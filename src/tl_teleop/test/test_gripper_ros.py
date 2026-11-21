"""Exercise the real ROS node against an isolated pseudo-terminal Modbus slave.

Only the newly allocated PTY is opened. All input and output topics are remapped
to a UUID namespace; this test never starts or calls the robot driver.
"""

import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import threading
import time
import tty
import uuid

import rclpy
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


def _crc16(payload):
    crc = 0xFFFF
    for value in payload:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc.to_bytes(2, 'little')


class _ModbusSlave:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.port = os.ttyname(self.slave)
        assert self.port.startswith('/dev/pts/')
        self.registers = {0x9C45: 37, 0x9C46: 0, 0x9C47: 1, 0x9C49: 0}
        self.requests = []
        self.errors = []
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.corrupt_crc = threading.Event()
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _reply(self, frame):
        assert frame[-2:] == _crc16(frame[:-2]), 'Bad request CRC'
        slave, function = frame[:2]
        register = int.from_bytes(frame[2:4], 'big')
        assert slave == 1 and frame[4:6] == b'\x00\x01'
        if function == 3:
            assert register in self.registers, f'Unexpected read: {register:#x}'
            value = self.registers[register]
            payload = bytes([slave, function, 2]) + value.to_bytes(2, 'big')
        else:
            assert function == 0x10 and frame[6] == 2
            assert register in {0x9C40, 0x9C41, 0x9C4A}, 'Forbidden write'
            value = int.from_bytes(frame[7:9], 'big')
            if register == 0x9C40:
                assert value in (0, 100)
                self.registers[0x9C45] = value
            elif register == 0x9C41:
                assert 1 <= value <= 20
            else:
                assert 1 <= value <= 30
            payload = frame[:6]
        with self.lock:
            self.requests.append((function, register, value))
        response = payload + _crc16(payload)
        if self.corrupt_crc.is_set():
            response = response[:-1] + bytes([response[-1] ^ 1])
        os.write(self.master, response)

    def _serve(self):
        buffer = bytearray()
        try:
            while not self.stop.is_set():
                if not select.select([self.master], [], [], .02)[0]:
                    continue
                buffer.extend(os.read(self.master, 256))
                while len(buffer) >= 2:
                    function = buffer[1]
                    assert function in (3, 0x10), 'Unexpected Modbus function'
                    if function == 0x10 and len(buffer) < 7:
                        break
                    size = 8 if function == 3 else 9 + buffer[6]
                    if len(buffer) < size:
                        break
                    frame = bytes(buffer[:size])
                    del buffer[:size]
                    self._reply(frame)
        except Exception as error:
            if not self.stop.is_set():
                self.errors.append(repr(error))

    def writes(self):
        with self.lock:
            return [(register, value) for function, register, value in self.requests
                    if function == 0x10]

    def targets(self):
        return [value for register, value in self.writes() if register == 0x9C40]

    def disconnect(self):
        self.stop.set()
        self.thread.join(timeout=1)
        if self.master is not None:
            os.close(self.master)
            self.master = None

    def close(self):
        self.disconnect()
        os.close(self.slave)


class _RosHarness:
    def __init__(self, context, tmp_path):
        self.slave = _ModbusSlave()
        suffix = uuid.uuid4().hex
        topic = f'/isolated_gripper_test_{suffix}'
        self.node = rclpy.create_node(f'gripper_test_{suffix}', context=context)
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        self.ready = []
        self.states = []
        self.publisher = self.node.create_publisher(
            Joy, topic + '/input', QoSProfile(depth=1, lifespan=Duration(seconds=.2)))
        self.subscriptions = [
            self.node.create_subscription(
                Bool, topic + '/ready',
                lambda msg: self.ready.append((time.monotonic(), msg.data)), 10),
            self.node.create_subscription(
                String, topic + '/state',
                lambda msg: self.states.append(json.loads(msg.data)), 10),
        ]
        package = Path(__file__).resolve().parents[1]
        env = os.environ.copy()
        env['PYTHONPATH'] = str(package) + os.pathsep + env.get('PYTHONPATH', '')
        env['ROS_LOG_DIR'] = str(tmp_path / suffix)
        self.log_path = tmp_path / f'{suffix}.log'
        self.log = self.log_path.open('w')
        self.process = subprocess.Popen([
            '/usr/bin/python3', str(package / 'scripts' / 'tl_gripper'),
            '--ros-args', '-p', f'port:={self.slave.port}',
            '-p', 'serial_timeout:=0.15',
            '-r', f'__node:=gripper_under_test_{suffix}',
            '-r', f'/tl_teleop/gripper_input:={topic}/input',
            '-r', f'/tl_teleop/gripper_ready:={topic}/ready',
            '-r', f'/tl_teleop/gripper_state:={topic}/state',
        ], env=env, stdout=self.log, stderr=subprocess.STDOUT)

    def _check(self):
        assert self.process.poll() is None, self.log_path.read_text()
        assert not self.slave.errors, self.slave.errors

    def pump(self, duration, trigger=None, enabled=True, age=0.):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._check()
            if trigger is not None:
                message = Joy()
                message.header.stamp = (
                    self.node.get_clock().now() - Duration(seconds=age)).to_msg()
                message.axes = [float(trigger)]
                message.buttons = [int(enabled)]
                self.publisher.publish(message)
            # Refresh at 50 Hz throughout the SDK's four blocking status reads.
            self.executor.spin_once(timeout_sec=.005)
            time.sleep(.015)

    def wait_for(self, predicate, timeout=6., **input_values):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.pump(.02, **input_values)
        assert predicate(), self.log_path.read_text()

    def await_ready(self):
        self.wait_for(lambda: self.ready and self.ready[-1][1]
                      and self.publisher.get_subscription_count() == 1)

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self.log.close()
        self.executor.shutdown()
        self.node.destroy_node()
        self.slave.close()


def test_ros_trigger_and_serial_failures_are_isolated(tmp_path):
    context = Context()
    rclpy.init(context=context)
    harness = None
    try:
        harness = _RosHarness(context, tmp_path)
        harness.await_ready()
        harness.pump(.55)
        assert harness.slave.writes() == [], 'Startup must only read status'
        assert {request[1] for request in harness.slave.requests} == {
            0x9C45, 0x9C46, 0x9C47, 0x9C49}
        assert harness.states[-1]['state']['position'] == 37

        harness.pump(.6, trigger=0.)
        assert harness.slave.writes() == [], 'Engaging Grip must not open the claw'
        harness.wait_for(lambda: harness.slave.targets() == [0], trigger=1.)
        harness.pump(.4, trigger=1.)
        assert harness.slave.writes() == [(0x9C41, 10), (0x9C4A, 20), (0x9C40, 0)]
        harness.wait_for(lambda: harness.slave.targets() == [0, 100], trigger=0.)
        harness.wait_for(lambda: harness.slave.targets() == [0, 100, 0], trigger=1.)

        # Grip release while carrying an object must not issue an open command.
        previous_writes = harness.slave.writes()
        harness.pump(.6, trigger=0., enabled=False)
        assert harness.slave.writes() == previous_writes

        # A delayed released-trigger frame must not open the currently held claw.
        harness.pump(.35, trigger=1.)
        harness.pump(.6, trigger=0., age=1.)
        harness.pump(.3)
        assert harness.slave.writes() == previous_writes

        harness.slave.corrupt_crc.set()
        harness.wait_for(lambda: harness.states and harness.states[-1]['error']
                         and harness.ready and not harness.ready[-1][1])
        assert 'CRC' in harness.states[-1]['error']
        fault_time = time.monotonic()
        harness.pump(.3, trigger=1.)
        harness.pump(.3, trigger=0.)
        assert harness.slave.writes() == previous_writes
        assert sum(stamp >= fault_time and not ready for stamp, ready in harness.ready) >= 2
        harness.close()
        harness = None

        # A physical serial disconnect also clears readiness and stays latched.
        harness = _RosHarness(context, tmp_path)
        harness.await_ready()
        harness.slave.disconnect()
        harness.wait_for(lambda: harness.states and harness.states[-1]['error']
                         and harness.ready and not harness.ready[-1][1])
        harness.pump(.3, trigger=1.)
        assert not harness.states[-1]['running']
        assert harness.slave.writes() == []
    finally:
        if harness is not None:
            harness.close()
        context.shutdown()
