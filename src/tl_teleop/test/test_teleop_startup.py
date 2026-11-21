"""Run the real executable without a driver or any hardware connection.

The domain, runtime lock directory and every hardware-facing ROS name are
isolated. No driver, VR service client or serial device is supplied by the test.
"""

import os
from pathlib import Path
import secrets
import signal
import subprocess
import time
import uuid

import pytest


class _TeleopProcess:
    def __init__(self, command, environment, log_path, deadline):
        self.log_path = log_path
        self.deadline = deadline
        self.log_file = log_path.open('w')
        self.process = subprocess.Popen(
            command, env=environment, stdout=self.log_file, stderr=subprocess.STDOUT)

    def output(self):
        return self.log_path.read_text(errors='replace')

    def assert_no_sdk(self):
        output = self.output()
        if 'Initializing PXREA' in output or 'PXREA SDK initialized' in output:
            # Fail immediately if startup regresses to acquiring the shared VR
            # client before discovering the deliberately absent driver.
            self.kill()
            pytest.fail('PXREA initialization occurred without a driver:\n' + output)
        return output

    def wait(self, predicate, timeout, description):
        deadline = min(self.deadline, time.monotonic() + timeout)
        while time.monotonic() < deadline:
            output = self.assert_no_sdk()
            if predicate(output):
                return output
            if self.process.poll() is not None:
                output = self.assert_no_sdk()
                if predicate(output):
                    return output
                pytest.fail(f'Process exited while {description}:\n{self.output()}')
            time.sleep(.01)
        self.kill()
        pytest.fail(f'Timed out while {description}:\n{self.output()}')

    def await_driver_wait(self):
        self.wait(lambda output: 'Waiting for tl_driver services' in output,
                  3., 'waiting for driver discovery')
        assert self.process.poll() is None, self.output()

    def interrupt(self):
        started = time.monotonic()
        self.process.send_signal(signal.SIGINT)
        self.wait(lambda _: self.process.poll() is not None, 2.5,
                  'cancelling startup with SIGINT')
        elapsed = time.monotonic() - started
        assert elapsed < 2.5, f'SIGINT took {elapsed:.3f}s'
        assert self.process.returncode == 0, self.output()
        assert 'cancelled while waiting for tl_driver' in self.output()
        self.assert_no_sdk()

    def kill(self):
        if self.process.poll() is None:
            self.process.kill()
            self.process.wait(timeout=1.)

    def close(self):
        self.kill()
        self.log_file.close()


def test_wait_cancel_and_single_instance_lock(tmp_path):
    package = Path(__file__).resolve().parents[1]
    default_executable = package.parents[1] / 'build' / 'tl_teleop' / 'tl_teleop'
    executable = Path(os.environ.get('TL_TELEOP_TEST_EXECUTABLE', default_executable))
    assert executable.is_absolute() and executable.is_file(), (
        'Build tl_teleop first, or set TL_TELEOP_TEST_EXECUTABLE to its absolute path')
    # Do not accidentally execute an old pre-fix binary that initializes the
    # real SDK immediately. CMake passes the newly built target explicitly.
    assert b'Waiting for tl_driver services (up to 30s' in executable.read_bytes(), (
        'Rebuild tl_teleop before this hardware-isolated startup regression test')

    tmp_path.chmod(0o700)
    environment = os.environ.copy()
    environment.update({
        'ROS_DOMAIN_ID': str(100 + secrets.randbelow(81)),
        'ROS_LOCALHOST_ONLY': '1',
        'XDG_RUNTIME_DIR': str(tmp_path),
        'TL_TELEOP_INSTANCE_LOCK': str(tmp_path / 'instance.lock'),
        'ROS_LOG_DIR': str(tmp_path / 'log'),
        'RCUTILS_LOGGING_BUFFERED_STREAM': '0',
    })
    namespace = '/isolated_teleop_startup_' + uuid.uuid4().hex
    command = [
        str(executable), '--ros-args', '--params-file',
        str(package / 'config' / 'tl_teleop_6axis_config.yaml'),
        '-p', 'gripper_enabled:=false',
    ]
    # A unique service remap guarantees absence even if another test happens to
    # choose the same nonzero domain. Also isolate every arm command/feedback.
    for name in (
            '/tl_driver/set_speed', '/tl_driver/set_current_mode',
            '/tl_driver/open_servoj', '/tl_driver/close_servoj',
            '/tl_driver/coord_transform', '/tl_driver/set_servoj_pos',
            '/joint_states', '/tcp_pose'):
        command.extend(['-r', f'{name}:={namespace}{name}'])

    deadline = time.monotonic() + 12.
    processes = []

    def start(label):
        process = _TeleopProcess(command, environment, tmp_path / f'{label}.log', deadline)
        processes.append(process)
        return process

    try:
        first = start('first')
        first.await_driver_wait()
        lock_path = tmp_path / 'instance.lock'
        assert lock_path.read_text().strip() == str(first.process.pid)

        second = start('second')
        second.wait(lambda _: second.process.poll() is not None, 1.5,
                    'rejecting a duplicate teleop process')
        assert second.process.returncode != 0, second.output()
        assert 'Another tl_teleop instance is running' in second.output()
        assert f'PID {first.process.pid}' in second.output()
        assert 'TL_Teleop node initialized' not in second.output()
        assert first.process.poll() is None, first.output()
        assert lock_path.read_text().strip() == str(first.process.pid)
        first.interrupt()

        # Reuse the same inode and directory after orderly shutdown: release of
        # the OS lock, rather than deletion of the file, permits the new owner.
        assert lock_path.is_file()
        third = start('after_exit')
        third.await_driver_wait()
        assert lock_path.read_text().strip() == str(third.process.pid)
        third.interrupt()
    finally:
        for process in processes:
            process.close()
