"""Trigger intent and serial worker; no ROS or arm SDK dependency."""

import math
import threading
import time

from .claw import CheckedClaw


class GripperController:
    """Keep slow serial transactions separate from input and arm control.

    Input callbacks only update memory. The worker reads feedback and sends
    changed targets after checking the newest input immediately before each
    write. Losing input cancels unsent targets; it does not release an object or
    cancel a motion that has already been accepted by the gripper controller.
    """

    def __init__(self, port, slave_id=1, baudrate=115200, timeout=.5,
                 force=10, speed=20, input_timeout=.2, threshold=.5,
                 claw_factory=CheckedClaw, clock=time.monotonic):
        if not isinstance(port, str) or not port:
            raise ValueError('A gripper serial port is required')
        for name, value, lower, upper in (
                ('slave_id', slave_id, 1, 247), ('force', force, 1, 20),
                ('speed', speed, 1, 30), ('baudrate', baudrate, 1, 4000000)):
            if isinstance(value, bool) or not isinstance(value, int) or not lower <= value <= upper:
                raise ValueError(f'{name} must be an integer within {lower}..{upper}')
        for name, value, upper in (('timeout', timeout, 1.), ('input_timeout', input_timeout, .5)):
            if not math.isfinite(value) or not 0 < value <= upper:
                raise ValueError(f'{name} must be finite and within (0, {upper}]')
        if not math.isfinite(threshold) or not 0 < threshold < 1:
            raise ValueError('trigger threshold must be within (0, 1)')
        self._connection = dict(port=port, slave_id=slave_id, baudrate=baudrate,
                                timeout=timeout, debug=False)
        self._force, self._speed = force, speed
        self._input_timeout, self._threshold = input_timeout, threshold
        self._claw_factory, self._clock = claw_factory, clock
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._running = False
        self._started = False
        self._error = None
        self._state = None
        self._state_stamp = None
        self._input_stamp = None
        self._active = False
        self._pressed = False
        self._intent = None
        self._last_command = None
        self._commands_sent = 0
        # These flags are only used by the serial worker.
        self._force_set = False
        self._speed_set = False

    def _clear_input(self):
        self._active = False
        self._pressed = False
        self._intent = None
        self._input_stamp = None

    def update_input(self, trigger, enabled, age_s=0.0):
        """Receive a current input frame, preserving its original age.

        First engagement with a released trigger deliberately produces no open
        command. Engagement with a pressed trigger closes; subsequent trigger
        transitions while engaged close/open as in the original project.
        """
        now = self._clock()
        try:
            valid = (math.isfinite(trigger) and 0 <= trigger <= 1
                     and math.isfinite(age_s) and 0 <= age_s < self._input_timeout)
        except (TypeError, ValueError):
            valid = False
        with self._lock:
            if self._error or self._stop.is_set() or not enabled or not valid:
                self._clear_input()
                return
            if self._input_stamp is None or now - self._input_stamp >= self._input_timeout:
                self._clear_input()
            pressed = trigger >= self._threshold
            if (not self._active and pressed) or (self._active and pressed != self._pressed):
                self._intent = 0 if pressed else 100
            self._active, self._pressed = True, pressed
            self._input_stamp = now - age_s

    def _ready(self, now):
        return (self._running and self._error is None and self._state is not None
                and self._state['calibrated'] and self._state_stamp is not None
                and 0 <= now - self._state_stamp < 1.0 and not self._stop.is_set())

    def snapshot(self):
        now = self._clock()
        with self._lock:
            return {
                'ready': self._ready(now), 'error': self._error,
                'state': None if self._state is None else dict(self._state),
                'state_age_s': None if self._state_stamp is None else max(0., now - self._state_stamp),
                'last_command': self._last_command, 'commands_sent': self._commands_sent,
                'running': self._running,
            }

    def _current_goal(self):
        now = self._clock()
        with self._lock:
            if (not self._ready(now) or not self._active or self._input_stamp is None
                    or not 0 <= now - self._input_stamp < self._input_timeout):
                self._clear_input()
                return None
            return self._intent if self._intent != self._last_command else None

    def _service_cycle(self, claw):
        # A state consists of four 50 ms transactions in the migrated SDK.
        # Record the start, so the age also includes the query duration.
        started = self._clock()
        state = claw.get_state()
        with self._lock:
            self._state = dict(state)
            self._state_stamp = started
        if not state['calibrated']:
            raise RuntimeError('Gripper is not calibrated; automatic homing is disabled')
        if self._current_goal() is None:
            return
        claw.motion_allowed = True
        try:
            if not self._force_set:
                if self._current_goal() is None:
                    return
                claw.set_force(self._force)
                self._force_set = True
            if not self._speed_set:
                if self._current_goal() is None:
                    return
                claw.set_speed(self._speed, save=False)
                self._speed_set = True
            # Re-read the latest goal after configuration, rather than retaining
            # a command received before the blocking reads/writes above.
            goal = self._current_goal()
            if goal is None:
                return
            claw.move(goal, wait=False)
            with self._lock:
                self._last_command = goal
                self._commands_sent += 1
        finally:
            claw.motion_allowed = False

    def _run(self):
        try:
            with self._claw_factory(**self._connection) as claw:
                while not self._stop.is_set():
                    self._service_cycle(claw)
                    self._stop.wait(.05)
        except Exception as error:
            with self._lock:
                self._error = str(error) or type(error).__name__
                self._clear_input()
        finally:
            with self._lock:
                self._running = False

    def start(self):
        with self._lock:
            if self._started:
                raise RuntimeError('Gripper worker cannot restart; create a new controller after resolving the fault')
            if self._stop.is_set():
                raise RuntimeError('Gripper controller is closed')
            self._started = True
            self._running = True
            self._thread = threading.Thread(target=self._run, name='gripper-serial', daemon=True)
            self._thread.start()

    def close(self):
        self._stop.set()
        with self._lock:
            self._clear_input()
            thread = self._thread
        if thread is not None:
            # Four reads plus bounded configuration/write timeouts. No extra
            # opening, closing or homing command is issued during shutdown.
            thread.join(timeout=6.0)
