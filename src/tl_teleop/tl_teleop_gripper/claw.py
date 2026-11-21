"""Validated Lebai transactions, migrated from vr_tcb610.hardware.claw."""

import time

from .vendor import claw_sdk as vendor


class CheckedClaw(vendor.LebaiClaw):
    READ_REGISTERS = {0x9C45, 0x9C46, 0x9C47, 0x9C49, 0x9C4A}
    WRITE_REGISTERS = {0x9C40, 0x9C41, 0x9C4A}

    def __init__(self, *args, packet_logger=None, **kwargs):
        self.motion_allowed = False
        self.packet_logger = packet_logger
        super().__init__(*args, **kwargs)
        try:
            # pyserial uses an exclusive flock on Linux. No requests have been
            # sent yet; reject a second cooperating process before any I/O.
            self._ser.exclusive = True
            self._ser.write_timeout = .5
        except Exception:
            self.disconnect()
            raise

    def _send(self, payload, read_len):
        if len(payload) < 6 or payload[0] != self._slave_id:
            raise ValueError('Invalid Modbus request')
        function, register = payload[1], int.from_bytes(payload[2:4], 'big')
        if function == 3:
            if (len(payload) != 6 or register not in self.READ_REGISTERS
                    or payload[4:6] != b'\x00\x01'):
                raise PermissionError('Only reviewed single-register reads are allowed')
        elif function == 0x10:
            if not self.motion_allowed or register not in self.WRITE_REGISTERS:
                raise PermissionError('Motion is disabled; homing and persistent writes are forbidden')
            if len(payload) != 9 or payload[4:7] != b'\x00\x01\x02':
                raise ValueError('Invalid single-register write')
            value = int.from_bytes(payload[7:9], 'big')
            if register == 0x9C40 and value not in (0, 100):
                raise ValueError('Only open (100) and close (0) targets are supported')
            if register == 0x9C41 and not 1 <= value <= 20:
                raise ValueError('Gripper force must be within 1..20')
            if register == 0x9C4A and not 1 <= value <= 30:
                raise ValueError('Gripper speed must be within 1..30')
        else:
            raise PermissionError('Unsupported Modbus function')
        started = time.monotonic()
        response = None
        error = None
        try:
            response = super()._send(payload, read_len)
            if response[:2] != payload[:2]:
                raise ValueError('Response slave address or function does not match')
            if function == 3 and (len(response) != 7 or response[2] != 2):
                raise ValueError('Invalid read response length')
            if function == 0x10 and (len(response) != 8 or response[2:6] != payload[2:6]):
                raise ValueError('Write acknowledgement register/count does not match')
            return response
        except BaseException as exc:
            error = repr(exc)
            raise
        finally:
            if self.packet_logger:
                self.packet_logger({
                    'function': function, 'register': hex(register),
                    'tx_hex': (payload + self._crc16(payload)).hex(' '),
                    'rx_hex': None if response is None else response.hex(' '),
                    'transaction_ms': (time.monotonic() - started) * 1000,
                    'error': error,
                })

    def get_state(self):
        position, torque = self.get_position(), self.get_torque()
        done = self._read(self._REG_CMD_DONE)
        uncalibrated = self._read(self._REG_NOT_CALIBRATED)
        if not (0 <= position <= 100 and 0 <= torque <= 100
                and done in (0, 1) and uncalibrated in (0, 1)):
            raise ValueError('Invalid gripper feedback')
        return {'position': position, 'torque': torque, 'is_done': bool(done),
                'calibrated': not bool(uncalibrated)}
