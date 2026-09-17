"""Read the known mainboard's cached gripper angle using SWD HotPlug.

No serial commands, writes, halt, reset, or programming. This is a temporary
readback path until the installed firmware exposes gripper telemetry over CDC.
"""
import math
from pathlib import Path
import re
import struct
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parent.parent
CLI = ROOT / 'tools/stlink-utility-4.6.0/extracted/program files/STMicroelectronics/STM32 ST-LINK Utility/ST-LINK Utility/ST-LINK_CLI.exe'
PROBE_SN = '2302340225005A504E413836'
POINTER = 0x200036A8
SIGNATURES = {
    0x080149F0: bytes.fromhex('594982aa284617f055fb012852d1829a'),
    0x08014B54: bytes.fromhex('7c79030888790308a8350020333313400000824298790308'),
}


class ReadbackError(Exception):
    pass


class GripperFeedback:
    def __init__(self, cli=CLI, run=subprocess.run):
        self.cli, self._run = Path(cli), run
        self._lock = threading.Lock()
        self._last_attempt = float('-inf')
        self._last = self.unavailable('not_sampled', '尚未读取夹爪角度。')

    @staticmethod
    def unavailable(code, message):
        return {'status': 'unavailable', 'angle_deg': None, 'unit': 'degree',
                'source': 'stlink_mainboard_cache', 'sampled_at_ms': None,
                'monotonic_ns': None, 'device_sample_at_ms': None,
                'reason': code, 'message': message}

    def _memory(self, ranges):
        args = [str(self.cli), '-c', 'SN=' + PROBE_SN, 'SWD', 'HOTPLUG']
        for address, count in ranges:
            args += ['-r32', hex(address), hex(count)]
        result = self._run(args, capture_output=True, timeout=1.5,
                           creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        output = result.stdout.decode('utf-8', errors='replace')
        if (result.returncode != 0 or 'Connection mode: HotPlug' not in output
                or 'Device ID: 0x413' not in output or PROBE_SN not in output):
            raise ReadbackError('stlink_unavailable')
        memory = {}
        for match in re.finditer(r'^0x([0-9A-Fa-f]{8})\s*:\s*((?:[0-9A-Fa-f]{8}(?:[ \t]+|$))+)', output, re.M):
            address = int(match.group(1), 16)
            for word in re.findall(r'[0-9A-Fa-f]{8}', match.group(2)):
                for byte in struct.pack('<I', int(word, 16)):
                    memory[address] = byte
                    address += 1
        try:
            return {address: bytes(memory[address + i] for i in range(count))
                    for address, count in ranges}
        except KeyError as exc:
            raise ReadbackError('stlink_incomplete_read') from exc

    def read(self):
        # A concurrent caller shares a recent result; no unbounded probe queue.
        if not self._lock.acquire(blocking=False):
            return self.unavailable('stlink_busy', '夹爪角度正在读取，请稍后刷新。')
        try:
            if time.monotonic() - self._last_attempt < 0.5:
                return dict(self._last)
            self._last_attempt = time.monotonic()
            try:
                if not self.cli.is_file():
                    raise ReadbackError('stlink_tool_missing')
                ranges = [(address, len(signature)) for address, signature in SIGNATURES.items()]
                header = self._memory(ranges + [(POINTER, 4)])
                if any(header[address] != signature for address, signature in SIGNATURES.items()):
                    raise ReadbackError('gripper_firmware_mismatch')
                pointer = struct.unpack('<I', header[POINTER])[0]
                if pointer % 4 or not 0x20000000 <= pointer <= 0x2001FFB0:
                    raise ReadbackError('gripper_pointer_invalid')
                # Recheck pointer after object read to catch reboot/reallocation.
                values = self._memory([(pointer, 0x50), (POINTER, 4)])
                obj = values[pointer]
                if values[POINTER] != header[POINTER] or obj[4] != 7 or obj[0x18:0x1A] != b'\x00\x08':
                    raise ReadbackError('gripper_object_invalid')
                angle = struct.unpack_from('<f', obj, 8)[0]
                if not math.isfinite(angle):
                    raise ReadbackError('gripper_angle_invalid')
                self._last = {'status': 'available', 'angle_deg': angle, 'unit': 'degree',
                              'source': 'stlink_mainboard_cache', 'sampled_at_ms': time.time_ns() // 1_000_000,
                              'monotonic_ns': time.monotonic_ns(), 'device_sample_at_ms': None,
                              'reason': None, 'message': None}
            except (OSError, subprocess.TimeoutExpired, ReadbackError) as exc:
                code = str(exc) if isinstance(exc, ReadbackError) else 'stlink_read_failed'
                messages = {'gripper_firmware_mismatch': '主控固件已变化，夹爪只读地址需要重新核验。',
                            'stlink_tool_missing': '未找到 ST-LINK 读取工具。'}
                self._last = self.unavailable(code, messages.get(code, '夹爪读取不可用，请检查 ST-LINK 和主控供电。'))
            return dict(self._last)
        finally:
            self._lock.release()
