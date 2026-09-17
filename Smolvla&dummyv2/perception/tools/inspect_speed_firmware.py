"""Offline speed audit for the hash-identified installed firmware; no device I/O."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

WORKSPACE = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WORKSPACE / 'tools/firmware-inspect-deps'))
from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB, CS_MODE_LITTLE_ENDIAN

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('directory', type=Path)
args = parser.parse_args()
flash = (args.directory / 'flash.bin').read_bytes()
sram = (args.directory / 'sram.bin').read_bytes()
expected = 'f52d9209fe2eeca72b6eb0235e6c5446c2a326b2a0f1a6f4ecaf6c590076c5af'
assert len(flash) == 0x100000 and hashlib.sha256(flash).hexdigest() == expected
assert len(sram) == 0x20000
base = 0x200035A8
offset = base - 0x20000000
def floats(at, count=1):
    return list(struct.unpack_from('<' + 'f'*count, sram, offset+at))
def safe(values):
    return [v if math.isfinite(v) else 'NaN' if math.isnan(v) else str(v) for v in values]

motors = []
for axis, address in enumerate(struct.unpack_from('<6I', sram, offset+0xE4), 1):
    p = address - 0x20000000
    assert 0 <= p < len(sram)-0x30 and sram[p+4] == axis
    motors.append({'axis': axis, 'object': hex(address), 'reduction': sram[p+0x19],
                   'inverse': bool(sram[p+0x18]),
                   'can_payload_position_speed': safe(struct.unpack_from('<2f', sram, p+0x20))})

md = Cs(CS_ARCH_ARM, CS_MODE_THUMB | CS_MODE_LITTLE_ENDIAN)
md.skipdata = True
regions = [(0x08012798, 0x080127D4), (0x08012A50, 0x08012A92),
           (0x08012B2C, 0x08012B80), (0x080114FC, 0x08011560),
           (0x08011A34, 0x08011D2C), (0x080119FC, 0x08011A34),
           (0x0800FCD8, 0x0800FD26), (0x080126B4, 0x08012720)]
lines = []
decoded = {}
for start, end in regions:
    lines.append(f'\nREGION {start:08x}..{end:08x}')
    for instruction in md.disasm(flash[start-0x08000000:end-0x08000000], start):
        decoded[instruction.address] = (instruction.mnemonic, instruction.op_str)
        lines.append(f'{instruction.address:08x} {instruction.mnemonic:12s} {instruction.op_str}')
assert decoded[0x08012B7A] == ('bl', '#0x8011a34')
assert decoded[0x08011C2A] == ('vdiv.f32', 's13, s10, s12')
assert decoded[0x08011C6E] == ('vstr', 's14, [r4, #0x158]')
assert decoded[0x08011D16] == ('vstr', 's15, [r4, #0x16c]')
assert decoded[0x080126D4] == ('bl', '#0x8011a34')
assert struct.unpack_from('<f', flash, 0x11D2C)[0] == struct.unpack('<f', struct.pack('<f', .1))[0]
assert struct.unpack_from('<f', flash, 0x12B04)[0] == 100.

report = {'flash_sha256': expected, 'robot_object': hex(base),
          'command_mode': sram[offset+0xDD], 'enabled': bool(sram[offset+0x174]),
          'effective_common_speed': floats(0x150)[0], 'speed_ratio': floats(0x154)[0],
          'current_angles': floats(0x54, 6), 'target_angles': floats(0x6C, 6),
          'dynamic_speed_cache': safe(floats(0x158, 6)), 'motors': motors,
          'formula': 'v_i = abs(delta_i) * R_i * (clamp(S,0,100)*ratio) * 0.1 / (max(abs(delta))*R_m)',
          'm_definition': 'axis with largest absolute joint-angle error',
          'all_zero_error_case': 'no denominator guard; 0 times infinity yields NaN',
          'hardware_motion_sent': False, 'snapshot_atomic': False}
(args.directory / 'speed-disassembly.txt').write_text('\n'.join(lines), encoding='utf-8')
(args.directory / 'speed-audit.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
print(json.dumps(report, allow_nan=False))
