"""Offline exception-frame candidates from saved Cortex-M RAM; no device access."""
import argparse
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools/firmware-inspect-deps'))
from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB, CS_MODE_LITTLE_ENDIAN

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('directory', type=Path)
parser.add_argument('--flash', type=Path, default=ROOT / 'firmware_backups/dummy-mainboard-20260915-175818/flash-read1.bin')
args = parser.parse_args()
flash = args.flash.read_bytes()
decoder = Cs(CS_ARCH_ARM, CS_MODE_THUMB | CS_MODE_LITTLE_ENDIAN)
decoder.skipdata = True
for filename, base in [('sram-1.bin', 0x20000000), ('ccm.bin', 0x10000000)]:
    data = (args.directory / filename).read_bytes()
    print(filename)
    for offset in range(0, len(data) - 32, 4):
        words = struct.unpack_from('<8I', data, offset)
        r0, r1, r2, r3, r12, lr, pc, psr = words
        if not (0x08000000 <= pc < 0x08040000 and psr & (1 << 24) and not psr & 0x00F00000):
            continue
        if psr & 0x1ff > 110:
            continue
        instructions = list(decoder.disasm(flash[(pc & ~1)-0x08000000:(pc & ~1)-0x08000000+12], pc & ~1))
        print(f'frame candidate @{base+offset:08x}: ' + ' '.join(f'{n}={v:08x}' for n,v in zip(
            ['r0','r1','r2','r3','r12','lr','pc','xpsr'], words)))
        print(' ; '.join(f'{i.address:08x} {i.mnemonic} {i.op_str}' for i in instructions[:3]))
    for needle in [struct.pack('<I', 0x32333937), b'7932', b'terminate', b'bad_alloc']:
        hits, start = [], 0
        while True:
            found = data.find(needle, start)
            if found < 0:
                break
            hits.append(hex(base+found))
            start = found+1
        print(repr(needle), hits[:30])
