"""Passively watch bridge logs and save STM32 memory through ST-LINK HotPlug.

No USB CDC requests, halt, reset, register writes, or programming commands.
Snapshots of running memory are not atomic and are not a CPU register dump.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[1]
TRACE = WORKSPACE / 'windows_dummy_bridge/runtime/transport_trace.jsonl'
CLI = WORKSPACE / ('tools/stlink-utility-4.6.0/extracted/program files/'
                   'STMicroelectronics/STM32 ST-LINK Utility/ST-LINK Utility/ST-LINK_CLI.exe')
SERIAL = '2302340225005A504E413836'


def fault_reason(row):
    if row.get('kind') != 'result':
        return None
    raw = row.get('raw_response', '')
    if 'terminate called after throwing an instance of' in raw:
        return 'firmware_termination'
    if '\x00' in raw:
        return 'nul_in_ascii_reply'
    error = row.get('error') or {}
    if isinstance(error, dict) and error.get('code') in {
        'worker_unavailable', 'read_error', 'write_error', 'open_error'
    }:
        return error['code']
    return None


def capture(trigger):
    directory = ROOT / 'runtime' / ('mcu-fault-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    directory.mkdir()
    (directory / 'trigger.json').write_text(json.dumps(trigger, indent=2, ensure_ascii=False), encoding='utf-8')
    # Preserve the log immediately, before a high-rate stream can rotate it.
    for source, name in [(TRACE, 'transport.jsonl'), (ROOT / 'runtime/service.stderr.log', 'service.log')]:
        try:
            shutil.copyfile(source, directory / name)
        except OSError as exc:
            (directory / (name + '.error')).write_text(str(exc), encoding='utf-8')
    commands = []
    for index in (1, 2):
        args = [str(CLI), '-c', 'SN=' + SERIAL, 'SWD', 'HOTPLUG', '-SCore',
                '-r32', '0xE000ED00', '0x40', '-r32', '0xE000EDF0', '0x4',
                '-Dump', '0x20000000', '0x20000', str(directory / f'sram-{index}.bin')]
        if index == 1:
            args += ['-Dump', '0x10000000', '0x10000', str(directory / 'ccm.bin')]
        commands.append(args)
        began = time.time_ns() // 1_000_000
        try:
            run = subprocess.run(args, capture_output=True, timeout=20,
                                 creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            output = run.stdout + run.stderr
            (directory / f'capture-{index}.log').write_bytes(output)
            valid = (run.returncode == 0 and b'Connection mode: HotPlug' in output
                     and SERIAL.encode() in output and b'Device ID: 0x413' in output
                     and (directory / f'sram-{index}.bin').stat().st_size == 0x20000)
            if not valid:
                raise RuntimeError('ST-LINK did not confirm expected target and complete SRAM capture')
        except Exception as exc:
            (directory / 'capture-error.txt').write_text(str(exc), encoding='utf-8')
            break
        finally:
            (directory / f'timing-{index}.json').write_text(json.dumps({
                'started_at_ms': began, 'finished_at_ms': time.time_ns() // 1_000_000}), encoding='utf-8')
    manifest = {'trigger': trigger, 'commands': commands, 'hardware_motion_sent': False,
                'halt_or_reset_requested': False, 'atomic_snapshot': False,
                'files': {p.name: {'bytes': p.stat().st_size, 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
                          for p in directory.glob('*.bin')}}
    (directory / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps({'capture': str(directory), 'files': list(manifest['files']),
                      'error': (directory / 'capture-error.txt').exists()}), flush=True)
    return directory


def watch(seconds):
    started_ms = time.time_ns() // 1_000_000
    deadline = time.monotonic() + seconds
    print(json.dumps({'watching': str(TRACE), 'started_at_ms': started_ms,
                      'expires_in_seconds': seconds, 'sends_usb_commands': False}), flush=True)
    while time.monotonic() < deadline:
        try:
            with TRACE.open('rb') as stream:
                stream.seek(0, 2)
                stream.seek(max(0, stream.tell() - 65536))
                lines = stream.read().splitlines()
        except OSError:
            time.sleep(.2)
            continue
        for line in lines:
            try:
                row = json.loads(line)
            except (ValueError, UnicodeDecodeError):
                continue
            if row.get('at_ms', 0) < started_ms:
                continue
            reason = fault_reason(row)
            if reason:
                capture({'reason': reason, 'row': row})
                return  # One bounded capture; never repeatedly probe a failed target.
        time.sleep(.1)
    print(json.dumps({'result': 'watch_expired_without_new_fault'}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=900)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 1800:
        parser.error('--seconds must be between 1 and 1800')
    watch(args.seconds)
