"""Bounded GETJPOS-only diagnostic; capture worker stack on a stalled request."""
import argparse
import json
from pathlib import Path
import statistics
import subprocess
import threading
import time

import requests

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--worker-pid', type=int, required=True)
    parser.add_argument('--seconds', type=int, default=150)
    parser.add_argument('--hz', type=int, choices=[25, 40, 50], default=25)
    parser.add_argument('--passive', action='store_true', help='Watch existing bridge traffic only; send no requests')
    args = parser.parse_args()
    directory = ROOT / 'runtime' / ('serial-diagnostic-' + time.strftime('%Y%m%d-%H%M%S'))
    directory.mkdir()
    profiler = ROOT / 'runtime/diagnostic-tools/bin/py-spy.exe'
    session = requests.Session()
    samples, dumps = [], []
    start = time.monotonic()
    next_status = 0.
    reason = 'duration_complete'
    def capture(index):
        try:
            result = subprocess.run([str(profiler), 'dump', '--pid', str(args.worker_pid)],
                                    capture_output=True, timeout=2.5, creationflags=0x08000000)
            path = directory / f'stack-{index}.txt'
            path.write_bytes(result.stdout + result.stderr)
            dumps.append(str(path))
        except Exception as exc:
            dumps.append(str(exc))
    if args.passive:
        trace = ROOT.parents[1] / 'windows_dummy_bridge/runtime/transport_trace.jsonl'
        seen = set()
        while time.monotonic()-start < min(args.seconds, 300):
            with trace.open('rb') as stream:
                stream.seek(0, 2)
                stream.seek(max(0, stream.tell()-16384))
                lines = stream.read().splitlines()
            rows = []
            for line in lines:
                try: rows.append(json.loads(line))
                except ValueError: pass
            if rows:
                last = rows[-1]
                age = (time.time_ns()//1_000_000-last['at_ms'])/1000
                if last['kind'] == 'tx_attempt' and .6 < age < 3.2 and last['id'] not in seen:
                    seen.add(last['id'])
                    capture(last['id'])
                    print(json.dumps({'slow_transaction':last, 'stack_dumps':dumps}), flush=True)
                if (last.get('error') or {}).get('code') == 'worker_unavailable' and age < 2:
                    (directory/'failure.json').write_text(json.dumps(rows,indent=2), encoding='utf-8')
                    print(json.dumps({'result':'captured_failure','directory':str(directory)}), flush=True)
                    return
            time.sleep(.1)
        print(json.dumps({'result':'passive_timeout','stack_dumps':dumps,'directory':str(directory)}), flush=True)
        return
    while time.monotonic() - start < min(args.seconds, 300):
        now = time.monotonic()
        if now >= next_status:
            state = session.get('http://127.0.0.1:8770/api/status', timeout=3).json()
            if state['control']['armed'] or state['robot']['enabled_latch'] == 'confirmed':
                reason = 'operator_enabled_robot'
                break
            next_status = now + 1
        timer = threading.Timer(.6, capture, args=(len(samples),))
        timer.daemon = True
        began = time.monotonic()
        timer.start()
        try:
            response = session.post('http://127.0.0.1:8765/command', json={
                'command': '#GETJPOS', 'read_timeout_ms': 100}, timeout=6)
            value = response.json()
            sample = dict(at_ms=time.time_ns()//1_000_000,
                          elapsed_ms=round((time.monotonic()-began)*1000, 3), **value)
        except Exception as exc:
            sample = {'error': str(exc), 'elapsed_ms': (time.monotonic()-began)*1000}
        finally:
            timer.cancel()
            if timer.is_alive(): timer.join(3)
        samples.append(sample)
        if sample.get('error') or sample.get('read_timed_out'):
            reason = 'transport_failure'
            break
        if len(samples) % 500 == 0:
            print(json.dumps({'reads': len(samples), 'elapsed_seconds': round(time.monotonic()-start)}), flush=True)
        time.sleep(max(.001, 1/args.hz-(time.monotonic()-began)))
    session.close()
    elapsed = time.monotonic()-start
    summary = dict(reason=reason, seconds=round(elapsed, 2), reads=len(samples),
                   requested_hz=args.hz,
                   hz=round(len(samples)/elapsed, 2), motion_sent=False,
                   median_ms=statistics.median(s['elapsed_ms'] for s in samples) if samples else None,
                   stack_dumps=dumps, last=samples[-1] if samples else None)
    (directory/'report.json').write_text(json.dumps({'summary':summary,'samples':samples}, indent=2), encoding='utf-8')
    print(json.dumps({'directory':str(directory), **summary}), flush=True)


if __name__ == '__main__':
    main()
