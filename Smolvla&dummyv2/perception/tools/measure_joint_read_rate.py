"""Read-only 25 Hz bridge check: GETJPOS only, never enable or motion commands."""
import json
from pathlib import Path
import statistics
import time
import sys

import requests

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from perception_service.robot import RobotClient


def main():
    samples = []
    session = requests.Session()
    for _ in range(75):
        started = time.monotonic()
        try:
            reply = session.post('http://127.0.0.1:8765/command', json={
                'command': '#GETJPOS', 'read_timeout_ms': 100}, timeout=2)
            reply.raise_for_status()
            value = reply.json()
            valid = not value.get('error') and RobotClient._parse_positions(value.get('raw_response', '')) is not None
            sample = {'started': started, 'elapsed_ms': (time.monotonic()-started)*1000,
                      'valid': valid, 'early_reply': value.get('read_ended_on_joint_reply', False),
                      'error': value.get('error')}
        except Exception as exc:
            sample = {'started': started, 'elapsed_ms': (time.monotonic()-started)*1000,
                      'valid': False, 'early_reply': False, 'error': str(exc)}
        samples.append(sample)
        if len(samples) >= 3 and not any(s['valid'] for s in samples[-3:]):
            break
        time.sleep(max(.001, .04-(time.monotonic()-started)))
    session.close()
    duration = samples[-1]['started']-samples[0]['started']
    latencies = sorted(s['elapsed_ms'] for s in samples)
    report = {'command': '#GETJPOS', 'motion_sent': False, 'target_hz': 25,
              'requests': len(samples), 'valid_replies': sum(s['valid'] for s in samples),
              'early_replies': sum(s['early_reply'] for s in samples),
              'request_rate_hz': round((len(samples)-1)/duration, 2),
              'median_roundtrip_ms': round(statistics.median(latencies), 2),
              'p95_roundtrip_ms': round(latencies[int((len(latencies)-1)*.95)], 2),
              'samples': samples}
    (ROOT/'runtime/joint-read-25hz.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({key: value for key, value in report.items() if key != 'samples'}))


if __name__ == '__main__':
    main()
