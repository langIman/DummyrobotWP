"""Read-only recording validation; never contacts a service or robot."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def rows(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line]


def validate(root):
    session = json.loads((root / 'session.json').read_text(encoding='utf-8'))
    assert session['status'] == 'complete', session['status']
    assert not session['errors'], session['errors']
    wrist = rows(root / 'wrist/frames.jsonl')
    oak = rows(root / 'oak/frames.jsonl')
    robot = rows(root / 'robot_state.jsonl')
    events = rows(root / 'events.jsonl')
    alignment = rows(root / 'alignment.jsonl')
    for name, stream in [('wrist', wrist), ('oak', oak), ('robot', robot), ('events', events)]:
        assert len(stream) == session['counts'][name], name
        assert [row['sequence'] for row in stream] == list(range(len(stream))), name
        times = [row['monotonic_ns'] for row in stream if row.get('monotonic_ns') is not None]
        assert all(b >= a for a, b in zip(times, times[1:])), name
    assert wrist and oak and robot
    device_times = [row['device_timestamp_ns'] for row in oak]
    assert all(b > a for a, b in zip(device_times, device_times[1:]))
    assert len(list((root / 'wrist/rgb').glob('*.jpg'))) == len(wrist)
    assert len(list((root / 'oak/rgb').glob('*.jpg'))) == len(oak)
    assert len(list((root / 'oak/depth').glob('*.png'))) == len(oak)
    valid_pixels = pixels = 0
    for channel, stream in [('wrist', wrist), ('oak', oak)]:
        for row in stream:
            rgb = root / channel / row['file' if channel == 'wrist' else 'rgb_file']
            image = cv2.imdecode(np.frombuffer(rgb.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
            assert image is not None, rgb
            size = (row['height'], row['width']) if channel == 'wrist' else (row['rgb_height'], row['rgb_width'])
            assert image.shape[:2] == size
            if channel == 'oak':
                depth_path = root / channel / row['depth_file']
                depth = cv2.imdecode(np.frombuffer(depth_path.read_bytes(), dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                assert depth is not None and depth.dtype == np.uint16
                assert depth.shape == (row['depth_height'], row['depth_width'])
                valid_pixels += np.count_nonzero(depth)
                pixels += depth.size
    assert valid_pixels > 0
    assert json.loads((root / 'oak/calibration.json').read_text(encoding='utf-8'))
    assert len(alignment) == len(oak)
    for row, frame in zip(alignment, oak):
        assert row['oak_sequence'] == frame['sequence']
        for stream_name, stream in [('wrist', wrist), ('robot', robot)]:
            chosen = stream[row[stream_name + '_sequence']]
            difference = (chosen['monotonic_ns'] - frame['monotonic_ns']) / 1e6
            assert abs(row[stream_name + '_delta_ms'] - difference) <= .00051
            nearest = min(abs(item['monotonic_ns'] - frame['monotonic_ns']) for item in stream if item.get('monotonic_ns')) / 1e6
            assert abs(abs(difference) - nearest) <= .00051
    return {
        'session_id': session['session_id'], 'result': 'passed',
        'duration_ms': session['duration_ms'], 'counts': session['counts'],
        'dropped': session['dropped'], 'all_images_decoded': True,
        'depth_dtype': 'uint16', 'valid_depth_fraction': round(valid_pixels / pixels, 4),
        'nearest_pairs_verified': len(alignment),
        'max_wrist_pair_delta_ms': max(abs(row['wrist_delta_ms']) for row in alignment),
        'max_robot_pair_delta_ms': max(abs(row['robot_delta_ms']) for row in alignment),
        'robot_stream_targets': sum(row.get('type') == 'robot_stream_target' for row in events),
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    result = validate(args.session)
    content = json.dumps(result, ensure_ascii=False, indent=2)
    if args.report:
        args.report.write_text(content + '\n', encoding='utf-8')
    print(content)
