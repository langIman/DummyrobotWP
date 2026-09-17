"""Isolated ROS replay: no bridge, USB, HTTP commands or hardware driver."""
import os
os.environ['ROS_DOMAIN_ID'] = '194'
import json
import subprocess
import sys
import time
from pathlib import Path
import rclpy
from moveit_servo_gateway import GatewayNode
from audit_joint_mapping import audit
from continuous_target import HARDWARE_LIMITS

def main():
    root = Path(__file__).parent
    log = open(root / '../runtime/servo-offline.log', 'w')
    launch = subprocess.Popen([sys.executable, str(root / 'run_safe_launch.py')], stdout=log, stderr=log)
    rclpy.init()
    node = GatewayNode()
    import threading
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        deadline = time.monotonic() + 20
        while node.health()['status'] != 'ready' and time.monotonic() < deadline:
            time.sleep(.1)
        for label, pose in [('observed', [.1,-75.11,180.44,-.01,.15,.09]),
                            ('above_old_stop', [0,0,90,0,1,0]),
                            ('bent_wrist', [0,0,90,0,45,0])]:
            if label == 'above_old_stop':
                condition = audit(pose)['condition']
                assert 500 < condition < float('inf'), condition
            node.set_feedback(pose)
            okay, reason = node.arm()
            if not okay:
                raise RuntimeError(reason)
            results = []
            for step in range(100):
                node.set_feedback(pose)
                node.set_command(dict(linear_x=.10, linear_y=0., linear_z=0., angular_x=0., angular_y=0., angular_z=0.))
                time.sleep(.04)
                if step % 25 == 24:
                    results.append(node.latest_trajectory())
            node.disarm()
            print(json.dumps({'pose':label,'samples':results}), flush=True)
            if label in {'bent_wrist', 'above_old_stop'}:
                assert all(sample['positions'] for sample in results)
                assert all(sample['servo_status']['code'] == 0 for sample in results[-2:]), (
                    'ordinary control pose must run without singularity slowdown', results[-2:])
                gaps = [max(abs(a-b) for a,b in zip(sample['positions'],pose)) for sample in results]
                assert max(gaps) > 8.0, ('feedback lead cap must be disabled',gaps)
                assert all(low-1e-6 <= q <= high+1e-6 for sample in results
                           for q, (low, high) in zip(sample['positions'], HARDWARE_LIMITS))
            else:
                # This recorded pose used to trigger the 500 condition-number
                # stop. Joint-bound/collision checks may still stop this pose,
                # but finite singularity thresholds must no longer do so.
                assert all(sample['servo_status']['code'] not in {1, 2, 3} for sample in results), results
                assert all(not sample.get('guard_warning') for sample in results), results
            time.sleep(.2)
    finally:
        node.disarm()
        launch.send_signal(2)
        try:
            launch.wait(timeout=8)
        except subprocess.TimeoutExpired:
            launch.terminate()
        rclpy.shutdown()
        spinner.join(timeout=2)
        log.close()

if __name__ == '__main__':
    main()
