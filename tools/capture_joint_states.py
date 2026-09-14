import json
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

rclpy.init()
n = Node('capture')
samples = []
t0 = time.time()


def cb(m):
    samples.append({
        't': time.time() - t0,
        'p': list(m.position),
        'v': list(m.velocity),
    })


n.create_subscription(JointState, '/joint_states', cb, 100)
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
end = time.time() + dur
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.02)

out = sys.argv[2] if len(sys.argv) > 2 else '/tmp/capture.json'
with open(out, 'w') as f:
    json.dump(samples, f)
print(f'captured {len(samples)} samples -> {out}')
n.destroy_node()
rclpy.shutdown()
