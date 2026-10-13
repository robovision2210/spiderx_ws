"""Diagnosis only: with a 0-25 ms delay between a tick computing its stamp and publishing it, do
received /joint_states stamps go backwards? usage: reorder_demo.py original|repo DOMAIN OUT_JSON"""
import importlib.util, json, os, random, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
variant, domain, out = sys.argv[1], int(sys.argv[2]), sys.argv[3]
os.environ['ROS_LOCALHOST_ONLY'] = '1'
path = (os.path.join(HERE, 'original_stack', 'm6d_isolated_stack.py') if variant == 'original'
        else '/home/user/spiderx_ws/src/spiderx_controller/test/m6d_isolated_stack.py')
spec = importlib.util.spec_from_file_location('stack_' + variant, path)
mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
rng = random.Random(7)
class Slow(mod.IsolatedFakeStack):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        real = self.js_pub
        class P:
            def publish(self, msg):
                time.sleep(rng.uniform(0.0, 0.025)); real.publish(msg)
        self.js_pub = P()
stack = Slow(domain, [f'j{i}' for i in range(12)], [0.0] * 12)
ctx = Context(); rclpy.init(context=ctx, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node('reorder_probe', context=ctx, enable_rosout=False, start_parameter_services=False)
st = []
node.create_subscription(JointState, '/joint_states', lambda m: st.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), 50)
ex = SingleThreadedExecutor(context=ctx); ex.add_node(node)
end = time.monotonic() + 5.0
while time.monotonic() < end:
    ex.spin_once(timeout_sec=0.05)
ex.shutdown(); node.destroy_node(); rclpy.try_shutdown(context=ctx); stack.stop()
back = [i for i in range(1, len(st)) if st[i] <= st[i - 1]]
w = [st[i:i + 200] for i in range(0, len(st) - 199, 200)]
res = {'variant': variant, 'received': len(st), 'backward_steps': len(back),
       'windows_of_200': len(w),
       'windows_failing_joint_states_stale': sum(1 for x in w if any(b <= a for a, b in zip(x, x[1:]))),
       'examples': [(st[i - 1], st[i]) for i in back[:5]]}
json.dump(res, open(out, 'w'), indent=1); print(json.dumps(res))
