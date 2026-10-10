"""Mechanism experiment (diagnosis only): does the fake controller stack publish /joint_states
stamps out of order, as seen by a single-threaded subscriber in the same process (the test's
topology)? No gate, no goal, no action client: only the stack's timer, /clock and /joint_states.

usage: stress_order.py STACK(original|repo) DOMAIN SECONDS OUT_JSON
"""
import importlib.util
import json
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WS = '/home/user/spiderx_ws'


def load_stack(variant):
    path = (os.path.join(HERE, 'original_stack', 'm6d_isolated_stack.py') if variant == 'original'
            else os.path.join(WS, 'src/spiderx_controller/test/m6d_isolated_stack.py'))
    spec = importlib.util.spec_from_file_location('m6d_isolated_stack_' + variant, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod, path


def main():
    variant, domain, seconds, out = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    mod, path = load_stack(variant)
    cls = mod.IsolatedFakeStack
    state = {'active': 0, 'overlaps': 0, 'max': 0, 'ticks': 0}
    lock = threading.Lock()
    orig_tick = cls._tick

    def _tick(self):
        with lock:
            state['active'] += 1
            state['ticks'] += 1
            state['overlaps'] += state['active'] > 1
            state['max'] = max(state['max'], state['active'])
        try:
            return orig_tick(self)
        finally:
            with lock:
                state['active'] -= 1
    cls._tick = _tick

    names = [f'j{i}' for i in range(12)]
    stack = cls(domain, names, [0.0] * 12)
    ctx = Context()
    rclpy.init(context=ctx, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('stress_order_subscriber', context=ctx, enable_rosout=False,
                             start_parameter_services=False)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    stamps = []
    node.create_subscription(JointState, '/joint_states',
                             lambda m: stamps.append(m.header.stamp.sec +
                                                     m.header.stamp.nanosec * 1e-9), 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        ex.spin_once(timeout_sec=0.05)
    wall = time.monotonic() - t0
    ex.shutdown()
    node.destroy_node()
    rclpy.try_shutdown(context=ctx)
    stack.stop()
    bad = [i for i in range(1, len(stamps)) if stamps[i] <= stamps[i - 1]]
    # windows of 200 consecutive samples (the readiness probe keeps the first 200 of its window)
    windows = max(1, len(stamps) // 200)
    bad_windows = len({i // 200 for i in bad})
    res = {'variant': variant, 'stack_file': path, 'domain': domain, 'wall_s': wall,
           'loadavg': os.getloadavg(), 'ticks': state['ticks'],
           'tick_overlaps': state['overlaps'], 'max_concurrent_ticks': state['max'],
           'received': len(stamps), 'order_violations': len(bad),
           'windows_of_200': windows, 'windows_with_violation': bad_windows,
           'examples': [{'i': i, 'prev': stamps[i - 1], 'cur': stamps[i],
                         'delta_s': stamps[i] - stamps[i - 1]} for i in bad[:10]]}
    with open(out, 'w') as f:
        json.dump(res, f, indent=1)
    print(json.dumps({k: res[k] for k in ('variant', 'wall_s', 'ticks', 'tick_overlaps',
                                          'max_concurrent_ticks', 'received',
                                          'order_violations', 'windows_of_200',
                                          'windows_with_violation')}))


if __name__ == '__main__':
    main()
