"""Mechanism experiment (diagnosis only): does IsolatedFakeStack.stop() let a queued callback run
after its entities are destroyed (rclpy prints 'The following exception was never retrieved:
cannot use Destroyable because destruction was requested' when the orphaned Task is collected)?
No gate, no goal: the stack is started and stopped CYCLES times; stderr is counted by the caller.

usage: stress_teardown.py STACK(original|repo) DOMAIN CYCLES
"""
import gc
import importlib.util
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WS = '/home/user/spiderx_ws'


def main():
    variant, domain, cycles = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    path = (os.path.join(HERE, 'original_stack', 'm6d_isolated_stack.py') if variant == 'original'
            else os.path.join(WS, 'src/spiderx_controller/test/m6d_isolated_stack.py'))
    spec = importlib.util.spec_from_file_location('m6d_isolated_stack_' + variant, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if os.environ.get('TRACE_ORPHANS'):
        import traceback
        from rclpy import task as rtask

        def _del(self):
            if self._exception is not None and not self._exception_fetched:
                print('ORPHAN', type(self).__name__, repr(self._exception), flush=True)
                print(''.join(traceback.format_exception(self._exception)), flush=True)
                h = getattr(self, '_handler', None)
                print('ORPHAN handler', getattr(h, '__qualname__', h), 'args',
                      [type(x).__name__ for x in (getattr(self, '_args', None) or ())], flush=True)
        rtask.Future.__del__ = _del
    rng = random.Random(1234)
    names = [f'j{i}' for i in range(12)]
    for i in range(cycles):
        stack = mod.IsolatedFakeStack(domain, names, [0.0] * 12)
        time.sleep(rng.uniform(0.05, 0.25))
        stack.stop()
        del stack
        gc.collect()
        print(f'cycle {i + 1} stopped', flush=True)
    print(f'DONE {variant} {cycles} cycles, file {path}', flush=True)


if __name__ == '__main__':
    main()
