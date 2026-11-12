#!/usr/bin/env python3
"""S2 script driver (docs/S2_FIXED_BASE_CRAWL_DESIGN.md section 3), status-driven.

Drives m55_teleop_keyboard through a pseudo-terminal (keys only) and follows the locomotion
node's status topic. It never talks to the controller: every goal is built and sent by the node,
and only when the node's gate allows it (shadow mode otherwise: nothing is sent).

Script: per level L in --levels: one cycle forward, one reverse (space while the cycle's LAST
phase is in flight, so the node stops at the cycle boundary); then at level 0: stop + home,
lease (teleop killed with SIGKILL: no clean quit) + re-arm + home, and the emergency stop last.
Expected: 9 goals per run, 54 for three levels, +5 exercise goals = 59, and 1 cancel. In shadow
mode 58: after the lease the measured posture is still neutral, so there is no home goal.

Stop conditions (the driver presses x and exits 2): an unplanned FAULTED, any command rejection,
a goal count above the script, a refused arm/home, or a wait exceeding its wall cap
(sim duration / min RTF 0.1 + 60 s).

Usage: s2_driver.py --out <dir> [--levels 0,1,2] [--domain-id 0]
Writes <dir>/driver.jsonl (keys, decisions, every status) and <dir>/driver_summary.json."""
import argparse
import json
import os
import pty
import select
import signal
import sys
import threading
import time

P = 9                    # phases per cycle
MIN_RTF = 0.1            # = m55 dispatch.min_real_time_factor
LONGEST_GOAL_S = 4.7     # sim, 60 mm level (incl. 0.2 s lead-in)
CYCLE_S = (18.8, 21.1, 25.6)   # sim per cycle incl. lead-ins, levels 0..2
STATUS_TOPIC = '/spiderx/locomotion/status'


class Abort(Exception):
    pass


class Driver:
    def __init__(self, out, domain_id):
        self.out = open(os.path.join(out, 'driver.jsonl'), 'w')
        self.lock = threading.Lock()
        self.status = None
        self.status_n = 0
        self.seen_seqs = set()
        self.max_seq = 0
        self.expected_goals = 0
        self.planned_estop = False
        self.events = []
        self.hold_key = None
        self.fd = None
        self.pid = None
        self._ros(domain_id)
        threading.Thread(target=self._repeat, daemon=True).start()

    def close(self):
        """Stop the executor thread and the context before exit (otherwise the C++ executor
        aborts at interpreter exit: 'terminate called without an active exception')."""
        import rclpy
        self.ex.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        rclpy.try_shutdown(context=self.ctx)
        self.out.close()

    # ------------------------------------------------------------------ logging
    def log(self, **k):
        k['mono'] = round(time.monotonic(), 4)
        with self.lock:
            self.out.write(json.dumps(k, sort_keys=True) + '\n')
            self.out.flush()

    # ------------------------------------------------------------------ status
    def _ros(self, domain_id):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=domain_id)
        self.node = rclpy.create_node('s2_driver_status', context=self.ctx)
        self.node.create_subscription(String, STATUS_TOPIC, self._on_status, 10)
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(self.node)
        self.spin_thread = threading.Thread(target=self._spin, daemon=True)
        self.spin_thread.start()

    def _spin(self):
        try:
            self.ex.spin()
        except Exception:  # noqa: BLE001  (shutdown during spin)
            pass

    def _on_status(self, msg):
        st = json.loads(msg.data)
        g = st.get('goal')
        with self.lock:
            self.status = st
            self.status_n += 1
            if g is not None:
                self.seen_seqs.add(g['seq'])
                self.max_seq = max(self.max_seq, g['seq'])
        self.log(status=st)

    def st(self):
        with self.lock:
            return self.status

    def check(self):
        st = self.st()
        if st is None:
            return
        if st.get('command_rejections'):
            raise Abort(f'command rejections {st["command_rejections"]}')
        if st['state'] == 'FAULTED' and not self.planned_estop:
            raise Abort(f'unplanned FAULTED: {st.get("faults")}')
        if self.max_seq > self.expected_goals:
            raise Abort(f'goal seq {self.max_seq} above the script ({self.expected_goals})')

    def wait(self, what, pred, sim_s):
        cap = sim_s / MIN_RTF + 60.0
        end = time.monotonic() + cap
        while time.monotonic() < end:
            self.check()
            st = self.st()
            if st is not None and pred(st):
                self.log(reached=what)
                return st
            time.sleep(0.05)
        raise Abort(f'wall cap {cap:.0f} s exceeded waiting for: {what}')

    # ------------------------------------------------------------------ teleop
    def spawn(self):
        pid, fd = pty.fork()
        if pid == 0:
            os.execvp('ros2', ['ros2', 'run', 'spiderx_controller', 'm55_teleop_keyboard'])
        self.pid, self.fd = pid, fd
        threading.Thread(target=self._pump, args=(fd,), daemon=True).start()
        self.log(event='teleop_started', pid=pid)

    def _pump(self, fd):
        while True:
            try:
                r, _, _ = select.select([fd], [], [], 0.1)
                if r:
                    data = os.read(fd, 4096)
                    if not data:
                        return
                    self.log(teleop_out=data.decode(errors='replace'))
            except (OSError, ValueError):
                return

    def key(self, k, why):
        os.write(self.fd, k.encode())
        self.log(key=repr(k), why=why)

    def _repeat(self):
        """Key repeat as a terminal does it: 0.5 s delay, then 30 Hz (the tested case)."""
        started = None
        while True:
            k = self.hold_key
            if k is None:
                started = None
            else:
                if started is None:
                    started = time.monotonic()
                if time.monotonic() - started >= 0.5:
                    try:
                        os.write(self.fd, k.encode())
                    except OSError:
                        pass
            time.sleep(1 / 30)

    def hold(self, k, why):
        self.key(k, why)
        self.hold_key = k
        self.log(hold=repr(k))

    def release(self, why):
        self.log(release=repr(self.hold_key), why=why)
        self.hold_key = None

    def kill_teleop(self):
        self.hold_key = None
        os.killpg(self.pid, signal.SIGKILL)          # the whole pty session: no clean quit
        try:
            os.waitpid(self.pid, 0)
        except ChildProcessError:
            pass
        self.log(event='teleop_killed', pid=self.pid, signal='SIGKILL')

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def ready_at(b):
        return lambda st: st['state'] == 'READY' and st['boundary'] == b and st['goal'] is None

    def last_phase_in_flight(self, direction):
        def pred(st):
            g = st.get('goal')
            return g is not None and g['direction'] == direction and \
                g['b_to'] == (P if direction > 0 else 0)
        return pred

    def run_cycle(self, k, direction, level, why):
        self.expected_goals += P
        self.hold(k, why)
        self.wait(f'{why}: last phase in flight', self.last_phase_in_flight(direction),
                  CYCLE_S[level])
        self.key(' ', f'{why}: stop at the cycle boundary')
        self.release(why)
        self.wait(f'{why}: READY at boundary 0', self.ready_at(0), LONGEST_GOAL_S + 2.0)
        time.sleep(1.5)                              # zeros after the hold window clear the latch

    def first_goal_in_flight(self, after_seq):
        return lambda st: st.get('goal') is not None and st['goal']['seq'] > after_seq

    def home(self, why):
        self.expected_goals += 1
        self.key('h', why)
        self.wait(f'{why}: homed at boundary 0', self.ready_at(0), 2 * LONGEST_GOAL_S + 2.0)

    def script(self, levels):
        self.spawn()
        self.wait('status with levels', lambda st: bool(st.get('levels_m_s')), 30.0)
        time.sleep(3.0)                               # the teleop receives the levels too
        self.key('a', 'arm')
        self.wait('READY at boundary 0 after arm', self.ready_at(0), 5.0)
        current = 0
        for L in levels:
            while current < L:
                self.key('+', f'level {L}')
                current += 1
                time.sleep(0.5)
            self.run_cycle('w', +1, L, f'level {L} forward cycle')
            self.run_cycle('s', -1, L, f'level {L} reverse cycle')
        while current > 0:
            self.key('-', 'back to level 0')
            current -= 1
            time.sleep(0.5)
        # exercise 1: stop (space while goal 1 in flight), then home
        self.expected_goals += 1
        seq = self.max_seq
        self.hold('w', 'stop exercise')
        self.wait('stop exercise: goal in flight', self.first_goal_in_flight(seq), 5.0)
        self.key(' ', 'stop exercise: space')
        self.release('stop exercise')
        self.wait('stop exercise: READY at boundary 1', self.ready_at(1), LONGEST_GOAL_S + 2.0)
        time.sleep(1.5)
        self.home('stop exercise: home')
        # exercise 2: lease (teleop killed while goal 1 in flight), re-arm at rest, home
        self.expected_goals += 1
        seq = self.max_seq
        self.hold('w', 'lease exercise')
        self.wait('lease exercise: goal in flight', self.first_goal_in_flight(seq), 5.0)
        self.kill_teleop()
        self.wait('lease exercise: DISARMED at boundary 1',
                  lambda st: st['state'] == 'DISARMED' and st['boundary'] == 1, 2 * LONGEST_GOAL_S + 2.0)
        self.spawn()
        time.sleep(4.0)                               # the new teleop receives the levels
        self.key('a', 'lease exercise: re-arm')
        st = self.wait('lease exercise: READY after re-arm',
                       lambda st: st['state'] == 'READY' and st['goal'] is None, 5.0)
        # dispatch: the joints are at boundary 1 and arm resolves there (this process's rest
        # record), then home walks 1 phase back. Shadow: nothing moved, the measured posture is
        # neutral, so arm resolves to boundary 0 and there is nothing to home (58 goals, not 59).
        want = 0 if st['mode'] == 'shadow' else 1
        if st['boundary'] != want:
            raise Abort(f're-armed at boundary {st["boundary"]}, expected {want} ({st["mode"]})')
        if want == 1:
            self.home('lease exercise: home')
        else:
            self.log(note='shadow: re-armed at boundary 0 (measured posture neutral); no home goal')
        # exercise 3: emergency stop while goal 1 in flight (last: the posture is mid-phase)
        self.expected_goals += 1
        seq = self.max_seq
        self.hold('w', 'estop exercise')
        self.wait('estop exercise: goal in flight', self.first_goal_in_flight(seq), 5.0)
        self.planned_estop = True
        self.key('x', 'estop exercise: x')
        self.release('estop exercise')
        self.wait('estop exercise: FAULTED', lambda st: st['state'] == 'FAULTED', 5.0)
        time.sleep(3.0)                               # the cancelled goal reports its result
        self.key('q', 'quit teleop')
        time.sleep(3.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--levels', default='0,1,2')
    ap.add_argument('--domain-id', type=int, default=0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    levels = [int(x) for x in a.levels.split(',')]
    d = Driver(a.out, a.domain_id)
    rc, why = 0, 'script complete'
    try:
        d.script(levels)
    except Abort as e:
        rc, why = 2, f'STOP CONDITION: {e}'
        try:
            d.key('x', 'stop condition: emergency stop')
        except OSError:
            pass
    st = d.st() or {}
    summary = {'result': why, 'levels': levels, 'expected_goals': d.expected_goals,
               'goal_seqs_seen': sorted(d.seen_seqs), 'max_seq': d.max_seq,
               'final_state': st.get('state'), 'faults': st.get('faults'),
               'command_rejections': st.get('command_rejections'),
               'goals_recorded': st.get('goals_recorded'), 'status_messages': d.status_n}
    with open(os.path.join(a.out, 'driver_summary.json'), 'w') as f:
        json.dump(summary, f, indent=1, sort_keys=True)
    d.log(summary=summary)
    print(json.dumps(summary, sort_keys=True), flush=True)
    if d.pid:
        try:
            os.killpg(d.pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
    try:
        d.close()
    except Exception as e:  # noqa: BLE001
        print(f'close: {e!r}', flush=True)
    sys.stdout.flush()
    os._exit(rc)      # skip interpreter teardown: the C++ executor can abort there (exit 134)


if __name__ == '__main__':
    main()
