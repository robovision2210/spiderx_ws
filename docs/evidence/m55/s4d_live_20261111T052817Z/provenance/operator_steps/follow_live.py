import json, os, sys, time
RUN = '/root/spiderx_evidence/s4d_live_20261111T052817Z/run'
DJ, ST, WR = RUN + '/driver/driver.jsonl', RUN + '/steps.log', RUN + '/run_wrapper.out'
pos = {DJ: 0, ST: 0}
seen_steps = sum(1 for _ in open(ST))
pos[ST] = os.path.getsize(ST)
def emit(s):
    print(s[:300], flush=True)
def brief(d):
    for k in ('cycle_check', 'release', 'switch', 'close', 'close_observation', 'probe', 'summary',
              'repeat_stop_before_q', 'speed_change', 'abort', 'stop', 'error', 'refused', 'emergency'):
        if k in d:
            v = d[k]
            if k == 'cycle_check':
                return (f"cycle_check {v.get('cycle')} dir {v.get('direction')} fwd {v.get('forward_m')} "
                        f"lat {v.get('lateral_m')} verdict {v.get('verdict', v.get('ok'))}")
            if k == 'summary':
                return f"summary: result {v.get('result')} goals {v.get('goals_sent', v.get('goals'))} cancels {v.get('cancels_sent', v.get('cancels'))}"
            return f"{k}: {json.dumps(v, default=str)[:240]}"
    ks = [k for k in d if k not in ('mono', 't_receipt', 'status', 'teleop_out', 'key', 'hold', 't_after_write',
                                   't_before_write', 'why', 'write_result', 'reached', 'event', 'pid')]
    if ks:
        return f"other {ks}: {json.dumps(d, default=str)[:220]}"
    if d.get('event') == 'teleop_closed':
        return f"teleop_closed {json.dumps(d)[:200]}"
    return None
while True:
    for p in (DJ, ST):
        if not os.path.exists(p):
            continue
        with open(p) as f:
            f.seek(pos[p]); data = f.read(); 
        if not data.endswith('\n'):
            data = data[:data.rfind('\n') + 1]
        pos[p] += len(data.encode())
        for ln in data.splitlines():
            if p == ST:
                emit('STEP ' + ln)
            else:
                try:
                    d = json.loads(ln)
                except Exception:
                    continue
                b = brief(d)
                if b:
                    emit(b)
    if os.path.exists(WR) and 'guarded run exit' in open(WR).read():
        emit('WRAPPER: ' + open(WR).read().strip().splitlines()[-1]); break
    time.sleep(2)
