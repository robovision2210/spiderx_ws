#!/usr/bin/env python3
"""S2 step 3: exercise the CURRENT driver's /spiderx/locomotion/estop service fallback on the real
graph against the DISABLED (shadow) M5.5 node. Uses s2_driver.Driver itself (status subscriber
and estop service client); no teleop is spawned, so emergency_stop() takes the service path, as
in the lease exercise. Writes <out>/estop_check.json. Usage: estop_check.py <s2_driver.py> <out>"""
import importlib.util
import json
import os
import sys
import time

spec = importlib.util.spec_from_file_location('s2_driver', sys.argv[1])
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
out = sys.argv[2]
os.makedirs(out, exist_ok=True)
d = mod.Driver(out, 0)
t0 = time.monotonic()
while d.st() is None and time.monotonic() - t0 < 30.0:
    time.sleep(0.1)
before = d.st()
time.sleep(1.0)
before = d.st()
d.emergency_stop('S2 step 3: disabled real-graph estop-service check')
time.sleep(3.0)                                     # >= 2 status periods after the request
after = d.st()
calls = []
with open(os.path.join(out, 'driver.jsonl')) as f:
    for line in f:
        e = json.loads(line)
        if e.get('event') in ('estop_service', 'estop_key_failed', 'emergency_stop_failed'):
            calls.append(e)


def pick(st):
    if st is None:
        return None
    return {k: st.get(k) for k in ('state', 'mode', 'dispatch_gate', 'faults', 'goal',
                                    'goals_recorded', 'last_refusal', 'boundary')}


res = {'teleop_alive': getattr(d, 'teleop_alive', False), 'service_calls': calls,
       'status_before': pick(before), 'status_after': pick(after),
       'status_messages': d.status_n}
with open(os.path.join(out, 'estop_check.json'), 'w') as f:
    json.dump(res, f, indent=1, sort_keys=True)
print(json.dumps(res, sort_keys=True), flush=True)
try:
    d.close()
except Exception as e:  # noqa: BLE001
    print(f'close: {e!r}', flush=True)
os._exit(0)
