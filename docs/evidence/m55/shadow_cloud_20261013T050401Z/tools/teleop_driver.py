#!/usr/bin/env python3
"""M5.5 phase-1 shadow session, Cloud: drive m55_teleop_keyboard through a pseudo-terminal.
Keys go only to the teleop (which publishes /cmd_vel to the locomotion node and calls its
services). Key repeat is emulated as 0.5 s delay + 30 Hz (the tested continuous case).
Every key write is logged with time.monotonic(), the node's event clock. Usage: <out.jsonl>"""
import json, os, pty, select, signal, sys, time

out = open(sys.argv[1], 'w')
log = lambda **k: (out.write(json.dumps(dict(k, mono=round(time.monotonic(), 4))) + '\n'), out.flush())


def spawn():
    pid, fd = pty.fork()
    if pid == 0:
        os.execvp('ros2', ['ros2', 'run', 'spiderx_controller', 'm55_teleop_keyboard'])
    return pid, fd


buf = {'text': ''}


def pump(fd, dur):
    end = time.monotonic() + dur
    while time.monotonic() < end:
        r, _, _ = select.select([fd], [], [], 0.02)
        if r:
            try:
                data = os.read(fd, 4096).decode(errors='replace')
            except OSError:
                return
            buf['text'] += data
            log(teleop_out=data)


def key(fd, k, step=None):
    os.write(fd, k.encode()); log(key=repr(k), step=step)


def hold(fd, k, seconds, step):
    key(fd, k, step)
    pump(fd, 0.5)
    t_end = time.monotonic() + max(0.0, seconds - 0.5)
    while time.monotonic() < t_end:
        os.write(fd, k.encode())
        pump(fd, 1 / 30)
    log(release=repr(k), step=step)


pid, fd = spawn(); log(event='teleop_started', pid=pid)
pump(fd, 6)
key(fd, 'a', '1.4 arm'); pump(fd, 4)
hold(fd, 'w', 30.0, '1.5 hold w 30 s'); pump(fd, 4)              # release -> 1.13a timing
key(fd, 'j', '1.6'); pump(fd, 1); key(fd, 'l', '1.6'); pump(fd, 3)
# 1.7: hold w, space while still holding, keep holding, release, then press-hold again
key(fd, 'w', '1.7 hold'); pump(fd, 0.5)
t1 = time.monotonic() + 4.5
while time.monotonic() < t1:
    os.write(fd, b'w'); pump(fd, 1 / 30)
key(fd, ' ', '1.7 space while holding w')
t2 = time.monotonic() + 6.0
while time.monotonic() < t2:
    os.write(fd, b'w'); pump(fd, 1 / 30)
log(release="'w'", step='1.7 release after space'); pump(fd, 3)
hold(fd, 'w', 4.0, '1.7 press again after release'); pump(fd, 6)
key(fd, 'x', '1.8 estop'); pump(fd, 2)
key(fd, 'a', '1.8 arm while faulted (refused)'); pump(fd, 2)
key(fd, 'r', '1.8 reset'); pump(fd, 2)
key(fd, 'a', '1.8 arm after reset'); pump(fd, 3)
# 1.9 / 1.13b: hold w, then kill the teleop (crash path: lease)
key(fd, 'w', '1.9 hold before kill'); pump(fd, 0.5)
t3 = time.monotonic() + 3.5
while time.monotonic() < t3:
    os.write(fd, b'w'); pump(fd, 1 / 30)
os.kill(pid, signal.SIGKILL); log(event='teleop_killed', step='1.9 teleop crash while walking')
try:
    os.waitpid(pid, 0)
except ChildProcessError:
    pass
time.sleep(8)
pid2, fd2 = spawn(); log(event='teleop2_started', pid=pid2)
pump(fd2, 6)
hold(fd2, 'w', 3.0, '1.9 new teleop, w without arm'); pump(fd2, 4)
key(fd2, 'q', 'quit'); pump(fd2, 3)
try:
    os.kill(pid2, signal.SIGTERM)
except ProcessLookupError:
    pass
log(event='done')
