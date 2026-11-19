#!/usr/bin/env python3
"""S4e terminal preflight (docs/S4E_CONTRACT.md section 4.1): a standalone keyboard diagnostic.

No ROS import, no Gazebo, no teleop, no commands and no goals. It only reads the characters this
terminal delivers and reports their timing, so that an operator can see, before any launch,
whether their key repeat suits the teleop's hold window (H = 0.8 s, teleop.hold_s).

It shows:
  - every chunk of characters as it arrives, with its time and the gap since the previous chunk;
  - for each run of one repeated character: the first-repeat delay, the repeat interval and
    rate, the largest gap, and when the repeat input stopped;
  - idle periods.

A terminal delivers characters, never key releases. "No more characters arrived" is NOT a key-up
event: the key was released some time after the last character, before the next repeat would
have come. The teleop works the same way: it treats a motion key as held until H after its last
character (m55_teleop.TeleopState).

Human check (the default): refused, before the terminal is touched, unless standard input and
output are an interactive terminal and this process is in that terminal's foreground. Scripted
pseudo-terminal tests must pass --scripted-pty-test: every line and the record are then labelled
"SCRIPTED PTY TEST - NOT HUMAN INPUT" with input_source 'scripted-pty-test'. Such a record can
never satisfy an S4e human-input criterion (contract section 8, H1 and H3).

The terminal is put in cbreak mode (no echo, no line buffering; Ctrl+C still interrupts) and is
restored on every exit: Ctrl+D, the duration cap, end of input, Ctrl+C, SIGTERM, SIGHUP and errors.

Verdict (motion keys w, W, s, S; H = 0.8 s):
  FAIL        a run's first repeat or any gap within it was >= H; or a single character was
              followed within H..5 s by a run of the same key (a first repeat slower than the
              teleop's 2 s repeat window would look like that)
  UNMEASURED  otherwise, if no forward (w/W) run lasted >= the minimum hold (10 s for S4e)
  PASS        otherwise

Usage: s4e_terminal_preflight.py [--out <new file.json>] [--duration 180] [--min-hold-s 10]
                                 [--no-env-query] [--scripted-pty-test]
Exit: 0 finished (Ctrl+D, the duration cap or end of input), 3 error, 4 interrupted, 5 refused."""
import argparse
import json
import os
import select
import shutil
import signal
import statistics
import subprocess
import sys
import termios
import time

SCHEMA = 'spiderx.s4e_preflight/1'
HOLD_S = 0.8                    # = teleop.hold_s (config/m55_locomotion.yaml)
REPEAT_WINDOW_S = 2.0           # = m55_teleop.REPEAT_GAP_WINDOW_S
SLOW_FIRST_REPEAT_MAX_S = 5.0   # one character, then a run of it this soon: a slow first repeat?
MOTION_KEYS = ('w', 'W', 's', 'S')
FORWARD_KEYS = ('w', 'W')
MIN_HOLD_S = 10.0               # contract section 4.1 and check 1.11
IDLE_NOTE_S = 2.0
DURATION_S = 180.0
CTRL_D = 0x04
HUMAN, SCRIPTED = 'human-tty', 'scripted-pty-test'
SCRIPTED_LABEL = 'SCRIPTED PTY TEST - NOT HUMAN INPUT'
HUMAN_LABEL = 'HUMAN CHECK (interactive terminal; the operator attests separately)'
NOT_KEY_UP = ('no more characters arrived: that is NOT a key-up event (a terminal never reports '
              'a key release; the key was released some time after this last character)')
ENV_QUERIES = {
    'xset_q': ['xset', 'q'],
    'gsettings_delay': ['gsettings', 'get', 'org.gnome.desktop.peripherals.keyboard', 'delay'],
    'gsettings_repeat_interval': ['gsettings', 'get', 'org.gnome.desktop.peripherals.keyboard',
                                  'repeat-interval'],
    'gsettings_repeat': ['gsettings', 'get', 'org.gnome.desktop.peripherals.keyboard', 'repeat'],
}


class Interrupted(Exception):
    """SIGTERM or SIGHUP (Ctrl+C arrives as KeyboardInterrupt)."""


# ------------------------------------------------------------------ analysis (pure)
def key_name(b):
    """A readable name for one byte value."""
    if b == 0x20:
        return 'space'
    if b == 0x1b:
        return 'Esc'
    if b < 0x20:
        return 'Ctrl+' + chr(b + 0x40)
    if b == 0x7f:
        return 'Backspace'
    if b < 0x7f:
        return chr(b)
    return f'0x{b:02x}'


def runs(events, window=REPEAT_WINDOW_S, hold_s=HOLD_S):
    """events: [(t, byte value)] in arrival order. A run is consecutive occurrences of one byte,
    no other byte between, each within `window` of the previous one (as the teleop groups a
    repeated key). Returns one summary per run, in order."""
    raw = []
    for t, b in events:
        if raw and raw[-1]['byte'] == b and t - raw[-1]['times'][-1] <= window:
            raw[-1]['times'].append(t)
        else:
            raw.append({'byte': b, 'times': [t]})
    return [summarize(r['byte'], r['times'], hold_s) for r in raw]


def summarize(b, times, hold_s=HOLD_S):
    gaps = [b_ - a for a, b_ in zip(times, times[1:])]
    intervals = gaps[1:]
    med = statistics.median(intervals) if intervals else None
    return {
        'key': key_name(b), 'byte': b, 'n': len(times), 'start_s': times[0], 'end_s': times[-1],
        'duration_s': times[-1] - times[0],
        'first_repeat_delay_s': gaps[0] if gaps else None,
        'median_interval_s': med, 'rate_hz': (1.0 / med) if med else None,
        'min_interval_s': min(intervals) if intervals else None,
        'max_gap_s': max(gaps) if gaps else None,
        'gaps_at_or_above_hold_s': [g for g in gaps if g >= hold_s],
        'motion_key': key_name(b) in MOTION_KEYS,
    }


def slow_first_repeat_suspects(run_list, hold_s=HOLD_S, max_s=SLOW_FIRST_REPEAT_MAX_S):
    """A single motion character followed, with nothing between, by a run of the same key that
    starts hold_s..max_s later: one hold whose first repeat came too late looks exactly so."""
    out = []
    for p, r in zip(run_list, run_list[1:]):
        if p['motion_key'] and p['byte'] == r['byte'] and p['n'] == 1 and r['n'] >= 2 and \
                hold_s <= r['start_s'] - p['end_s'] <= max_s:
            out.append({'key': r['key'], 'single_at_s': p['end_s'], 'run_from_s': r['start_s'],
                        'gap_s': r['start_s'] - p['end_s']})
    return out


def verdict(run_list, min_hold_s=MIN_HOLD_S, hold_s=HOLD_S):
    """{'verdict': PASS|FAIL|UNMEASURED, 'reasons': [...], 'qualifying_hold': run or None}."""
    reasons = []
    for r in run_list:
        if r['motion_key'] and r['n'] >= 2 and r['gaps_at_or_above_hold_s']:
            slow = ', '.join(f'{g * 1000:.0f} ms' for g in r['gaps_at_or_above_hold_s'])
            reasons.append(f"run of {r['key']!r} from {r['start_s']:.3f} s: gap(s) {slow}"
                           f" >= the hold window {hold_s * 1000:.0f} ms: a held key would stop "
                           'between repeats (if that was a release and a new press within '
                           f'{REPEAT_WINDOW_S:g} s, run it again and wait at least 3 s first)')
    for s in slow_first_repeat_suspects(run_list, hold_s):
        reasons.append(f"a single {s['key']!r} at {s['single_at_s']:.3f} s, then a run of it from "
                       f"{s['run_from_s']:.3f} s ({s['gap_s'] * 1000:.0f} ms later): if that was "
                       'one hold, its first repeat came after the hold window')
    if reasons:
        return {'verdict': 'FAIL', 'reasons': reasons, 'qualifying_hold': None}
    long = [r for r in run_list if r['key'] in FORWARD_KEYS and r['duration_s'] >= min_hold_s]
    if not long:
        return {'verdict': 'UNMEASURED', 'qualifying_hold': None,
                'reasons': [f'no forward (w) hold of at least {min_hold_s:g} s was recorded']}
    best = max(long, key=lambda r: r['duration_s'])
    return {'verdict': 'PASS', 'qualifying_hold': best,
            'reasons': [f"a {best['duration_s']:.1f} s hold of {best['key']!r}: first repeat "
                        f"{best['first_repeat_delay_s'] * 1000:.0f} ms, largest gap "
                        f"{best['max_gap_s'] * 1000:.0f} ms, both below the hold window "
                        f'{hold_s * 1000:.0f} ms']}


def parse_xset(text):
    """(delay ms, rate per s, auto repeat on) from `xset q`, None where absent."""
    delay = rate = on = None
    for ln in text.splitlines():
        s = ln.split()
        if 'delay:' in s and 'rate:' in s:
            try:
                delay = int(s[s.index('delay:') + 1])
                rate = int(s[s.index('rate:') + 1])
            except (IndexError, ValueError):
                pass
        if s[:2] == ['auto', 'repeat:'] and len(s) > 2:
            on = s[2] == 'on'
    return {'auto_repeat_delay_ms': delay, 'repeat_rate_per_s': rate, 'auto_repeat_on': on}


def environment(query=True):
    """Read-only facts about this terminal and the desktop's key-repeat settings."""
    env = {'TERM': os.environ.get('TERM'), 'DISPLAY_set': bool(os.environ.get('DISPLAY')),
           'over_ssh': bool(os.environ.get('SSH_CONNECTION') or os.environ.get('SSH_TTY')),
           'queries': {}}
    if env['over_ssh']:
        env['ssh_note'] = ('over SSH the client machine generates the key repeats, and network '
                           'delay adds jitter (M5.5 section 16.2)')
    for name, cmd in ENV_QUERIES.items():
        if not query:
            env['queries'][name] = 'not queried (--no-env-query)'
        elif name == 'xset_q' and not env['DISPLAY_set']:
            env['queries'][name] = 'not available (no DISPLAY)'
        elif shutil.which(cmd[0]) is None:
            env['queries'][name] = f'not available ({cmd[0]} not found)'
        else:
            try:
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0,
                                   stdin=subprocess.DEVNULL)
                env['queries'][name] = {'rc': r.returncode, 'out': r.stdout.strip()[-2000:],
                                        'err': r.stderr.strip()[-300:]}
            except (OSError, subprocess.SubprocessError) as e:
                env['queries'][name] = f'not available ({e!r})'
    xq = env['queries'].get('xset_q')
    env['xset'] = parse_xset(xq['out']) if isinstance(xq, dict) and xq['rc'] == 0 else None
    return env


# ------------------------------------------------------------------ terminal
def tty_checks(fd_in=0, fd_out=1):
    c = {'stdin_tty': os.isatty(fd_in), 'stdout_tty': os.isatty(fd_out), 'foreground': False,
         'tty_name': None}
    if c['stdin_tty']:
        try:
            c['tty_name'] = os.ttyname(fd_in)
            c['foreground'] = os.tcgetpgrp(fd_in) == os.getpgrp()
        except OSError:
            pass
    return c


def refusal(checks):
    if not checks['stdin_tty']:
        return ('the human check needs an interactive terminal, and standard input is not one: '
                'run it in the terminal window you will use for the teleop')
    if not checks['stdout_tty']:
        return 'standard output is not a terminal: do not redirect or pipe the output'
    if not checks['foreground']:
        return 'this process is not in the terminal\'s foreground: do not run it in the background'
    return None


def cbreak(fd):
    """Echo and line buffering off, signals (Ctrl+C) on; one character at a time."""
    mode = termios.tcgetattr(fd)
    mode[3] &= ~(termios.ECHO | termios.ICANON)
    mode[6][termios.VMIN], mode[6][termios.VTIME] = 1, 0
    termios.tcsetattr(fd, termios.TCSAFLUSH, mode)


class Preflight:
    def __init__(self, label, hold_s=HOLD_S, min_hold_s=MIN_HOLD_S, out=None):
        self.label, self.hold_s, self.min_hold_s = label, hold_s, min_hold_s
        self.out = out or (lambda s: (sys.stdout.write(s + '\n'), sys.stdout.flush()))
        self.t0 = None
        self.events, self.chunks, self.idle = [], [], []
        self.last_t = None
        self.reported_end = True
        self.idle_noted = False

    def say(self, text):
        self.out(f'[{self.label}] {text}' if self.label == SCRIPTED_LABEL else text)

    def chunk(self, data, now):
        """One chunk read from the terminal at monotonic time `now`."""
        t = now - self.t0
        gap = None if self.last_t is None else t - self.last_t
        names = [key_name(b) for b in data]
        if self.events and not self.reported_end and data[0] != self.events[-1][1]:
            self.report_run_end()
        self.chunks.append({'t_s': t, 'mono': now, 'hex': data.hex(), 'keys': names})
        for b in data:
            self.events.append((t, b))
        self.last_t, self.reported_end, self.idle_noted = t, False, False
        self.say(f'[{t:8.3f} s] {" ".join(repr(n) if len(n) == 1 else n for n in names):<14} '
                 f'gap {"   -  " if gap is None else f"{gap * 1000:6.0f} ms"}')

    def report_run_end(self):
        r = runs(self.events, hold_s=self.hold_s)[-1]
        self.reported_end = True
        if r['n'] == 1:
            self.say(f"  single {r['key']!r} at {r['end_s']:.3f} s (a tap: no repeat). "
                     f'{NOT_KEY_UP}')
            return
        slow_ms = ', '.join(f'{g * 1000:.0f}' for g in r['gaps_at_or_above_hold_s'])
        slow = f'; GAP(S) >= {self.hold_s * 1000:.0f} ms: {slow_ms} ms' if slow_ms else ''
        rate = f" ({r['rate_hz']:.1f} Hz)" if r['rate_hz'] else ''
        median_ms = (r['median_interval_s'] or 0.0) * 1000
        self.say(f"  repeat input of {r['key']!r} stopped: last character at {r['end_s']:.3f} s; "
                 f"{r['n']} characters over {r['duration_s']:.2f} s; first repeat "
                 f"{r['first_repeat_delay_s'] * 1000:.0f} ms; median interval {median_ms:.1f} ms"
                 f"{rate}; largest gap {r['max_gap_s'] * 1000:.0f} ms{slow}.")
        self.say(f'  {NOT_KEY_UP}')

    def tick(self, now):
        """Idle bookkeeping at monotonic time `now` (no character arrived)."""
        if self.last_t is None:
            return
        t = now - self.t0
        if t - self.last_t >= IDLE_NOTE_S and not self.idle_noted:
            if not self.reported_end:
                self.report_run_end()
            self.idle_noted = True
            self.idle.append({'from_s': self.last_t, 'noted_at_s': t})
            self.say(f'  idle: nothing for {IDLE_NOTE_S:g} s since {self.last_t:.3f} s (idle is '
                     'not a key-up either)')

    def result(self):
        rl = runs(self.events, hold_s=self.hold_s)
        return rl, verdict(rl, self.min_hold_s, self.hold_s)


def loop(p, fd, duration_s, monotonic=time.monotonic):
    """Read until Ctrl+D, end of input or the duration cap. Returns the reason."""
    end = p.t0 + duration_s
    while True:
        now = monotonic()
        if now >= end:
            return 'duration cap'
        r, _, _ = select.select([fd], [], [], min(0.1, max(0.0, end - now)))
        now = monotonic()
        if not r:
            p.tick(now)
            continue
        data = os.read(fd, 1024)
        if not data:
            return 'end of input'
        if CTRL_D in data:
            data = data[:data.index(CTRL_D)]
            if data:
                p.chunk(data, now)
            return 'Ctrl+D'
        p.chunk(data, now)
        test_raise = os.environ.get('S4E_PREFLIGHT_TEST_RAISE_AFTER_CHUNKS')
        if p.label == SCRIPTED_LABEL and test_raise and len(p.chunks) >= int(test_raise):
            raise RuntimeError('injected test error (scripted pty test only)')


def main(argv=None):
    a = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    a.add_argument('--out', help='write the record to this new JSON file (never overwritten)')
    a.add_argument('--duration', type=float, default=DURATION_S, help='seconds before it stops')
    a.add_argument('--min-hold-s', type=float, default=MIN_HOLD_S,
                   help='the forward hold the verdict needs (S4e: 10; other values are recorded '
                        'and do not satisfy the contract)')
    a.add_argument('--no-env-query', action='store_true',
                   help='do not query xset or gsettings for the repeat settings')
    a.add_argument('--scripted-pty-test', action='store_true',
                   help='TESTS ONLY: label everything SCRIPTED PTY TEST - NOT HUMAN INPUT')
    o = a.parse_args(argv)
    scripted = o.scripted_pty_test
    label = SCRIPTED_LABEL if scripted else HUMAN_LABEL
    if o.out and os.path.exists(o.out):
        print(f'REFUSED: {o.out} exists (a record is never overwritten)', file=sys.stderr)
        return 5
    checks = tty_checks()
    why = refusal(checks)
    if why:
        print(f'REFUSED: {why}', file=sys.stderr)
        return 5
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    record = {'schema': SCHEMA, 'input_source': SCRIPTED if scripted else HUMAN, 'label': label,
              'human_check': not scripted, 'tty': checks, 'hold_s': HOLD_S,
              'repeat_window_s': REPEAT_WINDOW_S, 'min_hold_s': o.min_hold_s,
              'contract_min_hold_s': MIN_HOLD_S, 'not_key_up_note': NOT_KEY_UP,
              'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}
    p = Preflight(label, HOLD_S, o.min_hold_s)

    def interrupted(signum, frame):
        raise Interrupted(signal.Signals(signum).name)
    for s in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, interrupted)
    rc, reason = 0, None
    try:
        record['environment'] = environment(not o.no_env_query)
        p.say(f'S4e terminal preflight - {label}')
        if scripted:
            p.say('This run is a scripted pseudo-terminal test. It is NOT human input and can '
                  'never satisfy an S4e human-input criterion.')
        xs = record['environment'].get('xset')
        p.say('desktop repeat settings: ' + (
            f"xset: delay {xs['auto_repeat_delay_ms']} ms, rate {xs['repeat_rate_per_s']}/s, "
            f"auto repeat {'on' if xs['auto_repeat_on'] else 'off'}" if xs else
            'xset not available (recorded; the measured timing below is what counts)'))
        p.say(f'Hold w for {o.min_hold_s:g}-20 s by a clock and release; wait at least 3 s; tap '
              f'w once; then press Ctrl+D. Hold window H = {HOLD_S * 1000:.0f} ms.')
        cbreak(fd)
        p.t0 = time.monotonic()
        reason = loop(p, fd, o.duration)
    except KeyboardInterrupt:
        rc, reason = 4, 'Ctrl+C (SIGINT)'
    except Interrupted as e:
        rc, reason = 4, f'{e} received'
    except Exception as e:  # noqa: BLE001 - always restore the terminal and keep the record
        rc, reason = 3, f'error: {e!r}'
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
            record['terminal_restored'] = termios.tcgetattr(fd) == saved
        except termios.error as e:
            record['terminal_restored'] = False
            record['terminal_restore_error'] = repr(e)
    if p.t0 is not None and p.events and not p.reported_end:
        p.report_run_end()
    rl, v = p.result()
    record.update(ended_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                  chunks=p.chunks, runs=rl, idle=p.idle, verdict=v,
                  exit={'code': rc, 'reason': reason})
    p.say(f'end: {reason}; terminal restored: {record["terminal_restored"]}')
    p.say(f"VERDICT ({label}): {v['verdict']}: {'; '.join(v['reasons'])}")
    if o.min_hold_s != MIN_HOLD_S:
        p.say(f'note: --min-hold-s {o.min_hold_s:g} is not the contract value {MIN_HOLD_S:g}; '
              'this record does not satisfy the S4e preflight')
    if o.out:
        with open(o.out, 'x') as f:
            json.dump(record, f, indent=1, sort_keys=True)
        p.say(f'record written: {o.out}')
    return rc


if __name__ == '__main__':
    sys.exit(main())
