"""M6.1 run evidence: log/m61_run/<UTC>/ with the outcome, the goal, CSV streams and gate traces.

Layout of one run directory (design section 5):
  live_outcome.json            live runs: the extended outcome (written through
                               m6_live_playback.EvidenceFile: reserved, pre_send, final)
  outcome.json                 mock runs: the same outcome document
  trajectory.json              the exact goal content (points, times, velocities, trajectory_id)
  goal_fingerprint.txt         the approved goal fingerprint, trajectory_id and content hash
  readiness_before.json        readiness observed before the confirmation
  readiness_after.json         readiness re-observed after the confirmation
  joint_states.csv             time_s, then the 12 joint positions (canonical order)
  body_pose.csv                time_s, x, y, z, roll, pitch, yaw (header only if no pose arrived)
  commanded_vs_observed.csv    time_s, ref_time_s, then <joint>_cmd/_obs/_err, max_abs_err_rad
  gates.json                   per-gate thresholds, worst values, margins and trips in order
  git_state.txt                commit, branch, porcelain status, both live gates
  environment.txt              ROS distro, Python, platform, ROS_DOMAIN_ID as found

The directory is created exclusively (an existing run is never touched) and every file is
created with O_EXCL, so no evidence is ever overwritten. A failed supporting file is recorded in
the outcome's `evidence_files` block; it never hides the outcome. time_s is the sim-time stamp
(/joint_states header, or /clock for the body pose); empty when unknown.
"""

import datetime
import json
import os
import platform
import subprocess
import sys

DEFAULT_ROOT = 'log/m61_run'
LIVE_OUTCOME = 'live_outcome.json'
MOCK_OUTCOME = 'outcome.json'
JOINT_STATES_CSV = 'joint_states.csv'
BODY_POSE_CSV = 'body_pose.csv'
COMMANDED_CSV = 'commanded_vs_observed.csv'
BODY_POSE_COLUMNS = ('time_s', 'x', 'y', 'z', 'roll', 'pitch', 'yaw')
SUPPORT_FILES = ('trajectory.json', 'goal_fingerprint.txt', 'readiness_before.json',
                 'readiness_after.json', JOINT_STATES_CSV, BODY_POSE_CSV, COMMANDED_CSV,
                 'gates.json', 'git_state.txt', 'environment.txt')


def utc_stamp(now=None):
    now = now or datetime.datetime.now(datetime.timezone.utc)
    return now.strftime('%Y%m%dT%H%M%SZ')


def reserve_run_dir(root, name):
    """Create <root>/<name> exclusively. Raises FileExistsError if it exists (never reused)."""
    os.makedirs(root, exist_ok=True)
    path = os.path.join(root, name)
    os.mkdir(path)
    return path


def write_exclusive(path, text):
    """Create `path` (O_EXCL), write all of `text`, fsync. Raises FileExistsError if present."""
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    try:
        view = memoryview(text.encode())
        while view:
            n = os.write(fd, view)
            if n <= 0:
                raise OSError(f'short write to {path}')
            view = view[n:]
        os.fsync(fd)
    finally:
        os.close(fd)
    return path


def dumps(data):
    return json.dumps(data, indent=1, sort_keys=True, allow_nan=False, default=str) + '\n'


def _num(v):
    if v is None:
        return ''
    return repr(float(v))


def joint_states_csv(names, rows):
    """rows: [(stamp_s, {joint: position})]. Missing joints are written as empty cells."""
    lines = [','.join(('time_s',) + tuple(names))]
    for stamp, pos in rows:
        lines.append(','.join([_num(stamp)] + [_num(pos.get(n)) for n in names]))
    return '\n'.join(lines) + '\n'


def body_pose_csv(rows):
    """rows: [(stamp_s, (x, y, z, roll, pitch, yaw))]."""
    lines = [','.join(BODY_POSE_COLUMNS)]
    for stamp, pose in rows:
        lines.append(','.join([_num(stamp)] + [_num(v) for v in pose]))
    return '\n'.join(lines) + '\n'


def commanded_csv(names, rows):
    """rows: [(stamp_s, ref_time_s, commanded[12], observed{joint: q})]."""
    head = ['time_s', 'ref_time_s']
    for n in names:
        head += [f'{n}_cmd', f'{n}_obs', f'{n}_err']
    lines = [','.join(head + ['max_abs_err_rad'])]
    for stamp, ref_t, cmd, obs in rows:
        cells, worst = [_num(stamp), _num(ref_t)], None
        for n, c in zip(names, cmd):
            o = obs.get(n)
            e = None if o is None else o - c
            if e is not None:
                worst = abs(e) if worst is None else max(worst, abs(e))
            cells += [_num(c), _num(o), _num(e)]
        lines.append(','.join(cells + [_num(worst)]))
    return '\n'.join(lines) + '\n'


def _run(cmd):
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=5, check=False)
        return out.stdout.strip() if out.returncode == 0 else f'unavailable ({out.stderr.strip()})'
    except Exception as e:  # noqa: BLE001 - recorded, never fatal
        return f'unavailable ({type(e).__name__}: {e})'


def git_state_text(gates):
    lines = [f'commit: {_run(["git", "rev-parse", "HEAD"])}',
             f'branch: {_run(["git", "rev-parse", "--abbrev-ref", "HEAD"])}',
             'status --porcelain:', _run(['git', 'status', '--porcelain']) or '(clean)']
    lines += [f'{k}: {v}' for k, v in sorted(gates.items())]
    return '\n'.join(lines) + '\n'


def environment_text():
    keys = ('ROS_DISTRO', 'ROS_VERSION', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION',
            'GZ_VERSION', 'IGN_GAZEBO_RESOURCE_PATH')
    lines = [f'{k}: {os.environ.get(k, "(unset)")}' for k in keys]
    lines += [f'python: {sys.version.split()[0]}', f'platform: {platform.platform()}',
              'note: ROS_DOMAIN_ID is recorded only; the tool uses the explicit --domain-id']
    return '\n'.join(lines) + '\n'


def write_support_files(run_dir, outcome, trajectory, rows, gates_state):
    """Every supporting file of one run. Returns {file: 'written' | 'error: ...'}; never raises."""
    names = list((trajectory or {}).get('joint_names') or [])
    readiness = {r.get('when'): r for r in outcome.get('readiness') or []}
    contents = {
        'trajectory.json': lambda: dumps(trajectory),
        'goal_fingerprint.txt': lambda: (
            f'goal_fingerprint: {outcome.get("goal_fingerprint")}\n'
            f'trajectory_id: {outcome.get("trajectory_id")}\n'
            f'content_sha256: {outcome.get("trajectory_content_sha256")}\n'),
        'readiness_before.json': lambda: dumps(readiness.get('before_confirmation')),
        'readiness_after.json': lambda: dumps(readiness.get('after_confirmation')),
        JOINT_STATES_CSV: lambda: joint_states_csv(names, rows.get('joint_states', [])),
        BODY_POSE_CSV: lambda: body_pose_csv(rows.get('body_pose', [])),
        COMMANDED_CSV: lambda: commanded_csv(names, rows.get('commanded_vs_observed', [])),
        'gates.json': lambda: dumps({'gates': outcome.get('gates'), 'body': outcome.get('body'),
                                     'contact': outcome.get('contact'),
                                     'cancel_reason': outcome.get('cancel_reason')}),
        'git_state.txt': lambda: git_state_text(gates_state),
        'environment.txt': environment_text,
    }
    status = {}
    for name in SUPPORT_FILES:
        try:
            write_exclusive(os.path.join(run_dir, name), contents[name]())
            status[name] = 'written'
        except Exception as e:  # noqa: BLE001 - recorded in the outcome, never fatal
            status[name] = f'error: {type(e).__name__}: {e}'
    return status


__all__ = ['DEFAULT_ROOT', 'LIVE_OUTCOME', 'MOCK_OUTCOME', 'SUPPORT_FILES', 'BODY_POSE_COLUMNS',
           'utc_stamp', 'reserve_run_dir', 'write_exclusive', 'joint_states_csv', 'body_pose_csv',
           'commanded_csv', 'write_support_files']
