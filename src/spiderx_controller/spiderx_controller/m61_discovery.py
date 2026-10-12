"""M6.1 bounded action-server discovery, run BEFORE the first readiness observation.

Why. The one authorized Cloud attempt (2026-10-12, docs/M61A_CLOUD_VERIFICATION.md section 11)
was refused at readiness #1 with `action_server_missing` while the server was running. The
M6.0-B probe (m6_live_preflight.GraphProbe) asks for the action servers ONCE, node by node,
after a fixed 2.0 s spin of a node created moments earlier; it never waits for discovery.
Discovery latency is the leading hypothesis for that refusal; it is not a proven cause.

What. wait_for_action_server() repeats the SAME per-node query every POLL_S until it sees the
approved action server, bounded by the existing server wait (m6_live_contract.SERVER_WAIT_S,
10 s). It permits nothing on its own: the readiness observation that follows is complete and
fresh (it starts after the wait, so the 10 s readiness expiry is unchanged) and only that
observation can permit dispatch. A server that is not seen within the bound is still refused:
the readiness that follows fails `action_server_missing`, and m6_gait_replay.with_action_discovery
adds DISCOVERY_TIMEOUT.

Telling "absent" from "not discovered yet", where the graph allows it. At a timeout the last
snapshot is classified:
  SERVER_ABSENT   the expected host node (the action name's namespace, e.g.
                  /leg_trajectory_controller) IS discovered, yet no node offers the action, no
                  publisher of the action's status topic is visible and this process's client is
                  not matched to a server: the strongest graph evidence of absence available.
                  It cannot exclude endpoint discovery lagging node discovery by more than the
                  bound.
  UNATTRIBUTED    the server's endpoints ARE visible (a status publisher, or the client is
                  matched) but no discovered node owns the action: present, node attribution
                  incomplete.
  NOT_DISCOVERED  neither the host node nor any server endpoint is visible: absence and late
                  discovery cannot be told apart.
  QUERY_FAILED    the last query raised (recorded with its message).
An operator interrupt (interrupted() true) ends the wait early with outcome INTERRUPTED; the
readiness that follows is then not READY either, and nothing can be sent meanwhile.
Pure Python: the graph query, the spin and the clock are injected (the rclpy transport in
m61a_live, or the in-memory fake in m61_mock). Read-only: it sends, cancels and publishes
nothing.
"""

import math

from spiderx_controller import m6_live_contract as m6d

SCHEMA = 'spiderx.m61.action_discovery/1'
POLL_S = 0.25
TIMEOUT_S = m6d.SERVER_WAIT_S                 # the existing 10 s server-wait limit
FOUND, TIMEOUT, INTERRUPTED = 'found', 'timeout', 'interrupted'
SERVER_ABSENT = 'server_absent_on_discovered_host'
UNATTRIBUTED = 'server_endpoints_without_node'
NOT_DISCOVERED = 'not_discovered_before_timeout'
QUERY_FAILED = 'query_failed'
DISCOVERY_TIMEOUT = 'action_discovery_timeout'           # readiness failure codes
DISCOVERY_INTERRUPTED = 'action_discovery_interrupted'
CLASSIFICATION_TEXT = {
    SERVER_ABSENT: 'the expected host node is discovered, but no node offers the action, no '
                   'status publisher is visible and the client is not matched: consistent with '
                   'an absent server (endpoint discovery lagging by more than the bound is not '
                   'excluded)',
    UNATTRIBUTED: 'server endpoints are visible (status publisher or matched client) but no '
                  'discovered node owns the action: present, node attribution incomplete',
    NOT_DISCOVERED: 'neither the host node nor any server endpoint is visible: absence and late '
                    'discovery cannot be told apart',
    QUERY_FAILED: 'the graph query raised; see the error',
}


def expected_host_node(action_name):
    """'/leg_trajectory_controller/follow_joint_trajectory' -> '/leg_trajectory_controller'
    (ros2_controllers serve the action on the controller's own node)."""
    parts = [p for p in str(action_name).split('/') if p]
    return '/' + '/'.join(parts[:-1]) if len(parts) > 1 else None


def classify_timeout(snapshot, host_node, error=None):
    """The classification of a snapshot that did not show the server (see the module doc)."""
    if error is not None:
        return QUERY_FAILED
    snap = snapshot or {}
    endpoints = bool(snap.get('status_publishers')) or bool(snap.get('client_matched'))
    if endpoints:
        return UNATTRIBUTED
    if host_node is not None and host_node in (snap.get('nodes') or ()):
        return SERVER_ABSENT
    return NOT_DISCOVERED


def _finite(v):
    return v if isinstance(v, (int, float)) and math.isfinite(v) else None


def wait_for_action_server(snapshot, spin, now, action_name, domain_id=None,
                           timeout_s=TIMEOUT_S, poll_s=POLL_S, interrupted=None):
    """Poll snapshot() until it shows the action server, for at most timeout_s. Never raises.

    snapshot() -> {'servers': [(node, [types])] from the readiness probe's own per-node query,
                   'nodes': [fully qualified names], 'status_publishers': int,
                   'client_matched': bool}; spin(seconds) processes discovery for that long;
    now() is the monotonic clock. Polls happen at 0, poll_s, 2 poll_s, ... and one last poll at
    the bound; a server counts only if a poll at or before the bound shows it. The total wait is
    at most timeout_s plus one spin's overrun. interrupted() (optional) is checked at every poll.
    """
    host = expected_host_node(action_name)
    start = now()
    polls = []
    snap, error, outcome = {}, None, None
    while True:
        t = now()
        elapsed = t - start
        try:
            snap = dict(snapshot() or {})
            error = None
        except Exception as e:  # noqa: BLE001 - recorded; the next poll retries
            snap, error = {}, f'{type(e).__name__}: {e}'
        servers = list(snap.get('servers') or [])
        nodes = list(snap.get('nodes') or [])
        polls.append({'t_s': round(elapsed, 6), 'monotonic': round(t, 6),
                      'servers': len(servers), 'nodes': len(nodes),
                      'host_node_visible': host in nodes,
                      'status_publishers': snap.get('status_publishers'),
                      'client_matched': snap.get('client_matched'), 'error': error})
        if servers and elapsed <= timeout_s:
            outcome = FOUND
            break
        if interrupted is not None and interrupted():
            outcome = INTERRUPTED
            break
        if elapsed >= timeout_s:
            outcome = TIMEOUT
            break
        spin(min(poll_s, timeout_s - elapsed))
    end = now()
    found = outcome == FOUND
    classification = (None if found or outcome == INTERRUPTED
                      else classify_timeout(snap, host, error))
    return {
        'schema': SCHEMA,
        'action_name': action_name,
        'expected_host_node': host,
        'domain_id': domain_id,
        'timeout_s': timeout_s,
        'poll_s': poll_s,
        'started_monotonic': round(start, 6),
        'ended_monotonic': round(end, 6),
        'duration_s': round(end - start, 6),
        'outcome': outcome,
        'found_after_s': polls[-1]['t_s'] if found else None,
        'servers': [[n, list(ty)] for n, ty in servers] if found else [],
        'classification': classification,
        'classification_text': CLASSIFICATION_TEXT.get(classification),
        'last_snapshot': {'nodes': list(snap.get('nodes') or []),
                          'status_publishers': snap.get('status_publishers'),
                          'client_matched': snap.get('client_matched'), 'error': error},
        'polls': polls,
        'note': 'bounded wait before the readiness observation; it permits nothing by itself',
    }


def summary_line(record):
    """One line for logs and the rehearsal CLI."""
    if not record:
        return 'action discovery: not run'
    if record.get('outcome') == FOUND:
        return (f'action discovery: found {record.get("action_name")} after '
                f'{record.get("found_after_s")} s ({len(record.get("polls") or [])} polls)')
    return (f'action discovery: {str(record.get("outcome")).upper()} after '
            f'{_finite(record.get("duration_s"))} s: {record.get("classification")}')


__all__ = ['wait_for_action_server', 'classify_timeout', 'expected_host_node', 'summary_line',
           'POLL_S', 'TIMEOUT_S', 'FOUND', 'TIMEOUT', 'INTERRUPTED', 'SERVER_ABSENT',
           'UNATTRIBUTED', 'NOT_DISCOVERED', 'QUERY_FAILED', 'DISCOVERY_TIMEOUT',
           'DISCOVERY_INTERRUPTED', 'SCHEMA']
