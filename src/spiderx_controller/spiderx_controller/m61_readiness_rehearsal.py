"""M6.1 READ-ONLY readiness rehearsal: the production readiness path, with no way to dispatch.

    ros2 run spiderx_controller m61_readiness_rehearsal --domain-id 0 [--out DIR]

Against the running simulation it runs what --live runs before the confirmation and before the
send, and nothing else:
  1. it opens the production M6.1-A transport (m61a_live.M61AFixedBaseTransport) as
     RehearsalTransport, whose send_goal and cancel_goal raise and whose single-use guards are
     spent at open, so no goal and no cancel can leave this process;
  2. readiness #1 through m6_gait_replay.live_readiness_provider (the --live composition): the
     bounded action-server discovery (m61_discovery, 10 s), then a complete, fresh observation
     and every M6.1 layer (body pose, fixed base, sim progress, command owner);
  3. readiness #2, re-observed without waiting (as --live does after the confirmation);
  4. the pre-send check of the streams NOW (m6_gait_replay.send_streams_check) and the M6.0-D
     age check of readiness #2 (m6_live_readiness.dispatch_permitted).
Like --live it stops at the first step that does not pass. There is no session, no
confirmation, no goal object and no call to any send: this module only observes. It neither reads
nor changes a dispatch gate, so it runs on the disabled build, and nothing here can enable one.
The complete record (every check, monotonic timestamps, the discovery record, domain and action
name) is written exclusively to <out>/<UTC>/rehearsal.json.

Exit codes: 0 READY (every step passed), 1 NOT READY, 2 refused (arguments or evidence).
"""

import os
import time

from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m6_live_contract as m6d
from spiderx_controller import m6_live_playback as lp
from spiderx_controller import m6_live_readiness as rd
from spiderx_controller import m61_discovery as dsc
from spiderx_controller import m61_evidence as evd
from spiderx_controller import m61_goal as goal61
from spiderx_controller import m61_live_contract as c61
from spiderx_controller import m61_trot_cycle as tc
from spiderx_controller import m61a_live
from spiderx_controller import m6_gait_replay as gr

SCHEMA = 'spiderx.m61.readiness_rehearsal/1'
DEFAULT_OUT = os.path.join('log', 'm61_rehearsal')
NODE_NAME = 'm61_readiness_rehearsal'
NO_DISPATCH = ('the readiness rehearsal cannot dispatch: it has no goal, no cancel and no '
               'session (m61_readiness_rehearsal)')
EXIT_READY, EXIT_NOT_READY, EXIT_REFUSED = 0, 1, 2
NON_CLAIMS = ('A rehearsal observes readiness only. It sends no goal and shows nothing about '
              'tracking, gates in motion, walking, contact or hardware.',)


class RehearsalTransport(m61a_live.M61AFixedBaseTransport):
    """The production M6.1-A transport, made incapable of dispatch: send_goal and cancel_goal
    raise (and are counted), and the inherited single-use guards are spent at open."""

    def __init__(self, expected_fingerprint, domain_id, fixed_base, **kw):
        kw.setdefault('node_name', NODE_NAME)
        super().__init__(expected_fingerprint, domain_id, fixed_base, **kw)
        self.dispatch_calls = 0

    def open(self):
        super().open()
        self._goal_sent = self._cancel_sent = True
        return self

    def send_goal(self, goal, binding):
        self.dispatch_calls += 1
        raise PermissionError(NO_DISPATCH)

    def cancel_goal(self):
        self.dispatch_calls += 1
        raise PermissionError(NO_DISPATCH)


def _default_transport_factory(fp, domain_id, fixed_base):
    return RehearsalTransport(fp, domain_id, fixed_base).open()


def _entry(res, now, t0, label):
    entry = res.to_dict(now) if isinstance(res, rd.ReadinessResult) else None
    permitted, code = rd.dispatch_permitted(res, now)
    if entry is not None:
        entry.update(gr.readiness_detail(res, now, t0))
    return {'when': label, 'result': entry, 'permitted': permitted, 'code': code}


def run_rehearsal(plan, domain_id, transport_factory=None, collect_factory=None):
    """The rehearsal record (dict). Never sends; an exception after opening is recorded."""
    traj = tc.build_trajectory(plan)
    fp = gf.fingerprint(goal61.approved_spec(traj, plan))     # identity only (transport binding)
    cfg = gr.load_fixed_base_config(plan.config_dir)
    rec = {'schema': SCHEMA, 'milestone': 'M6.1', 'mode': 'readiness_rehearsal',
           'domain_id': domain_id, 'action_name': None, 'node_name': None,
           'trajectory_id': traj['trajectory_id'],
           'trajectory_content_sha256': traj['provenance']['content_sha256'],
           'goal_fingerprint': fp, 'gates': gr._gate_state(), 'dispatch_possible': False,
           'goals_sent': 0, 'cancels_sent': 0, 'steps': [], 'streams_at_send': None,
           'age_check_at_send': None, 'verdict': 'NOT READY', 'stopped_at': None,
           'errors': [], 'non_claims': list(NON_CLAIMS)}
    t_req = time.monotonic()
    transport = (transport_factory or _default_transport_factory)(fp, domain_id, cfg)
    rec['transport_opened_monotonic'] = round(time.monotonic(), 6)
    rec['transport_open_s'] = round(time.monotonic() - t_req, 6)
    rec['action_name'] = getattr(transport, 'action_name', None)
    rec['node_name'] = getattr(transport, 'node_name', None)
    t0 = transport.wall_now()
    try:
        provide = gr.live_readiness_provider(transport, plan, cfg,
                                             collect_factory or gr._default_collect_factory)
        for label in ('before_confirmation', 'after_confirmation'):
            res = provide()
            step = _entry(res, transport.wall_now(), t0, label)
            rec['steps'].append(step)
            if not step['permitted']:
                rec['stopped_at'] = label
                break
        else:
            codes, detail = gr.send_streams_check(transport, cfg, plan.sources.joint_names)()
            now = transport.wall_now()
            rec['streams_at_send'] = evd.json_safe(dict(detail, ok=not codes, codes=list(codes)))
            ok, code = rd.dispatch_permitted(res, now)
            rec['age_check_at_send'] = {'permitted': ok, 'code': code,
                                        'age_s': round(res.age_s(now), 6),
                                        'max_age_s': m6d.READINESS_MAX_AGE_S}
            if codes or not ok:
                rec['stopped_at'] = 'at_send'
            else:
                rec['verdict'] = 'READY'
    except Exception as e:  # noqa: BLE001 - recorded; nothing can have been sent
        rec['errors'].append({'type': type(e).__name__, 'message': str(e)})
        rec['verdict'] = 'NOT READY'
    finally:
        rec['dispatch_calls'] = getattr(transport, 'dispatch_calls', 0)
        try:
            transport.close()
            rec['transport_closed'] = True
        except Exception as e:  # noqa: BLE001
            rec['transport_closed'] = False
            rec['errors'].append({'type': type(e).__name__, 'message': f'close: {e}'})
    first = rec['steps'][0]['result'] if rec['steps'] else None
    rec['discovery'] = ((first or {}).get('report') or {}).get('m61_action_discovery')
    return rec


def parse_args(argv):
    import argparse
    p = argparse.ArgumentParser(prog='m61_readiness_rehearsal', description=__doc__.split('\n')[0])
    p.add_argument('--domain-id', type=int, required=True,
                   help='ROS domain id of the running simulation, typed explicitly')
    p.add_argument('--out', default=DEFAULT_OUT, help=f'evidence root (default {DEFAULT_OUT})')
    return p.parse_args(argv)


def main(argv=None, plan=None, transport_factory=None, collect_factory=None, utc=None):
    import sys
    argv = list(argv if argv is not None else sys.argv)
    try:
        args = parse_args(argv[1:])
    except SystemExit:
        return EXIT_REFUSED
    print('SpiderX M6.1 READ-ONLY readiness rehearsal (no goal, no cancel, no session)')
    if not 0 <= args.domain_id <= lp.MAX_DOMAIN_ID:
        print(f'REFUSED: --domain-id must be in 0..{lp.MAX_DOMAIN_ID}')
        return EXIT_REFUSED
    stamp = utc or evd.utc_stamp()
    try:
        run_dir = evd.reserve_run_dir(args.out, stamp)
    except (FileExistsError, OSError) as e:
        print(f'REFUSED: cannot reserve {args.out}/{stamp} ({type(e).__name__}: {e})')
        return EXIT_REFUSED
    rec = run_rehearsal(plan or tc.load_plan(), args.domain_id, transport_factory,
                        collect_factory)
    rec['run_dir'] = run_dir
    path = evd.write_exclusive(os.path.join(run_dir, 'rehearsal.json'),
                               evd.dumps(evd.json_safe(rec)))
    print(dsc.summary_line(rec.get('discovery')))
    for step in rec['steps']:
        res = step['result'] or {}
        print(f'  readiness {step["when"]}: permitted {step["permitted"]} ({step["code"]}); '
              f'failure codes {res.get("failure_codes")}')
    if rec['streams_at_send'] is not None:
        print(f'  streams at send: ok {rec["streams_at_send"]["ok"]} '
              f'{rec["streams_at_send"]["codes"]}; age check {rec["age_check_at_send"]}')
    print(f'{rec["verdict"]}; goals sent 0, cancels sent 0, dispatch calls '
          f'{rec["dispatch_calls"]}; M6.1 gate {c61.M61_LIVE_DISPATCH_ENABLED}')
    print(f'Record: {path}')
    return EXIT_READY if rec['verdict'] == 'READY' else EXIT_NOT_READY


__all__ = ['RehearsalTransport', 'run_rehearsal', 'main', 'SCHEMA', 'NO_DISPATCH']
