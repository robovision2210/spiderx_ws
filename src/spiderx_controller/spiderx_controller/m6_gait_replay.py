"""M6.1 protected trot gait replay: ONE trot-cycle goal through the M6.0-D safety machinery.

Design: docs/M61_TROT_REPLAY_DESIGN.md (Option B). Implementation notes:
docs/M61_IMPLEMENTATION_NOTES.md.

M61Session subclasses the UNCHANGED M6.0-D m6_live_playback.LiveSession and re-uses, by import:
the one-shot dispatch latch, readiness before and after the typed confirmation, freshness at the
send, the fingerprint-verifying transport, the poll-based supervision loop (stream monitor, sim
stall, sample gap, controller presence, result watchdog), the single guarded cancel, interrupt
handling and EvidenceFile persistence. M6.1 replaces only what is gait specific:
  - the trajectory: config/m61_trot_cycle.yaml, preflighted by m61_trot_cycle against
    config/m61_limits.yaml (never the M6.0-D crouch trajectory or envelope);
  - the goal and its fingerprint (m61_goal, built on m6_goal_fingerprint);
  - the confirmation word SEND-ONE-TROT-CYCLE and the separate gate M61_LIVE_DISPATCH_ENABLED;
  - the tracking reference (cubic Hermite with the waypoint velocities);
  - the gait gates G1-G7 and body-pose freshness (m61_gates), which end the run in GATE_TRIPPED
    after the one cancel - never a return goal;
  - the evidence directory log/m61_run/<UTC>/ with CSV streams (m61_evidence);
  - M6.1-A: the FIXED BASE the replay requires (m61a_fixed_base): readiness checks the welded
    plant (fixed-base /robot_description with the approved mount, Gazebo body-link entry matching
    it, identity spawn, body at the weld pose), the body pose is COMPOSED
    (T_world_model * T_model_dummy * T_dummy_base, never the bare model root), and G8 cancels if
    the attachment is lost in flight. The plant evidence is recorded separately from the
    trajectory provenance, which is unchanged.

LIVE M6.1 DISPATCH IS HARD-DISABLED (m61_live_contract.M61_LIVE_DISPATCH_ENABLED = False).

    ros2 run spiderx_controller m6_gait_replay.py --dry-run
    echo SEND-ONE-TROT-CYCLE | ros2 run spiderx_controller m6_gait_replay.py --mock --scenario success
    ros2 run spiderx_controller m6_gait_replay.py --live --domain-id N     # refused: exit 3
"""

import dataclasses
import math
import os

from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m6_live_contract as m6d
from spiderx_controller import m6_live_playback as lp
from spiderx_controller import m6_live_readiness as rd
from spiderx_controller import m61_evidence as evd
from spiderx_controller import m61_gates as gates
from spiderx_controller import m61_goal as goal61
from spiderx_controller import m61_live_contract as c61
from spiderx_controller import m61_trot_cycle as tc
from spiderx_controller import m61a_fixed_base as fb

OUTCOME_SCHEMA = 'spiderx.m61.live_outcome/1'
GATE_TRIPPED = 'GATE_TRIPPED'
M61_TERMINAL = frozenset(lp.TERMINAL | {GATE_TRIPPED})
TICK = 'm61_tick'


class _TickTransport:
    """Delegates everything to the real transport and appends one tick event to every poll, so
    the re-used supervision loop hands the session a chance to check body-pose freshness even
    when no message arrives. It sends, cancels and publishes nothing of its own."""

    def __init__(self, inner):
        self.inner = inner

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def poll(self, timeout_s):
        events = list(self.inner.poll(timeout_s))
        events.append((TICK, self.inner.wall_now()))
        return events


class M61Session(lp.LiveSession):
    """One approved trot-cycle goal, at most once. Use run(); it returns a structured outcome."""

    def __init__(self, transport, plan, readiness_provider, confirm_reader, latch=None,
                 checkpoint=None, fixed_base=None):
        super().__init__(_TickTransport(transport), plan.sources, readiness_provider,
                         confirm_reader, latch=latch, checkpoint=checkpoint)
        self.plan = plan
        self.limits = plan.limits
        self.fixed_base = fixed_base             # m61a FixedBaseConfig (G8 active) or None
        self.gates = None
        self._preflight = None
        self._tracking_verdict = None
        self._js_rows = []
        self._pose_rows = []

    # ------------------------------------------------------------ pre-dispatch (M6.1 content)
    def _run(self):
        self._t0 = self._now()
        self._event('session start (M6.1 single trot cycle)')
        # 1. offline preflight + approved spec (no goal object yet)
        try:
            self._trajectory = tc.build_trajectory(self.plan)
            self._preflight = tc.preflight(self._trajectory, self.plan)
            self._spec = goal61.approved_spec(self._trajectory, self.plan)
            self._fingerprint = gf.fingerprint(self._spec)
        except (tc.TrajectoryBuildError, gf.FingerprintError) as e:
            return self._refuse('preflight_refused', str(e))
        self.gates = gates.GateMonitor(self.limits, self.sources.limits,
                                       fixed_base=self.fixed_base)
        self._set(lp.PREFLIGHTED, f'preflighted M6.1 trajectory '
                                  f'{self._trajectory["trajectory_id"]}, fingerprint '
                                  f'{self._fingerprint[:16]}')
        # 2. readiness (same process, fresh, not incompatible, fresh body pose)
        ok, code = self._readiness_ok('before_confirmation')
        if not ok:
            return self._refuse(code, 'readiness gate before confirmation')
        if self.latch.count:
            return self._refuse('operator_interrupt', 'interrupted before confirmation')
        # 3. typed confirmation (an interrupt while typing wins over the typed text)
        confirmed = c61.parse_confirmation(self.confirm_reader)
        if self.latch.count:
            return self._refuse('operator_interrupt', 'interrupted at confirmation')
        if not confirmed:
            return self._refuse('confirmation_refused', 'confirmation word not typed exactly')
        self._set(lp.CONFIRMED, 'operator confirmation accepted')
        # 4. readiness re-observed after confirmation
        ok, code = self._readiness_ok('after_confirmation')
        if not ok:
            return self._refuse(code, 'readiness gate after confirmation')
        # 5. goal construction + server + dispatch (the transport verifies the fingerprint)
        try:
            goal, _, _, fp = goal61.build_live_goal(self._trajectory, self.plan)
        except (gf.FingerprintError, ac.PreflightRefused) as e:
            return self._refuse('goal_construction_refused', str(e))
        if fp != self._fingerprint:
            return self._refuse('goal_fingerprint_mismatch', 'goal differs from preflighted spec')
        if not self.transport.server_ready(m6d.SERVER_WAIT_S):
            return self._refuse('action_server_unavailable', 'server not ready within 10 s')
        self._set(lp.DISPATCHING, 'dispatching the one approved trot-cycle goal')
        if self.checkpoint is not None:
            try:
                self.checkpoint(self.outcome())          # pre-send evidence, or no send at all
            except Exception as e:  # noqa: BLE001
                self._record_error('evidence_checkpoint', e)
                return self._refuse('evidence_persistence_failed',
                                    'pre-dispatch evidence could not be saved; nothing sent')
        # 6. last checks, immediately before the send
        if self.latch.count:
            return self._refuse('operator_interrupt', 'interrupted before dispatch')
        ok, code = self._fresh_at_send()
        if not ok:
            return self._refuse(code, 'readiness evidence no longer permitted at the send; '
                                      'no new observation, no retry')
        try:
            self._dispatch(goal)
        except gf.FingerprintError as e:
            self._send = 'refused_by_transport'
            return self._refuse('goal_fingerprint_mismatch', str(e))
        except ac.SecondGoalForbidden:
            raise
        except Exception as e:  # noqa: BLE001 - a failed send is reported, never retried
            self._record_error('send_goal', e)
            self.reason = 'transport_error'
            self._set(lp.HELD_ERROR, f'send failed: {type(e).__name__}: {e}; the goal may have '
                                     'reached the server; acceptance unknown; no retry')
            return self.outcome()
        return self._supervise()

    # ------------------------------------------------------------ gates (M6.1)
    def _gating(self):
        """The goal is running: a gate trip may request the one cancel."""
        return self.state == lp.ACTIVE and self._result is None

    def _observing(self):
        """Gates record trips while the goal runs, after a cancel request and in the settle."""
        return self.state in (lp.ACTIVE, lp.CANCEL_REQUESTED)

    def _on_event(self, ev, monitor):
        kind = ev[0] if ev else None
        if kind == TICK:
            if self.gates is not None and self._gating():
                reason = self.gates.check_pose_stale(ev[1])
                if reason:
                    self._request_cancel(reason)
            return None
        if kind == 'body_pose':
            _, wall, stamp, pose = ev
            self._pose_rows.append((stamp, tuple(pose)))
            if self.gates is not None:
                reason = self.gates.on_body_pose(wall, stamp, pose, gate=self._observing())
                if reason and self._gating():
                    self._request_cancel(reason)
            return None
        if kind == 'fixed_base':
            _, wall, stamp, codes, attachment = ev
            if self.gates is not None:
                reason = self.gates.on_fixed_base(wall, stamp, codes, attachment,
                                                  gate=self._observing())
                if reason and self._gating():
                    self._request_cancel(reason)
            return None
        if kind == 'joint_state' and self.gates is not None:
            _, wall, stamp, names, positions = ev
            self._js_rows.append((stamp, dict(zip(names, positions))))
            reason = self.gates.on_joint_state(wall, stamp, names, positions,
                                               gate=self._observing())
            if reason and self._gating():
                self._request_cancel(reason)            # before the tracking check below
        done = super()._on_event(ev, monitor)
        if kind == 'goal_response' and self.gates is not None and self._acceptance:
            self.gates.arm(self._now())
        return done

    def _inflight_tracking(self, stamp, names, positions):
        if self._t_accept_sim is None:
            return None
        ref = tc.reference_positions(self._trajectory, self._ref_time(stamp))
        if ref is None:
            return None
        pos = dict(zip(names, positions))
        errs = []
        for n, r in zip(self._trajectory['joint_names'], ref):
            v = pos.get(n)
            if v is None or not math.isfinite(v):
                return 'tracking_error'
            errs.append(abs(v - r))
        worst = max(errs)
        self._max_inflight_error = max(worst, self._max_inflight_error or 0.0)
        return 'tracking_error' if worst > self.limits.client_tracking_abort_rad else None

    def _tracking(self):
        if self._t_accept_sim is None:
            return ac.UNAVAILABLE, {'reason': 'goal never accepted'}
        shifted = [(self._ref_time(stamp), obs) for stamp, obs in self._samples]
        verdict, detail = tc.evaluate_tracking(self._trajectory, shifted,
                                               self.limits.path_position_tolerance_rad)
        self._tracking_verdict = verdict
        return verdict, detail

    def _finish_result(self):
        out = super()._finish_result()
        trips = self.gates.trips if self.gates is not None else []
        if trips and self.state == lp.SUCCEEDED:
            self.reason = trips[0]['reason']
            self._set(GATE_TRIPPED, f'gate {trips[0]["gate"]} tripped after the result '
                                    '(no cancel needed: the goal had ended); no new goal')
            return self.outcome()
        return out

    def _finish_cancel(self, confirmed, why):
        if self._cancel_reason in gates.M61_REASONS:
            self._cancel_confirmed = confirmed
            self._set(GATE_TRIPPED, f'gate {gates.REASON_GATE[self._cancel_reason]} tripped '
                                    f'({self._cancel_reason}); cancel '
                                    f'{"confirmed" if confirmed else "unconfirmed"} ({why}); '
                                    'no new goal, no return motion')
            return self.outcome()
        return super()._finish_cancel(confirmed, why)

    def outcome_after_exception(self, exc, where='session'):
        if self.state == GATE_TRIPPED:              # already terminal for M6.1
            self._record_error(where, exc)
            return self.outcome()
        return super().outcome_after_exception(exc, where)

    # ------------------------------------------------------------ report
    def channels(self):
        ch = super().channels()
        if self.state == GATE_TRIPPED and self._tracking_verdict is not None:
            ch['tracking'] = self._tracking_verdict
        return ch

    def evidence_rows(self):
        """Rows for the CSV files: all /joint_states and body poses seen by the session, and the
        commanded spline vs every observation after acceptance (the tracking samples)."""
        cov = []
        if self._trajectory is not None and self._t_accept_sim is not None:
            for stamp, obs in self._samples:
                rt = self._ref_time(stamp)
                ref = tc.reference_positions(self._trajectory, rt)
                if ref is not None:
                    cov.append((stamp, rt, ref, obs))
        return {'joint_states': list(self._js_rows), 'body_pose': list(self._pose_rows),
                'commanded_vs_observed': cov}

    def outcome(self):
        out = super().outcome()
        lim = self.limits
        traj = self._trajectory or {}
        pre = None
        if self._preflight is not None:
            pre = {k: v for k, v in self._preflight.to_dict().items() if k != 'non_claims'}
        out.update({
            'schema': OUTCOME_SCHEMA,
            'milestone': 'M6.1',
            'terminal': self.state in M61_TERMINAL,
            'limits': c61.as_dict(),
            'm61_limits': None if lim is None else lim.as_dict(),
            'envelope': {'source': 'config/m61_limits.yaml', 'm6d_envelope_used': False},
            'non_claims': list(c61.NON_CLAIMS),
            'gait': traj.get('gait'),
            'trajectory_content_sha256': (traj.get('provenance') or {}).get('content_sha256'),
            'preflight': pre,
            'gates': None if self.gates is None else self.gates.summary(self._cancel_reason),
            'body': None if self.gates is None else self.gates.body_summary(),
            'joint_extremes_observed': (None if self.gates is None
                                        else self.gates.joint_extremes()),
            'base_constraint': None if lim is None else lim.base_constraint,
            'contact': {'status': 'not_measured', 'note': gates.CONTACT_NOTE},
            'evidence_rows': {'joint_states': len(self._js_rows),
                              'body_pose': len(self._pose_rows)},
            'm61_live_dispatch_enabled': c61.M61_LIVE_DISPATCH_ENABLED,
            'fixed_base': fixed_base_record(self.fixed_base, self.transport),
        })
        return out


def fixed_base_record(cfg, transport):
    """The INSTANTIATED-PLANT evidence (M6.1-A), kept apart from the trajectory provenance: the
    fixed-base config and its SHA-256, and what the transport saw of /robot_description and the
    Gazebo pose entries. None when the run had no fixed-base configuration."""
    if cfg is None:
        return None
    import hashlib
    snap = getattr(transport, 'fixed_base_snapshot', None)
    snap = snap() if callable(snap) else {}
    desc = snap.get('description')
    rec = {'config': fb.CONFIG_FILE, 'config_sha256': cfg.raw.get('_sha256'),
           'mount_xyz_rpy': list(cfg.mount.xyz_rpy()),
           'min_mount_height_m': cfg.min_mount_height_m,
           'robot_description_sha256': (None if desc is None else
                                        hashlib.sha256(desc.encode()).hexdigest()),
           'pose_messages': snap.get('messages'), 'invalid_pose_samples':
               snap.get('invalid_samples'), 'plant': None}
    ok, codes, checks = fb.assess_plant(desc, snap.get('latest_usable_pose'), cfg)
    rec['plant'] = {'ok': ok, 'codes': codes, 'checks': checks}
    return rec


def with_fixed_base(base_provider, snapshot, cfg):
    """Wrap a readiness provider: also NOT READY unless the welded plant is verified (M6.1-A):
    a fixed-base /robot_description with the approved mount above the clearance minimum, Gazebo's
    body-link entry consistent with it, the model root at the identity spawn and the composed body
    at the weld pose (m61a_fixed_base.assess_plant). snapshot() -> tracker snapshot dict."""
    def provide():
        res = base_provider()
        if not isinstance(res, rd.ReadinessResult):
            return res
        snap = snapshot()
        ok, codes, checks = fb.assess_plant(snap.get('description'),
                                            snap.get('latest_usable_pose'), cfg)
        report = dict(res.report or {}, m61a_fixed_base={'ok': ok, 'codes': codes,
                                                         'checks': checks})
        if ok:
            return dataclasses.replace(res, report=report)
        return dataclasses.replace(
            res, ready=False, failure_codes=tuple(res.failure_codes) + tuple(codes),
            reasons=tuple(res.reasons) + tuple(f'fixed base: {c}' for c in codes),
            report=report)
    return provide


def with_command_owner(base_provider, owner_snapshot):
    """Wrap a readiness provider: also NOT READY while another commander is VISIBLE in the graph
    (a publisher on the controller's topic, or another FollowJointTrajectory client).
    owner_snapshot() -> {'command_publishers', 'foreign_action_clients'}. It is re-evaluated at
    every readiness observation, including the one immediately before dispatch. It is a
    point-in-time graph observation (m61a_fixed_base.command_owner_codes), not a lock."""
    def provide():
        res = base_provider()
        if not isinstance(res, rd.ReadinessResult):
            return res
        snap = owner_snapshot()
        codes = fb.command_owner_codes(snap.get('command_publishers'),
                                       snap.get('foreign_action_clients'))
        report = dict(res.report or {}, m61a_command_owner={'ok': not codes, 'codes': codes,
                                                            'snapshot': snap})
        if not codes:
            return dataclasses.replace(res, report=report)
        return dataclasses.replace(
            res, ready=False, failure_codes=tuple(res.failure_codes) + tuple(codes),
            reasons=tuple(res.reasons) + tuple(f'command owner: {c}' for c in codes),
            report=report)
    return provide


def load_fixed_base_config(config_dir=None):
    """The M6.1-A config with its file SHA-256 recorded (raw['_sha256'])."""
    import hashlib
    path = fb.config_path(config_dir)
    cfg = fb.load_config(os.path.dirname(path))
    with open(path, 'rb') as f:
        cfg.raw['_sha256'] = hashlib.sha256(f.read()).hexdigest()
    return cfg


def with_body_pose(base_provider, latest_pose, now, limits):
    """Wrap a readiness provider: also NOT READY without a fresh, upright, high-enough body pose
    (design section 3.2). latest_pose() -> (wall, stamp, (x, y, z, roll, pitch, yaw)) or None."""
    def provide():
        res = base_provider()
        if not isinstance(res, rd.ReadinessResult):
            return res
        ok, code, detail = gates.pose_readiness(latest_pose(), now(), limits)
        report = dict(res.report or {}, m61_body_pose=detail)
        if ok:
            return dataclasses.replace(res, report=report)
        return dataclasses.replace(
            res, ready=False, failure_codes=tuple(res.failure_codes) + (code,),
            reasons=tuple(res.reasons) + (f'body pose: {code}: {detail.get("reason")}',),
            report=report)
    return provide


__all__ = ['M61Session', 'GATE_TRIPPED', 'M61_TERMINAL', 'OUTCOME_SCHEMA', 'with_body_pose',
           'with_fixed_base', 'fixed_base_record', 'load_fixed_base_config']


# ==================================================================== CLI
# --dry-run    offline goal contract, no ROS graph
# --mock       full state machine against the in-memory M6.1 mock transport (body pose included)
# --live       one goal; refused (exit 3) while M61_LIVE_DISPATCH_ENABLED is False
# There is no option to supply a trajectory, skip the confirmation, retry, repeat or force, and
# no option or environment variable that changes the gate.
# Exit codes: 0 succeeded / dry-run PASS; 1 terminal failure; 2 refused; 3 live dispatch disabled.

EXIT_OK, EXIT_FAILED, EXIT_REFUSED, EXIT_DISABLED = (lp.EXIT_OK, lp.EXIT_FAILED,
                                                     lp.EXIT_REFUSED, lp.EXIT_DISABLED)
DEFAULT_OUT = evd.DEFAULT_ROOT
LIVE_STATE = ('ENABLED for exactly one goal' if c61.M61_LIVE_DISPATCH_ENABLED
              else 'HARD-DISABLED in this build')
CLI_SCOPE = (f'M6.1 protected trot gait replay. Live dispatch is {LIVE_STATE}; --dry-run and '
             '--mock never touch a ROS graph. Nothing here shows walking, locomotion, contact or '
             'balance.')


def parse_args(argv):
    import argparse
    from spiderx_controller import m61_mock
    p = argparse.ArgumentParser(prog='m6_gait_replay', description=CLI_SCOPE)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--dry-run', action='store_true',
                      help='build, preflight and fingerprint the one approved goal; send nothing')
    mode.add_argument('--mock', action='store_true',
                      help='run the full state machine against the in-memory mock transport; '
                           'reads the confirmation word from stdin')
    mode.add_argument('--live', action='store_true',
                      help=f'one live trot-cycle goal to the running simulation ({LIVE_STATE})')
    p.add_argument('--scenario', default='success',
                   help='--mock scenario: ' + ', '.join(sorted(m61_mock.SCENARIOS)))
    p.add_argument('--out', default=DEFAULT_OUT,
                   help=f'evidence root under the git-ignored log/ (default: {DEFAULT_OUT})')
    p.add_argument('--no-write', action='store_true', help='print only (not with --live)')
    p.add_argument('--domain-id', type=int, default=None,
                   help='--live only: the ROS domain id of the running simulation, typed '
                        'explicitly by the operator (never read from the environment)')
    return p.parse_args(argv)


def dry_run_report(plan):
    traj = tc.build_trajectory(plan)
    goal, report, spec, fp = goal61.build_live_goal(traj, plan)
    return {
        'schema': 'spiderx.m61.dry_run/1',
        'mode': 'dry-run',
        'verdict': 'PASS',
        'trajectory_id': traj['trajectory_id'],
        'trajectory_content_sha256': traj['provenance']['content_sha256'],
        'goal_fingerprint': fp,
        'goal_spec': spec,
        'preflight': report.to_dict(),
        'gait': traj['gait'],
        'm61_live_dispatch_enabled': c61.M61_LIVE_DISPATCH_ENABLED,
        'limits': c61.as_dict(),
        'm61_limits': plan.limits.as_dict(),
        'goals_sent': 0,
        'non_claims': list(c61.NON_CLAIMS),
    }


def run_mock(plan, scenario, reader):
    """(outcome, session, trajectory) of one mock run. Nothing reaches any ROS graph."""
    from spiderx_controller import m6_live_mock as m6mock
    from spiderx_controller import m61_mock
    if scenario not in m61_mock.SCENARIOS:
        raise ValueError(f'unknown scenario {scenario!r}; choose from '
                         f'{sorted(m61_mock.SCENARIOS)}')
    traj = tc.build_trajectory(plan)
    fp = gf.fingerprint(goal61.approved_spec(traj, plan))
    latch = lp.InterruptLatch()
    cfg = load_fixed_base_config(plan.config_dir)
    transport = m61_mock.M61FakeTransport(traj, fp, latch=latch, fixed_base=cfg,
                                          **m61_mock.SCENARIOS[scenario])
    report = m6mock.mock_readiness_report(plan.sources.joint_names, plan.neutral)
    provide = with_command_owner(with_fixed_base(
        with_body_pose(lambda: rd.assess(report, transport.wall_now()),
                       transport.latest_body_pose, transport.wall_now, plan.limits),
        transport.fixed_base_snapshot, cfg), transport.command_owner_snapshot)
    session = M61Session(transport, plan, provide, reader, latch=latch, fixed_base=cfg)
    out = session.run()
    out.update(mode='mock', scenario=scenario, mock_server_goals_received=len(transport.sent),
               mock_server_cancels_received=transport.cancels)
    return out, session, traj


def _gate_state():
    return {'m61_live_dispatch_enabled': c61.M61_LIVE_DISPATCH_ENABLED,
            'm6d_live_dispatch_enabled (not used by M6.1)': m6d.LIVE_DISPATCH_ENABLED}


def _write_run(root, name, outcome_file, data, session, trajectory):
    """Reserve <root>/<name>, write the outcome and every supporting file (exclusive)."""
    run_dir = evd.reserve_run_dir(root, name)
    rows = session.evidence_rows() if session is not None else {}
    data['evidence_files'] = evd.write_support_files(run_dir, data, trajectory, rows,
                                                     _gate_state())
    data['run_dir'] = run_dir
    evd.write_exclusive(os.path.join(run_dir, outcome_file), evd.dumps(data))
    return run_dir


def _reserved_record(args, utc, path):
    return {'schema': OUTCOME_SCHEMA, 'milestone': 'M6.1', 'mode': 'live',
            'state': 'NOT_DISPATCHED', 'terminal': False, 'passed': False, 'reason': None,
            'run_utc': utc, 'domain_id': args.domain_id, 'goals_sent': 0, 'cancels_sent': 0,
            'dispatch': lp.dispatch_record(), 'errors': [],
            'evidence': {'path': path, 'phase': 'reserved'},
            'evidence_phases': lp.EVIDENCE_PHASES, 'limits': c61.as_dict(),
            'non_claims': list(c61.NON_CLAIMS)}


def _setup_failure_outcome(exc):
    """Outcome when the run failed before any session ran (so nothing was sent)."""
    return {'schema': OUTCOME_SCHEMA, 'milestone': 'M6.1', 'state': lp.REFUSED, 'terminal': True,
            'passed': False, 'reason': 'session_setup_failed', 'goals_sent': 0,
            'cancels_sent': 0, 'retries': 0, 'automatic_return_goals': 0,
            'dispatch': lp.dispatch_record(),
            'errors': [{'where': 'setup', 'type': type(exc).__name__, 'message': str(exc)}],
            'limits': c61.as_dict(), 'non_claims': list(c61.NON_CLAIMS)}


def _default_transport_factory(fp, domain_id, fixed_base):
    from spiderx_controller import m61a_live
    return m61a_live.M61AFixedBaseTransport(fp, domain_id, fixed_base).open()


def _default_collect_factory(transport):
    return transport.graph_collector()


def _run_live(plan, transport_factory, collect_factory, reader, latch, domain_id,
              checkpoint=None):
    """The future live path: (outcome, session). Refuses unless M61_LIVE_DISPATCH_ENABLED.

    An exception that escapes this function was raised before any session ran, so nothing was
    sent; anything raised while the session runs becomes part of its outcome.
    """
    if not c61.M61_LIVE_DISPATCH_ENABLED:
        raise PermissionError(c61.M61_LIVE_DISPATCH_DISABLED_MESSAGE)
    traj = tc.build_trajectory(plan)
    fp = gf.fingerprint(goal61.approved_spec(traj, plan))
    cfg = load_fixed_base_config(plan.config_dir)
    transport = transport_factory(fp, domain_id, cfg)
    out = session = None
    try:
        base = rd.make_readiness_provider(collect_factory(transport), plan.sources.joint_names,
                                          plan.neutral, transport.wall_now)
        provide = with_command_owner(with_fixed_base(
            with_body_pose(base, transport.latest_body_pose, transport.wall_now, plan.limits),
            transport.fixed_base_snapshot, cfg), transport.command_owner_snapshot)
        session = M61Session(transport, plan, provide, reader, latch=latch,
                             checkpoint=checkpoint, fixed_base=cfg)
        out = session.run()
        return out, session
    finally:
        lp._close_transport(transport, out)


def _live_main(args, plan, reader, transport_factory=None, collect_factory=None, latch=None,
               utc_stamp=None, evidence_factory=None):
    """--live after the gate: one goal, persisted evidence under <out>/<UTC>/, never an overwrite.

    Reached only when M61_LIVE_DISPATCH_ENABLED is True. Before any ROS initialization the run
    directory and its live_outcome.json are reserved exclusively and a NOT_DISPATCHED record is
    saved; if either fails, nothing starts.
    """
    if not c61.M61_LIVE_DISPATCH_ENABLED:
        print(f'REFUSED: {c61.M61_LIVE_DISPATCH_DISABLED_MESSAGE}')
        return EXIT_DISABLED
    if args.no_write:
        print('REFUSED: --live always records evidence; --no-write is not allowed with --live')
        return EXIT_REFUSED
    if args.domain_id is None or not 0 <= args.domain_id <= lp.MAX_DOMAIN_ID:
        print(f'REFUSED: --live needs an explicit --domain-id in 0..{lp.MAX_DOMAIN_ID}')
        return EXIT_REFUSED
    utc = utc_stamp or evd.utc_stamp()
    try:
        run_dir = evd.reserve_run_dir(args.out, utc)
        target = os.path.join(run_dir, evd.LIVE_OUTCOME)
        evidence = (evidence_factory or lp.EvidenceFile.reserve)(target)
    except FileExistsError:
        print(f'REFUSED: {args.out}/{utc} already exists (evidence is never overwritten); '
              'nothing started')
        return EXIT_REFUSED
    except OSError as e:
        print(f'REFUSED: cannot reserve live evidence under {args.out}/{utc} '
              f'({type(e).__name__}: {e}); nothing started')
        return EXIT_REFUSED
    try:
        evidence.write(_reserved_record(args, utc, target))
    except Exception as e:  # noqa: BLE001
        evidence.close()
        print(f'REFUSED: the initial NOT_DISPATCHED evidence could not be saved '
              f'({type(e).__name__}: {e}); nothing started')
        return EXIT_REFUSED
    latch = latch or lp.InterruptLatch()
    if reader is None:             # called (and the prompt shown) only after readiness #1 passed
        reader = lp.interruptible_line_reader(
            latch, prompt=f'Readiness passed (including a fresh body pose and the M6.1-A '
                          f'fixed-base checks). Type '
                          f'{c61.CONFIRMATION_WORD} to send ONE trot-cycle goal (neutral -> one '
                          f'trot cycle -> neutral, 7.0 s sim time) to the running simulation on '
                          f'ROS domain {args.domain_id}; anything else, EOF or Ctrl+C refuses:')
    session = traj = None
    latch.install()
    try:
        if plan is None:
            plan = tc.load_plan()
        data, session = _run_live(plan, transport_factory or _default_transport_factory,
                                  collect_factory or _default_collect_factory, reader, latch,
                                  args.domain_id, checkpoint=evidence.checkpoint)
        traj = session._trajectory
    except Exception as e:  # noqa: BLE001 - raised before any session ran: nothing was sent
        data = _setup_failure_outcome(e)
    finally:
        latch.uninstall()
    data.update(mode='live', domain_id=args.domain_id, run_utc=utc, run_dir=run_dir,
                evidence_phases=lp.EVIDENCE_PHASES)
    rows = session.evidence_rows() if session is not None else {}
    data['evidence_files'] = evd.write_support_files(run_dir, data, traj, rows, _gate_state())
    return lp._finish_live(evidence, data)


def main(argv=None, plan=None, reader=None):
    import sys
    argv = list(argv if argv is not None else sys.argv)
    args = parse_args(argv[1:])
    print('SpiderX M6.1 protected trot gait replay')
    print(CLI_SCOPE)
    if args.live:
        # the M6.1 gate, checked before any configuration, ROS import or input
        if not c61.M61_LIVE_DISPATCH_ENABLED:
            print(f'REFUSED: {c61.M61_LIVE_DISPATCH_DISABLED_MESSAGE}')
            return EXIT_DISABLED
        return _live_main(args, plan, reader)
    if args.domain_id is not None:
        print('REFUSED: --domain-id is only valid with --live')
        return EXIT_REFUSED
    if plan is None:
        plan = tc.load_plan()
    if args.dry_run:
        try:
            data = dry_run_report(plan)
        except (tc.TrajectoryBuildError, gf.FingerprintError, ac.PreflightRefused) as e:
            print(f'REFUSED: {e}')
            return EXIT_REFUSED
        print(f'Dry run PASS: trajectory {data["trajectory_id"]}, content '
              f'{data["trajectory_content_sha256"][:16]}, goal fingerprint '
              f'{data["goal_fingerprint"]}; nothing sent')
        if not args.no_write:
            target = os.path.join(args.out, 'dry_run', data['trajectory_id'],
                                  'dry_run_report.json')
            try:
                os.makedirs(os.path.dirname(target), exist_ok=True)
                print(f'Report: {evd.write_exclusive(target, evd.dumps(data))}')
            except FileExistsError:
                print(f'REFUSED: {target} already exists (reports are never overwritten)')
                return EXIT_REFUSED
        return EXIT_OK
    if reader is None:
        print(f'Type {c61.CONFIRMATION_WORD} to continue the MOCK run (nothing is sent to any '
              'ROS graph):')
        reader = sys.stdin.readline
    try:
        data, session, traj = run_mock(plan, args.scenario, reader)
    except (ValueError, tc.TrajectoryBuildError, gf.FingerprintError) as e:
        print(f'REFUSED: {e}')
        return EXIT_REFUSED
    print(f'Mock {args.scenario}: final state {data["state"]}, reason {data["reason"]}, goals at '
          f'mock server {data["mock_server_goals_received"]}, cancels '
          f'{data["mock_server_cancels_received"]}')
    if not args.no_write:
        name = f'{evd.utc_stamp()}_{args.scenario}'
        try:
            run_dir = _write_run(os.path.join(args.out, 'mock'), name, evd.MOCK_OUTCOME, data,
                                 session, traj)
        except FileExistsError:
            print(f'REFUSED: {args.out}/mock/{name} already exists (never overwritten)')
            return EXIT_REFUSED
        print(f'Evidence: {run_dir}')
    return lp._exit_code(data['state'])
