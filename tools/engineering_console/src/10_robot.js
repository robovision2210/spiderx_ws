/* Robot math. Browser re-implementation of the project's published formulas, driven only by values
   extracted from spiderx.urdf.xacro. It is NOT the project's Python code and proves nothing about it. */
const V = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  norm: (a) => Math.hypot(a[0], a[1], a[2]),
  unit: (a) => { const n = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / n, a[1] / n, a[2] / n]; },
};
/* 3x3 rotation as row-major array of 9 */
function rotAxis(axis, ang) {
  const [x, y, z] = V.unit(axis), c = Math.cos(ang), s = Math.sin(ang), C = 1 - c;
  return [c + x * x * C, x * y * C - z * s, x * z * C + y * s,
    y * x * C + z * s, c + y * y * C, y * z * C - x * s,
    z * x * C - y * s, z * y * C + x * s, c + z * z * C];
}
function mm(a, b) {
  const o = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
    o[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
  return o;
}
const mv = (m, v) => [m[0] * v[0] + m[1] * v[1] + m[2] * v[2], m[3] * v[0] + m[4] * v[1] + m[5] * v[2], m[6] * v[0] + m[7] * v[1] + m[8] * v[2]];
const I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
function rotRpy(r, p, y) { return mm(rotAxis([0, 0, 1], y), mm(rotAxis([0, 1, 0], p), rotAxis([1, 0, 0], r))); }

class RobotModel {
  constructor(data) {
    this.data = data;
    this.links = data.links; this.joints = data.joints;
    this.jointByName = {}; this.childJoints = {}; this.jointOfChild = {};
    for (const j of this.joints) {
      this.jointByName[j.name] = j; this.jointOfChild[j.child] = j;
      (this.childJoints[j.parent] = this.childJoints[j.parent] || []).push(j);
    }
    this.revolute = this.joints.filter((j) => j.type === 'revolute');
    this.q = {}; for (const j of this.revolute) this.q[j.name] = 0;
    this.pose = {}; this.update();
  }
  /* URDF semantics: T_parent_child(q) = Trans(xyz) . Rz Ry Rx . Rot(axis, q) */
  update() {
    const walk = (link, R, p) => {
      this.pose[link] = { R, p };
      for (const j of this.childJoints[link] || []) {
        let Rj = mm(R, rotRpy(j.rpy[0], j.rpy[1], j.rpy[2]));
        const pj = V.add(p, mv(R, j.xyz));
        j.world = { p: pj, axis: j.axis ? V.unit(mv(Rj, j.axis)) : null };
        if (j.type === 'revolute') Rj = mm(Rj, rotAxis(j.axis, this.q[j.name] || 0));
        walk(j.child, Rj, pj);
      }
    };
    walk('dummy_link', I3, [0, 0, 0]);
  }
  setAll(names, values) { names.forEach((n, i) => { this.q[n] = values[i]; }); this.update(); }
  tip(legKey) {
    const leg = this.data.legs[legKey], ps = this.pose[leg.foot_link];
    return V.add(ps.p, mv(ps.R, leg.tip_local));
  }
  /* Chain from base_link to a link: list of joints */
  chain(link) { const out = []; let l = link; while (this.jointOfChild[l]) { out.unshift(this.jointOfChild[l]); l = this.jointOfChild[l].parent; } return out; }
  legGeometry(legKey) {
    const leg = this.data.legs[legKey];
    const saved = { ...this.q };
    for (const n of leg.joints) this.q[n] = 0; this.update();
    const js = leg.joints.map((n) => this.jointByName[n]);
    const g = { key: legKey, names: leg.joints, axisPoints: js.map((j) => j.world.p.slice()), axes: js.map((j) => j.world.axis.slice()),
      tip0: this.tip(legKey), limits: js.map((j) => [j.lower, j.upper]) };
    this.q = saved; this.update();
    return g;
  }
}
/* Tip by product of exponentials (same form as leg_kinematics.forward_tip_poe). */
function fkTip(g, q) {
  const [H, T, K] = g.axisPoints, [a1, a2, a3] = g.axes;
  const inner = V.add(V.sub(K, T), mv(rotAxis(a3, q[2]), V.sub(g.tip0, K)));
  const mid = V.add(V.sub(T, H), mv(rotAxis(a2, q[1]), inner));
  return V.add(H, mv(rotAxis(a1, q[0]), mid));
}
const wrapAng = (a) => Math.atan2(Math.sin(a), Math.cos(a));
function acosChecked(c) { if (c > 1 + 1e-12 || c < -1 - 1e-12) return null; return Math.acos(Math.max(-1, Math.min(1, c))); }
/* Analytic position IK, ported step by step from leg_kinematics.inverse (f22ac9f). Never clamps. */
function ikLeg(g, target, margin = 0.05, ref = [0, 0, 0]) {
  const res = { ok: false, reason: 'invalid_input', solution: null, candidates: [] };
  const [H, T, K] = g.axisPoints, [u, w, a3] = g.axes;
  const v = V.cross(u, w), sigma = V.dot(w, a3) > 0 ? 1 : -1;
  const r = V.sub(target, H), d = V.dot(V.sub(g.tip0, H), w);
  const rw = V.dot(r, w), rv = V.dot(r, v), rho = Math.hypot(rw, rv);
  if (rho < 1e-12) { res.reason = 'singular'; res.message = 'target lies on the hip axis'; return res; }
  const alpha = acosChecked(d / rho);
  if (alpha === null) { res.reason = 'unreachable_hip'; res.message = 'target is too close to the hip axis for this leg'; return res; }
  const phi = Math.atan2(rv, rw);
  const hips = alpha < 1e-12 ? [phi + alpha] : [phi + alpha, phi - alpha];
  const perp = (x) => V.sub(x, V.scale(w, V.dot(x, w)));
  const L1p = perp(V.sub(K, T)), L2p = perp(V.sub(g.tip0, K));
  const l1 = V.norm(L1p), l2 = V.norm(L2p), b1 = V.unit(L1p), b2 = V.cross(w, b1);
  const to2 = (x) => [V.dot(x, b1), V.dot(x, b2)];
  const A = to2(L1p), B = to2(L2p), bA = Math.atan2(A[1], A[0]), bB = Math.atan2(B[1], B[0]);
  for (const q1 of hips) {
    const e = V.sub(mv(rotAxis(u, -q1), r), V.sub(T, H));
    const E = to2(perp(e));
    const psi = acosChecked((E[0] * E[0] + E[1] * E[1] - l1 * l1 - l2 * l2) / (2 * l1 * l2));
    if (psi === null) continue;
    const th3s = psi < 1e-12 ? [psi - (bB - bA)] : [psi - (bB - bA), -psi - (bB - bA)];
    for (const th3 of th3s) {
      const c3 = Math.cos(th3), s3 = Math.sin(th3);
      const S = [A[0] + c3 * B[0] - s3 * B[1], A[1] + s3 * B[0] + c3 * B[1]];
      const q2 = Math.atan2(E[1], E[0]) - Math.atan2(S[1], S[0]);
      const cand = [wrapAng(q1), wrapAng(q2), wrapAng(sigma * th3)];
      const residual = V.norm(V.sub(fkTip(g, cand), target));
      const viol = [];
      cand.forEach((x, i) => { if (!(g.limits[i][0] + margin <= x && x <= g.limits[i][1] - margin)) viol.push(g.names[i]); });
      res.candidates.push({ q: cand, residual, viol, dist: Math.hypot(cand[0] - ref[0], cand[1] - ref[1], cand[2] - ref[2]), valid: residual <= 1e-6 && !viol.length });
    }
  }
  if (!res.candidates.length) { res.reason = 'unreachable_knee'; res.message = 'target is outside the reach of thigh plus foot'; return res; }
  const valid = res.candidates.filter((c) => c.valid);
  if (!valid.length) {
    if (res.candidates.every((c) => c.residual <= 1e-6)) { res.reason = 'joint_limits'; res.message = 'reachable by geometry, but every solution needs a joint beyond its URDF limit minus the 0.05 rad margin'; res.viol = res.candidates[0].viol; }
    else { res.reason = 'numerical'; res.message = 'no candidate reproduced the target'; }
    return res;
  }
  valid.sort((a, b) => a.dist - b.dist);
  res.ok = true; res.reason = 'ok'; res.solution = valid[0].q; res.residual = valid[0].residual;
  return res;
}
/* Gait phase and foot offset: same rules as gait_phase.leg_phase and gait_trajectory.foot_offset_and_velocity. */
const LEG_FULL = { lf: 'front_left', rf: 'front_right', lr: 'rear_left', rr: 'rear_right' };
function legPhase(gait, legKey, u) {
  let th = (((u - gait.phase_offsets[LEG_FULL[legKey]]) % 1) + 1) % 1;
  const sw = 1 - gait.duty_factor;
  if (th >= 1 - 1e-12) th = 0;
  if (Math.abs(th - sw) <= 1e-12) th = sw;
  return th < sw ? { phase: 'swing', s: th / sw } : { phase: 'stance', s: (th - sw) / gait.duty_factor };
}
function footOffset(gait, P, legKey, u) {
  const ph = legPhase(gait, legKey, u), L = P.step_length_m, h = P.step_height_m, v = P.body_speed_m_s;
  const beta = gait.duty_factor, T = L / (beta * v), S = L / beta, tau = (1 - beta) * T;
  if (ph.phase === 'stance') return { ph, off: [0, L / 2 - L * ph.s, 0] };
  const a = 2 * Math.PI * ph.s;
  return { ph, off: [0, -L / 2 + S * (ph.s - Math.sin(a) / (2 * Math.PI)) - v * (ph.s * tau), h * (1 - Math.cos(a)) / 2] };
}
if (typeof module !== 'undefined') module.exports = { V, RobotModel, fkTip, ikLeg, legPhase, footOffset, rotAxis, mv, mm };
