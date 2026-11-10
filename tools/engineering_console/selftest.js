// Offline self-check of the browser math against numbers documented in the repository.
const { execSync } = require('child_process');
const R = require('./src/10_robot.js');
const d = JSON.parse(execSync('python3 -c "import json,extract;d=extract.extract();print(json.dumps(d[\'robot\']))"', { cwd: __dirname, maxBuffer: 1e9 }));
const m = new R.RobotModel(d);
let fail = 0; const ok = (name, cond, info) => { console.log((cond ? 'PASS ' : 'FAIL ') + name + ' ' + (info || '')); if (!cond) fail++; };
const g = m.legGeometry('lf');
const near = (a, b, t) => a.every((x, i) => Math.abs(x - b[i]) <= t);
ok('H (M3_FRAME_CONVENTIONS)', near(g.axisPoints[0], [0.010, 0.0161, 0.07725], 1e-9));
ok('T', near(g.axisPoints[1], [-0.0237, 0.03885, 0.08875], 1e-9));
ok('K', near(g.axisPoints[2], [-0.0367, -0.038932, 0.010968], 1e-9));
ok('tip0', near(g.tip0, [-0.033548, 0.042264, -0.054543], 1e-6));
// tree FK == PoE FK at random configs; IK round trip
let worst = 0, worstIk = 0, n = 0;
for (const k of ['lf', 'rf', 'lr', 'rr']) {
  const gg = m.legGeometry(k);
  for (let i = 0; i < 300; i++) {
    const q = gg.limits.map(([lo, hi]) => lo + 0.06 + Math.random() * (hi - lo - 0.12));
    m.setAll(gg.names, q);
    const a = m.tip(k), b = R.fkTip(gg, q);
    worst = Math.max(worst, R.V.norm(R.V.sub(a, b)));
    const r = R.ikLeg(gg, a, 0.05, q);
    if (!r.ok) { worstIk = 1; continue; }
    worstIk = Math.max(worstIk, Math.hypot(...r.solution.map((x, j) => x - q[j]))); n++;
  }
  m.setAll(gg.names, [0, 0, 0]);
}
ok('tree FK equals PoE FK', worst < 1e-12, worst.toExponential(2));
ok('IK round trip over ' + n + ' configs', worstIk < 1e-6, worstIk.toExponential(2));
// crouch_10mm: documented max commanded displacement 0.1223 rad (STATUS.md, M6.0)
let mx = 0;
for (const k of ['lf', 'rf', 'lr', 'rr']) { const gg = m.legGeometry(k); const r = R.ikLeg(gg, R.V.add(gg.tip0, [0, 0, 0.010])); mx = Math.max(mx, ...r.solution.map(Math.abs)); }
ok('crouch_10mm max |q| = 0.1223 rad', Math.abs(mx - 0.1223) < 5e-5, mx.toFixed(5));
// negative targets from m4_pose_targets.yaml
const grr = m.legGeometry('rr'); const neg = R.ikLeg(grr, R.V.add(grr.tip0, [0, 0, -0.30]));
ok('rr 300 mm below is refused', !neg.ok && neg.reason.startsWith('unreachable'), neg.reason);
const glr = m.legGeometry('lr'); const neg2 = R.ikLeg(glr, R.fkTip(glr, [0, 1.2, 0]));
ok('lr thigh 1.2 rad is refused for joint limits', !neg2.ok && neg2.reason === 'joint_limits', neg2.reason);
// sign table: +lf_hip moves tip outward (-x); +lf_thigh rearward (-y); +lf_foot forward (+y) and up
ok('lf_hip sign', R.fkTip(g, [0.1, 0, 0])[0] < g.tip0[0]);
ok('lf_thigh sign', R.fkTip(g, [0, 0.1, 0])[1] < g.tip0[1]);
ok('lf_foot sign', R.fkTip(g, [0, 0, 0.1])[1] > g.tip0[1] && R.fkTip(g, [0, 0, 0.1])[2] > g.tip0[2]);
process.exit(fail ? 1 : 0);
