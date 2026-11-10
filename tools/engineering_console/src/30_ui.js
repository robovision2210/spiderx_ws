/* Interface. One shared 3D stage moves between stations. No network calls, no live robot connection. */
const $ = (s, r = document) => r.querySelector(s);
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const h = (html) => { const t = document.createElement('template'); t.innerHTML = html.trim(); return t.content.firstElementChild; };
const store = { get(k, d) { try { const v = localStorage.getItem('spx.' + k); return v == null ? d : JSON.parse(v); } catch (e) { return d; } }, set(k, v) { try { localStorage.setItem('spx.' + k, JSON.stringify(v)); } catch (e) { /* storage optional */ } } };
const EV = []; /* evidence entries opened by chips */
function chip(l, src, claim, note) { const i = EV.push({ l, src: src || [], claim, note }) - 1; return '<button class="chip ' + l + '" data-ev="' + i + '" title="' + esc(LABELS[l].name) + '. Open the evidence.">' + LABELS[l].short + '</button>'; }
function terms(text) {
  const keys = Object.keys(GLOSSARY).sort((a, b) => b.length - a.length), used = new Set();
  const re = new RegExp('(?<![\\w/-])(' + keys.map((k) => k.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')).join('|') + ')(?![\\w/])', 'gi');
  return esc(text).replace(re, (m) => { const k = keys.find((x) => x.toLowerCase() === m.toLowerCase()); if (!k || used.has(k) || (k.length < 4 && m !== k)) return m; used.add(k); return '<span class="term" tabindex="0" data-tip="' + esc(GLOSSARY[k]) + '">' + m + '</span>'; });
}
const fmt = (x, n = 3) => (x >= 0 ? '+' : '') + x.toFixed(n);
const SNAP = DATA.snapshot, model = new RobotModel(DATA.robot), LEGK = ['lf', 'rf', 'lr', 'rr'];
const GEO = {}; LEGK.forEach((k) => { GEO[k] = model.legGeometry(k); });
const ALLJ = LEGK.flatMap((k) => DATA.robot.legs[k].joints);
const state = { view: 'mission', motion: !matchMedia('(prefers-reduced-motion: reduce)').matches, sel: 'body', rview: 'whole', lab: 'fk', leg: 'lf', gait: 'trot', u: 0, playing: false, ik: [0, 0, 0], rp: 0, rj: 'lr_thigh_joint', flow: 2, flowPlay: false, map: 'packages', ms: 'M6.1', msFilter: null, lesson: store.get('lesson', 0), done: store.get('done', []) };
/* which leg and which driving joint each link belongs to */
const linkLeg = {}, linkDriver = {};
for (const l of model.links) { const ch = model.chain(l.name).filter((j) => j.type === 'revolute'); if (ch.length) { linkLeg[l.name] = ch[0].name.slice(0, 2); linkDriver[l.name] = ch.map((j) => j.name); } }
LEGK.forEach((k) => { COMPONENTS['leg_' + k] = { title: LEG_INFO[k].name, kind: 'Leg (3 joints)', what: 'A chain of three revolute joints from the hip servo body ' + LEG_INFO[k].servo + ' to the foot link ' + DATA.robot.legs[k].foot_link + ': ' + DATA.robot.legs[k].joints.join(', ') + '.', does: 'Places one foot. Hip swings the leg sideways, thigh and knee work as a two link arm in one plane.', why: 'Four such legs support and move the body. Each needs three joints because a foot position has three coordinates.', rel: 'Parent: the body. Its three joints are commanded together with the other nine as one 12 joint goal. ' + LEG_INFO[k].note, l: 'C', src: [CTL + 'config/spiderx_legs.yaml:27', DSC + 'urdf/spiderx.urdf.xacro:' + model.jointByName[DATA.robot.legs[k].joints[0]].line], links: model.links.filter((l) => linkLeg[l.name] === k).map((l) => l.name), leg: k }; });
for (const j of model.revolute) {
  const k = j.name.slice(0, 2), role = j.name.includes('hip') ? 'hip' : j.name.includes('thigh') ? 'thigh' : 'foot', R = JOINT_ROLE[role];
  COMPONENTS['j_' + j.name] = { title: j.name, kind: R.what, what: 'A revolute joint between ' + j.parent + ' and ' + j.child + '. Axis [' + j.axis.join(', ') + '] in its parent frame. URDF limits ' + j.lower.toFixed(3) + ' to ' + j.upper.toFixed(3) + ' rad (' + (j.lower * 57.2958).toFixed(0) + ' to ' + (j.upper * 57.2958).toFixed(0) + ' degrees).', does: R.does, why: R.why,
    rel: 'Part of the ' + LEG_INFO[k].name.toLowerCase() + '. Commanded through leg_trajectory_controller with a position interface. Tools keep 0.05 rad inside the limits. Effort ' + j.effort + ' and velocity ' + j.velocity + ' in the URDF are placeholders.', l: 'C',
    src: [DSC + 'urdf/spiderx.urdf.xacro:' + j.line, CTL + 'config/spiderx_legs.yaml:27', DSC + 'urdf/spiderx_fortress_control.xacro:29'], links: model.links.filter((l) => (linkDriver[l.name] || []).includes(j.name)).map((l) => l.name), joint: j.name, leg: k };
}
/* ---------- stage ---------- */
const stageEl = h('<div class="stage"><canvas tabindex="0" aria-label="Interactive 3D view of the SpiderX meshes. Drag to orbit, scroll to zoom, arrow keys to rotate."></canvas><div class="scan"></div><div class="labels"></div><div class="hud"></div><div class="hint">Drag to orbit · scroll or pinch to zoom · shift drag to pan · click a part to select it</div><div class="cams"></div></div>');
const viewer = new Viewer($('canvas', stageEl), model, DATA.mesh_b64);
if (!viewer.ok) stageEl.appendChild(h('<div class="fallback"><p><b>3D view unavailable.</b><br>This browser did not provide WebGL, so the real meshes cannot be drawn. Every other station still works. Nothing has been substituted for the robot.</p></div>'));
const CAMS = { Iso: { yaw: -0.9, pitch: 0.42, dist: 0.62, target: [0.051, -0.06, 0.02] }, Front: { yaw: Math.PI / 2, pitch: 0.08, dist: 0.6, target: [0.051, -0.06, 0.02] }, Side: { yaw: 0, pitch: 0.08, dist: 0.66, target: [0.051, -0.06, 0.02] }, Top: { yaw: -Math.PI / 2, pitch: 1.5, dist: 0.7, target: [0.051, -0.06, 0.02] } };
$('.cams', stageEl).innerHTML = Object.keys(CAMS).map((k) => '<button class="btn" data-cam="' + k + '">' + k + '</button>').join('');
$('.cams', stageEl).addEventListener('click', (e) => { const k = e.target.dataset.cam; if (k && viewer.ok) viewer.flyTo(CAMS[k]); });
function hud(tags) { $('.hud', stageEl).innerHTML = tags.map((t) => '<span class="tag ' + (t[1] || '') + '">' + esc(t[0]) + '</span>').join(''); }
function mount(slot) { if (slot) slot.appendChild(stageEl); if (viewer.ok) viewer.dirty = true; }
const TRI = [[[1, 0, 0], [1, 0.42, 0.3], '+x right'], [[0, 1, 0], [0.4, 0.9, 0.5], '+y front'], [[0, 0, 1], [0.35, 0.6, 1], '+z up']];
function triad(o, R, len, lines, labels, name) { TRI.forEach(([a, c, t]) => { const e = V.add(o, V.scale(mv(R, a), len)); lines.push([o, e, c, 1, true]); if (name) labels.push({ p: e, text: t.slice(0, 2) }); }); if (name) labels.push({ p: o, text: name, cls: 'cy' }); }
function resetPose() { model.setAll(ALLJ, ALLJ.map(() => 0)); }
function axesLines(only) { const out = []; for (const j of model.revolute) { if (only && !only(j)) continue; const a = j.world.axis, p = j.world.p; out.push([V.sub(p, V.scale(a, 0.035)), V.add(p, V.scale(a, 0.035)), [1, 0.57, 0.25], 1, true]); } return out; }
/* ---------- evidence drawer ---------- */
const drawer = $('#drawer');
function openEv(i) {
  const e = EV[i], L = LABELS[e.l];
  drawer.innerHTML = '<div class="row" style="justify-content:space-between"><h2>Evidence</h2><button class="btn" id="dClose">Close</button></div><div><span class="chip ' + e.l + '">' + L.name + '</span></div><p style="color:var(--ink2)">' + esc(L.help) + '</p>' +
    (e.claim ? '<div><h2>Claim</h2><p>' + esc(e.claim) + '</p></div>' : '') + (e.note ? '<div class="msg">' + esc(e.note) + '</div>' : '') +
    '<div><h2>Sources in the repository</h2><div class="srcs">' + (e.src.length ? e.src.map((s) => '<div><code>' + esc(s) + '</code><button class="btn" data-copy="' + esc(s) + '">Copy</button></div>').join('') : '<p>None.</p>') + '</div></div>' +
    '<p class="mono" style="color:var(--ink3)">Snapshot ' + SNAP.short + ' · ' + SNAP.date + '. Paths are relative to the repository root. A number after a colon is a line.</p>';
  drawer.hidden = false; $('#dClose').focus();
}
document.addEventListener('click', (e) => {
  const c = e.target.closest('[data-ev]'); if (c) { openEv(+c.dataset.ev); return; }
  const cp = e.target.closest('[data-copy]'); if (cp) { const t = cp.dataset.copy; (navigator.clipboard ? navigator.clipboard.writeText(t) : Promise.reject()).then(() => { cp.textContent = 'Copied'; }).catch(() => { const r = document.createRange(); r.selectNodeContents(cp.previousElementSibling); const s = getSelection(); s.removeAllRanges(); s.addRange(r); cp.textContent = 'Selected'; }); return; }
  if (e.target.id === 'dClose') drawer.hidden = true;
  const go = e.target.closest('[data-go]'); if (go) { e.preventDefault(); location.hash = go.dataset.go; }
});
document.addEventListener('keydown', (e) => { if (e.key === 'Escape') { drawer.hidden = true; $('#lb').hidden = true; } });
/* ---------- stations ---------- */
const VIEWS = [['mission', 'Mission Control'], ['robot', 'Explore the Robot'], ['flow', 'Command to Motion'], ['map', 'ROS 2 System Map'], ['lab', 'Kinematics and Gait Lab'], ['evidence', 'Evidence Gallery'], ['milestones', 'Milestone Map'], ['learn', 'Learn from Zero'], ['next', 'Limits and Next']];
$('#nav').innerHTML = VIEWS.map(([id, t], i) => '<a href="#' + id + '"><i>0' + (i + 1) + '</i>' + t + '</a>').join('');
const snapHtml = '<b>SNAPSHOT · NOT CONNECTED</b><span>Static reading of the repository. No link to any robot or simulator.</span><span>Branch<br><span class="mono">' + esc(SNAP.branch) + '</span></span><span>Commit <span class="mono">' + SNAP.short + '</span> · ' + SNAP.date + '</span>';
$('#snap').innerHTML = snapHtml;
const legendHtml = '<div class="legend">' + Object.keys(LABELS).map((k) => '<span><span class="chip ' + k + '">' + LABELS[k].name + '</span></span>').join('') + '<span>Select any label to open its sources.</span></div>';
const head = (n, title, sub) => '<div class="vh"><span class="eyebrow">Station 0' + n + '</span><h1>' + title + '</h1><p>' + sub + '</p></div>';
const R = {};
R.mission = (el) => {
  const li = (a) => '<li><span>' + terms(a.t) + '</span>' + chip(a.l, a.src, a.t, a.note) + '</li>';
  el.innerHTML = head(1, 'Mission Control', 'What SpiderX is, what it is for, what it can show today and what nobody has proven yet.') +
    '<div class="snap m">' + snapHtml + '</div>' +
    '<div class="grid2"><div class="panel plain"><div class="slot"></div>' + legendHtml + '</div>' +
    '<div style="display:flex;flex-direction:column;gap:16px"><div class="panel"><h2>What is SpiderX?</h2><p>' + terms(MISSION.what) + '</p><div class="read"><div><span>JOINTS</span><b>12</b></div><div><span>SIZE (m)</span><b style="font-size:13px">' + DATA.robot.bbox.size.map((x) => x.toFixed(2)).join(' × ') + '</b></div><div><span>LOCOMOTION</span><b style="font-size:13px;color:var(--or)">NOT YET</b></div></div><p style="font-size:12.5px;color:var(--ink3)">Size recomputed from the meshes at the neutral pose ' + chip('C', [DSC + 'meshes/', D + 'SPIDERX_URDF_AUDIT.md:111'], 'Bounding box 0.187 × 0.363 × 0.166 m at the neutral pose', 'Recomputed from the 30 STL files for this console. It matches the audit.') + '</p></div>' +
    '<div class="panel"><h2>Why is it being built?</h2><p>' + esc(MISSION.why) + '</p><p style="font-size:12.5px;color:var(--ink3)">The Torq purpose comes from your brief. The repository holds no evidence for it.</p></div></div></div>' +
    '<div class="grid2 even"><div class="panel"><h2>What it can demonstrate today</h2><ul class="clean">' + MISSION.achieved.map(li).join('') + '</ul></div>' +
    '<div style="display:flex;flex-direction:column;gap:16px"><div class="panel"><h2>Objectives, not yet achieved</h2><ul class="clean">' + MISSION.objectives.map(li).join('') + '</ul></div>' +
    '<div class="panel"><h2 style="color:var(--or)">Not yet proven</h2><ul class="dash">' + MISSION.unproven.map((t) => '<li>' + esc(t) + '</li>').join('') + '</ul></div></div></div>';
  mount($('.slot', el)); resetPose();
  if (viewer.ok) { viewer.highlight = null; viewer.lines = []; viewer.setLabels($('.labels', stageEl), [{ p: [0.051, 0.11, 0.06], text: 'front (+y)', cls: 'cy' }, { p: [0.051, -0.045, 0.16], text: 'lidar_link' }]); viewer.autoRotate = state.motion; viewer.onFrame = null; viewer.flyTo(CAMS.Iso); }
  hud([['REAL MESHES · 30 STL files from the repository', 'cy'], ['NEUTRAL POSE · all joints 0 rad']]);
};
function applyRobot() {
  const c = COMPONENTS[state.sel]; if (!viewer.ok) return;
  const lines = [], labels = []; viewer.hidden.clear(); viewer.highlight = c.links && c.links.length ? new Set(c.links) : null;
  if (state.rview === 'leg') { const k = c.leg || 'lf'; for (const l of model.links) if (linkLeg[l.name] && linkLeg[l.name] !== k) viewer.hidden.add(l.name); }
  if (state.rview === 'axes' || c.joint) { lines.push(...axesLines(c.joint && state.rview !== 'axes' ? (j) => j.name === c.joint : null)); if (state.rview === 'axes') for (const j of model.revolute) labels.push({ p: (() => j.world.p), text: j.name, cls: c.joint === j.name ? 'or' : '' }); else labels.push({ p: () => model.jointByName[c.joint].world.p, text: c.joint, cls: 'or' }); }
  if (state.rview === 'frames' || c.frame) {
    const fr = (n) => triad(model.pose[n].p, model.pose[n].R, 0.06, lines, labels, n);
    if (state.rview === 'frames') { fr('base_link'); fr('lidar_link'); for (const k of LEGK) triad(model.pose[DATA.robot.legs[k].foot_link].p, model.pose[DATA.robot.legs[k].foot_link].R, 0.025, lines, [], null); }
    else if (c.frame === 'tips') LEGK.forEach((k) => { const t = model.tip(k); lines.push([V.add(t, [-0.012, 0, 0]), V.add(t, [0.012, 0, 0]), [1, 0.57, 0.25], 1, true], [V.add(t, [0, -0.012, 0]), V.add(t, [0, 0.012, 0]), [1, 0.57, 0.25], 1, true]); labels.push({ p: t, text: k + ' tip', cls: 'or' }); });
    else fr(c.frame === 'dummy_link' ? 'base_link' : c.frame);
  }
  viewer.lines = lines; viewer.setLabels($('.labels', stageEl), labels); viewer.autoRotate = false;
}
R.robot = (el) => {
  const b = (id, t) => '<button class="btn" data-sel="' + id + '" aria-pressed="' + (state.sel === id) + '">' + t + '</button>';
  el.innerHTML = head(2, 'Explore the Robot', 'Select a part in the list or directly on the model. Names, axes and limits are read from the robot description file.') +
    '<div class="grid2"><div style="display:flex;flex-direction:column;gap:12px"><div class="row" id="rviews">' + [['whole', 'Whole robot'], ['leg', 'One leg'], ['axes', 'Joint axes'], ['frames', 'Frames']].map(([k, t]) => '<button class="btn" data-rv="' + k + '" aria-pressed="' + (state.rview === k) + '">' + t + '</button>').join('') + '<button class="btn" id="rReset">Reset pose</button></div><div class="slot"></div>' +
    '<div class="panel sel"><div class="grp"><span>Body</span>' + b('body', 'Body') + b('lidar', 'Lidar sensor') + '</div>' + LEGK.map((k) => '<div class="grp"><span>' + k.toUpperCase() + '</span>' + b('leg_' + k, 'Leg') + DATA.robot.legs[k].joints.map((j) => b('j_' + j, j.replace('_joint', '').slice(3))).join('') + '</div>').join('') + '<div class="grp"><span>Frames</span>' + b('f_dummy', 'dummy_link') + b('f_base', 'base_link') + b('f_lidar', 'lidar_link') + b('f_tip', 'foot tips') + '</div></div></div>' +
    '<div class="panel" id="insp" aria-live="polite"></div></div>';
  mount($('.slot', el));
  const insp = () => {
    const c = COMPONENTS[state.sel]; let extra = '';
    if (c.joint) { const j = model.jointByName[c.joint]; extra = '<div class="slider"><label for="jq">Angle of ' + c.joint + '</label><input id="jq" type="range" min="' + j.lower + '" max="' + j.upper + '" step="any" value="' + model.q[c.joint] + '"><output id="jqo">' + fmt(model.q[c.joint]) + ' rad</output></div><p style="font-size:12.5px;color:var(--ink3)">The slider moves the drawing only, using the joint origin and axis from the URDF. Nothing is sent anywhere.</p>'; }
    $('#insp').innerHTML = '<div class="row" style="justify-content:space-between"><span class="tag cy">' + esc(c.kind) + '</span>' + chip(c.l, c.src, c.title + ': ' + c.what) + '</div><h3>' + esc(c.title) + '</h3><dl class="kv"><dt>What it is</dt><dd>' + terms(c.what) + '</dd><dt>What it does</dt><dd>' + terms(c.does) + '</dd><dt>Why it exists</dt><dd>' + terms(c.why) + '</dd><dt>Connections</dt><dd>' + terms(c.rel) + '</dd><dt>Defined in</dt><dd>' + c.src.map((s) => '<code>' + esc(s) + '</code>').join('<br>') + '</dd></dl>' + extra;
    const s = $('#jq'); if (s) s.oninput = () => { model.q[c.joint] = +s.value; model.update(); $('#jqo').textContent = fmt(+s.value) + ' rad'; viewer.dirty = true; };
    el.querySelectorAll('[data-sel]').forEach((x) => x.setAttribute('aria-pressed', x.dataset.sel === state.sel));
    el.querySelectorAll('[data-rv]').forEach((x) => x.setAttribute('aria-pressed', x.dataset.rv === state.rview));
    applyRobot();
  };
  el.onclick = (e) => { const s = e.target.closest('[data-sel]'), v = e.target.closest('[data-rv]'); if (s) { state.sel = s.dataset.sel; insp(); } if (v) { state.rview = v.dataset.rv; if (state.rview === 'leg' && !COMPONENTS[state.sel].leg) state.sel = 'leg_lf'; insp(); } if (e.target.id === 'rReset') { resetPose(); insp(); } };
  if (viewer.ok) { viewer.onFrame = null; viewer.onPick = (name) => { if (!name || state.view !== 'robot') return; const d = linkDriver[name]; state.sel = name === 'lidar_link' ? 'lidar' : d ? (state.sel === 'leg_' + linkLeg[name] ? 'j_' + d[d.length - 1] : 'leg_' + linkLeg[name]) : 'body'; insp(); }; }
  hud([['REAL MESHES', 'cy'], ['DRAWING ONLY · not a simulation']]); insp();
};
R.flow = (el) => {
  el.innerHTML = head(3, 'How a Command Becomes Motion', 'The real chain, read from the launch and configuration files. Dashed stages are planned and do not exist yet. Select a stage or step through.') +
    '<div class="row"><span class="tag">CONCEPTUAL ANIMATION · not live ROS data</span><button class="btn" id="fPrev">Previous</button><button class="btn pri" id="fPlay">Play</button><button class="btn" id="fNext">Next</button><button class="btn" id="fRe">Restart</button></div>' +
    '<div class="flow" id="flowRow">' + FLOW.map((s, i) => (i ? '<span class="farrow" data-a="' + i + '"><span class="pulse"></span></span>' : '') + '<button class="fstage' + (s.planned ? ' planned' : '') + '" data-f="' + i + '"><b>' + esc(s.t) + '</b><small>' + esc(s.sub) + '</small><span><span class="chip ' + s.l + '">' + LABELS[s.l].short + '</span></span></button>').join('') + '</div><div class="panel" id="fDet" aria-live="polite"></div>';
  const show = () => {
    const s = FLOW[state.flow];
    el.querySelectorAll('.fstage').forEach((b, i) => { b.classList.toggle('on', i === state.flow); b.classList.toggle('past', i < state.flow && !FLOW[i].planned); });
    el.querySelectorAll('.farrow').forEach((a) => a.classList.toggle('live', +a.dataset.a === state.flow && state.motion));
    $('#fDet').innerHTML = '<div class="row" style="justify-content:space-between"><h3>' + (state.flow + 1) + ' of ' + FLOW.length + ' · ' + esc(s.t) + '</h3>' + chip(s.l, s.where, s.t + ': ' + s.plain) + '</div><p>' + terms(s.plain) + '</p><dl class="kv"><dt>Input</dt><dd>' + esc(s.io[0].replace(/^In: /, '')) + '</dd><dt>Output</dt><dd>' + esc(s.io[1].replace(/^Out: /, '')) + '</dd>' + (s.topic ? '<dt>Interface</dt><dd><code>' + esc(s.topic) + '</code></dd>' : '') + '<dt>Where</dt><dd>' + s.where.map((w) => '<code>' + esc(w) + '</code>').join('<br>') + '</dd><dt>Status</dt><dd>' + (s.planned ? 'Planned. Nothing in the repository implements this stage as a running node.' : LABELS[s.l].name + '. ' + LABELS[s.l].help) + '</dd></dl>';
    $('#fPlay').textContent = state.flowPlay ? 'Pause' : 'Play';
  };
  const step = (d) => { state.flow = (state.flow + d + FLOW.length) % FLOW.length; show(); };
  clearInterval(R.flow.t); R.flow.t = setInterval(() => { if (state.flowPlay && state.view === 'flow') { if (state.flow === FLOW.length - 1) { state.flowPlay = false; show(); } else step(1); } }, 3800);
  el.onclick = (e) => { const f = e.target.closest('[data-f]'); if (f) { state.flow = +f.dataset.f; state.flowPlay = false; show(); } const id = e.target.id; if (id === 'fPrev') step(-1); if (id === 'fNext') step(1); if (id === 'fRe') { state.flow = 0; show(); } if (id === 'fPlay') { state.flowPlay = !state.flowPlay; show(); } };
  show();
};
function svgGraph(items, edges, pos, w, hgt, selId) {
  const box = (it) => { const p = pos[it.id], tw = Math.max(90, it.t.length * (it.kind === 'topic' ? 7.4 : 7.6) + 22); return { x: p[0] - tw / 2, y: p[1] - 17, w: tw, h: 34, cx: p[0], cy: p[1] }; };
  const B = {}; items.forEach((it) => { B[it.id] = box(it); });
  const clip = (a, b) => { const dx = b.cx - a.cx, dy = b.cy - a.cy, t = Math.min(Math.abs((a.w / 2 + 4) / (dx || 1e-9)), Math.abs((a.h / 2 + 4) / (dy || 1e-9))); return [a.cx + dx * t, a.cy + dy * t]; };
  return '<svg viewBox="0 0 ' + w + ' ' + hgt + '" width="' + w + '" role="group" aria-label="Diagram"><defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0 0L10 5L0 10z" fill="#2a444d"/></marker></defs>' +
    edges.map(([a, b]) => { const s = clip(B[a], B[b]), e = clip(B[b], B[a]); return '<line class="gedge" x1="' + s[0].toFixed(1) + '" y1="' + s[1].toFixed(1) + '" x2="' + e[0].toFixed(1) + '" y2="' + e[1].toFixed(1) + '" marker-end="url(#ah)"/>'; }).join('') +
    items.map((it) => { const b = B[it.id]; return '<g class="gnode ' + (it.kind || '') + ' ' + (it.st || '') + (it.l === 'P' ? ' planned' : '') + (selId === it.id ? ' on' : '') + '" data-n="' + it.id + '" tabindex="0" role="button" aria-label="' + esc(it.t) + '"><rect x="' + b.x.toFixed(1) + '" y="' + b.y + '" width="' + b.w.toFixed(1) + '" height="34" rx="' + (it.kind === 'topic' ? 3 : 14) + '"/><text x="' + b.cx + '" y="' + (b.cy + 4.5) + '" text-anchor="middle">' + esc(it.t) + '</text></g>'; }).join('') + '</svg>';
}
R.map = (el) => {
  const tabs = [['packages', 'Packages'], ['nodes', 'Nodes and topics'], ['tf', 'TF tree'], ['sim', 'Simulator bridge']];
  el.innerHTML = head(4, 'ROS 2 System Map', 'Four separate layers. A node graph shows programs and messages. A TF tree shows coordinate frames. They answer different questions, so they are kept apart.') +
    '<div class="tabs">' + tabs.map(([k, t]) => '<button class="btn" data-mt="' + k + '" aria-pressed="' + (state.map === k) + '">' + t + '</button>').join('') + '</div><div id="mBody"></div>';
  let sel = null;
  const draw = () => {
    el.querySelectorAll('[data-mt]').forEach((x) => x.setAttribute('aria-pressed', x.dataset.mt === state.map));
    const body = $('#mBody'); const M = SYSMAP[state.map]; let html = '<p style="color:var(--ink2);max-width:75ch;margin-bottom:12px">' + esc(M.intro) + '</p>';
    const detail = (it) => it ? '<div class="panel"><div class="row" style="justify-content:space-between"><h3 class="mono">' + esc(it.t || it.id) + '</h3>' + chip(it.l, it.src, (it.t || it.id) + ': ' + it.role) + '</div><p>' + terms(it.role) + '</p><p>' + it.src.map((s) => '<code>' + esc(s) + '</code>').join('<br>') + '</p></div>' : '<div class="panel"><p>Select a box to read its role and its source.</p></div>';
    if (state.map === 'packages') {
      const items = M.items.map((p) => ({ ...p, t: p.id })), pos = {}; items.forEach((p) => { pos[p.id] = [150 + p.x * 290, 50 + p.y * 100]; });
      html += '<div class="scroll">' + svgGraph(items, M.edges, pos, 880, 300, sel) + '</div><div class="legend" style="margin:8px 0 14px"><span>Solid: used by the simulation today</span><span>Dotted grey: templates only</span><span>Dashed amber: configured but blocked</span></div>' + detail(items.find((i) => i.id === sel));
    } else if (state.map === 'nodes') {
      const pos = { gz: [110, 250], bridge: [330, 90], cm: [330, 250], gtb: [330, 430], t_clock: [560, 40], t_scan: [560, 100], jsb: [540, 200], jtc: [540, 300], t_gt: [600, 430], t_js: [760, 200], rsp: [960, 200], t_tf: [960, 110], rviz: [960, 40], t_act: [800, 360], client: [1000, 430], t_cmdvel: [110, 470] };
      const edges = [['gz', 'bridge'], ['bridge', 't_clock'], ['bridge', 't_scan'], ['gz', 'gtb'], ['gtb', 't_gt'], ['gz', 'cm'], ['cm', 'jsb'], ['cm', 'jtc'], ['jsb', 't_js'], ['t_js', 'rsp'], ['rsp', 't_tf'], ['t_tf', 'rviz'], ['client', 't_act'], ['t_act', 'jtc'], ['t_gt', 'client'], ['t_js', 'client']];
      const items = M.items.map((i) => i.id === 't_act' ? { ...i, t: 'follow_joint_trajectory (action)' } : i);
      html += '<div class="row" style="margin-bottom:8px"><span class="tag">CONFIGURATION DRAWING · not a live graph capture</span></div><div class="scroll">' + svgGraph(items, edges, pos, 1120, 500, sel) + '</div><div class="legend" style="margin:8px 0 14px"><span>Rounded: node</span><span>Square, cyan text: topic or action</span><span>Dashed grey: reserved name with no subscriber</span><span>Both controllers run inside the Gazebo process, so their joint commands reach the simulator without a ROS topic.</span></div>' + detail(M.items.find((i) => i.id === sel));
    } else if (state.map === 'tf') {
      const kids = (l) => model.childJoints[l] || [];
      const node = (l, open) => { const k = kids(l); const j = model.jointOfChild[l]; const lab = '<button data-tfn="' + l + '">' + (j ? '<span class="' + (j.type === 'revolute' ? 'rev' : 'fx') + '">' + esc(j.name) + (j.type === 'revolute' ? ' ⟲' : '') + '</span> → ' : '') + '<b>' + esc(l) + '</b></button>'; return k.length ? '<details' + (open ? ' open' : '') + '><summary>' + lab + '</summary>' + k.map((c) => node(c.child, open && (linkLeg[c.child] === 'lf' || !linkLeg[c.child]) && !/b_s[123]_1/.test(c.child))).join('') + '</details>' : '<div class="leaf">' + lab + '</div>'; };
      const t = sel && model.jointOfChild[sel];
      html += '<div class="grid2 even"><div class="panel"><h2>Frames from the URDF</h2><div class="tree" style="margin-left:-16px">' + node('dummy_link', true) + '</div><p style="font-size:12.5px;color:var(--ink3)">The front left leg is open. Open the others to compare. ⟲ marks a revolute joint.</p></div><div style="display:flex;flex-direction:column;gap:16px">' +
        (t ? '<div class="panel"><div class="row" style="justify-content:space-between"><h3 class="mono">' + esc(sel) + '</h3>' + chip('C', [DSC + 'urdf/spiderx.urdf.xacro:' + t.line], 'Frame ' + sel + ' is the child of ' + t.parent + ' through ' + t.name) + '</div><dl class="kv"><dt>Parent</dt><dd class="mono">' + esc(t.parent) + '</dd><dt>Joint</dt><dd class="mono">' + esc(t.name) + ' (' + t.type + ')</dd><dt>Offset</dt><dd class="mono">' + t.xyz.join(', ') + ' m</dd>' + (t.axis ? '<dt>Axis</dt><dd class="mono">[' + t.axis.join(', ') + ']</dd><dt>Limits</dt><dd class="mono">' + t.lower + ' to ' + t.upper + ' rad</dd>' : '') + '</dl></div>' : '<div class="panel"><p>Select a frame in the tree to read its parent, joint and offset.</p></div>') +
        '<div class="panel"><h2>What is missing above the root</h2><ul class="clean"><li><span><code>odom → dummy_link</code> would come from odometry. It does not exist.</span>' + chip('P', [D + 'SPIDERX_SYSTEM_ARCHITECTURE.md:51']) + '</li><li><span><code>map → odom</code> would come from SLAM or AMCL. It does not exist.</span>' + chip('P', [D + 'SPIDERX_SYSTEM_ARCHITECTURE.md:51']) + '</li><li><span>The fixed base test adds <code>world → dummy_link</code> at z = 0.125 m. That launch must never run with an odometry stack.</span>' + chip('C', [DSC + 'urdf/spiderx_fixed_base.urdf.xacro:34']) + '</li><li><span>Foot tips are derived points and are not TF frames.</span>' + chip('C', [CTL + 'spiderx_controller/leg_kinematics.py:17']) + '</li></ul></div></div></div>';
    } else {
      html += '<div class="panel"><div class="scroll"><table><thead><tr><th>Inside Gazebo</th><th>Carried across by</th><th>On the ROS 2 side</th><th>Label</th></tr></thead><tbody>' + M.rows.map((r) => '<tr><td>' + esc(r.gz) + '</td><td>' + esc(r.via) + '</td><td class="mono">' + esc(r.ros) + '</td><td>' + chip(r.l, r.src, r.gz + ' reaches ROS as ' + r.ros) + '</td></tr>').join('') + '</tbody></table></div><p>Every bridge is one way, from Gazebo to ROS. Commands enter the simulator only through the gz_ros2_control plugin.</p></div>';
    }
    body.innerHTML = html;
  };
  el.onclick = (e) => { const t = e.target.closest('[data-mt]'), n = e.target.closest('[data-n]'), f = e.target.closest('[data-tfn]'); if (t) { state.map = t.dataset.mt; sel = null; draw(); } if (n) { sel = n.dataset.n; draw(); } if (f) { sel = f.dataset.tfn; e.preventDefault(); draw(); } };
  el.onkeydown = (e) => { const n = e.target.closest && e.target.closest('[data-n]'); if (n && (e.key === 'Enter' || e.key === ' ')) { e.preventDefault(); sel = n.dataset.n; draw(); } };
  draw();
};
/* ---------- lab ---------- */
const PB = DATA.playback, pbv = (arr, i, j) => arr[i][j] * PB.scale;
function chartHtml() {
  const W = 720, H = 270, m = { l: 56, r: 16, t: 26, b: 46 }, ji = PB.joints.indexOf(state.rj), n = PB.t.length;
  let lo = 1e9, hi = -1e9; for (let i = 0; i < n; i++) { for (const a of [PB.cmd, PB.obs]) { const v = pbv(a, i, ji); lo = Math.min(lo, v); hi = Math.max(hi, v); } }
  if (hi - lo < 0.004) { hi += 0.002; lo -= 0.002; } const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
  const X = (t) => m.l + (t - 3) / 4 * (W - m.l - m.r), Y = (v) => m.t + (hi - v) / (hi - lo) * (H - m.t - m.b);
  const path = (a) => PB.t.map((t, i) => (i ? 'L' : 'M') + X(t).toFixed(1) + ' ' + Y(pbv(a, i, ji)).toFixed(1)).join('');
  let g = ''; for (let k = 0; k <= 4; k++) { const v = lo + (hi - lo) * k / 4; g += '<line x1="' + m.l + '" x2="' + (W - m.r) + '" y1="' + Y(v) + '" y2="' + Y(v) + '" stroke="#1d2e35"/><text x="' + (m.l - 8) + '" y="' + (Y(v) + 4) + '" text-anchor="end" font-size="12" fill="#7d989d" font-family="IBM Plex Mono,monospace">' + v.toFixed(3) + '</text>'; }
  for (let t = 3; t <= 7; t += 0.5) g += '<text x="' + X(t) + '" y="' + (H - 26) + '" text-anchor="middle" font-size="12" fill="#7d989d" font-family="IBM Plex Mono,monospace">' + t.toFixed(1) + '</text>';
  return { W, H, m, X, Y, html: '<svg viewBox="0 0 ' + W + ' ' + H + '" role="img" aria-label="Commanded and observed angle of ' + state.rj + ' over reference time"><rect x="0" y="0" width="' + W + '" height="' + H + '" fill="none"/>' + g + '<text x="' + (W - m.r) + '" y="' + (H - 6) + '" text-anchor="end" font-size="12" fill="#7d989d">reference time (s)</text><text x="4" y="12" font-size="12" fill="#7d989d">angle (rad)</text><path d="' + path(PB.cmd) + '" fill="none" stroke="#ff9140" stroke-width="2" stroke-dasharray="6 4"/><path d="' + path(PB.obs) + '" fill="none" stroke="#35d6e0" stroke-width="2"/><line id="cx" y1="' + m.t + '" y2="' + (H - m.b) + '" stroke="#e6f1f2" stroke-width="1" x1="0" x2="0"/><circle id="cdo" r="4.5" fill="#35d6e0" stroke="#070b0d" stroke-width="2"/><circle id="cdc" r="4.5" fill="#ff9140" stroke="#070b0d" stroke-width="2"/></svg>' };
}
R.lab = (el) => {
  const tabs = [['fk', 'Forward kinematics'], ['ik', 'Inverse kinematics'], ['gait', 'Gait phases'], ['replay', 'Recorded trot cycle']];
  el.innerHTML = head(5, 'Kinematics and Gait Lab', 'Four experiments. Three are calculated in your browser from the URDF values. One replays joint angles that were recorded in a real test run.') +
    '<div class="tabs">' + tabs.map(([k, t]) => '<button class="btn" data-lt="' + k + '">' + t + '</button>').join('') + '</div><div class="grid2"><div class="slot"></div><div class="panel" id="labP"></div></div>';
  mount($('.slot', el));
  const legSel = () => '<div class="row"><span style="color:var(--ink3);font-size:12.5px">Leg</span>' + LEGK.map((k) => '<button class="btn" data-leg="' + k + '" aria-pressed="' + (state.leg === k) + '">' + k.toUpperCase() + '</button>').join('') + '</div>';
  const portNote = '<p style="font-size:12.5px;color:var(--ink3)">Browser port of the project’s formulas. Checked offline against four documented points, the 0.1223 rad crouch figure and both negative targets ' + chip('C', [CTL + 'spiderx_controller/leg_kinematics.py:411', CTL + 'spiderx_controller/leg_kinematics.py:465', 'tools/engineering_console/selftest.js'], 'The browser math reproduces numbers documented by the project', 'This is a re-implementation. It is not the project’s Python code and does not prove that code correct.') + '</p>';
  let last = 0;
  const footMarks = (extra) => { const lines = (extra || []).slice(), labels = []; LEGK.forEach((k) => { const t = model.tip(k); lines.push([V.add(t, [-0.01, 0, 0]), V.add(t, [0.01, 0, 0]), [1, 0.57, 0.25], 1, true], [V.add(t, [0, -0.01, 0]), V.add(t, [0, 0.01, 0]), [1, 0.57, 0.25], 1, true]); }); viewer.lines = lines; return labels; };
  const draw = () => {
    el.querySelectorAll('[data-lt]').forEach((x) => x.setAttribute('aria-pressed', x.dataset.lt === state.lab));
    const P = $('#labP'), g = GEO[state.leg]; state.playing = false; resetPose();
    if (viewer.ok) { viewer.hidden.clear(); viewer.highlight = null; viewer.colors = {}; viewer.autoRotate = false; viewer.onFrame = null; viewer.setLabels($('.labels', stageEl), []); viewer.lines = []; }
    if (state.lab === 'fk') {
      hud([['COMPUTED IN BROWSER from URDF values', 'cy'], ['DRAWING ONLY · not a simulation']]);
      P.innerHTML = '<h2>Forward kinematics</h2><p>' + terms('Set three joint angles and read where the derived foot tip ends up in base_link. One set of angles always gives one foot position.') + '</p>' + legSel() + g.names.map((n, i) => '<div class="slider"><label for="fk' + i + '">' + n + '</label><input id="fk' + i + '" type="range" min="' + g.limits[i][0] + '" max="' + g.limits[i][1] + '" step="any" value="0"><output id="fko' + i + '">+0.000 rad</output></div>').join('') + '<div class="read" id="fkR"></div><div id="fkM"></div>' + portNote;
      const upd = () => { const q = [0, 1, 2].map((i) => +$('#fk' + i).value); model.setAll(g.names, q); const t = model.tip(state.leg); q.forEach((x, i) => { $('#fko' + i).textContent = fmt(x) + ' rad'; }); $('#fkR').innerHTML = ['x right', 'y front', 'z up'].map((a, i) => '<div><span>TIP ' + a.toUpperCase() + ' (mm)</span><b>' + fmt(t[i] * 1000, 1) + '</b></div>').join(''); const d = V.sub(t, g.tip0); $('#fkM').innerHTML = '<div class="msg ok">Moved ' + (V.norm(d) * 1000).toFixed(1) + ' mm from the neutral tip. ' + (q.some((x, i) => x < g.limits[i][0] + 0.05 || x > g.limits[i][1] - 0.05) ? 'At least one joint is inside the 0.05 rad safety margin, so the project’s tools would refuse this pose.' : 'All joints are within the limits minus the 0.05 rad margin.') + '</div>'; if (viewer.ok) { viewer.highlight = new Set(COMPONENTS['leg_' + state.leg].links); footMarks([[g.tip0, t, [1, 0.57, 0.25], 0.9, true]]); viewer.dirty = true; } };
      P.oninput = upd; upd();
    } else if (state.lab === 'ik') {
      hud([['COMPUTED IN BROWSER from URDF values', 'cy'], ['DRAWING ONLY · not a simulation']]);
      const ax = [['dx', 'right (+x)', -30, 30], ['dy', 'forward (+y)', -40, 40], ['dz', 'up (+z)', -40, 60]];
      P.innerHTML = '<h2>Inverse kinematics</h2><p>' + terms('Move the wanted foot position away from its neutral point. The solver returns the three joint angles, or refuses and says why. It never clamps.') + '</p>' + legSel() + ax.map((a, i) => '<div class="slider"><label for="ik' + i + '">Target ' + a[1] + '</label><input id="ik' + i + '" type="range" min="' + a[2] + '" max="' + a[3] + '" step="0.5" value="' + state.ik[i] + '"><output id="iko' + i + '"></output></div>').join('') + '<div class="row"><button class="btn" data-pose="0,0,0">Neutral</button><button class="btn" data-pose="0,0,10">Raise foot 10 mm</button><button class="btn" data-pose="0,0,15">Raise foot 15 mm</button><button class="btn" data-pose="0,0,-40">Unreachable example</button></div><div class="read" id="ikR"></div><div id="ikM"></div><p style="font-size:12.5px;color:var(--ink3)">The project’s static poses keep every offset within 20 mm ' + chip('C', [CTL + 'config/m4_pose_targets.yaml:31'], 'M4 safety bound: at most 0.02 m offset per axis') + '</p>' + portNote;
      const upd = () => { state.ik = [0, 1, 2].map((i) => +$('#ik' + i).value); state.ik.forEach((x, i) => { $('#iko' + i).textContent = fmt(x, 1) + ' mm'; }); const target = V.add(g.tip0, V.scale(state.ik, 0.001)); const r = ikLeg(g, target); const tl = [[V.add(target, [-0.014, 0, 0]), V.add(target, [0.014, 0, 0]), [0.2, 0.84, 0.88], 1, true], [V.add(target, [0, -0.014, 0]), V.add(target, [0, 0.014, 0]), [0.2, 0.84, 0.88], 1, true], [V.add(target, [0, 0, -0.014]), V.add(target, [0, 0, 0.014]), [0.2, 0.84, 0.88], 1, true]];
        if (r.ok) { model.setAll(g.names, r.solution); $('#ikR').innerHTML = g.names.map((n, i) => '<div><span>' + n.replace('_joint', '').toUpperCase() + ' (rad)</span><b>' + fmt(r.solution[i]) + '</b></div>').join(''); $('#ikM').innerHTML = '<div class="msg ok">Solution found. ' + r.solution.filter((x) => Math.abs(x) > 1e-4).length + ' of 3 joints changed. The forward check lands ' + r.residual.toExponential(1) + ' m from the target.</div>'; }
        else { $('#ikR').innerHTML = g.names.map((n) => '<div><span>' + n.replace('_joint', '').toUpperCase() + '</span><b style="color:var(--or)">refused</b></div>').join(''); $('#ikM').innerHTML = '<div class="msg"><b>Refused: ' + esc(r.reason) + '.</b> ' + esc(r.message || '') + ' The leg in the drawing stays where it was. No goal would be sent.</div>'; }
        if (viewer.ok) { viewer.highlight = new Set(COMPONENTS['leg_' + state.leg].links); viewer.lines = tl; viewer.setLabels($('.labels', stageEl), [{ p: target, text: 'target', cls: 'cy' }]); viewer.dirty = true; } };
      P.oninput = upd; P.onclick = (e) => { const p = e.target.dataset.pose; if (p) { p.split(',').forEach((v, i) => { $('#ik' + i).value = v; }); upd(); } }; upd();
    } else if (state.lab === 'gait') {
      hud([['CONCEPTUAL DEMONSTRATION', ''], ['Legs only · body does not travel · not a simulation · not walking']]);
      const G = GAITS[state.gait], T = GAIT_PARAMS.step_length_m / (G.duty_factor * GAIT_PARAMS.body_speed_m_s);
      P.innerHTML = '<h2>Gait phases</h2><p>' + terms('Each row is one leg over one gait cycle. Orange stripes are swing, teal is stance. The pattern comes from the duty factor and the phase offsets in the project’s gait file.') + '</p><div class="row">' + Object.keys(GAITS).map((k) => '<button class="btn" data-gait="' + k + '" aria-pressed="' + (k === state.gait) + '">' + k + '</button>').join('') + '</div><p><b>' + state.gait + '.</b> ' + esc(G.desc) + ' Duty factor ' + G.duty_factor + ', cycle ' + T.toFixed(2) + ' s at 5 mm/s, at least ' + G.minFeet + ' feet in stance. ' + chip('C', [CTL + 'config/m4_5_gaits.yaml:61', CTL + 'spiderx_controller/gait_phase.py:38'], 'Gait ' + state.gait + ': duty factor ' + G.duty_factor + ' and its phase offsets') + '</p>' +
        '<div class="gt" id="gtl">' + LEGK.map((k) => { const sw = 1 - G.duty_factor, o = G.phase_offsets[LEG_FULL[k]]; const seg = o + sw <= 1 ? [[o, sw]] : [[o, 1 - o], [0, o + sw - 1]]; return '<label>' + k.toUpperCase() + ' ' + LEG_FULL[k].replace('_', ' ') + '</label><div class="lane"><span class="st" style="left:0;right:0"></span>' + seg.map((s) => '<span class="sw" style="left:' + s[0] * 100 + '%;width:' + s[1] * 100 + '%"></span>').join('') + '<span class="ph"></span></div>'; }).join('') + '</div><div class="legend"><span><span class="key" style="background:var(--or)"></span>swing, foot in the air</span><span><span class="key" style="background:var(--cy2)"></span>stance, foot on the ground</span></div>' +
        '<div class="row"><button class="btn pri" id="gPlay">Play</button><input id="gU" type="range" min="0" max="1" step="0.002" value="' + state.u + '" aria-label="Position in the gait cycle" style="flex:1;min-width:120px"><output id="gUo" class="mono" style="min-width:132px;text-align:right"></output></div>' +
        '<div class="msg' + (G.verdict === 'PASS' ? ' ok' : '') + '"><b>Offline verdict reported by the project: ' + G.verdict + '.</b> ' + (G.failed ? 'Failed checks: ' + G.failed + '. ' : 'Its kinematic checks passed. ') + 'That verdict is about the model, not about walking. ' + chip('R', [D + 'M4_5_TEST_RESULTS.md:118'], 'M4.5 offline verdict for ' + state.gait + ': ' + G.verdict) + '</div>' +
        '<p style="font-size:12.5px;color:var(--ink3)">The leg motion uses the project’s stance and cycloid swing formulas with step 40 mm and lift 15 mm, solved by the browser port of the inverse kinematics. The cycle is shown much faster than its real ' + T.toFixed(0) + ' s. The project has never played any of these six gaits in the simulator.</p>';
      const pose = () => { const up = []; let down = 0; LEGK.forEach((k) => { const f = footOffset(G, GAIT_PARAMS, k, state.u), r = ikLeg(GEO[k], V.add(GEO[k].tip0, f.off)); if (r.ok) model.setAll(GEO[k].names, r.solution); if (f.ph.phase === 'stance') down++; if (viewer.ok) for (const l of COMPONENTS['leg_' + k].links) if (/foot/.test(l)) viewer.colors[l] = f.ph.phase === 'swing' ? [1, 0.57, 0.25] : [0.1, 0.62, 0.66]; });
        el.querySelectorAll('#gtl .ph').forEach((p) => { p.style.left = 'calc(' + state.u * 100 + '% - 1px)'; }); $('#gUo').textContent = 'u = ' + state.u.toFixed(2) + ' · ' + down + ' feet down'; $('#gU').value = state.u; if (viewer.ok) viewer.dirty = true; };
      if (viewer.ok) { viewer.flyTo({ yaw: -0.25, pitch: 0.2, dist: 0.7, target: [0.051, -0.06, 0.03] }); viewer.onFrame = (t) => { const dt = Math.min(0.05, (t - last) / 1000); last = t; if (state.playing && state.view === 'lab') { state.u = (state.u + dt / 5) % 1; pose(); return true; } return false; }; }
      P.onclick = (e) => { if (e.target.dataset.gait) { state.gait = e.target.dataset.gait; draw(); } if (e.target.id === 'gPlay') { state.playing = !state.playing; e.target.textContent = state.playing ? 'Pause' : 'Play'; } };
      P.oninput = (e) => { if (e.target.id === 'gU') { state.u = +e.target.value; state.playing = false; $('#gPlay').textContent = 'Play'; pose(); } }; pose();
    } else {
      hud([['RECORDED DATA · run 20261012T103615Z, cloud, welded body', 'cy'], ['PLAYBACK OF A PAST TEST · NOT LIVE']]);
      const n = PB.t.length;
      P.innerHTML = '<h2>Recorded trot cycle (M6.1)</h2><p>The observed joint angles of the one passed fixed base trot cycle, replayed on the model. The body was welded 0.125 m above the ground, so the feet move in the air. ' + chip('T', [PB.source, RUN3 + 'requirements.json', RUN3 + 'SHA256SUMS'], 'Commanded and observed joint angles of the passed cycle', 'The console reads ' + PB.rows_total + ' recorded rows and shows every second one. Maximum error recomputed from the file: ' + PB.max_abs_err_rad.toFixed(5) + ' rad on ' + PB.max_err_joint + ' at reference time ' + PB.max_err_ref_time_s.toFixed(2) + ' s. The run was made on source commit b0d8e75; src/ at this snapshot is identical to it.') + '</p>' +
        '<div class="row"><label for="rjs" style="font-size:12.5px;color:var(--ink3)">Joint</label><select id="rjs" class="btn">' + PB.joints.map((j) => '<option' + (j === state.rj ? ' selected' : '') + '>' + j + '</option>').join('') + '</select><span class="legend"><span><span class="key" style="background:var(--cy)"></span>observed</span><span><span class="key" style="background:repeating-linear-gradient(90deg,var(--or) 0 5px,transparent 5px 8px)"></span>commanded</span></span></div><div class="chart" id="rch"></div>' +
        '<div class="row"><button class="btn pri" id="rPlay">Play</button><input id="rT" type="range" min="0" max="' + (n - 1) + '" step="1" value="' + state.rp + '" aria-label="Position in the recording" style="flex:1;min-width:120px"><button class="btn" id="rMax">Go to largest error</button></div><div class="read" id="rR"></div>' +
        '<details><summary style="cursor:pointer;color:var(--cy)">Largest error per joint (table)</summary><div class="scroll"><table><thead><tr><th>Joint</th><th style="text-align:right">Max |error| (rad)</th></tr></thead><tbody>' + PB.joints.map((j) => '<tr><td class="mono">' + j + '</td><td class="num">' + PB.per_joint_max_err[j].toFixed(5) + '</td></tr>').join('') + '</tbody></table></div></details>' +
        '<div class="msg"><b>What this does not show:</b> walking, contact, balance or anything about hardware. The limit for this check was 0.05 rad.</div>';
      let C;
      const chart = () => { C = chartHtml(); $('#rch').innerHTML = C.html + '<div class="tip" hidden></div>'; const svg = $('#rch svg'); const mvh = (e) => { const r = svg.getBoundingClientRect(), x = (e.clientX - r.left) / r.width * C.W, t = 3 + (x - C.m.l) / (C.W - C.m.l - C.m.r) * 4; state.rp = Math.max(0, Math.min(n - 1, Math.round((t - 3) / 4 * (n - 1)))); state.playing = false; $('#rPlay').textContent = 'Play'; pose(e, r); }; svg.addEventListener('pointermove', mvh); svg.addEventListener('pointerdown', mvh); svg.addEventListener('pointerleave', () => { $('#rch .tip').hidden = true; }); };
      const pose = (e, r) => { const i = Math.round(state.rp), ji = PB.joints.indexOf(state.rj); model.setAll(PB.joints, PB.obs[i].map((v) => v * PB.scale)); const c = pbv(PB.cmd, i, ji), o = pbv(PB.obs, i, ji); let mx = 0; for (let j = 0; j < 12; j++) mx = Math.max(mx, Math.abs(pbv(PB.cmd, i, j) - pbv(PB.obs, i, j)));
        $('#rR').innerHTML = '<div><span>REFERENCE TIME (s)</span><b>' + PB.t[i].toFixed(2) + '</b></div><div><span>' + state.rj.replace('_joint', '').toUpperCase() + ' ERROR (rad)</span><b>' + fmt(o - c, 4) + '</b></div><div><span>WORST JOINT NOW (rad)</span><b>' + mx.toFixed(4) + '</b></div>';
        $('#cx').setAttribute('x1', C.X(PB.t[i])); $('#cx').setAttribute('x2', C.X(PB.t[i])); $('#cdo').setAttribute('cx', C.X(PB.t[i])); $('#cdo').setAttribute('cy', C.Y(o)); $('#cdc').setAttribute('cx', C.X(PB.t[i])); $('#cdc').setAttribute('cy', C.Y(c)); $('#rT').value = i;
        const tip = $('#rch .tip'); if (e && r) { tip.hidden = false; tip.innerHTML = 't = ' + PB.t[i].toFixed(2) + ' s<br>observed ' + fmt(o, 4) + '<br>commanded ' + fmt(c, 4); tip.style.left = Math.min(r.width - 170, Math.max(0, e.clientX - r.left + 12)) + 'px'; tip.style.top = '8px'; }
        if (viewer.ok) viewer.dirty = true; };
      if (viewer.ok) { viewer.flyTo({ yaw: -0.25, pitch: 0.2, dist: 0.7, target: [0.051, -0.06, 0.03] }); viewer.showGrid = true; viewer.onFrame = (t) => { const dt = Math.min(0.05, (t - last) / 1000); last = t; if (state.playing && state.view === 'lab') { state.rp += dt / 4 * (n - 1); if (state.rp >= n - 1) { state.rp = n - 1; state.playing = false; $('#rPlay').textContent = 'Replay'; } pose(); return true; } return false; }; }
      P.onclick = (e) => { if (e.target.id === 'rPlay') { if (state.rp >= n - 1) state.rp = 0; state.playing = !state.playing; e.target.textContent = state.playing ? 'Pause' : 'Play'; } if (e.target.id === 'rMax') { state.playing = false; state.rj = PB.max_err_joint; $('#rjs').value = state.rj; chart(); state.rp = PB.t.findIndex((t) => t >= PB.max_err_ref_time_s - 0.004); pose(); } };
      P.oninput = (e) => { if (e.target.id === 'rT') { state.rp = +e.target.value; state.playing = false; pose(); } }; P.onchange = (e) => { if (e.target.id === 'rjs') { state.rj = e.target.value; chart(); pose(); } };
      chart(); pose();
    }
  };
  el.addEventListener('click', (e) => { const t = e.target.closest('[data-lt]'), l = e.target.closest('[data-leg]'); if (t) { state.lab = t.dataset.lt; draw(); } if (l) { state.leg = l.dataset.leg; state.ik = [0, 0, 0]; draw(); } });
  R.lab.draw = draw; draw();
};
/* ---------- gallery ---------- */
function lightbox(key, title) {
  const im = DATA.images[key], lb = $('#lb'); let z = 1;
  lb.innerHTML = '<div class="bar"><b style="font-family:var(--fd)">' + esc(title) + '</b><span class="mono" style="color:var(--ink3)">' + esc(im.path) + ' · ' + im.w + ' × ' + im.h + ' px</span><span class="row"><button class="btn" data-z="-">Zoom out</button><button class="btn" data-z="1">Fit</button><button class="btn" data-z="+">Zoom in</button><button class="btn pri" data-z="x">Close</button></span></div><div class="pan" tabindex="0"><img alt="' + esc(title) + '" src="' + im.uri + '"></div>';
  const img = $('img', lb), pan = $('.pan', lb); const fit = () => Math.min(pan.clientWidth / im.w, pan.clientHeight / im.h, 1); const set = () => { img.style.width = im.w * z + 'px'; };
  lb.hidden = false; z = fit(); set();
  lb.onclick = (e) => { const k = e.target.dataset.z; if (!k) return; if (k === 'x') lb.hidden = true; else { z = k === '+' ? Math.min(4, z * 1.4) : k === '-' ? Math.max(0.2, z / 1.4) : fit(); set(); } };
  $('[data-z="x"]', lb).focus();
}
R.evidence = (el) => {
  el.innerHTML = head(6, 'Evidence Gallery', 'Only real files from the repository. Each item says what it shows, why it matters and what it does not prove. Select a picture to enlarge it.') +
    '<div class="gal">' + GALLERY.map((g) => { const im = DATA.images[g.img]; return '<article class="card"><button class="thumb" data-img="' + g.img + '" aria-label="Enlarge: ' + esc(g.title) + '"><img loading="lazy" alt="' + esc(g.shows) + '" src="' + im.uri + '"></button><div class="cb"><div class="row" style="justify-content:space-between"><h3>' + esc(g.title) + '</h3>' + chip(g.l, [im.path], g.title + ': ' + g.shows, 'A screenshot is evidence only of what is visible in it.') + '</div><dl><dt>Shows</dt><dd>' + esc(g.shows) + '</dd><dt>Why it matters</dt><dd>' + esc(g.matters) + '</dd><dt>Provenance</dt><dd>' + esc(g.prov) + ' Added in commit <code>' + im.commit + '</code> on ' + im.date + '.</dd><dt>Does not prove</dt><dd class="not">' + esc(g.not) + '</dd></dl></div></article>'; }).join('') + '</div>' +
    '<div class="panel"><h2>Recorded data</h2><p>The commanded and observed joint angles of the passed trot cycle are a committed CSV file, not a picture. Open it as an interactive plot.</p><div class="row"><a class="btn pri" href="#lab" data-go="lab" id="toReplay">Open the recorded trot cycle</a>' + chip('T', [PB.source], 'Recorded tracking data of the passed fixed base cycle') + '</div></div>' +
    '<div class="vh"><h1 style="font-size:20px">Code excerpts</h1></div><div class="gal" style="grid-template-columns:repeat(auto-fill,minmax(320px,1fr))">' + CODE_EXCERPTS.map((c) => '<article class="card"><div class="cb"><div class="row" style="justify-content:space-between"><h3>' + esc(c.title) + '</h3>' + chip(c.l, [c.path + ':' + c.line], c.title) + '</div><pre>' + esc(c.code) + '</pre><dl><dt>Source</dt><dd><code>' + esc(c.path + ':' + c.line) + '</code> at ' + SNAP.short + '</dd><dt>Why it matters</dt><dd>' + esc(c.matters) + '</dd><dt>Does not prove</dt><dd class="not">' + esc(c.not) + '</dd></dl></div></article>').join('') + '</div>' +
    '<div class="vh"><h1 style="font-size:20px">Assets not available</h1><p>These would belong here. They are not in the repository, so nothing is shown in their place.</p></div><div class="gal">' + ASSETS_NEEDED.map((a) => '<article class="card need"><div class="cb"><span class="tag">ASSET NEEDED</span><h3>' + esc(a.t) + '</h3><p>' + esc(a.why) + '</p><code>' + esc(a.src) + '</code></div></article>').join('') + '</div>';
  el.onclick = (e) => { const t = e.target.closest('[data-img]'); if (t) lightbox(t.dataset.img, GALLERY.find((g) => g.img === t.dataset.img).title); if (e.target.id === 'toReplay') state.lab = 'replay'; };
};
R.milestones = (el) => {
  el.innerHTML = head(7, 'Milestone Map', 'Built from the roadmap and the status file. The numbering changed on 2026-10-02: M6 now means gait playback safety, and odometry, SLAM, Nav2 and hardware moved to M7, M8, M9 and M10. Older documents keep the old numbers.') +
    '<div class="row" id="msF">' + Object.entries(CATS).map(([k, t]) => '<button class="btn" data-cat="' + k + '" aria-pressed="false">' + t + '</button>').join('') + '</div><div class="panel"><div class="track" id="msT">' + MILESTONES.map((m) => '<button class="ms ' + m.cat + '" data-ms="' + m.id + '"><i></i><b>' + m.id + '</b><small>' + esc(m.t) + '</small></button>').join('') + '</div><div class="legend"><span><span class="key" style="background:var(--ok);border-radius:50%;width:10px"></span>Completed</span><span><span class="key" style="background:linear-gradient(90deg,var(--or) 50%,transparent 50%);border:1px solid var(--or);border-radius:50%;width:10px"></span>In progress</span><span><span class="key" style="border:1px dashed var(--warn);border-radius:50%;width:10px"></span>Implemented but unverified</span><span><span class="key" style="border:1px dotted var(--gry);border-radius:50%;width:10px"></span>Planned</span></div></div><div class="panel" id="msD" aria-live="polite"></div>';
  const show = () => { const m = MILESTONES.find((x) => x.id === state.ms);
    el.querySelectorAll('.ms').forEach((b) => { b.classList.toggle('on', b.dataset.ms === state.ms); b.classList.toggle('dim', !!state.msFilter && !b.classList.contains(state.msFilter)); });
    el.querySelectorAll('[data-cat]').forEach((b) => b.setAttribute('aria-pressed', b.dataset.cat === state.msFilter));
    $('#msD').innerHTML = '<div class="row" style="justify-content:space-between"><h3>' + m.id + ' · ' + esc(m.t) + '</h3><span class="row"><span class="tag ' + (m.cat === 'done' ? 'cy' : '') + '">' + CATS[m.cat].toUpperCase() + '</span>' + chip(m.l, m.src, m.id + ' ' + m.t + ': ' + m.ev) + '</span></div><dl class="kv"><dt>Objective</dt><dd>' + terms(m.obj) + '</dd><dt>Implementation</dt><dd>' + esc(m.impl) + '</dd><dt>Evidence</dt><dd>' + esc(m.ev) + '</dd><dt>Limits</dt><dd>' + esc(m.lim) + '</dd>' + (m.date ? '<dt>Date</dt><dd>' + m.date + ' (merge or run date in the git history)</dd>' : '') + '</dl>'; };
  el.onclick = (e) => { const m = e.target.closest('[data-ms]'), c = e.target.closest('[data-cat]'); if (m) { state.ms = m.dataset.ms; show(); } if (c) { state.msFilter = state.msFilter === c.dataset.cat ? null : c.dataset.cat; show(); } };
  show();
};
R.learn = (el) => {
  el.innerHTML = head(8, 'Learn SpiderX from Zero', 'Thirteen short lessons, each tied to a real file. Work through them in order. Your progress is kept in this browser only.') + '<div class="lessons"><div class="llist" id="ll"></div><div class="panel" id="lp"></div></div>';
  const show = () => {
    const i = state.lesson, L = LESSONS[i]; store.set('lesson', i);
    $('#ll').innerHTML = LESSONS.map((x, k) => '<button data-l="' + k + '" class="' + (k === i ? 'on ' : '') + (state.done.includes(k) ? 'done' : '') + '"><i>' + (state.done.includes(k) ? '✓' : String(k + 1).padStart(2, '0')) + '</i><span>' + esc(x.t) + '</span></button>').join('');
    const [vk, va] = L.vis.split(':'); let vis = '';
    if (vk === 'loop') vis = '<svg viewBox="0 0 520 110" style="max-width:520px;width:100%" role="img" aria-label="Sense, decide, act loop"><g font-size="14" text-anchor="middle"><rect x="10" y="30" width="140" height="50" rx="6" fill="#131e23" stroke="#12939c"/><text x="80" y="52" fill="#e6f1f2">SENSE</text><text x="80" y="70" fill="#7d989d" font-size="12">lidar, joint angles</text><rect x="190" y="30" width="140" height="50" rx="6" fill="#131e23" stroke="#8a9aa0" stroke-dasharray="4 3"/><text x="260" y="52" fill="#e6f1f2">DECIDE</text><text x="260" y="70" fill="#ff9140" font-size="12">mostly missing</text><rect x="370" y="30" width="140" height="50" rx="6" fill="#131e23" stroke="#12939c"/><text x="440" y="52" fill="#e6f1f2">ACT</text><text x="440" y="70" fill="#7d989d" font-size="12">12 joints</text></g><path d="M152 55h34M332 55h34" stroke="#35d6e0" stroke-width="1.5" marker-end="url(#la)"/><path d="M440 82v16H80V84" stroke="#35d6e0" stroke-width="1.5" fill="none" marker-end="url(#la)"/><defs><marker id="la" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0 0L10 5L0 10z" fill="#35d6e0"/></marker></defs></svg>';
    else if (vk === 'labels') vis = '<div style="display:flex;flex-direction:column;gap:8px">' + Object.keys(LABELS).map((k) => '<div class="row"><span class="chip ' + k + '">' + LABELS[k].name + '</span><span style="color:var(--ink2);font-size:13px;flex:1;min-width:200px">' + esc(LABELS[k].help) + '</span></div>').join('') + '</div>';
    else if (vk === 'img') vis = '<button class="thumb" data-img="' + va + '" style="border:1px solid var(--line2);padding:0;background:#000;max-width:420px"><img alt="" src="' + DATA.images[va].uri + '" style="display:block;width:100%"></button>';
    else if (vk === 'robot' || vk === 'lab') vis = '<div class="slot"></div>';
    else vis = '<a class="btn pri" href="#' + (vk === 'flow' ? 'flow' : 'map') + '">Open ' + (vk === 'flow' ? 'How a Command Becomes Motion' : 'the ROS 2 System Map') + '</a>';
    $('#lp').innerHTML = '<div class="row" style="justify-content:space-between"><span class="tag cy">LESSON ' + (i + 1) + ' OF ' + LESSONS.length + '</span><span class="row"><button class="btn" id="lPrev"' + (i ? '' : ' disabled') + '>Previous</button><button class="btn" id="lNext"' + (i < LESSONS.length - 1 ? '' : ' disabled') + '>Next</button></span></div><h3 style="font-size:22px">' + esc(L.t) + '</h3><dl class="kv"><dt>Objective</dt><dd><b style="color:var(--ink)">' + esc(L.obj) + '</b></dd></dl><p style="max-width:68ch">' + terms(L.body) + '</p>' + vis +
      (vk === 'lab' ? '<a class="btn" href="#lab" id="lLab">Open this experiment in the Lab</a>' : vk === 'robot' ? '<a class="btn" href="#robot">Open Explore the Robot</a>' : '') +
      '<dl class="kv"><dt>Source file</dt><dd><code>' + esc(L.src) + '</code></dd><dt>Try it</dt><dd>' + esc(L.ex) + '</dd></dl><div><h2 style="margin-bottom:8px">Check your understanding</h2><p style="color:var(--ink);margin-bottom:8px">' + esc(L.q) + '</p><div class="opts">' + L.opts.map((o, k) => '<button data-o="' + k + '">' + esc(o) + '</button>').join('') + '</div><div id="lFb" aria-live="polite" style="margin-top:8px"></div></div>';
    const slot = $('#lp .slot');
    if (slot) { mount(slot); resetPose(); if (viewer.ok) { viewer.hidden.clear(); viewer.colors = {}; viewer.onFrame = null; viewer.highlight = null; viewer.lines = []; viewer.autoRotate = false; const labels = [];
      if (va === 'axes') { viewer.lines = axesLines(); } else if (va === 'leg') { viewer.highlight = new Set(COMPONENTS.leg_lf.links); viewer.lines = axesLines((j) => j.name.startsWith('lf')); } else if (va === 'frames') { triad(model.pose.base_link.p, model.pose.base_link.R, 0.06, viewer.lines, labels, 'base_link'); }
      else if (va === 'crouch' || va === 'ik') { LEGK.forEach((k) => { const r = ikLeg(GEO[k], V.add(GEO[k].tip0, [0, 0, 0.01])); if (va === 'crouch' || k === 'lf') model.setAll(GEO[k].names, r.solution); }); }
      else if (va === 'fk') { model.setAll(GEO.lf.names, [0, 0.2, 0]); viewer.highlight = new Set(COMPONENTS.leg_lf.links); }
      else if (va === 'gait' || va === 'replay') { let last = 0, u = 0; const G = GAITS.trot; viewer.onFrame = (t) => { const dt = Math.min(0.05, (t - last) / 1000); last = t; if (!state.motion || state.view !== 'learn') return false; u = (u + dt / 5) % 1; if (va === 'gait') LEGK.forEach((k) => { const r = ikLeg(GEO[k], V.add(GEO[k].tip0, footOffset(G, GAIT_PARAMS, k, u).off)); if (r.ok) model.setAll(GEO[k].names, r.solution); }); else model.setAll(PB.joints, PB.obs[Math.floor(u * (PB.t.length - 1))].map((v) => v * PB.scale)); return true; }; }
      viewer.setLabels($('.labels', stageEl), labels); viewer.flyTo(va === 'leg' || va === 'fk' || va === 'ik' ? { yaw: 2.6, pitch: 0.25, dist: 0.5, target: [0.0, 0.0, 0.02] } : CAMS.Iso); }
      hud(va === 'replay' ? [['RECORDED DATA · past test, welded body', 'cy']] : va === 'gait' ? [['CONCEPTUAL DEMONSTRATION · not walking']] : [['REAL MESHES · drawing only', 'cy']]); }
  };
  el.onclick = (e) => { const l = e.target.closest('[data-l]'), o = e.target.closest('[data-o]'), im = e.target.closest('[data-img]'); const L = LESSONS[state.lesson];
    if (l) { state.lesson = +l.dataset.l; show(); } if (e.target.id === 'lPrev') { state.lesson--; show(); } if (e.target.id === 'lNext') { state.lesson++; show(); }
    if (e.target.id === 'lLab') state.lab = L.vis.split(':')[1];
    if (im) lightbox(im.dataset.img, 'Lesson picture');
    if (o) { const k = +o.dataset.o, ok = k === L.a; o.classList.add(ok ? 'right' : 'wrong'); $('#lFb').innerHTML = '<div class="msg' + (ok ? ' ok' : '') + '"><b>' + (ok ? 'Correct.' : 'Not quite.') + '</b> ' + (ok ? esc(L.why) : 'Read the explanation again and try another answer.') + '</div>'; if (ok && !state.done.includes(state.lesson)) { state.done.push(state.lesson); store.set('done', state.done); const b = $('#ll [data-l="' + state.lesson + '"]'); b.classList.add('done'); $('i', b).textContent = '✓'; } } };
  show();
};
R.next = (el) => {
  const col = (t, key, c) => '<div class="panel"><h2' + (c ? ' style="color:' + c + '"' : '') + '>' + t + '</h2><ul class="clean">' + BOUNDARY[key].map((r) => '<li><span>' + terms(r[0]) + '</span>' + chip(r[1], r[2], r[0], r[3]) + '</li>').join('') + '</ul></div>';
  el.innerHTML = head(9, 'Limitations and Next Objectives', 'Where solid ground ends. The split below is the honest engineering boundary of the project at this snapshot.') +
    '<div class="cols">' + col('What is known', 'known') + col('What is uncertain', 'uncertain', 'var(--or)') + col('What needs another test', 'retest', 'var(--warn)') + col('What is simulation only', 'simonly') + col('Required before hardware', 'hardware', 'var(--gry)') + '</div>' +
    '<div class="vh"><h1 style="font-size:20px">Ranked next objectives</h1><p>The first four come from the project’s own "next" notes and open roadmap boxes. The order is this console’s recommendation, following the rule the roadmap states: each layer is proven before the next one starts.</p></div><ol class="rank">' + NEXT.map((n) => '<li><div><b>' + esc(n.t) + '</b><p style="color:var(--ink2)">' + esc(n.why) + '</p><small>' + esc(n.basis) + ' · ' + esc(n.src) + '</small></div></li>').join('') + '</ol>';
};
/* ---------- routing, motion, boot ---------- */
const main = $('#main');
function route() {
  const id = (location.hash || '#mission').slice(1), v = VIEWS.find((x) => x[0] === id) ? id : 'mission'; state.view = v; state.playing = false;
  document.querySelectorAll('#nav a').forEach((a) => { if (a.getAttribute('href') === '#' + v) a.setAttribute('aria-current', 'page'); else a.removeAttribute('aria-current'); });
  if (stageEl.parentNode) stageEl.remove(); if (viewer.ok) { viewer.onPick = null; viewer.onFrame = null; viewer.colors = {}; viewer.hidden.clear(); viewer.autoRotate = false; }
  EV.length = 0; const el = document.createElement('section'); el.className = 'view'; el.id = 'v-' + v; main.replaceChildren(el); R[v](el); window.scrollTo(0, 0);
  const a = $('#nav a[aria-current]'); if (a && a.scrollIntoView && innerWidth < 981) a.scrollIntoView({ inline: 'center', block: 'nearest' });
}
addEventListener('hashchange', route);
const mb = $('#motionBtn');
function setMotion(on) { state.motion = on; document.documentElement.classList.toggle('still', !on); mb.setAttribute('aria-pressed', on); mb.innerHTML = on ? '❙❙ <span>Pause motion</span>' : '▶ <span>Resume motion</span>'; if (!on) { state.playing = false; state.flowPlay = false; } if (viewer.ok) viewer.autoRotate = on && state.view === 'mission'; }
mb.onclick = () => { setMotion(!state.motion); if (state.view === 'flow') route(); };
setMotion(state.motion); route();
(function boot() {
  const b = $('#boot'), end = () => { b.classList.add('out'); setTimeout(() => b.remove(), 500); };
  if (!state.motion || store.get('booted', false)) { b.remove(); return; }
  const c = DATA.robot.counts, lines = ['Reading snapshot ' + SNAP.short + ' of ' + SNAP.date, c.links + ' links, ' + c.joints + ' joints, ' + c.revolute + ' revolute', c.meshes + ' meshes, ' + c.triangles.toLocaleString('en') + ' triangles', 'Not connected to a robot or simulator'];
  lines.forEach((t, i) => { const li = document.createElement('li'); li.textContent = '> ' + t; li.style.animationDelay = (0.35 + i * 0.45) + 's'; $('#bootLines').appendChild(li); });
  const t = setTimeout(end, 3000); store.set('booted', true);
  const skip = () => { clearTimeout(t); end(); }; $('#bootSkip').onclick = skip; b.addEventListener('keydown', skip); $('#bootSkip').focus();
})();
