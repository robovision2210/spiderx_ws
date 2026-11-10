/* Minimal WebGL viewer for the real SpiderX meshes. No external library. World frame = base_link (z up). */
function b64ToBytes(b64) { const s = atob(b64), u = new Uint8Array(s.length); for (let i = 0; i < s.length; i++) u[i] = s.charCodeAt(i); return u; }
const M4 = {
  persp(fov, asp, n, f) { const t = 1 / Math.tan(fov / 2); return [t / asp, 0, 0, 0, 0, t, 0, 0, 0, 0, (f + n) / (n - f), -1, 0, 0, 2 * f * n / (n - f), 0]; },
  lookAt(e, c, up) {
    const z = V.unit(V.sub(e, c)), x = V.unit(V.cross(up, z)), y = V.cross(z, x);
    return [x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0, -V.dot(x, e), -V.dot(y, e), -V.dot(z, e), 1];
  },
  mul(a, b) { const o = new Array(16); for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) { let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k]; o[c * 4 + r] = s; } return o; },
  fromPose(R, p) { return [R[0], R[3], R[6], 0, R[1], R[4], R[7], 0, R[2], R[5], R[8], 0, p[0], p[1], p[2], 1]; },
};
class Viewer {
  constructor(canvas, model, meshB64) {
    this.canvas = canvas; this.model = model;
    this.cam = { yaw: -0.9, pitch: 0.42, dist: 0.62, target: [0.051, -0.06, 0.02] };
    this.goal = null; this.highlight = null; this.colors = {}; this.lines = []; this.labels = []; this.hidden = new Set();
    this.autoRotate = false; this.dirty = true; this.onPick = null; this.onFrame = null; this.ok = false;
    const gl = canvas.getContext('webgl', { antialias: true, alpha: true, preserveDrawingBuffer: false });
    if (!gl) return;
    this.gl = gl; this.ok = true;
    const vs = 'attribute vec3 aP;attribute vec3 aN;uniform mat4 uVP;uniform mat4 uM;varying vec3 vN;varying vec3 vW;void main(){vec4 w=uM*vec4(aP,1.0);vW=w.xyz;vN=mat3(uM)*aN;gl_Position=uVP*w;}';
    const fs = 'precision mediump float;varying vec3 vN;varying vec3 vW;uniform vec3 uEye;uniform vec3 uCol;uniform vec3 uRim;uniform float uA;uniform float uFlat;void main(){if(uFlat>0.5){gl_FragColor=vec4(uCol,uA);return;}vec3 n=normalize(vN);vec3 e=normalize(uEye-vW);if(dot(n,e)<0.0)n=-n;float d=max(dot(n,normalize(vec3(0.35,-0.5,0.8))),0.0);float f=pow(1.0-max(dot(n,e),0.0),2.5);vec3 c=uCol*(0.32+0.68*d)+uRim*f;gl_FragColor=vec4(c,uA);}';
    const sh = (t, s) => { const o = gl.createShader(t); gl.shaderSource(o, s); gl.compileShader(o); return o; };
    const pr = gl.createProgram(); gl.attachShader(pr, sh(gl.VERTEX_SHADER, vs)); gl.attachShader(pr, sh(gl.FRAGMENT_SHADER, fs)); gl.linkProgram(pr);
    if (!gl.getProgramParameter(pr, gl.LINK_STATUS)) { this.ok = false; return; }
    gl.useProgram(pr); this.pr = pr;
    this.loc = { aP: gl.getAttribLocation(pr, 'aP'), aN: gl.getAttribLocation(pr, 'aN') };
    for (const n of ['uVP', 'uM', 'uEye', 'uCol', 'uRim', 'uA', 'uFlat']) this.loc[n] = gl.getUniformLocation(pr, n);
    this.meshes = {}; this.buildMeshes(b64ToBytes(meshB64));
    this.lineBuf = gl.createBuffer();
    this.bind();
    const ro = new ResizeObserver(() => { this.dirty = true; }); ro.observe(canvas);
    const loop = (t) => { this.tick(t); requestAnimationFrame(loop); }; requestAnimationFrame(loop);
  }
  buildMeshes(bytes) {
    const gl = this.gl, d = this.model.data, q = d.quant;
    const dv = new DataView(bytes.buffer);
    const mk = (pos) => {
      const n = pos.length / 3, arr = new Float32Array(n * 6);
      for (let t = 0; t < n; t += 3) {
        const a = [pos[t * 3], pos[t * 3 + 1], pos[t * 3 + 2]], b = [pos[t * 3 + 3], pos[t * 3 + 4], pos[t * 3 + 5]], c = [pos[t * 3 + 6], pos[t * 3 + 7], pos[t * 3 + 8]];
        const nn = V.unit(V.cross(V.sub(b, a), V.sub(c, a)));
        [a, b, c].forEach((p, k) => arr.set([p[0], p[1], p[2], nn[0], nn[1], nn[2]], (t + k) * 6));
      }
      const buf = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, buf); gl.bufferData(gl.ARRAY_BUFFER, arr, gl.STATIC_DRAW);
      return { buf, count: n };
    };
    for (const [name, m] of Object.entries(d.mesh_index)) {
      const pos = new Float32Array(m.i[1] * 3);
      for (let k = 0; k < m.i[1]; k++) { const vi = dv.getUint16(m.i[0] + k * 2, true), o = m.v[0] + vi * 6; pos[k * 3] = dv.getInt16(o, true) * q; pos[k * 3 + 1] = dv.getInt16(o + 2, true) * q; pos[k * 3 + 2] = dv.getInt16(o + 4, true) * q; }
      this.meshes[name] = mk(pos);
    }
    /* lidar_link has no CAD: the URDF gives a cylinder r 0.035, length 0.040, centred 0.010 below the frame. */
    const cyl = [], r = 0.035, z0 = -0.03, z1 = 0.01, N = 40;
    for (let i = 0; i < N; i++) {
      const a = i / N * 2 * Math.PI, b = (i + 1) / N * 2 * Math.PI, A = [r * Math.cos(a), r * Math.sin(a)], B = [r * Math.cos(b), r * Math.sin(b)];
      cyl.push(A[0], A[1], z0, B[0], B[1], z0, B[0], B[1], z1, A[0], A[1], z0, B[0], B[1], z1, A[0], A[1], z1, 0, 0, z1, A[0], A[1], z1, B[0], B[1], z1, 0, 0, z0, B[0], B[1], z0, A[0], A[1], z0);
    }
    this.meshes.lidar_link = mk(new Float32Array(cyl));
  }
  bind() {
    const c = this.canvas; let drag = null; const pts = new Map();
    c.addEventListener('pointerdown', (e) => { c.setPointerCapture(e.pointerId); pts.set(e.pointerId, [e.clientX, e.clientY]); drag = { x: e.clientX, y: e.clientY, moved: 0, pan: e.button === 2 || e.shiftKey }; this.goal = null; });
    c.addEventListener('pointermove', (e) => {
      if (!pts.has(e.pointerId)) return;
      if (pts.size === 2) { const o = [...pts.values()]; const d0 = Math.hypot(o[0][0] - o[1][0], o[0][1] - o[1][1]); pts.set(e.pointerId, [e.clientX, e.clientY]); const n = [...pts.values()]; const d1 = Math.hypot(n[0][0] - n[1][0], n[0][1] - n[1][1]); this.cam.dist = Math.min(2.5, Math.max(0.12, this.cam.dist * d0 / (d1 || 1))); this.dirty = true; drag.moved = 99; return; }
      pts.set(e.pointerId, [e.clientX, e.clientY]);
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y; drag.x = e.clientX; drag.y = e.clientY; drag.moved += Math.abs(dx) + Math.abs(dy);
      if (drag.pan) { const s = this.cam.dist * 0.0016, cy = Math.cos(this.cam.yaw), sy = Math.sin(this.cam.yaw); this.cam.target = V.add(this.cam.target, [sy * dx * s, -cy * dx * s, dy * s]); }
      else { this.cam.yaw -= dx * 0.008; this.cam.pitch = Math.max(-1.45, Math.min(1.5, this.cam.pitch + dy * 0.008)); }
      this.dirty = true;
    });
    const up = (e) => { if (drag && drag.moved < 5 && pts.size === 1 && this.onPick) { const r = c.getBoundingClientRect(); this.onPick(this.pick(e.clientX - r.left, e.clientY - r.top)); } pts.delete(e.pointerId); if (!pts.size) drag = null; };
    c.addEventListener('pointerup', up); c.addEventListener('pointercancel', (e) => { pts.delete(e.pointerId); });
    c.addEventListener('contextmenu', (e) => e.preventDefault());
    c.addEventListener('wheel', (e) => { e.preventDefault(); this.cam.dist = Math.min(2.5, Math.max(0.12, this.cam.dist * Math.exp(e.deltaY * 0.0012))); this.goal = null; this.dirty = true; }, { passive: false });
    c.addEventListener('keydown', (e) => {
      const k = e.key; let used = true;
      if (k === 'ArrowLeft') this.cam.yaw -= 0.12; else if (k === 'ArrowRight') this.cam.yaw += 0.12;
      else if (k === 'ArrowUp') this.cam.pitch = Math.min(1.5, this.cam.pitch + 0.1); else if (k === 'ArrowDown') this.cam.pitch = Math.max(-1.45, this.cam.pitch - 0.1);
      else if (k === '+' || k === '=') this.cam.dist *= 0.88; else if (k === '-') this.cam.dist /= 0.88; else used = false;
      if (used) { e.preventDefault(); this.goal = null; this.dirty = true; }
    });
  }
  flyTo(c) { this.goal = { ...this.cam, ...c }; this.dirty = true; }
  eye() { const c = this.cam, cp = Math.cos(c.pitch), d = c.dist * Math.max(1, 1.45 / ((this.canvas.width || 16) / (this.canvas.height || 11))); return V.add(c.target, [d * cp * Math.cos(c.yaw), d * cp * Math.sin(c.yaw), d * Math.sin(c.pitch)]); }
  vp() { const w = this.canvas.width, h = this.canvas.height; return M4.mul(M4.persp(0.62, w / h, 0.01, 20), M4.lookAt(this.eye(), this.cam.target, [0, 0, 1])); }
  project(p) {
    const m = this.vpm || this.vp(), x = m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12], y = m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13], w = m[3] * p[0] + m[7] * p[1] + m[11] * p[2] + m[15];
    if (w <= 0) return null; const r = this.canvas.getBoundingClientRect(); return [(x / w * 0.5 + 0.5) * r.width, (1 - (y / w * 0.5 + 0.5)) * r.height];
  }
  tick(t) {
    const c = this.canvas, dpr = Math.min(window.devicePixelRatio || 1, 2), w = Math.round(c.clientWidth * dpr), h = Math.round(c.clientHeight * dpr);
    if (!w || !h) return;
    if (c.width !== w || c.height !== h) { c.width = w; c.height = h; this.dirty = true; }
    if (this.goal) {
      let done = true; const k = 0.14;
      for (const f of ['yaw', 'pitch', 'dist']) { const d = this.goal[f] - this.cam[f]; if (Math.abs(d) > 1e-3) done = false; this.cam[f] += d * k; }
      const dt = V.sub(this.goal.target, this.cam.target); if (V.norm(dt) > 1e-4) done = false; this.cam.target = V.add(this.cam.target, V.scale(dt, k));
      if (done) this.goal = null; this.dirty = true;
    } else if (this.autoRotate) { this.cam.yaw += 0.0022; this.dirty = true; }
    if (this.onFrame && this.onFrame(t)) this.dirty = true;
    if (this.dirty) { this.dirty = false; this.draw(false); this.placeLabels(); }
  }
  drawLink(name, pickId) {
    const gl = this.gl, m = this.meshes[name], ps = this.model.pose[name]; if (!m || !ps || this.hidden.has(name)) return;
    gl.bindBuffer(gl.ARRAY_BUFFER, m.buf);
    gl.vertexAttribPointer(this.loc.aP, 3, gl.FLOAT, false, 24, 0); gl.vertexAttribPointer(this.loc.aN, 3, gl.FLOAT, false, 24, 12);
    gl.uniformMatrix4fv(this.loc.uM, false, new Float32Array(M4.fromPose(ps.R, ps.p)));
    if (pickId != null) { gl.uniform1f(this.loc.uFlat, 1); gl.uniform3f(this.loc.uCol, pickId / 255, 0, 1); gl.uniform1f(this.loc.uA, 1); }
    else {
      const hl = this.highlight ? (this.highlight.has(name) ? 1 : -1) : 0;
      const base = this.colors[name] || (name === 'lidar_link' ? [0.16, 0.2, 0.22] : [0.44, 0.52, 0.55]);
      const col = hl === 1 ? [1.0, 0.56, 0.2] : hl === -1 ? [0.2, 0.27, 0.3] : base;
      const rim = hl === 1 ? [0.9, 0.45, 0.1] : [0.1, 0.75, 0.8];
      gl.uniform1f(this.loc.uFlat, 0); gl.uniform3f(this.loc.uCol, col[0], col[1], col[2]); gl.uniform3f(this.loc.uRim, rim[0] * (hl === -1 ? 0.35 : 0.8), rim[1] * (hl === -1 ? 0.35 : 0.8), rim[2] * (hl === -1 ? 0.35 : 0.8)); gl.uniform1f(this.loc.uA, 1);
    }
    gl.drawArrays(gl.TRIANGLES, 0, m.count);
  }
  draw(pickMode) {
    const gl = this.gl; gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    gl.clearColor(0, 0, 0, 0); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT); gl.enable(gl.DEPTH_TEST); gl.disable(gl.BLEND);
    this.vpm = this.vp(); const eye = this.eye();
    gl.uniformMatrix4fv(this.loc.uVP, false, new Float32Array(this.vpm)); gl.uniform3f(this.loc.uEye, eye[0], eye[1], eye[2]);
    gl.enableVertexAttribArray(this.loc.aP); gl.enableVertexAttribArray(this.loc.aN);
    this.pickNames = Object.keys(this.meshes);
    this.pickNames.forEach((n, i) => this.drawLink(n, pickMode ? i + 1 : null));
    if (pickMode) return;
    /* lines: [p0, p1, [r,g,b], alpha] in base_link metres */
    const all = this.gridLines().concat(this.lines); if (!all.length) return;
    gl.disableVertexAttribArray(this.loc.aN); gl.vertexAttrib3f(this.loc.aN, 0, 0, 1);
    gl.uniformMatrix4fv(this.loc.uM, false, new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])); gl.uniform1f(this.loc.uFlat, 1);
    gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.lineBuf); gl.vertexAttribPointer(this.loc.aP, 3, gl.FLOAT, false, 0, 0);
    const groups = {};
    for (const l of all) { const k = l[2].join(',') + '|' + (l[3] == null ? 1 : l[3]) + '|' + (l[4] ? 1 : 0); (groups[k] = groups[k] || { c: l[2], a: l[3] == null ? 1 : l[3], top: l[4], v: [] }).v.push(...l[0], ...l[1]); }
    for (const g of Object.values(groups)) {
      if (g.top) gl.disable(gl.DEPTH_TEST); else gl.enable(gl.DEPTH_TEST);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(g.v), gl.DYNAMIC_DRAW); gl.uniform3f(this.loc.uCol, g.c[0], g.c[1], g.c[2]); gl.uniform1f(this.loc.uA, g.a); gl.drawArrays(gl.LINES, 0, g.v.length / 3);
    }
  }
  gridLines() {
    if (this._grid) return this.showGrid === false ? [] : this._grid;
    const g = [], z = -0.0545, c = [0.1, 0.62, 0.66];
    for (let i = -8; i <= 8; i++) { const a = i === 0 ? 0.5 : 0.2; g.push([[0.051 + i * 0.05, -0.46, z], [0.051 + i * 0.05, 0.34, z], c, a]); g.push([[-0.349, -0.06 + i * 0.05, z], [0.451, -0.06 + i * 0.05, z], c, a]); }
    this._grid = g; return g;
  }
  pick(x, y) {
    const gl = this.gl, dpr = this.canvas.width / this.canvas.clientWidth; this.draw(true);
    const px = new Uint8Array(4); gl.readPixels(Math.round(x * dpr), this.canvas.height - Math.round(y * dpr), 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, px);
    this.dirty = true; this.draw(false);
    return px[3] && px[0] ? this.pickNames[px[0] - 1] : null;
  }
  setLabels(host, labels) { this.labelHost = host; this.labels = labels; host.innerHTML = ''; for (const l of labels) { const el = document.createElement('span'); el.className = 'v-label ' + (l.cls || ''); el.textContent = l.text; host.appendChild(el); l.el = el; } this.dirty = true; }
  placeLabels() { for (const l of this.labels) { const p = this.project(typeof l.p === 'function' ? l.p() : l.p); if (!p) { l.el.hidden = true; continue; } l.el.hidden = false; l.el.style.transform = 'translate(' + (p[0] + 8).toFixed(1) + 'px,' + (p[1] - 8).toFixed(1) + 'px)'; } }
}
