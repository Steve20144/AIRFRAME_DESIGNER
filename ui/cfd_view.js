// CFD tab 3D view: the prepared foil surface coloured by pressure coefficient, turned to the attitude the user
// picks while the air always comes from the same direction (three.js -x). Frames: body FRD (x fwd, y right,
// z down) -> three.js (x, -z, y), as scene.js. Positive alpha = nose up, positive beta = wind from the right.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';

const frdToThree = (v) => new THREE.Vector3(v[0], -v[2], v[1]);

export function createCfdView(canvas, handlers = {}) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  const scene = new THREE.Scene();
  const THEMES = {
    light: { bg: 0xf6f6f8, grid: 0xc9cad4, text: 0x333 },
    dark: { bg: 0x131419, grid: 0x33363f, text: 0xddd },
  };
  let theme = THEMES.light;
  scene.background = new THREE.Color(theme.bg);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 200);
  camera.position.set(4.5, 2.6, 5.5);
  const orbit = new OrbitControls(camera, canvas);
  orbit.enableDamping = true; orbit.dampingFactor = 0.12;
  orbit.target.set(1.2, 0, 0);

  scene.add(new THREE.HemisphereLight(0xe8eeff, 0x2a2e36, 1.1));
  const sun = new THREE.DirectionalLight(0xffffff, 1.2); sun.position.set(6, 10, 4); scene.add(sun);
  const sun2 = new THREE.DirectionalLight(0xffffff, 0.5); sun2.position.set(-6, -4, -5); scene.add(sun2);

  const grid = new THREE.GridHelper(20, 20, theme.grid, theme.grid);
  grid.position.y = -2.0; grid.material.transparent = true; grid.material.opacity = 0.5;
  scene.add(grid);

  // the aircraft: a group rotated to the attitude; the mesh inside it stays in body coordinates
  const aircraft = new THREE.Group();
  scene.add(aircraft);
  let mesh = null;
  const mat = new THREE.MeshStandardMaterial({ vertexColors: true, metalness: 0.05, roughness: 0.75, side: THREE.DoubleSide });
  const cgMarker = new THREE.Mesh(new THREE.SphereGeometry(0.05, 16, 16), new THREE.MeshBasicMaterial({ color: 0xe0a53a }));
  cgMarker.visible = false; aircraft.add(cgMarker);

  // wind: thin arrows upstream, fixed in the world
  const windGroup = new THREE.Group(); scene.add(windGroup);
  const windMat = new THREE.LineBasicMaterial({ color: 0x4f8fdc, transparent: true, opacity: 0.55 });
  for (let i = -2; i <= 2; i++) for (let j = -1; j <= 1; j++) {
    const y = j * 0.9 + 0.2, z = i * 0.9;
    const pts = [new THREE.Vector3(5.5, y, z), new THREE.Vector3(3.6, y, z)];
    windGroup.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), windMat));
    const head = new THREE.Mesh(new THREE.ConeGeometry(0.05, 0.18, 10), new THREE.MeshBasicMaterial({ color: 0x4f8fdc, transparent: true, opacity: 0.7 }));
    head.position.set(3.55, y, z); head.rotation.z = Math.PI / 2; windGroup.add(head);
  }

  // force arrows at the CG (world frame: lift up, drag downwind, side along body y)
  const arrows = {
    lift: new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 1, 0x4caf7a, 0.18, 0.09),
    drag: new THREE.ArrowHelper(new THREE.Vector3(-1, 0, 0), new THREE.Vector3(), 1, 0xd9534f, 0.18, 0.09),
    side: new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 1, 0x9b6bd6, 0.18, 0.09),
    weight: new THREE.ArrowHelper(new THREE.Vector3(0, -1, 0), new THREE.Vector3(), 1, 0x8d9aa8, 0.18, 0.09),
  };
  for (const a of Object.values(arrows)) { a.visible = false; scene.add(a); }
  // pitching-moment ring: a partial arc around the body y axis at the CG, arrow head showing the sense
  const momentGroup = new THREE.Group(); scene.add(momentGroup);
  let momentArc = null;

  let alpha = 0, beta = 0, cg = [0, 0, 0], showArrows = true, forceScale = 1.0;
  let cpRange = [-1.5, 1.0];
  let cpData = null;

  function setMesh(payload) {
    if (mesh) { aircraft.remove(mesh); mesh.geometry.dispose(); mesh = null; }
    if (!payload || !payload.vertices || !payload.vertices.length) return;
    const v = payload.vertices, pos = new Float32Array(v.length);
    for (let i = 0; i < v.length; i += 3) { pos[i] = v[i]; pos[i + 1] = -v[i + 2]; pos[i + 2] = v[i + 1]; }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setIndex(v.length / 3 > 65535 ? new THREE.Uint32BufferAttribute(payload.indices, 1) : new THREE.Uint16BufferAttribute(payload.indices, 1));
    g.computeVertexNormals();
    const col = new Float32Array(v.length); col.fill(0.78);
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    mesh = new THREE.Mesh(g, mat);
    aircraft.add(mesh);
    g.computeBoundingSphere();
    const c = g.boundingSphere.center;
    orbit.target.copy(c);
    camera.position.set(c.x + 4.2, c.y + 2.4, c.z + 5.2);
    cpData = null;
  }

  // Cp colour map: red where the air pushes (Cp > 0), blue where it pulls (Cp < 0), pale near zero
  function cpColor(cp, out) {
    const [lo, hi] = cpRange;
    if (!Number.isFinite(cp)) { out[0] = out[1] = out[2] = 0.6; return; }
    if (cp >= 0) {
      const t = Math.min(1, cp / Math.max(hi, 1e-6));
      out[0] = 0.96; out[1] = 0.96 - 0.72 * t; out[2] = 0.94 - 0.76 * t;
    } else {
      const t = Math.min(1, cp / Math.min(lo, -1e-6));
      out[0] = 0.96 - 0.80 * t; out[1] = 0.96 - 0.46 * t; out[2] = 0.94 - 0.10 * t;
    }
  }
  function applyCp() {
    if (!mesh) return;
    const attr = mesh.geometry.getAttribute('color');
    const n = attr.count, tmp = [0, 0, 0];
    if (!cpData) { for (let i = 0; i < n; i++) attr.setXYZ(i, 0.78, 0.78, 0.8); attr.needsUpdate = true; return; }
    for (let i = 0; i < n; i++) { cpColor(cpData[i], tmp); attr.setXYZ(i, tmp[0], tmp[1], tmp[2]); }
    attr.needsUpdate = true;
  }
  function setCp(arr, range) {
    cpData = arr;
    if (range) cpRange = range;
    applyCp();
  }
  function setCpRange(range) { cpRange = range; applyCp(); if (flow) buildStreamlines(); }

  function setAttitude(a, b, rollDeg = 0) {
    alpha = a; beta = b;
    const qPitch = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), THREE.MathUtils.degToRad(alpha));
    const qYaw = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), THREE.MathUtils.degToRad(beta));
    // bank (body x axis, FRD right-wing-down positive = three.js rotation about +x by -roll since FRD z down maps to three -y)
    const qRoll = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -THREE.MathUtils.degToRad(rollDeg || 0));
    aircraft.quaternion.copy(qYaw).multiply(qPitch).multiply(qRoll);
    updateArrows();
  }

  function setCg(c) { cg = c || [0, 0, 0]; cgMarker.position.copy(frdToThree(cg)); cgMarker.visible = !!c; updateArrows(); if (c && Math.abs(sheetZ - c[2]) > 1e-6 && !sheetZPinned) { sheetZ = c[2]; if (flow) { buildParticles(); buildStreamlines(); } } }
  let sheetZPinned = false;

  let forces = null;   // {lift, drag, side, weight, My}
  function setForces(f) { forces = f; updateArrows(); }
  function updateArrows() {
    const origin = frdToThree(cg).applyQuaternion(aircraft.quaternion);
    const ref = forces && forces.weight ? forces.weight : 450;
    const len = (N) => Math.max(0.02, Math.abs(N) / ref * 1.3 * forceScale);
    for (const k of ['lift', 'drag', 'side', 'weight']) {
      const a = arrows[k];
      const N = forces ? forces[k] : null;
      a.visible = showArrows && N != null && Number.isFinite(N) && Math.abs(N) > 1e-6;
      if (!a.visible) continue;
      a.position.copy(origin);
      let dir;
      if (k === 'lift') dir = new THREE.Vector3(0, Math.sign(N), 0);
      else if (k === 'drag') dir = new THREE.Vector3(-Math.sign(N), 0, 0);
      else if (k === 'weight') dir = new THREE.Vector3(0, -1, 0);
      else dir = new THREE.Vector3(0, 0, 1).applyQuaternion(aircraft.quaternion).multiplyScalar(Math.sign(N));
      a.setDirection(dir.normalize());
      a.setLength(len(N), Math.min(0.18, len(N) * 0.4), Math.min(0.09, len(N) * 0.2));
    }
    if (momentArc) { momentGroup.remove(momentArc); momentArc.geometry.dispose(); momentArc = null; }
    if (forces && showArrows && Number.isFinite(forces.My) && Math.abs(forces.My) > 1e-3) {
      // arc radius 0.5 m around the body y axis (three z after rotation), swept nose-up for positive My
      const r = 0.55, sweep = Math.min(Math.PI * 0.9, Math.abs(forces.My) / Math.max(ref * 0.5, 1) * Math.PI);
      const pts = [];
      const s = Math.sign(forces.My);
      for (let i = 0; i <= 24; i++) { const t = -sweep / 2 + sweep * i / 24; pts.push(new THREE.Vector3(r * Math.cos(t), r * Math.sin(t) * s, 0)); }
      const g = new THREE.BufferGeometry().setFromPoints(pts);
      momentArc = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0xe0a53a }));
      momentArc.position.copy(origin);
      momentArc.quaternion.copy(aircraft.quaternion);
      momentGroup.add(momentArc);
    }
  }
  function setShowArrows(on) { showArrows = on; updateArrows(); }
  function setForceScale(s) { forceScale = s; updateArrows(); }

  // ---- the air: a sampled velocity / Cp grid in body axes (from /api/cfd/flow); particles are advected through it
  // and coloured by the local Cp with the same palette as the skin (red: the air is pushing, blue: pulling).
  // Everything lives inside the aircraft group, so turning the aircraft turns the flow with it and the far-field
  // air always comes from the fixed wind direction.
  let flow = null;        // {origin:[3], h, nx, ny, nz, U: Float32Array(n*3), cp: Float32Array(n), V}
  const flowGroup = new THREE.Group(); aircraft.add(flowGroup);
  let particles = null, pState = null, N_PART = 14000, pSpeed = 1.0, showParticles = true, showStreams = false;
  let streams = null, streamCount = 60, dashPhase = 0;
  const DASH = 0.45, GAP = 0.25, DASH_T = DASH + GAP;   // metres of free-stream travel per dash cycle
  let seedMode = 'sheets';       // 'sheets' (horizontal sheet through the CG + vertical sheet at y = 0), 'horizontal', 'vertical', 'volume'
  let sheetZ = 0;                // body-frame z of the horizontal sheet (FRD, down positive); set from the CG
  function seedPoint(out) {
    // a random seed on the upstream side in body axes, following the seeding mode
    const f = flow;
    const yr = f.origin[1] + f.h * (f.ny - 1), zr = f.origin[2] + f.h * (f.nz - 1);
    const x = f.origin[0] + (f.nx - 1.5) * f.h - Math.random() * 0.25 * f.h * f.nx;
    let y, z;
    const mode = seedMode === 'sheets' ? (Math.random() < 0.6 ? 'horizontal' : 'vertical') : seedMode;
    if (mode === 'horizontal') { y = f.origin[1] + 0.1 * (yr - f.origin[1]) + Math.random() * 0.8 * (yr - f.origin[1]); z = sheetZ + (Math.random() - 0.5) * 0.08; }
    else if (mode === 'vertical') { y = (Math.random() - 0.5) * 0.08; z = f.origin[2] + 0.1 * (zr - f.origin[2]) + Math.random() * 0.8 * (zr - f.origin[2]); }
    else { y = f.origin[1] + 0.08 * (yr - f.origin[1]) + Math.random() * 0.84 * (yr - f.origin[1]); z = f.origin[2] + 0.1 * (zr - f.origin[2]) + Math.random() * 0.8 * (zr - f.origin[2]); }
    out[0] = x; out[1] = y; out[2] = z;
  }
  const tmpC = [0, 0, 0];
  // the air uses the skin's palette darkened a little, so neutral (pale) air still shows against the background
  let airRangeScale = 0.3;
  function flowColor(cp, out) {
    const saved = cpRange;
    cpRange = [saved[0] * airRangeScale, saved[1] * airRangeScale];
    cpColor(cp, out);
    cpRange = saved;
    out[0] *= 0.8; out[1] *= 0.8; out[2] *= 0.85;
  }
  function setAirRangeScale(k) { airRangeScale = Math.max(0.02, k); if (flow) buildStreamlines(); }

  function sampleFlow(x, y, z, out) {
    // trilinear velocity and Cp at a body-frame point; returns false outside the grid or inside the body (NaN cells)
    const f = flow;
    const fx = (x - f.origin[0]) / f.h, fy = (y - f.origin[1]) / f.h, fz = (z - f.origin[2]) / f.h;
    const i = Math.floor(fx), j = Math.floor(fy), k = Math.floor(fz);
    if (i < 0 || j < 0 || k < 0 || i >= f.nx - 1 || j >= f.ny - 1 || k >= f.nz - 1) return false;
    const tx = fx - i, ty = fy - j, tz = fz - k;
    let u = 0, v = 0, w = 0, c = 0, wsum = 0;
    for (let di = 0; di < 2; di++) for (let dj = 0; dj < 2; dj++) for (let dk = 0; dk < 2; dk++) {
      const wt = (di ? tx : 1 - tx) * (dj ? ty : 1 - ty) * (dk ? tz : 1 - tz);
      if (wt <= 0) continue;
      const idx = ((i + di) * f.ny + (j + dj)) * f.nz + (k + dk);
      const cp = f.cp[idx];
      if (cp !== cp) continue;          // NaN: inside the body
      u += wt * f.U[idx * 3]; v += wt * f.U[idx * 3 + 1]; w += wt * f.U[idx * 3 + 2]; c += wt * cp; wsum += wt;
    }
    if (wsum < 0.5) return false;        // mostly inside the body: the particle has hit the skin
    out[0] = u / wsum; out[1] = v / wsum; out[2] = w / wsum; out[3] = c / wsum;
    return true;
  }

  const seedTmp = [0, 0, 0];
  function spawn(i) {
    const p = pState;
    seedPoint(seedTmp);
    p.pos[i * 3] = seedTmp[0]; p.pos[i * 3 + 1] = seedTmp[1]; p.pos[i * 3 + 2] = seedTmp[2];
    p.age[i] = 0;
  }

  function buildParticles() {
    if (particles) { flowGroup.remove(particles); particles.geometry.dispose(); particles.material.dispose(); particles = null; }
    if (!flow) return;
    pState = { pos: new Float32Array(N_PART * 3), age: new Float32Array(N_PART), col: new Float32Array(N_PART * 3), three: new Float32Array(N_PART * 3) };
    for (let i = 0; i < N_PART; i++) { spawn(i); pState.age[i] = Math.random() * 3; }
    // let the field carry them for a while so the first frame is already a full stream
    for (let s = 0; s < 120; s++) stepParticles(1 / 30, true);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pState.three, 3));
    g.setAttribute('color', new THREE.BufferAttribute(pState.col, 3));
    const m = new THREE.PointsMaterial({ size: 0.075, vertexColors: true, transparent: true, opacity: 0.95, sizeAttenuation: true, depthWrite: false });
    particles = new THREE.Points(g, m);
    particles.visible = showParticles;
    particles.frustumCulled = false;
    flowGroup.add(particles);
  }

  const smp = [0, 0, 0, 0];
  function stepParticles(dt, silent) {
    const p = pState, f = flow;
    const scale = dt * pSpeed;
    for (let i = 0; i < N_PART; i++) {
      let x = p.pos[i * 3], y = p.pos[i * 3 + 1], z = p.pos[i * 3 + 2];
      if (!sampleFlow(x, y, z, smp)) { spawn(i); x = p.pos[i * 3]; y = p.pos[i * 3 + 1]; z = p.pos[i * 3 + 2]; if (!sampleFlow(x, y, z, smp)) continue; }
      // midpoint step
      const hx = x + 0.5 * scale * smp[0], hy = y + 0.5 * scale * smp[1], hz = z + 0.5 * scale * smp[2];
      if (sampleFlow(hx, hy, hz, smp)) { x += scale * smp[0]; y += scale * smp[1]; z += scale * smp[2]; }
      else { x = hx; y = hy; z = hz; }
      p.pos[i * 3] = x; p.pos[i * 3 + 1] = y; p.pos[i * 3 + 2] = z;
      p.age[i] += dt;
      if (p.age[i] > 14) spawn(i);
      flowColor(smp[3], tmpC);
      p.col[i * 3] = tmpC[0]; p.col[i * 3 + 1] = tmpC[1]; p.col[i * 3 + 2] = tmpC[2];
      p.three[i * 3] = x; p.three[i * 3 + 1] = -z; p.three[i * 3 + 2] = y;     // FRD -> three
    }
    if (!silent && particles) { particles.geometry.attributes.position.needsUpdate = true; particles.geometry.attributes.color.needsUpdate = true; }
  }

  function buildStreamlines() {
    if (streams) { for (const t of streams) { flowGroup.remove(t.line); t.line.geometry.dispose(); t.line.material.dispose(); } streams = null; }
    if (!flow || streamCount <= 0) return;
    const f = flow;
    const yr = f.origin[1] + f.h * (f.ny - 1), zr = f.origin[2] + f.h * (f.nz - 1);
    const x0 = f.origin[0] + (f.nx - 1.5) * f.h;
    // seeds: evenly spaced across the sheets (or a coarse volume lattice), upstream
    const seeds = [];
    const mode = seedMode;
    const nH = mode === 'sheets' ? Math.ceil(streamCount * 0.6) : (mode === 'horizontal' ? streamCount : 0);
    const nV = mode === 'sheets' ? streamCount - nH : (mode === 'vertical' ? streamCount : 0);
    for (let i = 0; i < nH; i++) seeds.push([x0, f.origin[1] + 0.1 * (yr - f.origin[1]) + (i + 0.5) / nH * 0.8 * (yr - f.origin[1]), sheetZ]);
    for (let i = 0; i < nV; i++) seeds.push([x0, 0, f.origin[2] + 0.1 * (zr - f.origin[2]) + (i + 0.5) / nV * 0.8 * (zr - f.origin[2])]);
    if (mode === 'volume') {
      const n = Math.max(1, Math.round(Math.sqrt(streamCount)));
      for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) seeds.push([x0, f.origin[1] + 0.1 * (yr - f.origin[1]) + (i + 0.5) / n * 0.8 * (yr - f.origin[1]), f.origin[2] + 0.1 * (zr - f.origin[2]) + (j + 0.5) / n * 0.8 * (zr - f.origin[2])]);
    }
    const ds = 0.5 * f.h;
    streams = [];
    for (const sd of seeds) {
      let [x, y, z] = sd, t = 0;
      const pos = [], col = [], dist = [];
      for (let n = 0; n < 1500; n++) {
        if (!sampleFlow(x, y, z, smp)) break;           // left the box or hit the skin: the thread ends here
        const sp = Math.hypot(smp[0], smp[1], smp[2]);
        if (sp < 1e-3) break;
        flowColor(smp[3], tmpC);
        pos.push(x, -z, y); col.push(tmpC[0], tmpC[1], tmpC[2]); dist.push(t * f.V);
        x += ds * smp[0] / sp; y += ds * smp[1] / sp; z += ds * smp[2] / sp;
        t += ds / sp;                                   // time of flight: dashes run fast where the air is fast
      }
      if (pos.length < 6) continue;
      // fat line (pixels wide) with per-vertex colour and animated dashes; the dash distance is time of flight x V
      const g = new LineGeometry();
      g.setPositions(pos); g.setColors(col);
      const base = new Float32Array(dist);
      const mat = new LineMaterial({ vertexColors: true, linewidth: threadWidth, dashed: true, dashSize: DASH, gapSize: GAP, dashOffset: 0,
                                     transparent: true, opacity: 0.95, alphaToCoverage: false });
      mat.resolution.set(canvas.clientWidth || 800, canvas.clientHeight || 480);
      const line = new Line2(g, mat);
      line.computeLineDistances();                      // creates the instance distance attributes the dashes use
      line.visible = showStreams; line.frustumCulled = false;
      flowGroup.add(line);
      streams.push({ line, base });
    }
    animateThreads(0);
  }
  function animateThreads(dt) {
    if (!streams || !flow) return;
    dashPhase = (dashPhase + dt * pSpeed * flow.V) % DASH_T;
    for (const t of streams) {
      const g = t.line.geometry, b = t.base;
      const ds = g.attributes.instanceDistanceStart, de = g.attributes.instanceDistanceEnd;
      if (!ds || !de) continue;
      const n = Math.min(ds.count, b.length - 1);
      for (let i = 0; i < n; i++) { ds.array[i] = b[i] - dashPhase + DASH_T; de.array[i] = b[i + 1] - dashPhase + DASH_T; }
      ds.needsUpdate = true; de.needsUpdate = true;
    }
  }
  let threadWidth = 2.5;
  function setThreadWidth(w) { threadWidth = w; if (streams) for (const t of streams) t.line.material.linewidth = w; }

  function setFlow(fl) {
    // fl: {origin, spacing, shape, speed_ms, U: Float32Array, cp: Float32Array} in body axes, or null to clear
    flow = fl ? { origin: fl.origin, h: fl.spacing, nx: fl.shape[0], ny: fl.shape[1], nz: fl.shape[2], U: fl.U, cp: fl.cp, V: fl.speed_ms } : null;
    if (!flow) { buildParticles(); buildStreamlines(); return; }
    if (showParticles) buildParticles();
    else if (particles) { flowGroup.remove(particles); particles.geometry.dispose(); particles.material.dispose(); particles = null; }
    buildStreamlines();
  }
  function setSeedMode(m) { seedMode = m; if (flow) { buildParticles(); buildStreamlines(); } }
  function setSheetZ(z) { sheetZ = z; sheetZPinned = true; if (flow) { buildParticles(); buildStreamlines(); } }
  function setParticleCount(n) { N_PART = Math.max(100, Math.min(40000, n | 0)); if (flow) buildParticles(); }
  function setFlowSpeed(s) { pSpeed = s; }
  function setShowParticles(on) { showParticles = on; if (on && flow && !particles) buildParticles(); if (particles) particles.visible = on; }
  function setShowStreams(on) { showStreams = on; if (streams) for (const t of streams) t.line.visible = on; }
  function setStreamCount(n) { streamCount = n | 0; if (flow) buildStreamlines(); }

  // turn-the-aircraft mode: dragging changes alpha (vertical) and beta (horizontal) instead of orbiting
  let turnMode = false, dragging = false, last = null;
  function setTurnMode(on) { turnMode = on; orbit.enabled = !on; canvas.style.cursor = on ? 'grab' : ''; }
  canvas.addEventListener('pointerdown', (e) => { if (!turnMode) return; dragging = true; last = [e.clientX, e.clientY]; canvas.setPointerCapture(e.pointerId); canvas.style.cursor = 'grabbing'; });
  canvas.addEventListener('pointermove', (e) => {
    if (!turnMode || !dragging) return;
    const dx = e.clientX - last[0], dy = e.clientY - last[1]; last = [e.clientX, e.clientY];
    // the camera may look from either side: use its position to decide which way "drag up" tilts the nose
    const camRight = new THREE.Vector3(1, 0, 0).applyQuaternion(camera.quaternion);
    const sideSign = camRight.x >= 0 ? 1 : -1;
    const na = Math.max(-20, Math.min(25, alpha + dy * 0.12 * sideSign));
    const nb = Math.max(-25, Math.min(25, beta - dx * 0.12 * (new THREE.Vector3(0, 0, 1).applyQuaternion(camera.quaternion).x >= 0 ? -1 : 1)));
    if (handlers.onAttitude) handlers.onAttitude(na, nb); else setAttitude(na, nb);
  });
  const endDrag = () => { if (dragging) { dragging = false; canvas.style.cursor = turnMode ? 'grab' : ''; if (handlers.onAttitudeCommit) handlers.onAttitudeCommit(alpha, beta); } };
  canvas.addEventListener('pointerup', endDrag); canvas.addEventListener('pointercancel', endDrag);

  function setTheme(name) {
    theme = THEMES[name] || THEMES.light;
    scene.background = new THREE.Color(theme.bg);
    grid.material.color.set(theme.grid);
  }

  function resize() {
    const w = canvas.clientWidth || 800, h = canvas.clientHeight || 480;
    if (canvas.width !== Math.floor(w * renderer.getPixelRatio()) || canvas.height !== Math.floor(h * renderer.getPixelRatio())) {
      renderer.setSize(w, h, false);
      camera.aspect = w / h; camera.updateProjectionMatrix();
      if (streams) for (const t of streams) t.line.material.resolution.set(w, h);
    }
  }
  let active = true, lastT = performance.now();
  function loop() {
    const now = performance.now(), dt = Math.min(0.05, (now - lastT) / 1000); lastT = now;
    if (active && canvas.offsetParent !== null) {
      resize();
      if (flow && particles && showParticles) stepParticles(dt, false);
      if (flow && streams && showStreams) animateThreads(dt);
      orbit.update(); renderer.render(scene, camera);
    }
    requestAnimationFrame(loop);
  }
  loop();

  function resetCamera() {
    const c = orbit.target.clone();
    camera.position.set(c.x + 4.2, c.y + 2.4, c.z + 5.2);
  }
  function viewFrom(which) {
    const c = orbit.target.clone();
    if (which === 'side') camera.position.set(c.x, c.y + 0.2, c.z + 7);
    else if (which === 'top') camera.position.set(c.x + 0.01, c.y + 7, c.z);
    else if (which === 'front') camera.position.set(c.x + 7, c.y + 0.5, c.z);
    else if (which === 'below') camera.position.set(c.x + 2, c.y - 6, c.z + 3);
    else resetCamera();
  }

  return { setMesh, setCp, setCpRange, setAttitude, setCg, setForces, setShowArrows, setForceScale, setTurnMode, setTheme, resetCamera, viewFrom,
           setFlow, setParticleCount, setFlowSpeed, setShowParticles, setShowStreams, setStreamCount, setSeedMode, setSheetZ, setThreadWidth, setAirRangeScale, get hasFlow() { return !!flow; },
           get alpha() { return alpha; }, get beta() { return beta; } };
}
