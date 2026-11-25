import * as THREE from 'three';

const $ = (id) => document.getElementById(id);

let sceneLib = [];
let currentData = null;
let tolDirty = false;

// ---------- three.js viewer ----------
const canvas = $('canvas');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
camera.position.set(2.4, -2.6, 1.8);
const threeScene = new THREE.Scene();
threeScene.background = new THREE.Color(0x0a0e14);
threeScene.add(new THREE.AmbientLight(0xffffff, 0.7));
const keyLight = new THREE.DirectionalLight(0xffffff, 0.9);
keyLight.position.set(2, -3, 4);
threeScene.add(keyLight);
const orbit = makeOrbit(camera, canvas);

const patchColors = [0x4ea1ff, 0x37d399, 0xffb454, 0xc58fff, 0xff8fab, 0x7fd4ff];
let meshGroup = null;

function resize() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);

function frameCamera(positions) {
  const box = new THREE.Box3();
  for (let i = 0; i < positions.length; i += 3) {
    box.expandByPoint(new THREE.Vector3(positions[i], positions[i + 1], positions[i + 2]));
  }
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3()).length() || 1;
  camera.position.set(center.x + size * 0.9, center.y - size * 1.0, center.z + size * 0.7);
  orbit.target.copy(center);
  orbit.update();
}

function renderData(data) {
  if (meshGroup) {
    threeScene.remove(meshGroup);
    meshGroup.traverse((o) => {
      if (o.geometry) o.geometry.dispose();
      if (o.material) o.material.dispose();
    });
  }
  meshGroup = new THREE.Group();
  const { positions, faces, facePatches, faceUnsatisfied, seamVertexIds, patchNames } = data.geometry;
  const colorByPatch = new Map(patchNames.map((n, i) => [n, patchColors[i % patchColors.length]]));

  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geom.setIndex(faces.flat());
  geom.computeVertexNormals();

  const faceColors = new Float32Array(faces.length * 3);
  const tmp = new THREE.Color();
  faces.forEach((_, fi) => {
    let hex = colorByPatch.get(facePatches[fi]) ?? 0x8aa0b6;
    if ($('showUnsat').checked && faceUnsatisfied[fi]) hex = 0xff3b3b;
    tmp.setHex(hex);
    faceColors[fi * 3] = tmp.r;
    faceColors[fi * 3 + 1] = tmp.g;
    faceColors[fi * 3 + 2] = tmp.b;
  });
  geom.setAttribute('color', new THREE.BufferAttribute(faceColors, 3));

  const surface = new THREE.Mesh(
    geom,
    new THREE.MeshStandardMaterial({ vertexColors: true, side: THREE.DoubleSide, roughness: 0.55, metalness: 0.1 })
  );
  meshGroup.add(surface);

  if ($('showWire').checked) {
    const wire = new THREE.LineSegments(
      new THREE.WireframeGeometry(geom),
      new THREE.LineBasicMaterial({ color: 0x0a0e14, transparent: true, opacity: 0.35 })
    );
    meshGroup.add(wire);
  }

  if ($('showSeams').checked) addSeamLines(meshGroup, data);

  threeScene.add(meshGroup);
  frameCamera(positions);
  resize();
}

function addSeamLines(group, data) {
  const ids = data.geometry.seamVertexIds;
  if (!ids || !ids.length) return;
  const pos = data.geometry.positions;
  const pts = ids.map((id) => new THREE.Vector3(pos[id * 3], pos[id * 3 + 1], pos[id * 3 + 2]));
  const g = new THREE.BufferGeometry().setFromPoints(pts);
  group.add(new THREE.Points(g, new THREE.PointsMaterial({ color: 0xffe066, size: 0.03 })));
}

// Minimal orbit controls without external addons.
function makeOrbit(cam, dom) {
  const state = { target: new THREE.Vector3() };
  const active = new Map();
  dom.addEventListener('pointerdown', (e) => {
    active.set(e.pointerId, { x: e.clientX, y: e.clientY, button: e.button });
    dom.setPointerCapture(e.pointerId);
  });
  dom.addEventListener('pointermove', (e) => {
    const prev = active.get(e.pointerId);
    if (!prev) return;
    const dx = e.clientX - prev.x;
    const dy = e.clientY - prev.y;
    prev.x = e.clientX;
    prev.y = e.clientY;
    if (prev.button === 2 || e.shiftKey) {
      const panScale = state.target.distanceTo(cam.position) * 0.0015;
      cam.position.x -= dx * panScale;
      cam.position.y += dy * panScale;
      state.target.x -= dx * panScale;
      state.target.y += dy * panScale;
    } else {
      const off = cam.position.clone().sub(state.target);
      const sph = new THREE.Spherical().setFromVector3(off);
      sph.theta -= dx * 0.006;
      sph.phi = THREE.MathUtils.clamp(sph.phi - dy * 0.006, 0.08, Math.PI - 0.08);
      off.setFromSpherical(sph);
      cam.position.copy(state.target).add(off);
    }
  });
  dom.addEventListener('pointerup', (e) => active.delete(e.pointerId));
  dom.addEventListener('wheel', (e) => {
    e.preventDefault();
    const off = cam.position.clone().sub(state.target);
    off.multiplyScalar(e.deltaY > 0 ? 1.12 : 0.89);
    cam.position.copy(state.target).add(off);
  }, { passive: false });
  dom.addEventListener('contextmenu', (e) => e.preventDefault());
  return {
    target: state.target,
    update() { cam.lookAt(state.target); },
  };
}

function animate() {
  requestAnimationFrame(animate);
  orbit.update();
  renderer.render(threeScene, camera);
}
animate();

// ---------- API ----------
async function api(path, body) {
  const res = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const data = await res.json();
  if (!res.ok) throw Object.assign(new Error(data.reason || data.error || 'request failed'), { data, status: res.status });
  return data;
}

function currentParams() {
  const params = {
    scene: $('scene').value,
    vertexCap: Number($('vertexCap').value),
    mode: $('mode').value,
  };
  // Until the operator edits the tolerance fields, each patch keeps its own
  // declared tolerance (needed so welded neighbours can demand different
  // precision). A manual edit applies a global override to every patch.
  if (tolDirty) {
    params.positionTol = Number($('posTol').value);
    params.normalTolDeg = Number($('normalTol').value);
  }
  return params;
}

function setStatus(text, cls) {
  const el = $('status');
  el.textContent = text;
  el.className = `status ${cls || ''}`;
}

async function runMesh() {
  setStatus('正在执行自适应剖分…', 'warn');
  try {
    const data = await api('/api/mesh', currentParams());
    currentData = data;
    renderData(data);
    renderResults(data);
    if (data.summary.rejected) {
      setStatus(`导出被拒绝：${data.summary.rejectReason}`, 'err');
    } else {
      setStatus(`剖分完成：${data.summary.globalVertices} 顶点 / ${data.summary.totalTriangles} 单元`, 'ok');
    }
  } catch (err) {
    setStatus(`失败：${err.message}`, 'err');
  }
}

async function exportMesh(format) {
  if (!currentData) await runMesh();
  setStatus(`正在生成 ${format.toUpperCase()} 文件…`, 'warn');
  try {
    const info = await api('/api/export', { ...currentParams(), format });
    setStatus(`已写出真实网格文件 ${info.file}（${info.vertices} 顶点，${info.triangles} 面，${info.bytes} 字节）`, 'ok');
    const a = document.createElement('a');
    a.href = info.downloadUrl;
    a.download = info.file;
    a.click();
  } catch (err) {
    if (err.status === 409) {
      setStatus(`导出被拒绝：${err.data?.reason || err.message}；未达标片 ${JSON.stringify(err.data?.unsatisfied || [])}`, 'err');
    } else {
      setStatus(`导出失败：${err.message}`, 'err');
    }
  }
}

// ---------- results panels ----------
function renderResults(data) {
  const s = data.summary;
  const allMet = data.verification.patches.every((p) => p.tolerancesMet);
  $('summary').innerHTML = `
    <div class="card"><div class="k">全局顶点（去重后）</div><div class="v">${s.globalVertices}</div></div>
    <div class="card"><div class="k">三角单元</div><div class="v">${s.totalTriangles}</div></div>
    <div class="card"><div class="k">顶点硬上限</div><div class="v">${s.vertexCap}</div></div>
    <div class="card"><div class="k">预算模式</div><div class="v" style="font-size:14px">${s.mode}</div></div>
    <div class="card"><div class="k">导出状态</div><div class="v ${s.rejected ? 'bad' : 'good'}" style="font-size:15px">${s.rejected ? '已拒绝' : '接受'}</div></div>
    <div class="card"><div class="k">容差达标</div><div class="v ${allMet ? 'good' : 'bad'}" style="font-size:15px">${allMet ? '全部' : '有未达标'}</div></div>
  `;

  $('patchTable').innerHTML = table(
    ['面片', '顶点', '单元', '误差种类', '采样距离(实)', '保证距离界', '采样法角(实)', '保证法角界', '松弛', '未达标'],
    data.patches.map((p) => {
      return [
        p.name,
        p.vertices,
        p.triangles,
        p.guaranteed ? '<span class="tag guar">可保证</span>' : '<span class="tag est">采样估计</span>',
        fmt(p.sampleMax.dist),
        fmt(p.boundMax.dist),
        `${deg(p.sampleMax.angle)}°`,
        `${deg(p.boundMax.angle)}°`,
        `${p.relax}×`,
        p.unsatisfied.length
          ? `<span class="tag bad">${p.unsatisfied.length}</span>`
          : '<span class="tag good">0</span>',
      ];
    })
  );

  $('weldTable').innerHTML = data.verification.cracks.length
    ? table(
      ['焊缝', '采样数', '最大裂口', '顶点错配', '状态'],
      data.verification.cracks.map((c) => [
        c.weld, c.samples, fmt(c.maxGap), c.mismatchedVertices,
        c.crackFree ? '<span class="tag good">无裂缝</span>' : '<span class="tag bad">有裂缝</span>',
      ])
    ) + `<p class="desc">非流形边：${data.verification.nonManifoldEdgeCount}，悬空顶点：${data.verification.hangingVertices.length}，边界边：${data.verification.boundaryEdgeCount}</p>`
    : '<p class="desc">该场景无焊接边（如刻意分开的唇边保持独立顶点）。</p>';

  const reasons = data.patches.flatMap((p) =>
    Object.entries(p.reasonCounts).map(([code, n]) => `<span class="pill">${p.name} · ${reasonText(code)} × ${n}</span>`)
  );
  const events = (data.events || []).slice(0, 12).map((e) =>
    `<div class="event"><b>${e.type}</b> ${e.patch || e.weld || ''} ${e.reason ? '— ' + e.reason : ''}</div>`
  ).join('');
  $('reasonPanel').innerHTML =
    (reasons.length ? `<div>${reasons.join('')}</div>` : '<p class="desc">无细化（零曲率片仅保留种子单元）。</p>') +
    (events ? `<h3>关键事件</h3>${events}` : '');

  const overlay = $('overlay');
  overlay.classList.remove('hidden');
  overlay.innerHTML = `
    <b>${$('scene').selectedOptions[0].text}</b><br>
    顶点 ${s.globalVertices} · 单元 ${s.totalTriangles}<br>
    焊缝 ${data.welds.length} · 裂口 ${data.verification.cracks.every((c) => c.crackFree) ? '无' : '有'}<br>
    <span style="color:var(--${allMet ? 'good' : 'bad'})">${allMet ? '全部达标' : '存在未达标区域'}</span>
  `;
}

function reasonText(code) {
  return {
    'pos-bound': '位置保证界超差',
    'normal-bound': '法线保证界超差',
    'pos-estimate': '位置采样/估计超差',
    'normal-estimate': '法线采样/估计超差',
    'singular-node': '重复节点强制细化',
    'depth-cap': '达到深度上限',
    'budget': '顶点预算阻断',
    'budget-weld': '焊缝预算阻断',
  }[code] || code;
}

function table(headers, rows) {
  return `<table><thead><tr>${headers.map((h) => `<th>${h}</th>`).join('')}</tr></thead><tbody>` +
    rows.map((r) => `<tr>${r.map((c) => `<td>${c}</td>`).join('')}</tr>`).join('') +
    '</tbody></table>';
}
const fmt = (x) => (typeof x === 'number' ? x.toExponential(2) : x);
const deg = (rad) => (rad * 180 / Math.PI).toFixed(2);

// ---------- init ----------
async function init() {
  const res = await fetch('/api/scenes');
  const { scenes: list } = await res.json();
  sceneLib = list;
  $('scene').innerHTML = list.map((s) => `<option value="${s.name}">${s.name}</option>`).join('');
  $('scene').addEventListener('change', () => {
    const meta = sceneLib.find((s) => s.name === $('scene').value);
    $('scene-desc').textContent = meta.description;
    applyDefaults();
  });
  const defaults = {
    plane: [0.01, 10, 5000], bump: [0.005, 8, 20000], spike: [0.004, 10, 25000],
    polar: [0.01, 12, 30000], band: [0.01, 15, 20000], weldedPair: [0.02, 12, 10000],
    budgetWeld: [0.02, 12, 9000], lips: [0.01, 10, 4000], frame: [0.01, 8, 30000],
  };
  function applyDefaults() {
    const d = defaults[$('scene').value] || [0.01, 8, 10000];
    $('posTol').value = d[0];
    $('normalTol').value = d[1];
    $('vertexCap').value = d[2];
    $('mode').value = 'coarsen-remote';
    tolDirty = false;
  }
  window.applyDefaults = applyDefaults;
  applyDefaults();
  $('posTol').addEventListener('input', () => { tolDirty = true; });
  $('normalTol').addEventListener('input', () => { tolDirty = true; });
  $('run').addEventListener('click', runMesh);
  $('exportObj').addEventListener('click', () => exportMesh('obj'));
  $('exportStl').addEventListener('click', () => exportMesh('stl'));
  ['showWire', 'showUnsat', 'showSeams'].forEach((id) =>
    $(id).addEventListener('change', () => currentData && renderData(currentData)));
  $('scene-desc').textContent = sceneLib[0].description;
}
init();
