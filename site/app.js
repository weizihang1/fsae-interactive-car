import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import {
  createFlow, buildStreamlines, createParticles, recolorStreamlines,
  surfacePressure, clearSurfacePressure,
} from './tunnel.js';

const MANIFEST = window.__CAR_MANIFEST__;
const GLB_BASE64 = window.__CAR_GLB__;
// 支持的深链参数：#mode=whole|explode|tunnel|drive  #shell=off  #view=side|front  #debug
const HASH = window.location.hash || '';

const SYSTEM_COLORS = {
  frame: '#7f8ea6', susp_front: '#4ea1ff', susp_rear: '#38d0c0', wheels: '#c8a24a',
  steering: '#b07cff', brakes: '#ff7043', powertrain: '#4bd99a', aero: '#ff5c6e',
  aero_ground: '#ff9f43', body: '#ffd166', cockpit: '#e0e6f0',
};

document.getElementById('sub').textContent =
  MANIFEST.totals.parts + ' 个零件 · ' + MANIFEST.totals.mass_kg + ' kg · 长 '
  + MANIFEST.totals.length_m + ' m';

// ------------------------------------------------------------------ 场景
const canvas = document.getElementById('cv');
const renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0d12);
scene.fog = new THREE.Fog(0x0b0d12, 10, 26);

const camera = new THREE.PerspectiveCamera(38, window.innerWidth / window.innerHeight, 0.05, 140);
camera.position.set(2.6, 1.9, -2.6);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.075;
controls.target.set(0, 0.42, 0);
controls.minDistance = 0.9;
controls.maxDistance = 14;
// 解除水平线限制：允许 360° 环绕，包括从车底往上看
controls.minPolarAngle = 0.02;
controls.maxPolarAngle = Math.PI - 0.02;

scene.add(new THREE.HemisphereLight(0x9fc0ff, 0x141a24, 0.85));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.1);
keyLight.position.set(4, 6, 5);
scene.add(keyLight);
const fillLight = new THREE.DirectionalLight(0x88aaff, 0.7);
fillLight.position.set(-5, 3, -4);
scene.add(fillLight);
const rimLight = new THREE.DirectionalLight(0xff9aa5, 0.5);
rimLight.position.set(-3, 1.5, 6);
scene.add(rimLight);
// 车底补光：否则从下方看底盘时一片死黑，看不出结构
const underLight = new THREE.DirectionalLight(0xbcd4ff, 0.85);
underLight.position.set(0.6, -4.2, 1.6);
scene.add(underLight);

const grid = new THREE.GridHelper(14, 56, 0x2b3a55, 0x151d2b);
grid.material.transparent = true;
grid.material.opacity = 0.5;
scene.add(grid);

// 模型与流场都建立在"Z 轴朝上"的工程坐标系里，而 three.js 的世界是 Y 轴朝上。
// 用外层容器整体旋转 -90°，内部数据全部保持工程坐标不变，
// 这样建模、流场计算与显示三者不会互相污染。
const world = new THREE.Group();
world.rotation.x = -Math.PI / 2;
scene.add(world);

// 工程坐标 -> 世界坐标：(x, y, z) -> (x, z, -y)
function toWorld(v) {
  return new THREE.Vector3(v.x, v.z, -v.y);
}

// ------------------------------------------------------------------ 状态
const state = {
  mode: 'whole', explode: 0, shellOn: true, shellT: 1,
  selected: null, hidden: new Set(), isolated: null,
  cpOn: false, vinf: 18, yaw: 0,
};

const partMeshes = new Map();
const systemGroups = new Map();
const bodyMeshes = [];
let model = null;
const CAR_CENTER = new THREE.Vector3(0, 0, 0.45);

const tun = {
  group: new THREE.Group(), flow: null, lines: null,
  particles: null, speeds: null, built: false,
};
world.add(tun.group);
tun.group.visible = false;

function decodeBase64(b64) {
  const bin = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return bytes.buffer;
}

// ------------------------------------------------------------------ 载入模型
new GLTFLoader().parse(decodeBase64(GLB_BASE64), '', function (gltf) {
  model = gltf.scene;
  world.add(model);
  model.traverse(function (o) {
    if (!o.isMesh) return;
    // 节点名会被 GLTFLoader 清洗（点号被删掉），所以以 extras 里的 id 为准
    const id = o.userData.id || o.name;
    if (!id) return;
    o.material = o.material.clone();
    o.userData.partId = id;
    o.userData.base = o.position.clone();
    o.userData.system = o.userData.system
      || ((o.parent && o.parent.name) ? o.parent.name : '');
    if (o.userData.system === 'body') {
      const d = o.userData.base.clone().sub(CAR_CENTER);
      d.z += 0.30;
      if (d.lengthSq() < 1e-6) d.set(0, 0, 1);
      o.userData.shellDir = d.normalize().multiplyScalar(1.30);
      o.material.transparent = true;
      bodyMeshes.push(o);
    }
    partMeshes.set(id, o);
  });
  model.traverse(function (o) {
    // 注意：GLTFLoader 对"只有子节点、没有网格"的节点创建的是 Object3D 而非 Group，
    // 因此不能依赖 isGroup 判断，只能按名字匹配。
    if (o.name && !o.isMesh && MANIFEST.systems.some(function (s) { return s.id === o.name; })) {
      systemGroups.set(o.name, o);
    }
  });
  buildTunnel();
  renderTree();
  applyDeepLink();
  document.getElementById('loading').style.display = 'none';
}, function (err) {
  document.getElementById('loading').innerHTML =
    '<div style="color:#ff8a8a">模型载入失败：' + err + '</div>';
});

// ------------------------------------------------------------------ 选择
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
let downPos = null;

canvas.addEventListener('pointerdown', function (e) {
  downPos = { x: e.clientX, y: e.clientY };
});
canvas.addEventListener('pointerup', function (e) {
  if (!downPos) return;
  const moved = Math.hypot(e.clientX - downPos.x, e.clientY - downPos.y);
  downPos = null;
  if (moved > 4 || !model) return;
  pointer.x = (e.clientX / window.innerWidth) * 2 - 1;
  pointer.y = -(e.clientY / window.innerHeight) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  const hits = raycaster.intersectObject(model, true).filter(function (h) {
    return h.object.visible && h.object.isMesh;
  });
  selectPartMesh(hits.length ? hits[0].object : null);
});

function selectPartMesh(mesh) {
  if (state.selected) {
    const prev = state.selected;
    prev.material.emissive.setHex(prev.userData.baseEmissive || 0x000000);
  }
  state.selected = mesh || null;
  if (mesh) {
    mesh.userData.baseEmissive = mesh.userData.baseEmissiveFinal
      || mesh.material.emissive.getHex();
    if (mesh.userData.baseEmissiveFinal === undefined) {
      mesh.userData.baseEmissiveFinal = mesh.userData.baseEmissive;
    }
    mesh.material.emissive.setHex(0x8a5a00);
  }
  renderCard(mesh ? mesh.userData.partId : null);
}

// ------------------------------------------------------------------ 系统树
const tree = document.getElementById('tree');
const openSystems = new Set(['frame', 'body']);

function renderTree() {
  const maxMass = Math.max.apply(null, MANIFEST.systems.map(function (s) { return s.mass_kg; }));
  let html = '';
  for (const sys of MANIFEST.systems) {
    const color = SYSTEM_COLORS[sys.id] || '#8899aa';
    const open = openSystems.has(sys.id) ? ' open' : '';
    const dim = state.hidden.has(sys.id) ? ' opacity:.4' : '';
    html += '<div class="sys' + open + '" style="' + (dim ? 'opacity:.4' : '') +
      '" data-sys="' + sys.id + '">' +
      '<div class="row" data-toggle="' + sys.id + '">' +
      '<span class="dot" style="background:' + color + '"></span>' +
      '<span class="sysname">' + sys.label + '</span>' +
      '<span class="sysmass">' + sys.mass_kg + ' kg · ' + sys.count + '件</span>' +
      '</div>' +
      '<div class="bar"><i style="width:' + (sys.mass_kg / maxMass * 100).toFixed(1) +
      '%;background:' + color + '"></i></div>' +
      '<div class="parts">';
    for (const p of sys.parts) {
      html += '<div class="part" data-part="' + p.id + '">' + p.name +
        '<span class="m">' + p.mass_kg + ' kg</span></div>';
    }
    html += '</div></div>';
  }
  tree.innerHTML = html;

  tree.querySelectorAll('[data-toggle]').forEach(function (el) {
    el.addEventListener('click', function (ev) {
      const id = el.dataset.toggle;
      if (ev.altKey || ev.shiftKey) {
        toggleVisibility(id);
        return;
      }
      if (openSystems.has(id)) openSystems.delete(id); else openSystems.add(id);
      renderTree();
    });
    el.addEventListener('contextmenu', function (ev) {
      ev.preventDefault();
      toggleVisibility(el.dataset.toggle);
    });
  });
  tree.querySelectorAll('[data-part]').forEach(function (el) {
    el.addEventListener('click', function () {
      const mesh = partMeshes.get(el.dataset.part);
      if (!mesh) return;
      selectPartMesh(mesh);
      controls.target.copy(toWorld(mesh.userData.base));
    });
  });
}

function toggleVisibility(systemId) {
  if (state.hidden.has(systemId)) state.hidden.delete(systemId);
  else state.hidden.add(systemId);
  applyVisibility();
  renderTree();
}

function applyVisibility() {
  systemGroups.forEach(function (group, id) {
    group.visible = !state.hidden.has(id);
  });
}

// 隔离显示某个系统。传 null 表示恢复显示全部系统。
function isolateSystem(systemId) {
  state.hidden.clear();
  state.isolated = null;
  if (systemId) {
    MANIFEST.systems.forEach(function (s) {
      if (s.id !== systemId) state.hidden.add(s.id);
    });
    state.isolated = systemId;
  }
  applyVisibility();
  renderTree();
}

// ------------------------------------------------------------------ 信息卡
function renderCard(partId) {
  const card = document.getElementById('card');
  if (!partId) {
    card.innerHTML = '<div style="color:var(--dim);font-size:12px">尚未选择零件。</div>';
    return;
  }
  let found = null;
  let sys = null;
  for (const s of MANIFEST.systems) {
    const p = s.parts.find(function (x) { return x.id === partId; });
    if (p) { found = p; sys = s; break; }
  }
  if (!found) {
    card.innerHTML = '<div style="color:var(--dim)">未找到该零件。</div>';
    return;
  }
  const c = found.center;
  const z = found.size;
  card.innerHTML =
    '<div class="name">' + found.name + '</div>' +
    '<div class="sysname2">' + sys.label + '</div>' +
    (found.note ? '<div class="note">' + found.note + '</div>' : '') +
    '<div class="kv"><span>件号</span><span>' + (found.part_no || '—') + '</span></div>' +
    (found.std
      ? '<div class="kv"><span>类型</span><span>标准件</span></div>'
        + '<div class="kv"><span>标准规格</span><span>' + (found.spec || '—') + '</span></div>'
      : '<div class="kv"><span>类型</span><span>自制件</span></div>') +
    '<div class="kv"><span>材料</span><span>' + found.material_label + '</span></div>' +
    '<div class="kv"><span>估算质量</span><span>' + found.mass_kg + ' kg</span></div>' +
    '<div class="kv"><span>几何体积</span><span>' + (found.volume_m3 * 1e6).toFixed(1) + ' cm³</span></div>' +
    '<div class="kv"><span>结构填充系数</span><span>' + (found.fill * 100).toFixed(0) + '%</span></div>' +
    '<div class="kv"><span>外形尺寸</span><span>' +
      (z[0] * 1000).toFixed(0) + '×' + (z[1] * 1000).toFixed(0) + '×' +
      (z[2] * 1000).toFixed(0) + ' mm</span></div>' +
    '<div class="kv"><span>位置 (X,Y,Z)</span><span>' +
      c.map(function (v) { return (v * 1000).toFixed(0); }).join(', ') + ' mm</span></div>' +
    '<a class="btn" href="stl/' + found.id + '.stl" download>下载该零件 STL</a>' +
    '<div class="btn ghost" id="isoBtn">单独显示此零件</div>';
  const iso = document.getElementById('isoBtn');
  if (iso) {
    iso.textContent = state.isolated === sys.id ? '显示全部系统' : '单独显示此零件';
    iso.addEventListener('click', function () {
      isolateSystem(state.isolated === sys.id ? null : sys.id);
      renderCard(partId);
    });
  }
}

// ------------------------------------------------------------------ 模式
const exSlider = document.getElementById('ex');
const barBase = document.getElementById('barBase');
const barExplode = document.getElementById('barExplode');
const barTunnel = document.getElementById('barTunnel');
const legend = document.getElementById('legend');

document.querySelectorAll('.tab').forEach(function (tab) {
  tab.addEventListener('click', function () { setMode(tab.dataset.mode); });
});

function setMode(next) {
  state.mode = next;
  document.querySelectorAll('.tab').forEach(function (t) {
    t.classList.toggle('on', t.dataset.mode === next);
  });
  barExplode.style.display = next === 'explode' ? 'flex' : 'none';
  barTunnel.style.display = next === 'tunnel' ? 'flex' : 'none';
  barBase.style.display = (next === 'whole' || next === 'explode') ? 'flex' : 'none';
  legend.style.display = next === 'tunnel' ? 'block' : 'none';
  tun.group.visible = next === 'tunnel';
  if (next !== 'explode') {
    state.explode = 0;
    exSlider.value = 0;
    applyExplode();
  }
  if (next === 'tunnel') {
    document.getElementById('legendTxt').innerHTML =
      '流线颜色代表当地流速（低压区流速高）。本模式是<b>简化流场示意</b>：' +
      '由车体阻塞、前后翼下洗、底板加速与轮胎尾流叠加而成，用来讲清流动结构；' +
      '<b>不是 CFD 计算结果</b>，气动力数值需在 CI 上跑 OpenFOAM 后回填。';
  }
  if (next === 'drive') {
    document.getElementById('card').innerHTML =
      '<div class="name">场景驾驶</div>' +
      '<div style="color:var(--dim);font-size:12px;margin-top:8px">' +
      '此模式尚未接入。整车几何与零件层级已经就绪，下一步接入赛道场景、' +
      '车辆物理与悬架行程联动。</div>';
  }
}

exSlider.addEventListener('input', function () {
  state.explode = exSlider.value / 100;
  applyExplode();
});

function applyExplode() {
  document.getElementById('exv').textContent = Math.round(state.explode * 100) + '%';
  const k = 1 + state.explode * 2.4;
  partMeshes.forEach(function (mesh) {
    const base = mesh.userData.base;
    if (!base || mesh.userData.system === 'body') return;
    mesh.position.set(base.x * k, base.y * k, base.z * k);
  });
}

// ------------------------------------------------------------------ 视角预设
// 世界坐标 = (x, z, -y)，即工程坐标的 Z 轴映射到世界的 Y 轴。
const VIEWS = {
  iso:     { pos: [2.60, 1.90, -2.60], tgt: [0.00, 0.42, 0.00] },
  front:   { pos: [5.20, 0.75, 0.02],  tgt: [0.00, 0.55, 0.00] },
  side:    { pos: [0.02, 0.75, -5.20], tgt: [0.02, 0.55, 0.00] },
  top:     { pos: [0.02, 5.40, -0.45], tgt: [0.02, 0.40, 0.00] },
  bottom:  { pos: [0.02, -3.40, -0.45], tgt: [0.02, 0.18, 0.00] },
  chassis: { pos: [2.10, -1.90, -1.70], tgt: [0.15, 0.16, 0.00] },
};

function setView(name) {
  const v = VIEWS[name];
  if (!v) return;
  camera.position.set(v.pos[0], v.pos[1], v.pos[2]);
  controls.target.set(v.tgt[0], v.tgt[1], v.tgt[2]);
  controls.update();
  if (name === 'chassis') {
    // 底盘气动视角：直接进风洞模式并打开压力云图，一步看到车底低压区
    if (state.mode !== 'tunnel') setMode('tunnel');
    if (!state.cpOn) {
      state.cpOn = true;
      document.getElementById('cp').checked = true;
      applyCp();
    }
  }
  document.querySelectorAll('.vb').forEach(function (b) {
    b.classList.toggle('on', b.dataset.view === name);
  });
}

document.querySelectorAll('.vb').forEach(function (b) {
  b.addEventListener('click', function () { setView(b.dataset.view); });
});

// ------------------------------------------------------------------ 车壳
let shellDirty = true;
const shellBtn = document.getElementById('shellBtn');

shellBtn.addEventListener('click', function () {
  state.shellOn = !state.shellOn;
  shellDirty = true;
  document.getElementById('shellLabel').textContent = state.shellOn ? '拆下车壳' : '装回车壳';
  shellBtn.classList.toggle('off', !state.shellOn);
});

document.getElementById('showAllBtn').addEventListener('click', function () {
  isolateSystem(null);
  state.selected = null;
  renderCard(null);
});

function applyShell() {
  const t = state.shellT;
  for (const m of bodyMeshes) {
    const b = m.userData.base;
    const d = m.userData.shellDir;
    const k = 1 - t;
    m.position.set(b.x + d.x * k, b.y + d.y * k, b.z + d.z * k);
    m.material.opacity = t;
    m.material.transparent = t < 0.995;
    m.visible = t > 0.03;
  }
}

function updateShell(dt) {
  const target = state.shellOn ? 1 : 0;
  if (Math.abs(state.shellT - target) < 0.002) {
    if (!shellDirty) return;
    state.shellT = target;
    shellDirty = false;
  } else {
    state.shellT += (target - state.shellT) * Math.min(1, dt * 4.5);
  }
  applyShell();
}

// ------------------------------------------------------------------ 风洞
function wheelCenters() {
  const out = [];
  const sys = MANIFEST.systems.find(function (s) { return s.id === 'wheels'; });
  if (!sys) return out;
  for (const p of sys.parts) {
    if (p.id.indexOf('wheel.hub') === 0) {
      out.push(new THREE.Vector3(p.center[0], p.center[1], p.center[2]));
    }
  }
  return out;
}

function disposeTunnelMeshes() {
  if (tun.lines) {
    tun.group.remove(tun.lines);
    tun.lines.geometry.dispose();
    tun.lines.material.dispose();
    tun.lines = null;
  }
  if (tun.particles) {
    tun.group.remove(tun.particles.points);
    tun.particles.points.geometry.dispose();
    tun.particles.points.material.dispose();
    tun.particles = null;
  }
}

function buildTunnel() {
  disposeTunnelMeshes();
  const wheels = wheelCenters();
  tun.flow = createFlow(wheels);
  tun.flow.setSpeed(state.vinf);
  tun.flow.setYaw(state.yaw);
  // 近地面区域加密播种，否则车底那两条通道里几乎没有流线，看不到加速效果
  const built = buildStreamlines(tun.flow, wheels, {
    yValues: [-0.95, -0.80, -0.62, -0.46, -0.33, -0.22, -0.13, -0.05,
              0.05, 0.13, 0.22, 0.33, 0.46, 0.62, 0.80, 0.95],
    zValues: [0.030, 0.075, 0.130, 0.200, 0.290, 0.400, 0.540, 0.700, 0.880, 1.070, 1.250],
    steps: 180,
  });
  tun.lines = built.lines;
  tun.speeds = built.speeds;
  tun.group.add(tun.lines);
  tun.particles = createParticles(built.paths, 1500);
  tun.group.add(tun.particles.points);
  tun.built = true;
  tun.group.visible = state.mode === 'tunnel';
}

function applyCp() {
  if (!model || !tun.flow) return;
  const meshes = [];
  model.traverse(function (o) { if (o.isMesh && o.visible) meshes.push(o); });
  if (state.cpOn) surfacePressure(tun.flow, meshes);
  else clearSurfacePressure(meshes);
}

document.getElementById('vwind').addEventListener('input', function (e) {
  state.vinf = +e.target.value;
  document.getElementById('vwindv').textContent = state.vinf + ' m/s';
  if (!tun.built) return;
  tun.flow.setSpeed(state.vinf);
  recolorStreamlines(tun.lines, tun.speeds, state.vinf);
  if (state.cpOn) applyCp();
});

let yawTimer = null;
function rebuildForYaw() {
  buildTunnel();
  if (state.cpOn) applyCp();
}
document.getElementById('yaw').addEventListener('input', function (e) {
  state.yaw = +e.target.value;
  document.getElementById('yawv').textContent = state.yaw.toFixed(1) + '°';
  if (yawTimer) return;
  yawTimer = setTimeout(function () { yawTimer = null; rebuildForYaw(); }, 90);
});
document.getElementById('yaw').addEventListener('change', function () {
  if (yawTimer) { clearTimeout(yawTimer); yawTimer = null; }
  rebuildForYaw();
});

document.getElementById('cp').addEventListener('change', function (e) {
  state.cpOn = e.target.checked;
  applyCp();
});

// ------------------------------------------------------------------ 主循环
window.addEventListener('resize', function () {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

let last = performance.now();
function loop(now) {
  requestAnimationFrame(loop);
  const dt = Math.min(0.05, (now - last) / 1000);
  last = now;
  controls.update();
  updateShell(dt);
  if (state.mode === 'tunnel' && tun.built) {
    tun.particles.update(dt, state.vinf);
  }
  // 相机低于地面时收起地面网格，否则网格线会糊在车底上
  grid.visible = camera.position.y > -0.02;
  renderer.render(scene, camera);
}
requestAnimationFrame(loop);

// ------------------------------------------------------------------ 诊断开关
// 仅在网址末尾加 #debug 时生效：把关键状态写进页面标题，便于无人值守排查
// （可用无头浏览器 --dump-dom 直接读到），正常访问时完全不影响页面。
function applyDeepLink() {
  const viewMatch = HASH.match(/view=(iso|front|side|top|bottom|chassis)/);
  if (viewMatch) setView(viewMatch[1]);
  const modeMatch = HASH.match(/mode=(whole|explode|tunnel|drive)/);
  if (modeMatch) setMode(modeMatch[1]);
  if (HASH.indexOf('shell=off') >= 0) {
    state.shellOn = false;
    state.shellT = 0;
    shellDirty = true;
    applyShell();
    document.getElementById('shellLabel').textContent = '装回车壳';
    shellBtn.classList.add('off');
  }
  const selMatch = HASH.match(/select=([^&]+)/);
  if (selMatch) {
    const mesh = partMeshes.get(decodeURIComponent(selMatch[1]));
    if (mesh) {
      selectPartMesh(mesh);
      controls.target.copy(toWorld(mesh.userData.base));
    }
  }
  const zoomMatch = HASH.match(/zoom=([\d.]+)/);
  if (zoomMatch) {
    const dist = parseFloat(zoomMatch[1]);
    const dir = camera.position.clone().sub(controls.target).normalize();
    camera.position.copy(controls.target).addScaledVector(dir, dist);
    controls.update();
  }
  const isoMatch = HASH.match(/isolate=([a-z_]+)/);
  if (isoMatch) {
    isolateSystem(isoMatch[1]);
  }
}

if (HASH.indexOf('debug') >= 0) {
  const dbg = document.createElement('pre');
  dbg.style.cssText = 'position:fixed;left:8px;bottom:8px;z-index:99;margin:0;padding:10px 12px;'
    + 'background:#000c;border:1px solid #4ea1ff;border-radius:8px;color:#eaf2ff;'
    + 'font:11px/1.5 Consolas,monospace;white-space:pre-wrap;max-width:1200px';
  document.body.appendChild(dbg);
  setTimeout(function () {
    const box = new THREE.Box3().setFromObject(world);
    const fmt = function (v) {
      return v.toArray().map(function (x) { return x.toFixed(3); }).join(',');
    };
    const sample = partMeshes.get('wheel.hub.前左') || partMeshes.values().next().value;
    const keys = Array.from(partMeshes.keys()).slice(0, 4);
    const wheelInfo = [];
    partMeshes.forEach(function (m, id) {
      if (id.indexOf('wheel.hub') === 0) {
        wheelInfo.push(id + ' pos=' + fmt(m.position) + ' sys=' + m.userData.system);
      }
    });
    dbg.textContent = [
      'explode=' + state.explode + '  shellT=' + state.shellT.toFixed(3)
        + '  shellOn=' + state.shellOn,
      'cam=' + fmt(camera.position) + '  target=' + fmt(controls.target),
      'wheelBase=' + (sample ? fmt(sample.userData.base) : 'na')
        + '  wheelPos=' + (sample ? fmt(sample.position) : 'na')
        + '  sys=' + (sample ? sample.userData.system : 'na'),
      'worldBBox=' + fmt(box.min) + ' | ' + fmt(box.max),
      'bodyMeshes=' + bodyMeshes.length + '  partMeshes=' + partMeshes.size,
      'firstIds=' + keys.join(' | '),
      'wheels=' + wheelInfo.join('\n        '),
      'systemGroups=' + Array.from(systemGroups.keys()).join('/'),
      'modelScale=' + fmt(model.scale) + '  modelPos=' + fmt(model.position),
    ].join('\n');
  }, 3500);
}
