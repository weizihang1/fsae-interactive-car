// 风洞可视化模块
//
// 重要说明：这里的"风"不是 CFD 计算结果，而是一个简化流场模型，由四项叠加而成：
//   1) 均匀来流 + 车体阻塞（球体偶极子的解析解），让流线正确地绕开车身；
//   2) 前后翼的环量效应：翼前上洗、翼后下洗（下洗强度按展长与离翼距离衰减）；
//   3) 底板/扩散器：车下加速、扩散器出口上洗；
//   4) 轮胎尾流：轮后减速并叠加小幅扰动。
// 它的用途是把流动结构讲清楚（哪里加速、哪里下洗、尾流在哪）。
// 真正的气动力数值属于 S3 阶段：在 CI 上用 OpenFOAM 跑，结果再回填到这里。
import * as THREE from 'three';

const STOPS = [
  [0.00, 0.05, 0.09, 0.34],
  [0.22, 0.08, 0.50, 0.86],
  [0.45, 0.18, 0.86, 0.62],
  [0.70, 0.96, 0.86, 0.22],
  [1.00, 0.95, 0.24, 0.18],
];

export function speedColor(t, target) {
  t = Math.min(1, Math.max(0, t));
  let i = 0;
  while (i < STOPS.length - 2 && t > STOPS[i + 1][0]) i++;
  const a = STOPS[i];
  const b = STOPS[i + 1];
  const f = (t - a[0]) / Math.max(1e-6, b[0] - a[0]);
  target.setRGB(a[1] + (b[1] - a[1]) * f,
                a[2] + (b[2] - a[2]) * f,
                a[3] + (b[3] - a[3]) * f);
  return target;
}

export function createFlow(wheelCenters) {
  const state = { vinf: 18, yaw: 0 };
  const mid = new THREE.Vector3(0, 0, 0.45);
  const A = 0.88;
  const WINGS = [
    { x: 1.02, z: 0.15, span: 1.30, strength: 0.46 },
    { x: -1.05, z: 0.95, span: 1.10, strength: 0.36 },
  ];
  const uInf = new THREE.Vector3();
  const rel = new THREE.Vector3();
  const rHat = new THREE.Vector3();

  function evalAt(p, out) {
    const yaw = THREE.MathUtils.degToRad(state.yaw);
    uInf.set(-state.vinf * Math.cos(yaw), state.vinf * Math.sin(yaw), 0);
    out.copy(uInf);

    // 1) 车体阻塞：球体偶极子 u = U + (a³/2)[U/r³ - 3(U·r̂)r̂/r⁴]
    rel.copy(p).sub(mid);
    const r = rel.length();
    if (r > 1e-3) {
      rHat.copy(rel).divideScalar(r);
      const k1 = (A * A * A) / (2 * r * r * r);
      const k2 = -3 * (A * A * A) * uInf.dot(rHat) / (2 * r * r * r * r);
      out.addScaledVector(uInf, k1);
      out.addScaledVector(rHat, k2);
    }

    // 2) 前后翼的环量效应
    for (let i = 0; i < WINGS.length; i++) {
      const w = WINGS[i];
      const dx = p.x - w.x;
      const dy = Math.abs(p.y);
      const dz = p.z - w.z;
      const spanFactor = 1 - Math.max(0, dy - w.span * 0.5) / 0.55;
      if (spanFactor <= 0) continue;
      const decayZ = Math.exp(-Math.abs(dz) / 0.42);
      const trail = dx < 0 ? Math.exp(dx / 0.85) : Math.exp(-dx / 0.32);
      const sign = dx < 0 ? -1.0 : 0.45;
      out.z += sign * w.strength * state.vinf * 0.17 * spanFactor * decayZ * trail;
    }

    // 3) 底板与扩散器
    if (p.z < 0.26 && Math.abs(p.y) < 0.42 && p.x < 0.60 && p.x > -1.30) {
      const tz = Math.min(1, Math.max(0, (0.26 - p.z) / 0.26));
      const tx = Math.min(1, Math.max(0, (0.60 - p.x) / 1.90));
      out.x *= 1 + 0.50 * tz * tx;
      out.y *= 1 + 0.35 * tz * tx;
      if (p.x < -0.40) out.z += 0.075 * tz * tx * state.vinf;
    }

    // 4) 轮胎尾流
    for (let i = 0; i < wheelCenters.length; i++) {
      const w = wheelCenters[i];
      if (p.x >= w.x) continue;
      const dxw = p.x - w.x;
      const dy2 = p.y - w.y;
      const dz2 = p.z - w.z;
      const d = Math.sqrt(dxw * dxw * 0.35 + dy2 * dy2 + dz2 * dz2);
      if (d >= 0.55) continue;
      const f = (1 - d / 0.55) * Math.exp(dxw / 0.55);
      out.x *= 1 - 0.40 * f;
      out.z += 0.045 * f * state.vinf * Math.sin(p.y * 11 + p.z * 7);
    }
    return out;
  }

  return {
    state: state,
    evalAt: evalAt,
    setSpeed: function (s) { state.vinf = s; },
    setYaw: function (y) { state.yaw = y; },
  };
}

export function buildStreamlines(flow, wheelCenters, opts) {
  opts = opts || {};
  const ny = opts.ny || 11;
  const nz = opts.nz || 7;
  const steps = opts.steps || 200;
  const h = opts.h || 0.032;
  const x0 = opts.x0 || 1.95;
  const yValues = opts.yValues || null;
  const zValues = opts.zValues || null;
  const nyCount = yValues ? yValues.length : ny;
  const nzCount = zValues ? zValues.length : nz;

  const positions = [];
  const colors = [];
  const speeds = [];
  const paths = [];
  const col = new THREE.Color();
  const p = new THREE.Vector3();
  const k1 = new THREE.Vector3();
  const k2 = new THREE.Vector3();
  const pm = new THREE.Vector3();

  function integrate(seed) {
    p.copy(seed);
    const pts = [];
    const spd = [];
    for (let i = 0; i < steps; i++) {
      const s = flow.evalAt(p, k1).length();
      if (s < 1e-5) break;
      pts.push(p.x, p.y, p.z);
      spd.push(s);
      k1.divideScalar(s);
      pm.copy(p).addScaledVector(k1, h * 0.5);
      flow.evalAt(pm, k2);
      const s2 = k2.length();
      if (s2 < 1e-5) break;
      k2.divideScalar(s2);
      p.addScaledVector(k2, h);
      if (p.x < -2.05 || p.z < 0.008 || Math.abs(p.y) > 1.75) break;
    }
    return { pts: pts, spd: spd };
  }

  for (let iy = 0; iy < nyCount; iy++) {
    const y = yValues ? yValues[iy] : (-0.98 + (1.96 * iy) / (ny - 1));
    for (let iz = 0; iz < nzCount; iz++) {
      const z = zValues ? zValues[iz] : (0.05 + (1.20 * iz) / (nz - 1));
      const path = integrate(new THREE.Vector3(x0, y, z));
      const n = path.spd.length;
      if (n < 5) continue;
      paths.push(path);
      const pc = new Float32Array(n * 3);
      for (let k = 0; k < n; k++) {
        speedColor(Math.min(1, path.spd[k] / (flow.state.vinf * 2.0)), col);
        pc[k * 3] = col.r;
        pc[k * 3 + 1] = col.g;
        pc[k * 3 + 2] = col.b;
      }
      for (let k = 0; k < n - 1; k++) {
        const a = k * 3;
        const b = (k + 1) * 3;
        positions.push(path.pts[a], path.pts[a + 1], path.pts[a + 2],
                       path.pts[b], path.pts[b + 1], path.pts[b + 2]);
        colors.push(pc[a], pc[a + 1], pc[a + 2], pc[b], pc[b + 1], pc[b + 2]);
        speeds.push(path.spd[k], path.spd[k + 1]);
      }
    }
  }

  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geom.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  const mat = new THREE.LineBasicMaterial({
    vertexColors: true, transparent: true, opacity: 0.62,
    blending: THREE.NormalBlending, depthWrite: false,
  });
  const lines = new THREE.LineSegments(geom, mat);
  lines.renderOrder = 3;

  // 为粒子准备弧长表
  for (let i = 0; i < paths.length; i++) {
    const pts = paths[i].pts;
    const n = pts.length / 3;
    const cum = new Float32Array(n);
    let acc = 0;
    for (let k = 1; k < n; k++) {
      const dx = pts[k * 3] - pts[(k - 1) * 3];
      const dy = pts[k * 3 + 1] - pts[(k - 1) * 3 + 1];
      const dz = pts[k * 3 + 2] - pts[(k - 1) * 3 + 2];
      acc += Math.sqrt(dx * dx + dy * dy + dz * dz);
      cum[k] = acc;
    }
    paths[i].cum = cum;
    paths[i].total = acc || 1;
  }
  return { lines: lines, paths: paths, speeds: speeds };
}

export function recolorStreamlines(lines, speeds, vinf) {
  const attr = lines.geometry.attributes.color;
  const col = new THREE.Color();
  const n = speeds.length;
  for (let i = 0; i < n; i++) {
    speedColor(Math.min(1, speeds[i] / Math.max(1, vinf * 2.0)), col);
    attr.setXYZ(i, col.r, col.g, col.b);
  }
  attr.needsUpdate = true;
}

export function createParticles(paths, count) {
  const positions = new Float32Array(count * 3);
  const colors = new Float32Array(count * 3);
  const assign = [];
  const col = new THREE.Color();
  const usable = paths.filter(function (p) { return p.spd.length > 6; });

  for (let i = 0; i < count; i++) {
    const idx = i % usable.length;
    assign.push({
      path: usable[idx],
      s: Math.random() * usable[idx].total,
      vi: 1,
    });
  }

  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  const mat = new THREE.PointsMaterial({
    size: 0.019, vertexColors: true, transparent: true, opacity: 0.95,
    blending: THREE.AdditiveBlending, depthWrite: false, sizeAttenuation: true,
  });
  const points = new THREE.Points(geom, mat);
  points.renderOrder = 4;

  function sample(path, s) {
    const cum = path.cum;
    let i = Math.max(1, Math.min(cum.length - 1, path.vi));
    while (i < cum.length - 1 && cum[i] < s) i++;
    while (i > 1 && cum[i - 1] > s) i--;
    path.vi = i;
    const c0 = cum[i - 1];
    const c1 = cum[i];
    const f = c1 > c0 ? (s - c0) / (c1 - c0) : 0;
    const a = (i - 1) * 3;
    const b = i * 3;
    const pts = path.pts;
    return [
      pts[a] + (pts[b] - pts[a]) * f,
      pts[a + 1] + (pts[b + 1] - pts[a + 1]) * f,
      pts[a + 2] + (pts[b + 2] - pts[a + 2]) * f,
      path.spd[i - 1] + (path.spd[i] - path.spd[i - 1]) * f,
    ];
  }

  function update(dt, vinf) {
    for (let i = 0; i < count; i++) {
      const it = assign[i];
      const sp = Math.max(0.15, it.speed === undefined ? vinf : it.speed);
      it.s += vinf * dt * 0.24;
      if (it.s >= it.path.total) {
        it.s = 0;
        it.path.vi = 1;
      }
      const q = sample(it.path, it.s);
      positions[i * 3] = q[0];
      positions[i * 3 + 1] = q[1];
      positions[i * 3 + 2] = q[2];
      speedColor(Math.min(1, q[3] / Math.max(1, vinf * 2.0)), col);
      colors[i * 3] = col.r;
      colors[i * 3 + 1] = col.g;
      colors[i * 3 + 2] = col.b;
    }
    geom.attributes.position.needsUpdate = true;
    geom.attributes.color.needsUpdate = true;
  }

  return { points: points, update: update };
}

// 表面压力系数示意：Cp ≈ 1 - (|u|/U∞)²（不可压势流的伯努利关系）
export function surfacePressure(flow, meshes, range) {
  const pos = new THREE.Vector3();
  const out = new THREE.Vector3();
  const col = new THREE.Color();
  const lo = range ? range[0] : -1.6;
  const hi = range ? range[1] : 1.0;
  for (let mi = 0; mi < meshes.length; mi++) {
    const mesh = meshes[mi];
    const g = mesh.geometry;
    const P = g.attributes.position;
    const N = g.attributes.normal;
    if (!N) continue;
    const total = P.count;
    const colors = new Float32Array(total * 3);
    const wm = mesh.matrixWorld;
    for (let i = 0; i < total; i++) {
      pos.set(P.getX(i), P.getY(i), P.getZ(i)).applyMatrix4(wm);
      const nx = N.getX(i);
      const ny = N.getY(i);
      const nz = N.getZ(i);
      const q = new THREE.Vector3(pos.x + nx * 0.03, pos.y + ny * 0.03, pos.z + nz * 0.03);
      const sp = flow.evalAt(q, out).length();
      const ratio = sp / Math.max(1e-3, flow.state.vinf);
      const cp = 1 - ratio * ratio;
      const t = (cp - lo) / (hi - lo);
      speedColor(t, col);
      colors[i * 3] = col.r;
      colors[i * 3 + 1] = col.g;
      colors[i * 3 + 2] = col.b;
    }
    g.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
    for (let k = 0; k < mats.length; k++) {
      mats[k].vertexColors = true;
      mats[k].needsUpdate = true;
    }
  }
}

export function clearSurfacePressure(meshes) {
  for (let i = 0; i < meshes.length; i++) {
    const mesh = meshes[i];
    mesh.geometry.deleteAttribute('color');
    const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
    for (let k = 0; k < mats.length; k++) {
      mats[k].vertexColors = false;
      mats[k].needsUpdate = true;
    }
  }
}
