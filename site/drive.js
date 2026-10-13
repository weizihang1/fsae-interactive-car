// 驾驶模式：简化车辆动力学 + 赛道场景
//
// 重要说明：这是"游戏级"简化模型，**不是圈速仿真工具**。
// 车辆用 3 自由度自行车模型（纵向 / 侧向 / 横摆），轮胎为线性侧偏刚度 +
// 摩擦圆饱和，含纵向与侧向载荷转移，悬架行程按准静态算。
// 它能反映质量、轴距、质心高、轮胎抓地、驱动力与制动力对操控的影响，
// 但要做圈速结论必须另接准稳态（QSS）求解器。
import * as THREE from 'three';

const G = 9.81;

// ------------------------------------------------------------------ 赛道
function buildTrack(opts) {
  const N = 14;
  const ctrl = [];
  for (let i = 0; i < N; i++) {
    const a = (2 * Math.PI * i) / N;
    const rr = 46 + 15 * Math.sin(3 * a) + 8 * Math.cos(5 * a);
    ctrl.push(new THREE.Vector3(rr * Math.cos(a), rr * Math.sin(a), 0));
  }
  const curve = new THREE.CatmullRomCurve3(ctrl, true, 'centripetal', 0.5);
  const n = 420;
  const center = [];
  const tangent = [];
  for (let i = 0; i < n; i++) {
    const t = i / n;
    center.push(curve.getPointAt(t));
    tangent.push(curve.getTangentAt(t).normalize());
  }
  const halfW = opts.width / 2.0;
  const pos = [];
  const uvs = [];
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    const a = center[i];
    const b = center[j];
    const ta = tangent[i];
    const tb = tangent[j];
    const la = new THREE.Vector3(-ta.y, ta.x, 0).multiplyScalar(halfW);
    const lb = new THREE.Vector3(-tb.y, tb.x, 0).multiplyScalar(halfW);
    const p0 = a.clone().add(la);
    const p1 = a.clone().sub(la);
    const p2 = b.clone().add(lb);
    const p3 = b.clone().sub(lb);
    pos.push(p0.x, p0.y, 0, p1.x, p1.y, 0, p3.x, p3.y, 0);
    pos.push(p0.x, p0.y, 0, p3.x, p3.y, 0, p2.x, p2.y, 0);
    const v0 = i / n * 60.0;
    const v1 = (i + 1) / n * 60.0;
    uvs.push(0, v0, 1, v0, 1, v1, 0, v0, 1, v1, 0, v1);
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  geom.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
  geom.computeVertexNormals();
  const asphalt = new THREE.MeshStandardMaterial({
    color: 0x2a2d33, roughness: 0.92, metalness: 0.02, side: THREE.DoubleSide,
  });
  const surface = new THREE.Mesh(geom, asphalt);
  surface.position.z = 0.012;
  surface.receiveShadow = true;

  // 路肩：左右各一条细带，画出赛道边界
  const kerbPos = [];
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    for (const sgn of [1, -1]) {
      const a = center[i];
      const b = center[j];
      const ta = tangent[i];
      const tb = tangent[j];
      const na = new THREE.Vector3(-ta.y, ta.x, 0).multiplyScalar(sgn * halfW);
      const nb = new THREE.Vector3(-tb.y, tb.x, 0).multiplyScalar(sgn * halfW);
      const oa = na.clone().normalize().multiplyScalar(0.35);
      const ob = nb.clone().normalize().multiplyScalar(0.35);
      const p0 = a.clone().add(na);
      const p1 = a.clone().add(na).add(oa);
      const p2 = b.clone().add(nb);
      const p3 = b.clone().add(nb).add(ob);
      kerbPos.push(p0.x, p0.y, 0, p1.x, p1.y, 0, p3.x, p3.y, 0);
      kerbPos.push(p0.x, p0.y, 0, p3.x, p3.y, 0, p2.x, p2.y, 0);
    }
  }
  const kerbGeom = new THREE.BufferGeometry();
  kerbGeom.setAttribute('position', new THREE.Float32BufferAttribute(kerbPos, 3));
  kerbGeom.computeVertexNormals();
  const kerb = new THREE.Mesh(kerbGeom, new THREE.MeshStandardMaterial({
    color: 0xd8dde4, roughness: 0.75, metalness: 0.05, side: THREE.DoubleSide,
  }));
  kerb.position.z = 0.020;

  const group = new THREE.Group();
  group.add(surface);
  group.add(kerb);
  let total = 0;
  for (let i = 0; i < n; i++) {
    const a = center[i];
    const b = center[(i + 1) % n];
    total += Math.hypot(b.x - a.x, b.y - a.y);
  }
  return { group, center, tangent, halfW, n, length: total, segLen: total / n };
}

function nearestOnTrack(track, p) {
  let best = 1e9;
  let bestI = 0;
  const n = track.n;
  for (let i = 0; i < n; i++) {
    const c = track.center[i];
    const dx = p.x - c.x;
    const dy = p.y - c.y;
    const d = dx * dx + dy * dy;
    if (d < best) { best = d; bestI = i; }
  }
  return { dist: Math.sqrt(best), index: bestI };
}

// ------------------------------------------------------------------ 车辆
function createVehicle(manifest) {
  const dry = (manifest && manifest.totals) ? manifest.totals.mass_kg : 268;
  const m = dry + 78;                       // 加 78 kg 车手
  const L = 1.55;
  const lf = 0.47 * L;
  const lr = 0.53 * L;
  return {
    dryMass: dry,
    mass: m,
    wheelbase: L,
    lf, lr,
    cgHeight: 0.30,
    trackF: 1.22,
    trackR: 1.18,
    // 侧偏刚度（N/rad，整轴）：FSAE 光头胎的常见量级
    cf: 55000.0,
    cr: 62000.0,
    // 横摆转动惯量：用 m·a·b 估算
    Iz: m * lf * lr,
    mu: 1.45,
    dragArea: 1.15,                          // Cd·A
    rollRes: 0.016,
    power: 80000.0,                          // 电机峰值功率 W
    maxDrive: 2200.0,                        // 低速最大驱动力 N
    brakeBias: 0.62,                         // 前轴制动占比
    maxSteer: 24.0 * Math.PI / 180.0,
    steerRate: 2.6,                          // rad/s
    wheelRate: 28000.0,                      // 轮心刚度 N/m
    maxLateralG: 0,
  };
}

function createState(veh, track) {
  const p0 = track.center[0];
  const t0 = track.tangent[0];
  return {
    pos: new THREE.Vector2(p0.x, p0.y),
    heading: Math.atan2(t0.y, t0.x),
    vx: 0, vy: 0, r: 0,
    steer: 0,
    throttle: 0, brake: 0,
    wheelSpin: 0,
    travel: { fl: 0, fr: 0, rl: 0, rr: 0 },
    ax: 0, ay: 0,
    offTrack: false,
    lapTime: 0,
    lastIndex: 0,
    laps: 0,
    bestLap: 0,
    prevLineSide: -1,
    veh,
  };
}

function step(st, dt, input) {
  const v = st.veh;
  // 转向：向目标角以限速逼近
  const target = THREE.MathUtils.clamp(input.steer, -1, 1) * v.maxSteer;
  const ds = target - st.steer;
  const maxDs = v.steerRate * dt;
  st.steer += THREE.MathUtils.clamp(ds, -maxDs, maxDs);

  const speed = Math.abs(st.vx);
  const sinS = Math.sin(st.steer);

  // 载荷转移（纵向用上一帧的加速度，侧向用上一帧的）
  const Fz = v.mass * G;
  const dFzLong = v.mass * st.ax * v.cgHeight / v.wheelbase;
  const dFzLat = v.mass * st.ay * v.cgHeight / (0.5 * (v.trackF + v.trackR));
  const FzF = THREE.MathUtils.clamp(Fz * v.lr / v.wheelbase - dFzLong, 0, 3 * Fz);
  const FzR = THREE.MathUtils.clamp(Fz * v.lf / v.wheelbase + dFzLong, 0, 3 * Fz);

  // 纵向力
  let drive = 0;
  if (input.throttle > 0.001 && st.vx > -0.5) {
    const fMotor = Math.min(v.maxDrive, v.power / Math.max(speed, 2.0));
    drive = input.throttle * Math.min(fMotor, v.mu * FzR);
  }
  const brakeMax = v.mu * Fz;
  const braking = input.brake * brakeMax;
  const drag = 0.5 * 1.225 * v.dragArea * st.vx * st.vx;
  const roll = v.rollRes * v.mass * G;
  let fx = drive - drag;
  if (st.vx > 0.05) fx -= braking + roll;
  else if (st.vx < -0.05) fx += braking - roll;
  else fx -= Math.sign(fx) * Math.min(Math.abs(fx), roll * 1.2);

  // 轮胎侧偏（自行车模型）
  const FzFhalf = Math.max(FzF, 1.0);
  const FzRhalf = Math.max(FzR, 1.0);
  let alphF = 0;
  let alphR = 0;
  if (speed > 0.4) {
    alphF = Math.atan2(st.vy + v.lf * st.r, speed) - st.steer;
    alphR = Math.atan2(st.vy - v.lr * st.r, speed);
  }
  const capF = v.mu * FzFhalf;
  const capR = v.mu * FzRhalf;
  const FyF = THREE.MathUtils.clamp(-v.cf * alphF, -capF, capF);
  const FyR = THREE.MathUtils.clamp(-v.cr * alphR, -capR, capR);

  const grip = st.offTrack ? 0.55 : 1.0;
  const ax = (fx - FyF * sinS) / v.mass;
  const ay = (FyF * Math.cos(st.steer) + FyR) / v.mass * grip;
  const rDot = (v.lf * FyF * Math.cos(st.steer) - v.lr * FyR) / v.Iz * grip;

  st.ax = ax;
  st.ay = ay;
  // 标准三自由度自行车模型：m(v̇x − vy·r) = Fx，m(v̇y + vx·r) = Fy。
  // 纵向方程里的 vy·r 项不能省：省掉之后能量不守恒，转弯时会出现
  // 一个虚假的能量注入项 −2·vx·vy·r，车速会失控发散（实测冲到 460 km/h）。
  st.vy += (ay - st.vx * st.r) * dt;
  st.vx += (ax + st.vy * st.r) * dt;
  if (Math.abs(st.vx) < 0.02 && input.throttle < 0.01 && Math.abs(st.vy) < 0.05) {
    st.vx = 0;
    st.vy = 0;
  }
  st.r += rDot * dt;

  // 世界位姿
  const c = Math.cos(st.heading);
  const s = Math.sin(st.heading);
  st.pos.x += (st.vx * c - st.vy * s) * dt;
  st.pos.y += (st.vx * s + st.vy * c) * dt;
  st.heading += st.r * dt;

  // 车轮滚动角
  const tyreR = 0.26;
  st.wheelSpin += (st.vx / tyreR) * dt;

  // 准静态悬架行程：相对静态位置的压缩量（mm，正为压缩）
  // 制动时前轴加载 → 前轮相对车身向上；左转时载荷往右 → 右侧压缩。
  const longHalf = dFzLong / 2.0;
  const latHalf = dFzLat / 2.0;
  const mm = function (load) { return load / v.wheelRate * 1000.0; };
  st.travel.fl = mm(-longHalf - latHalf);
  st.travel.fr = mm(-longHalf + latHalf);
  st.travel.rl = mm(+longHalf - latHalf);
  st.travel.rr = mm(+longHalf + latHalf);
  return st;
}

export function createDrive(cfg) {
  const track = buildTrack({ width: 5.0 });
  const veh = createVehicle(cfg.manifest);
  let st = createState(veh, track);
  const group = new THREE.Group();
  group.add(track.group);

  // 场景附加：地面与远景
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(400, 400),
    new THREE.MeshStandardMaterial({ color: 0x11161f, roughness: 1.0, metalness: 0.0 })
  );
  ground.position.z = 0.0;
  group.add(ground);

  // 圆锥桩：沿赛道两侧间隔布置，给速度感
  const coneGeom = new THREE.ConeGeometry(0.16, 0.42, 10);
  const coneMat = new THREE.MeshStandardMaterial({ color: 0xff6a1a, roughness: 0.6 });
  const cones = new THREE.InstancedMesh(coneGeom, coneMat, 220);
  const dummy = new THREE.Object3D();
  let ci = 0;
  for (let i = 0; i < track.n && ci < 220; i += 9) {
    const c = track.center[i];
    const t = track.tangent[i];
    for (const sgn of [1, -1]) {
      const nrm = new THREE.Vector3(-t.y, t.x, 0).multiplyScalar(sgn * (track.halfW + 0.55));
      dummy.position.set(c.x + nrm.x, c.y + nrm.y, 0.21);
      dummy.updateMatrix();
      cones.setMatrixAt(ci++, dummy.matrix);
    }
  }
  cones.count = ci;
  group.add(cones);

  group.visible = false;
  cfg.world.add(group);

  // 车辆各角零件分组，用于悬挂与转向动画
  const corners = { fl: [], fr: [], rl: [], rr: [] };
  const wheels = [];
  function cornerOf(id) {
    if (id.indexOf('susp.front.left') === 0) return 'fl';
    if (id.indexOf('susp.front.right') === 0) return 'fr';
    if (id.indexOf('susp.rear.left') === 0) return 'rl';
    if (id.indexOf('susp.rear.right') === 0) return 'rr';
    if (id.indexOf('前左') >= 0) return 'fl';
    if (id.indexOf('前右') >= 0) return 'fr';
    if (id.indexOf('后左') >= 0) return 'rl';
    if (id.indexOf('后右') >= 0) return 'rr';
    return null;
  }
  cfg.partMeshes.forEach(function (mesh, id) {
    const c = cornerOf(id);
    if (!c) return;
    const rec = { mesh, base: mesh.userData.base.clone(), corner: c };
    corners[c].push(rec);
    if (/^(wheel\.tire|wheel\.rim|wheel\.hub|brake\.disc)/.test(id)) {
      rec.spin = true;
      wheels.push(rec);
    }
  });
  const wheelCenters = {
    fl: new THREE.Vector3(), fr: new THREE.Vector3(),
    rl: new THREE.Vector3(), rr: new THREE.Vector3(),
  };
  for (const k in corners) {
    let best = null;
    for (const rec of corners[k]) {
      if (rec.mesh.userData.partId.indexOf('wheel.hub') === 0) best = rec.base;
    }
    wheelCenters[k].copy(best || new THREE.Vector3());
  }

  const q = new THREE.Quaternion();
  const qSteer = new THREE.Quaternion();
  const qSpin = new THREE.Quaternion();
  const tmp = new THREE.Vector3();
  const Z = new THREE.Vector3(0, 0, 1);
  const Y = new THREE.Vector3(0, 1, 0);

  function applyAnimation() {
    const model = cfg.model;
    model.position.set(st.pos.x, st.pos.y, 0);
    model.rotation.z = st.heading;
    const steerDeg = st.steer;
    for (const k in corners) {
      const travel = st.travel[k] / 1000.0;
      const isFront = (k === 'fl' || k === 'fr');
      const wc = wheelCenters[k];
      qSteer.setFromAxisAngle(Z, isFront ? steerDeg : 0);
      for (const rec of corners[k]) {
        tmp.copy(rec.base).sub(wc);
        tmp.applyQuaternion(qSteer);
        tmp.add(wc);
        rec.mesh.position.set(tmp.x, tmp.y, tmp.z + travel);
        if (isFront && !rec.spin) {
          rec.mesh.quaternion.copy(qSteer);
        } else if (isFront) {
          qSpin.setFromAxisAngle(Y, st.wheelSpin);
          q.copy(qSteer).multiply(qSpin);
          rec.mesh.quaternion.copy(q);
        } else if (rec.spin) {
          qSpin.setFromAxisAngle(Y, st.wheelSpin);
          rec.mesh.quaternion.copy(qSpin);
        }
      }
    }
  }

  const input = { throttle: 0, brake: 0, steer: 0 };
  const flags = { lapTracking: true };
  let acc = 0;
  const fixed = 1 / 120;

  function update(dt) {
    acc += Math.min(dt, 0.1);
    let guard = 0;
    while (acc >= fixed && guard < 40) {
      step(st, fixed, input);
      acc -= fixed;
      guard++;
    }
    const near = nearestOnTrack(track, st.pos);
    st.offTrack = near.dist > track.halfW + 0.25;
    // 计圈用"穿越起跑线"而不是"最近点索引回绕"：车一旦冲出路外，
    // 最近点会跳到赛道另一侧，索引法会凭空生成 2 秒一圈的假圈数。
    if (flags.lapTracking) {
      const relX = st.pos.x - track.center[0].x;
      const relY = st.pos.y - track.center[0].y;
      const fx = track.tangent[0].x;
      const fy = track.tangent[0].y;
      const along = relX * fx + relY * fy;
      const lateral = -relX * fy + relY * fx;
      const side = along >= 0 ? 1 : -1;
      if (side > 0 && st.prevLineSide < 0
          && Math.abs(lateral) < track.halfW + 1.5 && st.lapTime > 5.0) {
        st.laps++;
        if (st.bestLap === 0 || st.lapTime < st.bestLap) st.bestLap = st.lapTime;
        st.lapTime = 0;
      }
      st.prevLineSide = side;
    }
    st.lapTime += dt;
    applyAnimation();
    const sp = Math.hypot(st.vx, st.vy) * 3.6;
    return {
      speed: sp,
      ax: st.ax / G, ay: st.ay / G,
      steerDeg: st.steer * 180 / Math.PI,
      throttle: input.throttle, brake: input.brake,
      travel: st.travel, offTrack: st.offTrack,
      lapTime: st.lapTime, laps: st.laps, bestLap: st.bestLap,
    };
  }

  function reset() {
    st = createState(veh, track);
    acc = 0;
    applyAnimation();
  }

  const camDbg = { carWorld: [0, 0, 0], pos: [0, 0], heading: 0 };

  function camera(cam, tgt) {
    const c = Math.cos(st.heading);
    const s = Math.sin(st.heading);
    const fwd = new THREE.Vector3(c, 0, -s);
    // 用模型的实际世界坐标定位相机，避免依赖另一条独立推算的坐标链
    const carWorld = new THREE.Vector3();
    cfg.model.updateWorldMatrix(true, false);
    cfg.model.getWorldPosition(carWorld);
    carWorld.y = 0;
    camDbg.carWorld = carWorld.toArray();
    camDbg.pos = [st.pos.x, st.pos.y];
    camDbg.heading = st.heading;
    const speedFactor = 1.0 + Math.min(Math.abs(st.vx) * 0.035, 0.55);
    cam.copy(carWorld).addScaledVector(fwd, -4.6 * speedFactor).setY(1.75 + 0.25 * speedFactor);
    tgt.copy(carWorld).addScaledVector(fwd, 2.4).setY(0.55);
  }

  function telemetry() { return st; }

  // 中心线上位于车前方 ahead 米的点（自动驾驶演示与航向偏差估算用）
  function aimPoint(ahead) {
    const near = nearestOnTrack(track, st.pos);
    const step = Math.max(1, Math.round(ahead / track.segLen));
    const i = (near.index + step) % track.n;
    return { point: track.center[i], index: near.index, dist: near.dist };
  }

  return {
    group: group,
    input: input,
    update: update,
    reset: reset,
    camera: camera,
    telemetry: telemetry,
    aimPoint: aimPoint,
    vehicle: veh,
    track: track,
    setVisible: function (on) { group.visible = on; },
    setLapTracking: function (on) { flags.lapTracking = on; },
    camDbg: camDbg,
  };
}
