// 在 Node 里跑驾驶模式的车辆动力学，验证长时间行为（不必开浏览器）。
// 用法：node tools/run_test_drive.mjs
// 说明：无头浏览器在虚拟时间下只推进极短的仿真时间，无法验证转弯、
// 循迹与圈速等长时间行为，因此把物理单独拎出来在 Node 里跑。
import * as THREE from 'three';
import { createDrive } from '../site/drive.js';

const world = new THREE.Group();
const model = new THREE.Object3D();
const drive = createDrive({
  manifest: { totals: { mass_kg: 267.8 } },
  model: model,
  partMeshes: new Map(),
  world: world,
});

const veh = drive.vehicle;
console.log('车辆参数：整车 ' + veh.mass.toFixed(0) + ' kg（含车手）  轴距 '
  + veh.wheelbase.toFixed(2) + ' m  质心高 ' + veh.cgHeight.toFixed(2)
  + ' m  横摆惯量 ' + veh.Iz.toFixed(1) + ' kg·m²');
console.log('赛道：周长 ' + drive.track.length.toFixed(1) + ' m，宽 '
  + (drive.track.halfW * 2).toFixed(1) + ' m');
console.log('');

const input = drive.input;
const dt = 1 / 60;
let t = 0;
let maxSpeed = 0;
let maxLat = 0;
let offCount = 0;
const marks = [];
for (let i = 0; i < 60 * 150; i++) {
  const st = drive.telemetry();
  const aim = drive.aimPoint(7.0 + Math.abs(st.vx) * 0.55);
  const dx = aim.point.x - st.pos.x;
  const dy = aim.point.y - st.pos.y;
  let err = Math.atan2(dy, dx) - st.heading;
  while (err > Math.PI) err -= 2 * Math.PI;
  while (err < -Math.PI) err += 2 * Math.PI;
  input.steer = Math.max(-1, Math.min(1, err * 2.2));
  const targetV = 15.0 - Math.min(8.0, Math.abs(err) * 14.0);
  input.throttle = st.vx < targetV ? 0.45 : 0.0;
  input.brake = st.vx > targetV + 1.5 ? 0.25 : 0.0;
  const o = drive.update(dt);
  t += dt;
  maxSpeed = Math.max(maxSpeed, o.speed);
  maxLat = Math.max(maxLat, Math.abs(o.ay));
  if (o.offTrack) offCount++;
  if (i % (60 * 15) === 0) {
    marks.push([t, o]);
  }
}
for (const m of marks) {
  const o = m[1];
  console.log('t=' + m[0].toFixed(0).padStart(3) + 's  车速 '
    + o.speed.toFixed(1).padStart(5) + ' km/h  纵向 ' + o.ax.toFixed(2).padStart(6)
    + 'g  横向 ' + o.ay.toFixed(2).padStart(6) + 'g  前轮 '
    + o.steerDeg.toFixed(1).padStart(5) + '°  圈速 ' + o.lapTime.toFixed(1).padStart(5)
    + 's  圈数 ' + o.laps);
}
console.log('');
console.log('峰值车速 ' + maxSpeed.toFixed(1) + ' km/h   峰值横向加速度 '
  + maxLat.toFixed(2) + ' g');
console.log('出界帧占比 ' + (offCount / (60 * 150) * 100).toFixed(1) + '%');
console.log('悬架行程（压缩为正，mm）：左前 ' + drive.telemetry().travel.fl.toFixed(1)
  + '  右前 ' + drive.telemetry().travel.fr.toFixed(1)
  + '  左后 ' + drive.telemetry().travel.rl.toFixed(1)
  + '  右后 ' + drive.telemetry().travel.rr.toFixed(1));
