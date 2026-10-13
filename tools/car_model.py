"""FSAE 整车参数化模型。

由 params/car.json 驱动，输出零件清单（每个零件一个独立网格 + 材料 + 质量）。
坐标系：X 向前，Y 向左，Z 向上；原点在轴距中点、地面高度处。

重要说明：默认参数是一套"原型车"数据，来自 FSAE 常见量级，
不是你车队的真实尺寸。所有数值都在 params/car.json 里，改完重跑即可。
"""
from __future__ import annotations

import math

import geom as g

# --------------------------------------------------------------------------
# 材料表：颜色给浏览器，密度用于质量估算
# --------------------------------------------------------------------------

MATERIALS = {
    "steel_tube": {"color": [0.30, 0.32, 0.35, 1.0], "metal": 0.85, "rough": 0.45, "density": 7850},
    "steel":      {"color": [0.40, 0.42, 0.45, 1.0], "metal": 0.85, "rough": 0.42, "density": 7850},
    "chrome":     {"color": [0.68, 0.70, 0.74, 1.0], "metal": 1.00, "rough": 0.22, "density": 7850},
    "aluminum":   {"color": [0.72, 0.74, 0.77, 1.0], "metal": 0.90, "rough": 0.35, "density": 2700},
    "carbon":     {"color": [0.11, 0.11, 0.13, 1.0], "metal": 0.30, "rough": 0.35, "density": 1600},
    "carbon_sandwich": {"color": [0.13, 0.13, 0.15, 1.0], "metal": 0.25, "rough": 0.40, "density": 480},
    "rubber":     {"color": [0.05, 0.05, 0.06, 1.0], "metal": 0.00, "rough": 0.92, "density": 1100},
    "foam":       {"color": [0.28, 0.08, 0.09, 1.0], "metal": 0.00, "rough": 0.85, "density": 90},
    "fabric":     {"color": [0.14, 0.15, 0.18, 1.0], "metal": 0.00, "rough": 0.90, "density": 300},
    "copper":     {"color": [0.70, 0.40, 0.20, 1.0], "metal": 1.00, "rough": 0.28, "density": 8900},
    "brake_disc": {"color": [0.32, 0.30, 0.28, 1.0], "metal": 0.95, "rough": 0.55, "density": 7200},
    "body_red":   {"color": [0.68, 0.09, 0.11, 1.0], "metal": 0.20, "rough": 0.28, "density": 1600},
    "body_white": {"color": [0.88, 0.89, 0.91, 1.0], "metal": 0.05, "rough": 0.42, "density": 1600},
    "body_dark":  {"color": [0.16, 0.17, 0.19, 1.0], "metal": 0.10, "rough": 0.50, "density": 1600},
    "battery":    {"color": [0.18, 0.20, 0.26, 1.0], "metal": 0.20, "rough": 0.55, "density": 1900},
}

SYSTEMS = [
    ("frame", "车架"),
    ("susp_front", "前悬架"),
    ("susp_rear", "后悬架"),
    ("wheels", "轮胎与轮辋"),
    ("steering", "转向系统"),
    ("brakes", "制动系统"),
    ("powertrain", "动力总成"),
    ("aero", "空气动力学套件"),
    ("cockpit", "座舱与安全"),
]

MATERIAL_LABELS = {
    "steel_tube": "钢管（车架）",
    "steel": "钢件",
    "chrome": "镀铬钢",
    "aluminum": "铝合金",
    "carbon": "碳纤维复合材料",
    "carbon_sandwich": "碳纤维夹芯板",
    "rubber": "橡胶",
    "foam": "座椅泡沫",
    "fabric": "织物",
    "copper": "铜",
    "brake_disc": "铸铁制动盘",
    "body_red": "车身覆盖件（红）",
    "body_white": "车身覆盖件（白）",
    "body_dark": "工程塑料",
    "battery": "动力电池模组",
}


class Builder:
    """收集零件。每个零件带独立网格、材料、体积与质量。"""

    def __init__(self, params):
        self.p = params
        self.parts = []

    def add(self, pid, label, system, material, mesh, fill=1.0):
        """fill = 结构填充系数。
        实心图元的几何体积对空心结构（轮胎、轮辋、翼片壳、箱体）会显著高估，
        因此按零件的实际构造给一个可审计的折减系数，质量 = 体积 × 密度 × fill。
        """
        if mesh is None or not mesh.verts:
            return
        volume = mesh.volume_or_computed()
        spec = MATERIALS[material]
        self.parts.append({
            "id": pid,
            "name": label,
            "system": system,
            "material": material,
            "mesh": mesh,
            "volume_m3": volume,
            "fill": fill,
            "mass_kg": volume * spec["density"] * fill,
            "_rgb": tuple(spec["color"][:3]),
        })

    # ---------------- 车架 ----------------

    def build_frame(self):
        p = self.p["frame"]
        seg = 8
        r_main, w_main = p["tube_main_r"], p["tube_main_wall"]
        r_sec, w_sec = p["tube_sec_r"], p["tube_sec_wall"]
        # 节点表（左侧），右侧由镜像生成
        base = {
            "fb_bl": (0.600, 0.215, 0.105),   # 前舱壁 下
            "fb_tl": (0.600, 0.190, 0.400),   # 前舱壁 上
            "fh_bl": (0.300, 0.245, 0.105),   # 前环 下
            "fh_tl": (0.300, 0.205, 0.700),   # 前环 上
            "mh_bl": (-0.440, 0.275, 0.105),  # 主环 下
            "mh_mid": (-0.440, 0.268, 0.560), # 主环 中
            "mh_tl": (-0.440, 0.245, 1.020),  # 主环 上
            "ds_bl": (0.430, 0.205, 0.620),   # 仪表板横杆端
            "si_out": (0.020, 0.335, 0.150),  # 侧碰结构 外下
            "si_top": (0.020, 0.335, 0.480),  # 侧碰结构 外上
            "rk_mnt": (0.720, 0.240, 0.440),  # 前摇臂支座
            "rm_bl": (-0.860, 0.215, 0.180),  # 后安装点 下
            "rm_tl": (-0.800, 0.175, 0.560),  # 后安装点 上
            "fl_f": (0.560, 0.150, 0.070),    # 前地板梁端
        }
        n = {}
        for key, (x, y, z) in base.items():
            n[key + "_l"] = (x, y, z)
            n[key + "_r"] = (x, -y, z)

        def tube(a, b, radius, wall, label, system="frame"):
            mesh = g.tube(n[a], n[b], radius, seg=seg, wall=wall)
            self.add("frame.%s.%s" % (a, b), label, system, "steel_tube", mesh)

        def cross(a, radius, wall, label):
            self.add("frame.%s.cross" % a, label, "frame", "steel_tube",
                     g.tube(n[a + "_l"], n[a + "_r"], radius, seg=seg, wall=wall))

        pairs = [
            ("fb_bl", "fh_bl", r_main, w_main, "前舱下纵梁"),
            ("fh_bl", "mh_bl", r_main, w_main, "座舱下纵梁"),
            ("fb_tl", "fh_tl", r_main, w_main, "前舱上纵梁"),
            ("fh_tl", "mh_tl", r_main, w_main, "座舱上纵梁"),
            ("fb_bl", "fb_tl", r_main, w_main, "前舱壁立柱"),
            ("fh_bl", "fh_tl", r_main, w_main, "前环立柱"),
            ("mh_bl", "mh_tl", r_main, w_main, "主环立柱"),
            ("fb_bl", "fh_tl", r_sec, w_sec, "前舱斜撑"),
            ("fh_bl", "mh_tl", r_sec, w_sec, "座舱侧斜撑"),
            ("fb_tl", "fh_bl", r_sec, w_sec, "前舱壁对角撑"),
            ("ds_bl", "fh_tl", r_sec, w_sec, "仪表板支撑"),
            ("ds_bl", "mh_bl", r_sec, w_sec, "座舱上侧纵梁"),
            ("si_out", "fb_bl", r_sec, w_sec, "侧碰结构前撑"),
            ("si_out", "fh_bl", r_sec, w_sec, "侧碰结构后撑"),
            ("si_top", "fh_tl", r_sec, w_sec, "侧碰结构上撑"),
            ("si_out", "si_top", r_sec, w_sec, "侧碰结构外立柱"),
            ("fb_tl", "si_top", r_sec, w_sec, "侧碰结构前斜撑"),
            ("fh_tl", "mh_mid", r_sec, w_sec, "座舱上缘纵梁"),
            ("mh_bl", "rm_bl", r_main, w_main, "后部下纵梁"),
            ("mh_tl", "rm_tl", r_main, w_main, "后部上纵梁"),
            ("mh_mid", "rm_tl", r_sec, w_sec, "后部斜撑"),
            ("mh_tl", "rm_bl", r_sec, w_sec, "主环后长斜撑"),
            ("rm_bl", "rm_tl", r_main, w_main, "后安装立柱"),
            ("fl_f", "fb_bl", r_sec, w_sec, "前底板纵梁"),
            ("fl_f", "fh_bl", r_sec, w_sec, "中底板纵梁"),
            ("fh_bl", "fl_f", r_sec, w_sec, "底板支撑"),
            ("rk_mnt", "fh_tl", r_sec, w_sec, "摇臂支座支撑"),
            ("rk_mnt", "fh_bl", r_sec, w_sec, "摇臂支座立柱"),
        ]
        for a, b, radius, wall, label in pairs:
            tube(a + "_l", b + "_l", radius, wall, label + "（左）")
            tube(a + "_r", b + "_r", radius, wall, label + "（右）")

        for a, label in [("fb_bl", "前舱壁下横杆"), ("fb_tl", "前舱壁上横杆"),
                         ("fh_bl", "前环下横杆"), ("fh_tl", "前环上横杆"),
                         ("mh_bl", "主环下横杆"), ("mh_mid", "主环中横杆"),
                         ("mh_tl", "主环上横杆"), ("ds_bl", "仪表板横杆"),
                         ("si_top", "侧碰结构上横杆"), ("rm_bl", "后安装下横杆"),
                         ("rm_tl", "后安装上横杆"), ("rk_mnt", "摇臂支座横杆")]:
            cross(a, r_sec if a != "mh_tl" else r_main,
                  w_sec if a != "mh_tl" else w_main, label)

    # ---------------- 座舱与安全 ----------------

    def build_cockpit(self):
        p = self.p["cockpit"]
        self.add("cockpit.floor", "座舱底板", "cockpit", "carbon_sandwich",
                 g.box((0.08, 0.0, 0.075), (1.05, 0.44, p["floor_thick"])))
        self.add("cockpit.firewall", "防火墙", "cockpit", "carbon",
                 g.box((-0.40, 0.0, 0.42), (0.010, 0.46, 0.62)), fill=0.90)
        self.add("cockpit.harness_bar", "安全带横杆", "cockpit", "steel",
                 g.tube((-0.34, -0.24, 0.86), (-0.34, 0.24, 0.86), 0.017, wall=0.0016))
        seat = g.loft_rect((0.16, 0.0, 0.30), (0.06, 0.0, 0.0), (0.0, 0.21, 0.0),
                           (-0.28, 0.0, 0.30), (0.06, 0.0, 0.0), (0.0, 0.21, 0.0))
        seat.add(g.loft_rect((0.16, 0.0, 0.30), (0.0, 0.0, 0.0), (0.0, 0.21, 0.0),
                             (0.10, 0.0, 0.62), (0.0, 0.0, 0.0), (0.0, 0.19, 0.0)))
        self.add("cockpit.seat", "座椅", "cockpit", "foam", seat)
        self.add("cockpit.headrest", "头枕", "cockpit", "foam",
                 g.box((-0.40, 0.0, 0.74), (0.10, 0.24, 0.16)))

    # ---------------- 轮胎与轮辋 ----------------

    def build_wheels(self):
        p = self.p["wheel"]
        tire_r = p["tire_od"] / 2.0
        rim_r = p["rim_dia"] / 2.0
        half_w = p["tire_width"] / 2.0
        half_rim = p["rim_width"] / 2.0

        # 轮胎剖面（闭合环）：从内侧轮辋座 → 胎肩 → 胎面 → 外侧轮辋座
        tire_profile = [
            (rim_r, -half_rim),
            (rim_r + 0.020, -half_rim - 0.004),
            (tire_r - 0.030, -half_w - 0.006),
            (tire_r, -half_w + 0.012),
            (tire_r, half_w - 0.012),
            (tire_r - 0.030, half_w + 0.006),
            (rim_r + 0.020, half_rim + 0.004),
            (rim_r, half_rim),
            (rim_r, -half_rim),
        ]
        rim_profile = [
            (0.055, -half_rim),
            (rim_r, -half_rim),
            (rim_r, half_rim),
            (0.055, half_rim),
            (0.055, -half_rim),
        ]
        disc_profile = [
            (0.075, -0.003),
            (0.145, -0.003),
            (0.145, 0.003),
            (0.075, 0.003),
            (0.075, -0.003),
        ]

        for side, tag in ((1, "左"), (-1, "右")):
            for axle, half_track, atag in ((1, p["track_front"] / 2.0, "前"),
                                           (-1, p["track_rear"] / 2.0, "后")):
                wheel_x = axle * self.p["general"]["wheelbase"] / 2.0
                center = (wheel_x, side * half_track, tire_r)
                axis = (0.0, 1.0, 0.0)
                self.add("wheel.tire.%s%s" % (atag, tag), "%s%s轮胎" % (atag, tag),
                         "wheels", "rubber",
                         g.revolve(tire_profile, center, axis, seg=22), fill=0.28)
                self.add("wheel.rim.%s%s" % (atag, tag), "%s%s轮辋" % (atag, tag),
                         "wheels", "aluminum",
                         g.revolve(rim_profile, center, axis, seg=22), fill=0.10)
                self.add("wheel.hub.%s%s" % (atag, tag), "%s%s轮毂" % (atag, tag),
                         "wheels", "aluminum",
                         g.revolve([(0.0, -0.045), (0.055, -0.045),
                                    (0.055, 0.045), (0.0, 0.045)],
                                   center, axis, seg=14), fill=0.55)
                self.add("brake.disc.%s%s" % (atag, tag), "%s%s制动盘" % (atag, tag),
                         "brakes", "brake_disc",
                         g.revolve(disc_profile,
                                   (wheel_x, side * (half_track - 0.075), tire_r), axis, seg=20),
                         fill=0.75)
                caliper_center = (wheel_x - 0.10, side * (half_track - 0.075), tire_r - 0.02)
                self.add("brake.caliper.%s%s" % (atag, tag), "%s%s制动卡钳" % (atag, tag),
                         "brakes", "aluminum",
                         g.box(caliper_center, (0.075, 0.035, 0.11)))

    # ---------------- 悬架 ----------------

    def build_suspension(self):
        wheel = self.p["wheel"]
        tire_r = wheel["tire_od"] / 2.0
        wb = self.p["general"]["wheelbase"]
        seg = 8
        r_arm, w_arm = 0.0110, 0.0012

        for axle, atag, system in ((1, "前", "susp_front"), (-1, "后", "susp_rear")):
            ax = axle * wb / 2.0
            half_track = (wheel["track_front"] if axle > 0 else wheel["track_rear"]) / 2.0
            for side, stag in ((1, "左"), (-1, "右")):
                hy = side * half_track
                pid = "susp.%s.%s" % ("front" if axle > 0 else "rear", "left" if side > 0 else "right")
                lab = atag + stag
                lca_in_f = (ax - 0.165, side * 0.245, 0.115)
                lca_in_r = (ax + 0.165, side * 0.245, 0.115)
                lca_out = (ax, hy - side * 0.028, 0.125)
                uca_in_f = (ax - 0.135, side * 0.290, 0.300)
                uca_in_r = (ax + 0.135, side * 0.290, 0.300)
                uca_out = (ax, hy - side * 0.032, 0.330)
                wc = (ax, hy, tire_r)
                rocker = (ax - 0.100, side * 0.235, 0.455)
                push_lo = (ax, hy - side * 0.105, 0.140)
                push_hi = (ax - 0.048, side * 0.222, 0.470)
                damp_lo = (ax - 0.160, side * 0.245, 0.410)
                damp_hi = (ax - 0.250, side * 0.115, 0.230)

                for name, a, b in (("uca_front", uca_in_f, uca_out),
                                   ("uca_rear", uca_in_r, uca_out)):
                    self.add("%s.%s" % (pid, name), "%s上叉臂%s" % (lab, "前支" if "front" in name else "后支"),
                             system, "steel_tube", g.tube(a, b, r_arm, seg=seg, wall=w_arm))
                for name, a, b in (("lca_front", lca_in_f, lca_out),
                                   ("lca_rear", lca_in_r, lca_out)):
                    self.add("%s.%s" % (pid, name), "%s下叉臂%s" % (lab, "前支" if "front" in name else "后支"),
                             system, "steel_tube", g.tube(a, b, r_arm, seg=seg, wall=w_arm))

                upright = g.loft_rect(lca_out, (0.050, 0.0, 0.0), (0.0, 0.040, 0.0),
                                      uca_out, (0.042, 0.0, 0.0), (0.0, 0.034, 0.0))
                upright.add(g.revolve([(0.0, -0.055), (0.048, -0.055),
                                       (0.048, 0.055), (0.0, 0.055)],
                                      wc, (0.0, 1.0, 0.0), seg=14))
                self.add("%s.upright" % pid, "%s立柱" % lab, system, "aluminum", upright)

                self.add("%s.pushrod" % pid, "%s推杆" % lab, system, "steel_tube",
                         g.tube(push_lo, push_hi, 0.0095, seg=seg, wall=0.0012))
                rocker_outline = [(0.060, 0.0), (-0.050, 0.048), (-0.050, -0.048)]
                self.add("%s.rocker" % pid, "%s摇臂" % lab, system, "aluminum",
                         g.extrude_dir(rocker_outline, rocker, (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 0.024))
                self.add("%s.damper_body" % pid, "%s减震器筒" % lab, system, "aluminum",
                         g.tube(damp_lo, damp_hi, 0.019, seg=seg))
                self.add("%s.spring" % pid, "%s减震弹簧" % lab, system, "chrome",
                         g.tube(damp_lo, damp_hi, 0.031, seg=seg, wall=0.004))

    # ---------------- 转向 ----------------

    def build_steering(self):
        wheel = self.p["wheel"]
        wb = self.p["general"]["wheelbase"]
        seg = 8
        rack_x, rack_z = 0.630, 0.160
        self.add("steering.rack_housing", "转向齿条壳", "steering", "aluminum",
                 g.tube((rack_x, -0.230, rack_z), (rack_x, 0.230, rack_z), 0.024, seg=seg))
        self.add("steering.rack", "转向齿条", "steering", "chrome",
                 g.tube((rack_x, -0.250, rack_z), (rack_x, 0.250, rack_z), 0.012, seg=seg))
        hub = (0.280, 0.0, 0.620)
        rack_center = (rack_x, 0.0, rack_z)
        self.add("steering.column", "转向柱", "steering", "chrome",
                 g.tube(rack_center, hub, 0.013, seg=seg))
        column_dir = g.vnorm(g.vsub(hub, rack_center))
        ring = []
        for i in range(14):
            ang = 2 * math.pi * i / 14
            ring.append((0.115 + 0.014 * math.cos(ang), 0.014 * math.sin(ang)))
        ring.append(ring[0])
        self.add("steering.wheel", "方向盘", "steering", "body_dark",
                 g.revolve(ring, hub, column_dir, seg=16))
        for side, stag in ((1, "左"), (-1, "右")):
            tie_in = (rack_x, side * 0.215, rack_z)
            tie_out = (wb / 2.0, side * (wheel["track_front"] / 2.0 - 0.055), 0.185)
            self.add("steering.tie_rod.%s" % ("left" if side > 0 else "right"),
                     "%s转向拉杆" % stag, "steering", "steel_tube",
                     g.tube(tie_in, tie_out, 0.0085, seg=seg, wall=0.0012))
        self.add("brake.master_cyl", "制动主缸", "brakes", "aluminum",
                 g.box((0.560, 0.0, 0.240), (0.16, 0.075, 0.075)))
        self.add("brake.pedal", "制动踏板", "brakes", "steel",
                 g.box((0.480, 0.0, 0.230), (0.02, 0.09, 0.16)))

    # ---------------- 动力总成（电动方案） ----------------

    def build_powertrain(self):
        wb = self.p["general"]["wheelbase"]
        rear_ax = -wb / 2.0
        self.add("powertrain.motor", "驱动电机", "powertrain", "aluminum",
                 g.revolve([(0.0, -0.120), (0.098, -0.120), (0.098, 0.120), (0.0, 0.120)],
                           (-0.590, 0.0, 0.290), (1.0, 0.0, 0.0), seg=18), fill=0.60)
        self.add("powertrain.inverter", "电机控制器", "powertrain", "body_dark",
                 g.box((-0.470, 0.0, 0.520), (0.20, 0.26, 0.10)), fill=0.50)
        self.add("powertrain.battery", "动力电池模组", "powertrain", "battery",
                 g.box((-0.300, 0.0, 0.290), (0.44, 0.44, 0.22)), fill=0.75)
        self.add("powertrain.battery_lid", "电池箱上盖", "powertrain", "carbon",
                 g.box((-0.300, 0.0, 0.410), (0.46, 0.46, 0.020)), fill=0.85)
        self.add("powertrain.diff", "差速器", "powertrain", "aluminum",
                 g.revolve([(0.0, -0.085), (0.072, -0.085), (0.072, 0.085), (0.0, 0.085)],
                           (rear_ax, 0.0, 0.260), (0.0, 1.0, 0.0), seg=18), fill=0.85)
        for side, stag in ((1, "左"), (-1, "右")):
            half_track = self.p["wheel"]["track_rear"] / 2.0
            self.add("powertrain.driveshaft.%s" % ("left" if side > 0 else "right"),
                     "%s半轴" % stag, "powertrain", "steel",
                     g.tube((rear_ax, side * 0.075, 0.260),
                            (rear_ax, side * (half_track - 0.06), 0.260), 0.016, seg=8))
        self.add("powertrain.mount", "电机安装座", "powertrain", "steel",
                 g.box((-0.590, 0.0, 0.185), (0.26, 0.30, 0.02)))

    # ---------------- 空气动力学套件 ----------------

    def build_aero(self):
        aero = self.p["aero"]
        fw_span = aero["front_wing_span"]
        rw_span = aero["rear_wing_span"]
        chord = aero["wing_chord"]
        aoa = math.radians(aero["wing_aoa_deg"])
        chord_dir = (math.cos(aoa), 0.0, -math.sin(aoa))
        span_dir = (0.0, 1.0, 0.0)

        # 前翼主翼 + 襟翼
        ang = math.radians(aoa)
        cd = (math.cos(ang), 0.0, -math.sin(ang))
        fw_main = g.extrude_dir(g.airfoil_outline(chord, 0.10, 0.055),
                                (aero["front_wing_x"], -fw_span / 2.0, aero["front_wing_z"]),
                                cd, span_dir, fw_span)
        self.add("aero.front_wing.main", "前翼主翼", "aero", "carbon_sandwich", fw_main, fill=0.45)
        flap_ang = math.radians(aoa + 18.0)
        flap_cd = (math.cos(flap_ang), 0.0, -math.sin(flap_ang))
        fw_flap = g.extrude_dir(g.airfoil_outline(chord * 0.45, 0.09, 0.06),
                                (aero["front_wing_x"] - chord * 0.50,
                                 -fw_span / 2.0, aero["front_wing_z"] + 0.055),
                                flap_cd, span_dir, fw_span)
        self.add("aero.front_wing.flap", "前翼襟翼", "aero", "carbon_sandwich", fw_flap, fill=0.45)
        for side, stag in ((1, "左"), (-1, "右")):
            ep = [(-0.02, -0.03), (chord * 1.35, -0.02), (chord * 1.30, 0.20), (-0.02, 0.16)]
            self.add("aero.front_wing.endplate.%s" % ("left" if side > 0 else "right"),
                     "前翼%s端板" % stag, "aero", "carbon_sandwich",
                     g.extrude_dir(ep, (aero["front_wing_x"] - 0.02, side * fw_span / 2.0,
                                        aero["front_wing_z"]),
                                   (1.0, 0.0, 0.0), span_dir, side * 0.006), fill=0.40)
        for side, stag in ((1, "左"), (-1, "右")):
            self.add("aero.front_wing.support.%s" % ("left" if side > 0 else "right"),
                     "前翼%s支撑" % stag, "aero", "carbon",
                     g.tube((aero["front_wing_x"] + 0.05, side * 0.18, aero["front_wing_z"] + 0.02),
                            (0.620, side * 0.14, 0.310), 0.012, seg=8))

        # 鼻锥
        nose = g.loft_rect((0.860, 0.0, 0.290), (0.0, 0.215, 0.0), (0.0, 0.0, 0.150),
                           (1.100, 0.0, 0.190), (0.0, 0.115, 0.0), (0.0, 0.0, 0.070))
        self.add("aero.nose", "鼻锥", "aero", "body_red", nose, fill=0.20)

        # 后翼主翼 + 两级襟翼
        rw_main = g.extrude_dir(g.airfoil_outline(aero["rear_wing_chord"], 0.11, 0.05),
                                (aero["rear_wing_x"], -rw_span / 2.0, aero["rear_wing_z"]),
                                cd, span_dir, rw_span)
        self.add("aero.rear_wing.main", "后翼主翼", "aero", "carbon_sandwich", rw_main, fill=0.45)
        for k, (dz, dd) in enumerate(((0.075, 0.42), (0.135, 0.30))):
            fang = math.radians(aoa + 14.0 + 8.0 * k)
            fcd = (math.cos(fang), 0.0, -math.sin(fang))
            flap = g.extrude_dir(g.airfoil_outline(aero["rear_wing_chord"] * dd, 0.10, 0.055),
                                 (aero["rear_wing_x"] - aero["rear_wing_chord"] * (0.45 + 0.25 * k),
                                  -rw_span / 2.0, aero["rear_wing_z"] + dz),
                                 fcd, span_dir, rw_span)
            self.add("aero.rear_wing.flap%d" % (k + 1), "后翼襟翼%d" % (k + 1),
                     "aero", "carbon_sandwich", flap, fill=0.45)
        for side, stag in ((1, "左"), (-1, "右")):
            ep = [(-0.03, -0.05), (aero["rear_wing_chord"] * 1.5, -0.02),
                  (aero["rear_wing_chord"] * 1.45, 0.24), (-0.03, 0.20)]
            self.add("aero.rear_wing.endplate.%s" % ("left" if side > 0 else "right"),
                     "后翼%s端板" % stag, "aero", "carbon_sandwich",
                     g.extrude_dir(ep, (aero["rear_wing_x"] - 0.03, side * rw_span / 2.0,
                                        aero["rear_wing_z"]),
                                   (1.0, 0.0, 0.0), span_dir, side * 0.008), fill=0.40)
        for side, stag in ((1, "左"), (-1, "右")):
            self.add("aero.rear_wing.support.%s" % ("left" if side > 0 else "right"),
                     "后翼%s支撑" % stag, "aero", "carbon",
                     g.tube((aero["rear_wing_x"] + 0.06, side * 0.22, aero["rear_wing_z"] - 0.02),
                            (-0.760, side * 0.16, 0.500), 0.014, seg=8))

        # 侧箱
        for side, stag in ((1, "左"), (-1, "右")):
            pod = g.loft_rect((0.420, side * 0.330, 0.230), (0.0, 0.105, 0.0), (0.0, 0.0, 0.140),
                              (-0.430, side * 0.300, 0.250), (0.0, 0.130, 0.0), (0.0, 0.0, 0.170))
            self.add("aero.sidepod.%s" % ("left" if side > 0 else "right"),
                     "%s侧箱" % stag, "aero", "body_red", pod, fill=0.12)

        # 底板与扩散器
        self.add("aero.floor", "空气动力学底板", "aero", "carbon_sandwich",
                 g.box((0.05, 0.0, 0.055), (1.55, 0.72, 0.008)))
        diff = g.loft_rect((-0.500, 0.0, 0.090), (0.0, 0.340, 0.0), (0.0, 0.0, 0.030),
                           (-1.020, 0.0, 0.210), (0.0, 0.330, 0.0), (0.0, 0.0, 0.100))
        self.add("aero.diffuser", "扩散器", "aero", "carbon_sandwich", diff, fill=0.25)


def build(params):
    """按参数字典生成全部零件。"""
    b = Builder(params)
    b.build_frame()
    b.build_cockpit()
    b.build_wheels()
    b.build_suspension()
    b.build_steering()
    b.build_powertrain()
    b.build_aero()
    return b.parts
