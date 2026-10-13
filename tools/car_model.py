"""FSAE 整车参数化模型。

由 params/car.json 驱动，输出零件清单（每个零件一个独立网格 + 材料 + 质量）。
坐标系：X 向前，Y 向左，Z 向上；原点在轴距中点、地面高度处。

重要说明：默认参数是一套"原型车"数据，来自 FSAE 常见量级，
不是你车队的真实尺寸。所有数值都在 params/car.json 里，改完重跑即可。
"""
from __future__ import annotations

import math

import geom as g
import hardware as hw

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
    ("aero_ground", "文丘里与地面效应套件"),
    ("body", "车壳"),
    ("cockpit", "座舱与安全"),
]

# 件号前缀，按系统分配
SYSTEM_CODE = {
    "frame": "FRM", "susp_front": "SUF", "susp_rear": "SUR", "wheels": "WHL",
    "steering": "STR", "brakes": "BRK", "powertrain": "PWR", "aero": "AER",
    "aero_ground": "GND", "body": "BDY", "cockpit": "CPK",
}

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
        self.seq = {}

    def add(self, pid, label, system, material, mesh, fill=1.0, note=None,
            std=False, std_spec=None):
        """fill = 结构填充系数。
        实心图元的几何体积对空心结构（轮胎、轮辋、翼片壳、箱体）会显著高估，
        因此按零件的实际构造给一个可审计的折减系数，质量 = 体积 × 密度 × fill。
        note = 该零件的设计意图/气动功能说明，会显示在网页的信息卡里。
        std/std_spec = 是否为标准件及其标准规格（对应 FS 规则 T 10 的标注要求）。
        """
        if mesh is None or not mesh.verts:
            return
        volume = mesh.volume_or_computed()
        spec = MATERIALS[material]
        code = SYSTEM_CODE.get(system, "GEN")
        self.seq[code] = self.seq.get(code, 0) + 1
        self.parts.append({
            "id": pid,
            "name": label,
            "part_no": "26B-%s-%03d" % (code, self.seq[code]),
            "system": system,
            "material": material,
            "mesh": mesh,
            "volume_m3": volume,
            "fill": fill,
            "mass_kg": volume * spec["density"] * fill,
            "note": note or "",
            "std": bool(std),
            "spec": std_spec or "",
            "_rgb": tuple(spec["color"][:3]),
        })

    # ---------------- 通用造型工具 ----------------

    @staticmethod
    def skin(sections):
        """按一组横截面放样出光滑外壳。
        sections = [(x, 半宽, 半高, 中心高度), ...]，沿 X 方向串联。"""
        mesh = g.Mesh()
        for a, b in zip(sections, sections[1:]):
            mesh.add(g.loft_rect(
                (a[0], 0.0, a[3]), (0.0, a[1], 0.0), (0.0, 0.0, a[2]),
                (b[0], 0.0, b[3]), (0.0, b[1], 0.0), (0.0, 0.0, b[2])))
        return mesh

    @staticmethod
    def panel(x0, x1, y, z_center, half_height, half_thickness=0.006):
        """竖向薄板件（侧板、导流板、侧裙）。half_thickness=0.006 即 12 mm 夹芯板。"""
        return g.loft_rect((x0, y, z_center), (0.0, half_thickness, 0.0), (0.0, 0.0, half_height),
                           (x1, y, z_center), (0.0, half_thickness, 0.0), (0.0, 0.0, half_height))

    @staticmethod
    def profiled_panel(sections, y0, y1, thickness=0.014, mode="roof", z_floor=0.014):
        """沿 X 逐段放样、高度按 sections 变化的板件 —— 文丘里通道靠它成形。

        sections = [(x, z), ...]
        mode="roof"：水平板，板面中心高度按 z 变化（通道顶板）
        mode="wall"：竖直板，底边固定在 z_floor，顶边按 z 变化（侧壁、隔板、龙骨）
        """
        mesh = g.Mesh()
        y_center = (y0 + y1) / 2.0
        half_y = abs(y1 - y0) / 2.0
        for (xa, za), (xb, zb) in zip(sections, sections[1:]):
            if mode == "roof":
                mesh.add(g.loft_rect(
                    (xa, y_center, za), (0.0, half_y, 0.0), (0.0, 0.0, thickness / 2.0),
                    (xb, y_center, zb), (0.0, half_y, 0.0), (0.0, 0.0, thickness / 2.0)))
            else:
                mesh.add(g.loft_rect(
                    (xa, y_center, (z_floor + za) / 2.0),
                    (0.0, half_y, 0.0), (0.0, 0.0, (za - z_floor) / 2.0),
                    (xb, y_center, (z_floor + zb) / 2.0),
                    (0.0, half_y, 0.0), (0.0, 0.0, (zb - z_floor) / 2.0)))
        return mesh

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

        welds = g.Mesh()

        def tube(a, b, radius, wall, label, system="frame"):
            p0, p1 = n[a], n[b]
            self.add("frame.%s.%s" % (a, b), label, system, "steel_tube",
                     hw.tube_chamfered(p0, p1, radius, wall=wall, seg=seg),
                     note="车架主结构管件。两端倒角去毛刺，接头处另加焊缝凸起。")
            welds.add(hw.weld_bead(p0, g.vsub(p1, p0), radius))
            welds.add(hw.weld_bead(p1, g.vsub(p0, p1), radius))

        def cross(a, radius, wall, label):
            p0, p1 = n[a + "_l"], n[a + "_r"]
            self.add("frame.%s.cross" % a, label, "frame", "steel_tube",
                     hw.tube_chamfered(p0, p1, radius, wall=wall, seg=seg))
            welds.add(hw.weld_bead(p0, g.vsub(p1, p0), radius))
            welds.add(hw.weld_bead(p1, g.vsub(p0, p1), radius))

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

        self.add("frame.weld_beads", "车架焊缝", "frame", "steel", welds,
                 note="全部管件接头处的焊缝凸起，按节点表自动生成，便于核对焊接工作量。")

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
                hub_y = side * (half_track + p["tire_width"] / 2.0 - 0.022)
                naxis = (0.0, side, 0.0)
                self.add("wheel.nut.%s%s" % (atag, tag), "%s%s中心锁紧螺母" % (atag, tag),
                         "wheels", "chrome",
                         hw.hex_nut((wheel_x, hub_y, tire_r), naxis, 0.046, 0.022, 0.016),
                         std=True, std_spec="中心锁紧螺母 M30",
                         note="中心单螺母锁紧。属关键紧固件，须有防松措施。")
                self.add("wheel.washer.%s%s" % (atag, tag), "%s%s轮毂垫圈" % (atag, tag),
                         "wheels", "chrome",
                         hw.washer((wheel_x, hub_y - side * 0.013, tire_r), naxis, 0.016, 0.052, 0.004),
                         std=True, std_spec="平垫圈 DIN 125")

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
                    tag2 = "前支" if "front" in name else "后支"
                    self.add("%s.%s" % (pid, name), "%s上叉臂%s" % (lab, tag2),
                             system, "steel_tube",
                             hw.tube_chamfered(a, b, r_arm, wall=w_arm, seg=seg))
                    d = g.vsub(b, a)
                    self.add("%s.%s.rodend" % (pid, name), "%s上叉臂%s关节轴承" % (lab, tag2),
                             system, "chrome",
                             hw.rod_end(a, d, g.vnorm(g.vcross(d, (0.0, 0.0, 1.0))),
                                        bore_r=0.004, thread_d=0.010, shank_len=0.042, nut_af=0.017),
                             std=True, std_spec="GE8DO 关节轴承 + M10 锁紧螺母",
                             note="叉臂内端经关节轴承与车架耳片铰接。FS 规则 T 10.2.5 要求"
                                  "球面轴承须有大于轴承座内径的挡圈保持，T 10.2.6 要求"
                                  "可调拉杆端用锁紧螺母防松。")
                for name, a, b in (("lca_front", lca_in_f, lca_out),
                                   ("lca_rear", lca_in_r, lca_out)):
                    tag2 = "前支" if "front" in name else "后支"
                    self.add("%s.%s" % (pid, name), "%s下叉臂%s" % (lab, tag2),
                             system, "steel_tube",
                             hw.tube_chamfered(a, b, r_arm, wall=w_arm, seg=seg))
                    d = g.vsub(b, a)
                    self.add("%s.%s.rodend" % (pid, name), "%s下叉臂%s关节轴承" % (lab, tag2),
                             system, "chrome",
                             hw.rod_end(a, d, g.vnorm(g.vcross(d, (0.0, 0.0, 1.0))),
                                        bore_r=0.004, thread_d=0.010, shank_len=0.042, nut_af=0.017),
                             std=True, std_spec="GE8DO 关节轴承 + M10 锁紧螺母",
                             note="下叉臂内端铰接点。关节轴承允许叉臂在跳动中自由转动，"
                                  "同时通过螺纹调整长度来设定倾角与前束。")

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
            sfx = "left" if side > 0 else "right"
            self.add("steering.tie_rod.%s" % ("left" if side > 0 else "right"),
                     "%s转向拉杆" % stag, "steering", "steel_tube",
                     hw.tube_chamfered(tie_in, tie_out, 0.0085, wall=0.0012, seg=seg))
            d = g.vsub(tie_out, tie_in)
            self.add("steering.tie_rod_rodend.%s" % sfx, "%s拉杆外端关节轴承" % stag,
                     "steering", "chrome",
                     hw.rod_end(tie_out, g.vmul(d, -1.0),
                                g.vnorm(g.vcross(d, (0.0, 0.0, 1.0))),
                                bore_r=0.004, thread_d=0.010, shank_len=0.040, nut_af=0.017),
                     std=True, std_spec="GE8DO 关节轴承",
                     note="拉杆外端接转向节臂。T 2.8.10 要求转向系统所有接头为机械连接"
                          "且在技术检查时可见。")
            self.add("steering.tie_rod_nut.%s" % sfx, "%s拉杆锁紧螺母" % stag,
                     "steering", "steel",
                     hw.hex_nut(g.vadd(tie_in, g.vmul(g.vnorm(d), 0.028)),
                                g.vnorm(d), 0.013, 0.0064, 0.0051, nylon=True),
                     std=True, std_spec="M8 尼龙锁紧螺母 ISO 7040",
                     note="T 10.2.6：可调拉杆端必须用锁紧螺母约束防松；"
                          "T 10.2.4：锁紧螺母外须露出至少两圈完整螺纹。")
        # 齿条正限位：T 2.8.3 要求限位必须布置在齿条上
        for side, stag in ((1, "左"), (-1, "右")):
            self.add("steering.rack_stop.%s" % ("left" if side > 0 else "right"),
                     "齿条%s正限位" % stag, "steering", "aluminum",
                     g.revolve([(0.012, -0.005), (0.019, -0.005), (0.019, 0.005), (0.012, 0.005),
                                (0.012, -0.005)],
                               (rack_x, side * 0.238, rack_z), (0.0, 1.0, 0.0), seg=16),
                     note="T 2.8.3：限位必须做在齿条上，防止连杆越过死点锁死，"
                          "并防止轮胎轮辋碰到其他部件。")
        # 方向盘快拆：T 2.8.5
        hub_dir = g.vnorm(g.vsub(hub, rack_center))
        self.add("steering.quick_release", "方向盘快拆", "steering", "chrome",
                 g.revolve([(0.0, 0.0), (0.026, 0.0), (0.026, 0.014), (0.030, 0.018),
                            (0.030, 0.030), (0.0, 0.030)],
                           g.vadd(hub, g.vmul(hub_dir, -0.030)), hub_dir, seg=18),
                 std=True, std_spec="方向盘快拆连接器",
                 note="T 2.8.5：方向盘必须用快拆连接转向柱，驾驶员戴手套在正常坐姿下可操作。")

    # ---------------- 制动踏板箱总成 ----------------

    def build_pedal_box(self):
        """双主缸踏板箱总成。

        结构对应 FS 规则的三条硬性要求：
        - T 6.1.2 必须有两套独立液压回路 → 两套主缸、两个独立油壶；
        - T 6.1.10 踏板与踏板箱须承受 2 kN → 踏板与支架按此校核；
        - T 3.20.1 踏板与主缸必须位于前撞击结构后方。
        平衡杆把驾驶员的踩踏力按可调比例分配给前后回路，用来跟随理想制动力分配曲线。
        """
        x_mc, z_pb, y_off = 0.585, 0.300, 0.085
        for side, stag in ((1, "左"), (-1, "右")):
            sfx = "left" if side > 0 else "right"
            y = side * y_off
            self.add("brake.mc_body.%s" % sfx, "%s制动主缸" % stag, "brakes", "aluminum",
                     g.revolve([(0.0, -0.048), (0.019, -0.048), (0.019, 0.048), (0.0, 0.048)],
                               (x_mc, y, z_pb), (1.0, 0.0, 0.0), seg=18),
                     std=True, std_spec="Ø19 串联主缸",
                     note="双回路之一。T 6.1.2：任一点泄漏后仍须保证至少两轮有效制动。")
            self.add("brake.mc_port.%s" % sfx, "%s主缸出油口" % stag, "brakes", "aluminum",
                     g.tube((x_mc + 0.026, y, z_pb + 0.016), (x_mc + 0.026, y, z_pb + 0.042),
                            0.0055, seg=10))
            self.add("brake.reservoir.%s" % sfx, "%s制动油壶" % stag, "brakes", "body_dark",
                     g.revolve([(0.0, 0.0), (0.013, 0.0), (0.013, 0.030), (0.0, 0.030)],
                               (x_mc - 0.010, y, z_pb + 0.020), (0.0, 0.0, 1.0), seg=14),
                     std=True, std_spec="独立油壶",
                     note="T 6.1.2 要求每套回路有独立流体储备，可用独立油壶或分隔油壶。")
            self.add("brake.clevis.%s" % sfx, "%s主缸推杆叉头" % stag, "brakes", "steel",
                     hw.clevis((0.487, y, z_pb), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                               bore_r=0.0042, gap=0.009, ear_w=0.005,
                               thread_d=0.008, shank_len=0.030, nut_af=0.013),
                     std=True, std_spec="CNC 叉头 + M8 锁紧螺母",
                     note="把平衡杆的力传给主缸推杆。叉口内嵌关节轴承，允许平衡杆摆动。")
            self.add("brake.balance_nut.%s" % sfx, "%s平衡杆锁紧螺母" % stag,
                     "brakes", "steel",
                     hw.hex_nut((0.487, side * 0.068, z_pb), (0.0, 1.0, 0.0),
                                0.013, 0.0064, 0.0051, nylon=True),
                     std=True, std_spec="M8 尼龙锁紧螺母 ISO 7040",
                     note="T 10.2.1：关键紧固件必须用尼龙锁紧螺母等防松措施。")
            self.add("brake.balance_washer.%s" % sfx, "%s平衡杆垫圈" % stag,
                     "brakes", "chrome",
                     hw.washer((0.487, side * 0.080, z_pb), (0.0, 1.0, 0.0), 0.0043, 0.011, 0.0016),
                     std=True, std_spec="平垫圈 DIN 125")
        self.add("brake.balance_bar", "平衡杆", "brakes", "chrome",
                 hw.thread_shank((0.487, -0.085, z_pb), (0.487, 0.085, z_pb), 0.008),
                 std=True, std_spec="M8 螺纹平衡杆",
                 note="把踩踏力按可调比例分给前后回路，跟随理想制动力分配曲线；"
                      "两端螺纹用于调节前后配比。")
        self.add("brake.balance_pivot", "平衡杆球面支点", "brakes", "chrome",
                 hw.rod_end((0.487, 0.0, z_pb), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                            bore_r=0.004, thread_d=0.010, shank_len=0.028, nut_af=None),
                 std=True, std_spec="GE8DO 关节轴承",
                 note="平衡杆绕竖直轴摆动，实现左右主缸的力分配。")
        self.add("brake.pedal_arm", "制动踏板臂", "brakes", "steel",
                 self.panel(0.440, 0.472, 0.0, 0.230, 0.075, 0.004),
                 note="T 6.1.11：踏板含传力面必须用钢或铝制造；"
                      "T 6.1.10：须承受 2 kN 不失效。")
        self.add("brake.pedal_face", "制动踏板面", "brakes", "aluminum",
                 g.box((0.476, 0.0, 0.242), (0.014, 0.090, 0.072)),
                 note="蹬踏面。按 2 kN 踏板力校核根部弯曲应力。")
        self.add("brake.pedal_pivot", "踏板枢轴", "brakes", "chrome",
                 g.tube((0.440, -0.055, 0.152), (0.440, 0.055, 0.152), 0.006, seg=10),
                 std=True, std_spec="M12 枢轴螺栓 ISO 4017",
                 note="T 10.1.2：关键紧固件须为钢制、≥M4、8.8 级以上；"
                      "T 10.1.3：只允许六角头螺栓或内六角圆柱头螺钉。")

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

        # 前翼细节件：古尼襟翼、端板涡片、翼下涡流发生器
        chord = aero["wing_chord"]
        te_x = aero["front_wing_x"] + chord * 0.995
        te_z = aero["front_wing_z"] - chord * 0.105
        gurney = g.loft_rect((te_x, 0.0, te_z + 0.009), (0.0, fw_span / 2.0, 0.0), (0.0, 0.0, 0.009),
                             (te_x + 0.006, 0.0, te_z + 0.009), (0.0, fw_span / 2.0, 0.0), (0.0, 0.0, 0.009))
        self.add("aero.front_wing.gurney", "前翼古尼襟翼", "aero", "carbon_sandwich", gurney,
                 fill=0.60,
                 note="在襟翼后缘竖起一道 12 mm 小挡板，把驻点压力留在翼后，"
                      "等效增加翼型弯度，是投入产出比最高的一片。")
        for side, stag in ((1, "左"), (-1, "右")):
            sfx = "left" if side > 0 else "right"
            strakes = g.Mesh()
            for k in range(3):
                yv = side * (fw_span / 2.0 - 0.025)
                xv = aero["front_wing_x"] + 0.055 + 0.080 * k
                strakes.add(g.loft_rect(
                    (xv, yv, aero["front_wing_z"] + 0.080), (0.0, 0.010, 0.0), (0.0, 0.0, 0.024),
                    (xv - 0.045, yv - side * 0.022, aero["front_wing_z"] + 0.074),
                    (0.0, 0.010, 0.0), (0.0, 0.0, 0.022)))
            self.add("aero.front_wing.strake.%s" % sfx, "前翼%s端板涡片" % stag, "aero",
                     "carbon_sandwich", strakes, fill=0.85,
                     note="端板内侧的三片涡片：制造稳定涡抑制翼尖绕流，"
                          "让前翼展向载荷更均匀、减小诱导阻力。")
            vgs = g.Mesh()
            for k in range(3):
                yv = side * (0.12 + 0.16 * k)
                xv = aero["front_wing_x"] + 0.10
                vgs.add(g.loft_rect(
                    (xv, yv, aero["front_wing_z"] - 0.004), (0.0, 0.009, 0.0), (0.0, 0.0, 0.020),
                    (xv - 0.050, yv + side * 0.024, aero["front_wing_z"] - 0.008),
                    (0.0, 0.009, 0.0), (0.0, 0.0, 0.018)))
            self.add("aero.front_wing.vortex_gen.%s" % sfx, "前翼%s涡流发生器" % stag, "aero",
                     "carbon_sandwich", vgs, fill=0.85,
                     note="翼下三片小涡流发生器：给附面层注入能量，"
                          "推迟大攻角下的翼面分离。")

    # ---------------- 车壳（按气动功能分片设计） ----------------

    def build_body(self):
        """车壳不是"包一层皮"，每一片都有明确的气动职责：
        1. 鼻锥与舱盖把来流平顺地导向上方与车身两侧，避免钝体分离（降阻）。
        2. 座舱侧板把气流"关"在座舱外侧，不让它窜进驾驶舱开口产生涡。
        3. 头盔整流罩是降阻收益最大的一片：把驾驶员头部+主环这个钝体
           整流成流线型尾锥，显著减小尾流分离区。
        4. 侧箱外壳承担两件事：下方收窄给扩散器让出流道，上方引导
           冷却气流平顺排出，尾端渐收避免脱体。
        5. 侧裙压低车身下方的压力，增强底板/扩散器的抽吸效果。
        6. 前轮导流板把前轮前方的气流拨开，减弱轮胎尾流对侧箱与
           后翼的污染。
        """
        # 鼻锥：四段放样，截面连续收缩，避免出现逆压梯度突变
        nose = self.skin([(0.600, 0.235, 0.170, 0.300),
                          (0.820, 0.222, 0.152, 0.288),
                          (1.010, 0.168, 0.108, 0.268),
                          (1.130, 0.098, 0.064, 0.252)])
        tip = self.skin([(1.130, 0.098, 0.064, 0.252),
                         (1.185, 0.030, 0.020, 0.248)])
        nose.add(tip)
        self.add("body.nose", "鼻锥", "body", "body_red", nose, fill=0.05,
                 note="把来流平顺分流到车身上下两侧，避免钝头分离；同时为前翼提供干净来流。")

        # 座舱侧板
        for side, stag in ((1, "左"), (-1, "右")):
            self.add("body.cockpit_panel.%s" % ("left" if side > 0 else "right"),
                     "座舱%s侧板" % stag, "body", "carbon_sandwich",
                     self.panel(0.300, -0.330, side * 0.315, 0.425, 0.180, 0.006),
                     fill=0.50,
                     note="封住座舱开口两侧，防止气流窜入驾驶舱形成涡流与附加阻力。")

        # 仪表罩
        self.add("body.dash_cover", "仪表罩", "body", "carbon_sandwich",
                 g.loft_rect((0.250, 0.0, 0.640), (0.0, 0.230, 0.0), (0.0, 0.0, 0.045),
                             (0.430, 0.0, 0.625), (0.0, 0.185, 0.0), (0.0, 0.0, 0.035)),
                 fill=0.20, note="整流仪表板区域，把气流从座舱开口上方引开。")

        # 头盔整流罩
        fairing = self.skin([(-0.320, 0.242, 0.150, 0.870),
                             (-0.430, 0.222, 0.155, 0.900),
                             (-0.540, 0.185, 0.125, 0.880),
                             (-0.650, 0.135, 0.080, 0.820)])
        self.add("body.helmet_fairing", "头盔整流罩", "body", "body_red", fairing, fill=0.05,
                 note="全车减阻收益最大的一片：把驾驶员头部与主环这个钝体整流为流线尾锥，"
                      "显著缩小尾部低压分离区。")

        # 尾部舱盖
        cover = self.skin([(-0.560, 0.238, 0.170, 0.600),
                           (-0.720, 0.215, 0.150, 0.545),
                           (-0.880, 0.170, 0.115, 0.480),
                           (-1.000, 0.115, 0.075, 0.430)])
        self.add("body.engine_cover", "尾部舱盖", "body", "body_red", cover, fill=0.05,
                 note="覆盖动力舱并把尾流向上收拢，与后翼配合；尾端渐收以避免脱体分离。")

        # 侧箱外壳
        for side, stag in ((1, "左"), (-1, "右")):
            pod = g.loft_rect((0.430, side * 0.330, 0.245), (0.0, 0.100, 0.0), (0.0, 0.0, 0.135),
                              (0.000, side * 0.330, 0.255), (0.0, 0.115, 0.0), (0.0, 0.0, 0.150))
            pod.add(g.loft_rect((0.000, side * 0.330, 0.255), (0.0, 0.115, 0.0), (0.0, 0.0, 0.150),
                                (-0.450, side * 0.292, 0.235), (0.0, 0.082, 0.0), (0.0, 0.0, 0.115)))
            self.add("body.sidepod.%s" % ("left" if side > 0 else "right"),
                     "%s侧箱外壳" % stag, "body", "body_red", pod, fill=0.03,
                     note="下缘收窄给底板与扩散器让出流道，上方引导冷却气流平顺排出，"
                          "尾端渐收避免脱体。")

        # 前轮导流板
        for side, stag in ((1, "左"), (-1, "右")):
            self.add("body.wheel_deflector.%s" % ("left" if side > 0 else "right"),
                     "前轮%s导流板" % stag, "body", "carbon_sandwich",
                     self.panel(0.880, 0.985, side * 0.505, 0.245, 0.145, 0.008),
                     fill=0.70,
                     note="把前轮前方的气流向外拨开，减弱轮胎旋转尾流对侧箱和后翼的污染。")

        # 侧裙
        for side, stag in ((1, "左"), (-1, "右")):
            self.add("body.side_skirt.%s" % ("left" if side > 0 else "right"),
                     "%s侧裙" % stag, "body", "carbon_sandwich",
                     self.panel(0.420, -0.360, side * 0.415, 0.078, 0.048, 0.008),
                     fill=0.70,
                     note="压低车身下方的压力并抑制两侧气流内窜，增强底板与扩散器的抽吸效果。")

    # ---------------- 文丘里通道与地面效应套件 ----------------

    def build_ground_effect(self):
        """地面效应套件。车底两条文丘里通道是全车下压力的主要来源之一。

        通道纵剖面按"入口高 → 喉部最窄 → 出口扩张"布置：气流在喉部加速、
        静压降低，把车身吸向地面；出口接扩散器把加速后的气流平稳排出，
        避免在车尾突然扩压导致分离。配套布置喉部前的涡流发生器（搅动附面层、
        推迟分离）与扩散器内的隔板（抑制横向串流、保住压力恢复）。
        """
        cfg = self.p.get("undercarriage", {})
        profile = [(float(x), float(z)) for x, z in cfg.get(
            "profile", [[0.620, 0.075], [0.200, 0.055], [-0.100, 0.048],
                        [-0.500, 0.130], [-1.000, 0.320]])]
        y_in = float(cfg.get("tunnel_inner_y", 0.060))
        y_out = float(cfg.get("tunnel_outer_y", 0.400))
        z_floor = float(cfg.get("ground_gap", 0.014))
        tail = [(x, z) for (x, z) in profile if x <= -0.40]

        self.add("aero_ground.keel", "中央龙骨", "aero_ground", "carbon_sandwich",
                 self.profiled_panel(profile, -0.016, 0.016, mode="wall", z_floor=z_floor),
                 fill=0.35,
                 note="把车底切成左右两条独立通道，避免两侧气流互相干扰，"
                      "同时封住中央这条最容易漏压的路径。")

        for side, stag in ((1, "左"), (-1, "右")):
            sfx = "left" if side > 0 else "right"
            self.add("aero_ground.underbody.%s" % sfx, "%s文丘里通道顶板" % stag,
                     "aero_ground", "carbon_sandwich",
                     self.profiled_panel(profile, side * y_in, side * y_out, 0.014, "roof"),
                     fill=0.45,
                     note="通道顶板：截面从入口 75 mm 收敛到喉部 48 mm，再扩张到出口 320 mm。"
                          "喉部流速最高、压力最低，这就是地面效应的来源。")
            self.add("aero_ground.tunnel_wall.%s" % sfx, "%s通道外壁" % stag,
                     "aero_ground", "carbon_sandwich",
                     self.profiled_panel(profile, side * (y_out - 0.012), side * y_out,
                                         mode="wall", z_floor=z_floor),
                     fill=0.60,
                     note="通道外侧封板：防止车侧高压气流灌进车底。"
                          "否则低压区会被吹平，下压力大幅流失。")
            fins = g.Mesh()
            for k in range(4):
                xv = 0.42 - 0.085 * k
                yv = side * (y_in + 0.055 + 0.055 * k)
                fins.add(g.loft_rect(
                    (xv, yv, 0.090), (0.0, 0.008, 0.0), (0.0, 0.0, 0.026),
                    (xv - 0.055, yv + side * 0.024, 0.084), (0.0, 0.008, 0.0), (0.0, 0.0, 0.024)))
            self.add("aero_ground.vortex_generator.%s" % sfx, "%s涡流发生器" % stag,
                     "aero_ground", "carbon_sandwich", fins, fill=0.85,
                     note="喉部前四片小涡流发生器：把附面层低能气流搅成涡，"
                          "提高抗逆压梯度能力，推迟喉后扩张段的分离。")

        for yv in (-0.30, -0.10, 0.10, 0.30):
            self.add("aero_ground.diffuser_fence.%d" % int(round(yv * 100)),
                     "扩散器隔板 %+.2f m" % yv, "aero_ground", "carbon_sandwich",
                     self.profiled_panel(tail, yv - 0.008, yv + 0.008, mode="wall", z_floor=z_floor),
                     fill=0.60,
                     note="扩散器内的纵向隔板：抑制横向串流与角部涡，"
                          "保住扩张段的压力恢复效率。")

        self.add("aero_ground.nose_duct", "鼻锥下进气通道", "aero_ground", "carbon_sandwich",
                 self.profiled_panel([(1.030, 0.170), (0.820, 0.110), (0.620, 0.075)],
                                     -0.300, 0.300, 0.014, "roof"),
                 fill=0.45,
                 note="把鼻锥下方的来流平顺导入车底通道入口，减小入口损失，"
                      "避免这部分气流被卷进前轮尾流白白浪费。")

        self.add("aero_ground.splitter_lip", "前唇", "aero_ground", "carbon_sandwich",
                 g.loft_rect((0.700, 0.0, 0.052), (0.0, 0.400, 0.0), (0.0, 0.0, 0.008),
                             (0.640, 0.0, 0.060), (0.0, 0.400, 0.0), (0.0, 0.0, 0.010)),
                 fill=0.80,
                 note="车底入口前缘的水平唇：把来流分成上下两路，"
                      "并阻止车头高压区从下方泄漏进通道。")


def build(params):
    """按参数字典生成全部零件。"""
    b = Builder(params)
    b.build_frame()
    b.build_cockpit()
    b.build_wheels()
    b.build_suspension()
    b.build_steering()
    b.build_pedal_box()
    b.build_powertrain()
    b.build_aero()
    b.build_ground_effect()
    b.build_body()
    return b.parts
