"""标准件与工业细节库。

本内核没有布尔运算，所以"从实心体上挖孔"这类操作无法直接表达。
带孔的形状（螺母通孔、关节轴承眼环、垫圈）一律用环形实体或回转体构造，
形状是准确的；紧固件由若干相互重叠的图元叠合而成。

用途说明：这些零件适合装配表达与渲染，不是水密单实体。
若要 3D 打印或做 CFD，需要先在真正的 CAD 里做布尔合并。
"""
from __future__ import annotations

import math

import geom as g


# --------------------------------------------------------------------------
# 基础构造
# --------------------------------------------------------------------------

def prism(profile, origin, u, v, w, length):
    """把二维凸多边形沿 w 拉伸 length。(u, v, w) 需构成右手系。"""
    n = len(profile)
    m = g.Mesh()
    for s in (0.0, length):
        for (a, b) in profile:
            m.verts.append(g.vadd(g.vadd(origin, g.vmul(w, s)),
                                  g.vadd(g.vmul(u, a), g.vmul(v, b))))
    for i in range(1, n - 1):
        m.tris.append((0, i, i + 1))
        m.tris.append((n, n + i + 1, n + i))
    for i in range(n):
        j = (i + 1) % n
        m.tris.append((i, n + i, n + j))
        m.tris.append((i, n + j, j))
    return m.fix_winding()


def resample(poly, n):
    """按周长等分把多边形重采样成 n 个点（用于把多边形与圆对接）。"""
    pts = list(poly)
    count = len(pts)
    lengths = []
    total = 0.0
    for i in range(count):
        a, b = pts[i], pts[(i + 1) % count]
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        lengths.append(d)
        total += d
    out = []
    for k in range(n):
        target = total * k / n
        acc = 0.0
        for i, d in enumerate(lengths):
            if d <= 0.0:
                continue
            if acc + d >= target - 1e-12:
                f = (target - acc) / d
                a, b = pts[i], pts[(i + 1) % count]
                out.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
                break
            acc += d
        else:
            out.append(pts[0])
    return out


def ring_solid(outer2d, inner_r, origin, u, v, w, height):
    """外轮廓为多边形、内轮廓为圆的环形实体（六角螺母、套筒的本体）。"""
    n = len(outer2d)
    m = g.Mesh()

    def at2(pt, s):
        return g.vadd(g.vadd(origin, g.vmul(w, s)),
                      g.vadd(g.vmul(u, pt[0]), g.vmul(v, pt[1])))

    def circle(r, s):
        return [at2((r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)), s)
                for i in range(n)]

    o0 = [at2(p, 0.0) for p in outer2d]
    o1 = [at2(p, height) for p in outer2d]
    i0 = circle(inner_r, 0.0)
    i1 = circle(inner_r, height)
    for ring in (o0, o1, i0, i1):
        m.verts.extend(ring)

    def vid(base, i):
        return base + i % n

    for i in range(n):
        j = (i + 1) % n
        # 外壁
        m.tris.append((vid(0, i), vid(0, j), vid(n, j)))
        m.tris.append((vid(0, i), vid(n, j), vid(n, i)))
        # 内壁（朝向相反）
        m.tris.append((vid(2 * n, i), vid(3 * n, j), vid(2 * n, j)))
        m.tris.append((vid(2 * n, i), vid(3 * n, i), vid(3 * n, j)))
        # 顶面
        m.tris.append((vid(n, i), vid(n, j), vid(3 * n, j)))
        m.tris.append((vid(n, i), vid(3 * n, j), vid(3 * n, i)))
        # 底面
        m.tris.append((vid(0, i), vid(2 * n, j), vid(0, j)))
        m.tris.append((vid(0, i), vid(2 * n, i), vid(2 * n, j)))
    return m.fix_winding()


def hex_profile(af):
    """对边距为 af 的六边形。"""
    r = 2.0 * af / math.sqrt(3.0)
    return [(r * math.cos(math.pi / 6.0 + i * math.pi / 3.0),
             r * math.sin(math.pi / 6.0 + i * math.pi / 3.0)) for i in range(6)]


# --------------------------------------------------------------------------
# 紧固件
# --------------------------------------------------------------------------

def hex_prism(origin, axis, af, length):
    a = g.vnorm(axis)
    u, v = g.orthogonal_basis(a)
    return prism(hex_profile(af), origin, u, v, a, length)


def hex_nut(center, axis, af, height, bore_r, seg=24, nylon=False):
    """六角螺母（带通孔）。nylon=True 时在端部加一圈尼龙锁紧环。"""
    a = g.vnorm(axis)
    u, v = g.orthogonal_basis(a)
    n = seg
    body = ring_solid(resample(hex_profile(af), n), bore_r,
                      g.vadd(center, g.vmul(a, -height / 2.0)), u, v, a, height)
    if not nylon:
        return body
    collar_h = height * 0.22
    collar = g.revolve(
        [(bore_r, 0.0), (af * 0.5, 0.0), (af * 0.5, collar_h), (bore_r, collar_h), (bore_r, 0.0)],
        g.vadd(center, g.vmul(a, height / 2.0)), a, seg=n)
    body.add(collar)
    return body


def washer(center, axis, id_r, od_r, t):
    """平垫圈：闭合回转剖面，中心自然留孔。"""
    return g.revolve(
        [(id_r, -t / 2.0), (od_r, -t / 2.0), (od_r, t / 2.0), (id_r, t / 2.0), (id_r, -t / 2.0)],
        center, axis, seg=20)


def thread_shank(p0, p1, d, pitch=None, seg=14):
    """螺纹杆：圆柱 + 若干道牙顶环，用来读作螺纹。"""
    r = d / 2.0
    if pitch is None:
        pitch = d * 0.16
    m = g.tube(p0, p1, r * 0.86, seg=seg)
    axis = g.vsub(p1, p0)
    length = g.vlen(axis)
    if length <= pitch:
        return m
    a = g.vnorm(axis)
    count = max(1, int(length / pitch))
    step = (length - pitch) / count
    for k in range(count):
        c = g.vadd(p0, g.vmul(a, pitch / 2.0 + k * step))
        m.add(g.tube(g.vadd(c, g.vmul(a, -pitch * 0.30)), g.vadd(c, g.vmul(a, pitch * 0.30)),
                     r, seg=seg, caps=False))
    return m


def hex_bolt(head_center, axis, d, length, af=None, head_h=None, seg=14):
    """六角头螺栓。axis 由螺栓头指向被连接件内部。"""
    a = g.vnorm(axis)
    if af is None:
        af = d * 1.6
    if head_h is None:
        head_h = d * 0.65
    m = hex_prism(g.vadd(head_center, g.vmul(a, -head_h)), a, af, head_h)
    m.add(thread_shank(head_center, g.vadd(head_center, g.vmul(a, length)), d, seg=seg))
    return m


def socket_screw(head_center, axis, d, length, head_d=None, head_h=None, seg=16):
    """内六角圆柱头螺钉：圆柱头 + 倒角 + 螺纹杆，头部端面加一圈沉孔示意。"""
    a = g.vnorm(axis)
    if head_d is None:
        head_d = d * 1.6
    if head_h is None:
        head_h = d
    m = g.revolve(
        [(0.20 * head_d, 0.0), (0.5 * head_d - 0.0006, 0.0), (0.5 * head_d, 0.0012),
         (0.5 * head_d, head_h - 0.0012), (0.5 * head_d - 0.0006, head_h),
         (0.20 * head_d, head_h), (0.20 * head_d, 0.0)],
        g.vadd(head_center, g.vmul(a, head_h)), a, seg=seg)
    m.add(thread_shank(head_center, g.vadd(head_center, g.vmul(a, length)), d, seg=14))
    return m


# --------------------------------------------------------------------------
# 杆端件
# --------------------------------------------------------------------------

def rod_end(center, axis, bore_axis, bore_r=0.0040, eye_r=0.0105, eye_w=0.0090,
            thread_d=0.0100, shank_len=0.042, nut_af=None, seg=16):
    """关节轴承（杆端关节轴承 / heim joint）：眼环 + 颈部 + 螺纹杆 + 锁紧螺母。"""
    a = g.vnorm(axis)
    b = g.vnorm(bore_axis)
    m = g.revolve(
        [(bore_r, -eye_w / 2), (eye_r, -eye_w / 2), (eye_r, eye_w / 2),
         (bore_r, eye_w / 2), (bore_r, -eye_w / 2)],
        center, b, seg=seg)
    neck0 = g.vadd(center, g.vmul(a, -eye_r * 0.85))
    neck1 = g.vadd(center, g.vmul(a, -eye_r * 2.30))
    m.add(g.tube(neck0, neck1, eye_r * 0.78, thread_d / 2.0, seg=seg))
    shank1 = g.vadd(neck1, g.vmul(a, -shank_len))
    m.add(thread_shank(neck1, shank1, thread_d, seg=14))
    if nut_af:
        m.add(hex_nut(g.vadd(neck1, g.vmul(a, -thread_d * 1.9)), a, nut_af,
                      thread_d * 0.85, thread_d * 0.55, seg=20))
    return m


def clevis(center, axis, bore_axis, bore_r=0.0032, gap=0.012, ear_w=0.006,
           thread_d=0.0100, shank_len=0.030, nut_af=None, seg=12):
    """U 形叉头（拨叉）：两片耳板 + 过渡块 + 螺纹杆 + 锁紧螺母。"""
    a = g.vnorm(axis)
    b = g.vnorm(bore_axis)
    side = g.vnorm(g.vcross(b, a))
    m = g.Mesh()
    for sgn in (1.0, -1.0):
        off = g.vmul(side, sgn * (gap / 2.0 + ear_w / 2.0))
        m.add(g.revolve(
            [(bore_r, -ear_w / 2), (bore_r * 2.6, -ear_w / 2), (bore_r * 2.6, ear_w / 2),
             (bore_r, ear_w / 2), (bore_r, -ear_w / 2)],
            g.vadd(center, off), b, seg=seg))
    block0 = g.vadd(center, g.vmul(a, -bore_r * 2.6))
    block1 = g.vadd(center, g.vmul(a, -bore_r * 2.6 - thread_d * 1.3))
    u, v = g.orthogonal_basis(a)
    m.add(prism([(gap / 2.0 + ear_w, -bore_r * 2.6), (gap / 2.0 + ear_w, bore_r * 2.6),
                 (-(gap / 2.0 + ear_w), bore_r * 2.6), (-(gap / 2.0 + ear_w), -bore_r * 2.6)],
                block0, u, v, a, -(block1[0] - block0[0]) if False else
                g.vlen(g.vsub(block1, block0))))
    shank1 = g.vadd(block1, g.vmul(a, -shank_len))
    m.add(thread_shank(block1, shank1, thread_d, seg=14))
    if nut_af:
        m.add(hex_nut(g.vadd(block1, g.vmul(a, -thread_d * 1.9)), a, nut_af,
                      thread_d * 0.85, thread_d * 0.55, seg=20))
    return m


# --------------------------------------------------------------------------
# 焊接与管件细节
# --------------------------------------------------------------------------

def weld_bead(point, direction, r, scale=1.30, width=0.0075, seg=10):
    """管端焊缝：在接头处加一圈略粗的凸起。"""
    a = g.vnorm(direction)
    return g.tube(g.vadd(point, g.vmul(a, -width * 0.5)),
                  g.vadd(point, g.vmul(a, width * 0.5)),
                  r * scale, seg=seg, caps=False)


def tube_chamfered(p0, p1, r, wall=None, chamfer=None, seg=10):
    """两端倒角的管件：管身 + 两端倒角锥，去除毛刺感。"""
    if chamfer is None:
        chamfer = max(0.0006, min(r * 0.18, 0.0015))
    axis = g.vsub(p1, p0)
    length = g.vlen(axis)
    if length < chamfer * 3.0:
        return g.tube(p0, p1, r, seg=seg, wall=wall)
    a = g.vnorm(axis)
    m = g.tube(g.vadd(p0, g.vmul(a, chamfer)), g.vadd(p1, g.vmul(a, -chamfer)), r,
               seg=seg, wall=wall)
    m.add(g.tube(p0, g.vadd(p0, g.vmul(a, chamfer)), r - chamfer, r, seg=seg))
    m.add(g.tube(g.vadd(p1, g.vmul(a, -chamfer)), p1, r, r - chamfer, seg=seg))
    m.volume = None
    return m
