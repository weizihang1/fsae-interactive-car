"""纯 Python 参数化几何内核：图元 -> 三角网格 -> STL / GLB / 预览图。

设计原则
--------
1. 零第三方依赖（只用标准库 math / json / struct / zlib / array）。
   本机与 GitHub Actions 都能直接跑，不需要 500MB 级的 CAD 内核。
2. 每个图元自带解析体积，用于反算质量；缺失时用散度定理计算。
3. 闭合体自动修正三角面绕向，避免法线朝内导致的黑面。
4. 导出 GLB 时把零件顶点平移到自身包围盒中心，节点平移量即该中心，
   浏览器里做爆炸视图只需把节点位移乘以系数，动画代价为零。
"""
from __future__ import annotations

import array
import json
import math
import struct
import zlib


# --------------------------------------------------------------------------
# 向量工具
# --------------------------------------------------------------------------

def vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vmul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def vlen(a):
    return math.sqrt(vdot(a, a))


def vnorm(a):
    length = vlen(a)
    if length < 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0] / length, a[1] / length, a[2] / length)


def orthogonal_basis(axis):
    """返回与 axis 正交的一对单位向量 (u, v)，满足 u × v = axis。"""
    a = vnorm(axis)
    ref = (0.0, 0.0, 1.0) if abs(a[2]) < 0.9 else (1.0, 0.0, 0.0)
    u = vnorm(vcross(ref, a))
    return u, vcross(a, u)


def rot_z(p, deg, origin=(0.0, 0.0, 0.0)):
    ang = math.radians(deg)
    c, s = math.cos(ang), math.sin(ang)
    x, y, z = vsub(p, origin)
    return (origin[0] + x * c - y * s, origin[1] + x * s + y * c, origin[2] + z)


# --------------------------------------------------------------------------
# 网格
# --------------------------------------------------------------------------

class Mesh:
    __slots__ = ("verts", "tris", "volume")

    def __init__(self):
        self.verts = []
        self.tris = []
        self.volume = None

    def add(self, other):
        off = len(self.verts)
        self.verts.extend(other.verts)
        self.tris.extend((a + off, b + off, c + off) for a, b, c in other.tris)
        return self

    def add_all(self, others):
        for other in others:
            self.add(other)
        return self

    def mirrored_x(self):
        m = Mesh()
        m.verts = [(x, -y, z) for (x, y, z) in self.verts]
        m.tris = [(a, c, b) for (a, b, c) in self.tris]
        m.volume = self.volume
        return m

    def moved(self, delta):
        m = Mesh()
        m.verts = [vadd(v, delta) for v in self.verts]
        m.tris = list(self.tris)
        m.volume = self.volume
        return m

    def scaled(self, factor):
        m = Mesh()
        m.verts = [vmul(v, factor) for v in self.verts]
        m.tris = list(self.tris)
        m.volume = None if self.volume is None else self.volume * factor ** 3
        return m

    def signed_volume(self):
        total = 0.0
        for a, b, c in self.tris:
            total += vdot(self.verts[a], vcross(self.verts[b], self.verts[c]))
        return total / 6.0

    def volume_or_computed(self):
        if self.volume is not None:
            return self.volume
        return abs(self.signed_volume())

    def bounds(self):
        xs = [v[0] for v in self.verts]
        ys = [v[1] for v in self.verts]
        zs = [v[2] for v in self.verts]
        return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))

    def center(self):
        lo, hi = self.bounds()
        return ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)

    def size(self):
        lo, hi = self.bounds()
        return (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])

    def fix_winding(self):
        if self.signed_volume() < 0:
            self.tris = [(a, c, b) for a, b, c in self.tris]
        return self


def merge(meshes):
    m = Mesh()
    m.add_all(meshes)
    return m


# --------------------------------------------------------------------------
# 图元
# --------------------------------------------------------------------------

def tube(p0, p1, r0, r1=None, seg=10, wall=None, caps=True):
    """圆管 / 圆锥管。wall 给定时按空心管计算体积，用于质量估算。"""
    if r1 is None:
        r1 = r0
    m = Mesh()
    axis = vsub(p1, p0)
    length = vlen(axis)
    if length < 1e-9 or r0 <= 0:
        return m
    u, v = orthogonal_basis(axis)
    ring0, ring1 = [], []
    for i in range(seg):
        ang = 2 * math.pi * i / seg
        ca, sa = math.cos(ang), math.sin(ang)
        ring0.append(vadd(p0, vadd(vmul(u, ca * r0), vmul(v, sa * r0))))
        ring1.append(vadd(p1, vadd(vmul(u, ca * r1), vmul(v, sa * r1))))
    base = len(m.verts)
    m.verts.extend(ring0)
    m.verts.extend(ring1)
    i0, i1 = base, base + seg
    for i in range(seg):
        j = (i + 1) % seg
        m.tris.append((i0 + i, i0 + j, i1 + j))
        m.tris.append((i0 + i, i1 + j, i1 + i))
    if caps:
        c0 = len(m.verts)
        m.verts.append(p0)
        c1 = len(m.verts)
        m.verts.append(p1)
        for i in range(seg):
            j = (i + 1) % seg
            m.tris.append((c0, i0 + j, i0 + i))
            m.tris.append((c1, i1 + i, i1 + j))
    volume = math.pi * length * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0
    if wall is not None:
        ri0, ri1 = max(r0 - wall, 1e-6), max(r1 - wall, 1e-6)
        volume -= math.pi * length * (ri0 * ri0 + ri0 * ri1 + ri1 * ri1) / 3.0
    m.volume = volume
    return m.fix_winding()


def box(center, size, rz=0.0, rz_origin=None):
    """长方体，可绕 Z 轴旋转。size 为整体尺寸 (dx, dy, dz)。"""
    hx, hy, hz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
    corners = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
               (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
    origin = rz_origin if rz_origin else center
    m = Mesh()
    m.verts = [rot_z(vadd(center, c), rz, origin) for c in corners]
    m.tris = [(0, 2, 1), (0, 3, 2),
              (4, 5, 6), (4, 6, 7),
              (0, 1, 5), (0, 5, 4),
              (2, 3, 7), (2, 7, 6),
              (1, 2, 6), (1, 6, 5),
              (3, 0, 4), (3, 4, 7)]
    m.volume = size[0] * size[1] * size[2]
    return m.fix_winding()


def loft_rect(c0, w0, h0, c1, w1, h1):
    """两截面放样体。w / h 是半宽、半高的三维向量，可表达锥体、楔形、
    侧箱、扩散器、发动机包等大量车身结构。"""
    corners = []
    for center, w, h in ((c0, w0, h0), (c1, w1, h1)):
        for sw, sh in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            corners.append(vadd(center, vadd(vmul(w, sw), vmul(h, sh))))
    m = Mesh()
    m.verts = corners
    m.tris = [(0, 1, 2), (0, 2, 3),
              (4, 6, 5), (4, 7, 6),
              (0, 4, 5), (0, 5, 1),
              (1, 5, 6), (1, 6, 2),
              (2, 6, 7), (2, 7, 3),
              (3, 7, 4), (3, 4, 0)]
    return m.fix_winding()


def sphere(center, r, seg=12, rings=8):
    verts = [(center[0], center[1], center[2] + r)]
    for j in range(1, rings):
        phi = math.pi * j / rings
        z = math.cos(phi) * r
        rr = math.sin(phi) * r
        for i in range(seg):
            ang = 2 * math.pi * i / seg
            verts.append((center[0] + rr * math.cos(ang),
                          center[1] + rr * math.sin(ang),
                          center[2] + z))
    verts.append((center[0], center[1], center[2] - r))
    top, bottom = 0, len(verts) - 1
    tris = []
    for i in range(seg):
        tris.append((top, 1 + (i + 1) % seg, 1 + i))
    for j in range(rings - 2):
        a0 = 1 + j * seg
        b0 = 1 + (j + 1) * seg
        for i in range(seg):
            i2 = (i + 1) % seg
            tris.append((a0 + i, a0 + i2, b0 + i2))
            tris.append((a0 + i, b0 + i2, b0 + i))
    last = 1 + (rings - 2) * seg
    for i in range(seg):
        tris.append((bottom, last + i, last + (i + 1) % seg))
    m = Mesh()
    m.verts = verts
    m.tris = tris
    m.volume = 4.0 / 3.0 * math.pi * r ** 3
    return m.fix_winding()


def revolve(profile, origin, axis, seg=16):
    """把 (半径, 沿轴高度) 剖面绕轴旋转成回转体。
    适合轮胎、轮辋、制动盘、轮毂、头锥。"""
    axis_n = vnorm(axis)
    u, v = orthogonal_basis(axis_n)
    n = len(profile)
    m = Mesh()
    if n < 2:
        return m
    for i in range(seg):
        ang = 2 * math.pi * i / seg
        radial = vadd(vmul(u, math.cos(ang)), vmul(v, math.sin(ang)))
        for (rr, hh) in profile:
            m.verts.append(vadd(vadd(origin, vmul(axis_n, hh)), vmul(radial, rr)))

    def vid(i, j):
        return i * n + j

    for i in range(seg):
        i2 = (i + 1) % seg
        for j in range(n - 1):
            m.tris.append((vid(i, j), vid(i2, j), vid(i2, j + 1)))
            m.tris.append((vid(i, j), vid(i2, j + 1), vid(i, j + 1)))
    closed = abs(profile[0][0] - profile[-1][0]) < 1e-9 and abs(profile[0][1] - profile[-1][1]) < 1e-9
    if not closed and profile[0][0] > 1e-6:
        c = len(m.verts)
        m.verts.append(vadd(origin, vmul(axis_n, profile[0][1])))
        for i in range(seg):
            m.tris.append((c, vid(i, 0), vid((i + 1) % seg, 0)))
    if not closed and profile[-1][0] > 1e-6:
        c = len(m.verts)
        m.verts.append(vadd(origin, vmul(axis_n, profile[-1][1])))
        for i in range(seg):
            m.tris.append((c, vid((i + 1) % seg, n - 1), vid(i, n - 1)))
    return m.fix_winding()


def extrude_dir(outline, origin, dir_x, dir_y, depth):
    """把二维闭合折线沿 dir_y 拉伸 depth。
    outline 的点位于 (dir_x, dir_x×dir_y) 平面，适合翼片、端板、平板件。"""
    dx = vnorm(dir_x)
    dy = vnorm(dir_y)
    dz = vcross(dx, dy)
    n = len(outline)
    m = Mesh()
    for s in (0.0, depth):
        for (a, b) in outline:
            m.verts.append(vadd(vadd(origin, vmul(dy, s)),
                                vadd(vmul(dx, a), vmul(dz, b))))
    for i in range(1, n - 1):
        m.tris.append((0, i, i + 1))
        m.tris.append((n, n + i + 1, n + i))
    for i in range(n):
        j = (i + 1) % n
        # 侧面绕向必须与端面一致（都朝外）。原先写成 (i, j, n+j) 会让侧面朝内，
        # 与端面朝向相反，散度定理算出的体积只有真值的 1/3。
        m.tris.append((i, n + i, n + j))
        m.tris.append((i, n + j, j))
    return m.fix_winding()


def airfoil_outline(chord, thick=0.12, camber=0.04, n=20):
    """简化 NACA 四位翼型轮廓，返回 (x, y) 列表（单位米）。"""
    m_ = camber
    p_ = 0.4

    def half(x, sign):
        yt = 5 * thick * (0.2969 * math.sqrt(x) - 0.1260 * x
                          - 0.3516 * x ** 2 + 0.2843 * x ** 3 - 0.1036 * x ** 4)
        if x < p_:
            yc = m_ / (p_ ** 2) * (2 * p_ * x - x ** 2)
        else:
            yc = m_ / ((1 - p_) ** 2) * ((1 - 2 * p_) + 2 * p_ * x - x ** 2)
        return (x * chord, (yc + sign * yt) * chord)

    upper = [half(0.5 * (1 - math.cos(math.pi * i / n)), 1) for i in range(n + 1)]
    lower = [half(0.5 * (1 - math.cos(math.pi * i / n)), -1) for i in range(n, 0, -1)]
    return upper + lower


def plate(p0, p1, width, thick, up=(0.0, 0.0, 1.0)):
    """p0-p1 之间的平板，用于底板、防火墙、隔板。"""
    d = vsub(p1, p0)
    n = vnorm(vcross(d, up))
    if vlen(n) < 1e-9:
        n = (0.0, 1.0, 0.0)
    w = vmul(vnorm(up), width / 2.0)
    t = vmul(n, thick / 2.0)
    c0, c1 = vsub(p0, t), vadd(p0, t)
    c2, c3 = vsub(p1, t), vadd(p1, t)
    m = Mesh()
    m.verts = [vsub(c0, w), vadd(c0, w), vadd(c1, w), vsub(c1, w),
               vsub(c2, w), vadd(c2, w), vadd(c3, w), vsub(c3, w)]
    m.tris = [(0, 1, 2), (0, 2, 3),
              (4, 5, 6), (4, 6, 7),
              (0, 4, 5), (0, 5, 1),
              (1, 5, 6), (1, 6, 2),
              (2, 6, 7), (2, 7, 3),
              (3, 7, 4), (3, 4, 0)]
    m.volume = vlen(d) * width * thick
    return m.fix_winding()


# --------------------------------------------------------------------------
# 导出：二进制 STL（给 CFD 与三维打印用）
# --------------------------------------------------------------------------

def write_stl(path, mesh):
    with open(path, "wb") as handle:
        handle.write(b"\0" * 80)
        handle.write(struct.pack("<I", len(mesh.tris)))
        for a, b, c in mesh.tris:
            p, q, r = mesh.verts[a], mesh.verts[b], mesh.verts[c]
            n = vnorm(vcross(vsub(q, p), vsub(r, p)))
            handle.write(struct.pack("<3f", *n))
            for v in (p, q, r):
                handle.write(struct.pack("<3f", *v))
            handle.write(struct.pack("<H", 0))


# --------------------------------------------------------------------------
# 导出：GLB（glTF 2.0 二进制，给浏览器用）
# --------------------------------------------------------------------------

def _flat_arrays(verts, tris, translate):
    """平坦着色，按 位置 + 法线 去重，显著压缩体积。"""
    positions, normals, indices = [], [], []
    cache = {}
    for a, b, c in tris:
        p = vsub(verts[a], translate)
        q = vsub(verts[b], translate)
        r = vsub(verts[c], translate)
        n = vnorm(vcross(vsub(q, p), vsub(r, p)))
        if n == (0.0, 0.0, 0.0):
            continue
        nk = (round(n[0], 3), round(n[1], 3), round(n[2], 3))
        for v in (p, q, r):
            key = (round(v[0], 4), round(v[1], 4), round(v[2], 4)) + nk
            index = cache.get(key)
            if index is None:
                index = len(positions)
                cache[key] = index
                positions.append(v)
                normals.append(n)
            indices.append(index)
    return positions, normals, indices


def write_glb(path, parts, materials=None):
    """写单个 GLB，内含全部零件。
    节点层级 root -> system -> part，part 节点自带平移，爆炸视图可直接使用。"""
    materials = materials or {}
    chunks = []
    offset = 0
    buffer_views, accessors, meshes, nodes = [], [], [], []
    material_ids = {}
    material_list = []

    def add_view(data, target):
        nonlocal offset
        pad = (-len(data)) % 4
        buffer_views.append({"buffer": 0, "byteOffset": offset,
                             "byteLength": len(data), "target": target})
        chunks.append(data + b"\0" * pad)
        offset += len(data) + pad
        return len(buffer_views) - 1

    def add_accessor(view, comp_type, count, acc_type, minmax=None):
        acc = {"bufferView": view, "componentType": comp_type,
               "count": count, "type": acc_type}
        if minmax:
            acc["min"], acc["max"] = minmax
        accessors.append(acc)
        return len(accessors) - 1

    for part in parts:
        name = part.get("material", "default")
        if name not in material_ids:
            spec = materials.get(name, {"color": [0.7, 0.7, 0.72, 1.0],
                                        "metal": 0.1, "rough": 0.6})
            material_list.append({
                "name": name,
                "pbrMetallicRoughness": {
                    "baseColorFactor": spec["color"],
                    "metallicFactor": spec.get("metal", 0.1),
                    "roughnessFactor": spec.get("rough", 0.6),
                },
                "doubleSided": True,
            })
            material_ids[name] = len(material_list) - 1

    total_verts = 0
    total_tris = 0
    for part in parts:
        mesh = part["mesh"]
        center = mesh.center()
        positions, normals, indices = _flat_arrays(mesh.verts, mesh.tris, center)
        if not positions:
            continue
        total_verts += len(positions)
        total_tris += len(indices) // 3
        pos_bytes = b"".join(struct.pack("<3f", *v) for v in positions)
        nrm_bytes = b"".join(struct.pack("<3f", *v) for v in normals)
        idx_bytes = b"".join(struct.pack("<I", i) for i in indices)
        view_pos = add_view(pos_bytes, 34962)
        view_nrm = add_view(nrm_bytes, 34962)
        view_idx = add_view(idx_bytes, 34963)
        mn = [min(v[i] for v in positions) for i in range(3)]
        mx = [max(v[i] for v in positions) for i in range(3)]
        acc_pos = add_accessor(view_pos, 5126, len(positions), "VEC3", [mn, mx])
        acc_nrm = add_accessor(view_nrm, 5126, len(normals), "VEC3")
        acc_idx = add_accessor(view_idx, 5125, len(indices), "SCALAR")
        meshes.append({"name": part["id"], "primitives": [{
            "attributes": {"POSITION": acc_pos, "NORMAL": acc_nrm},
            "indices": acc_idx,
            "material": material_ids[part.get("material", "default")],
        }]})
        nodes.append({
            "name": part["id"],
            "mesh": len(meshes) - 1,
            "translation": [round(center[0], 6), round(center[1], 6), round(center[2], 6)],
            # 注意：three.js 的 GLTFLoader 会用 PropertyBinding.sanitizeNodeName 清洗节点名，
            # 把其中的 "." 等字符删掉。因此零件 ID 必须另存一份在 extras 里（extras 不会被清洗），
            # 网页端一律以 extras.id 为准。
            "extras": {"id": part["id"], "system": part.get("system", ""),
                       "label": part.get("name", "")},
        })
        part["_node"] = len(nodes) - 1

    grouped = {}
    for part in parts:
        if "_node" in part:
            grouped.setdefault(part.get("system", "misc"), []).append(part["_node"])
    system_nodes = []
    for system, children in grouped.items():
        nodes.append({"name": system, "children": children})
        system_nodes.append(len(nodes) - 1)
    nodes.append({"name": "car", "children": system_nodes})
    root = len(nodes) - 1

    blob = b"".join(chunks)
    gltf = {
        "asset": {"version": "2.0", "generator": "fsae-geom (pure python)"},
        "scene": 0,
        "scenes": [{"nodes": [root]}],
        "nodes": nodes,
        "meshes": meshes,
        "materials": material_list,
        "accessors": accessors,
        "bufferViews": buffer_views,
        "buffers": [{"byteLength": len(blob)}],
    }
    payload = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    payload += b" " * ((-len(payload)) % 4)
    blob += b"\0" * ((-len(blob)) % 4)
    total = 12 + 8 + len(payload) + 8 + len(blob)
    with open(path, "wb") as handle:
        handle.write(struct.pack("<III", 0x46546C67, 2, total))
        handle.write(struct.pack("<II", len(payload), 0x4E4F534A))
        handle.write(payload)
        handle.write(struct.pack("<II", len(blob), 0x004E4942))
        handle.write(blob)
    return {"bytes": total, "vertices": total_verts, "triangles": total_tris,
            "nodes": len(nodes), "materials": len(material_list)}


# --------------------------------------------------------------------------
# 导出：PNG 预览（纯标准库光栅化，用于无人值守时的视觉校验）
# --------------------------------------------------------------------------

def _png_write(path, width, height, rows):
    raw = bytearray()
    for row in rows:
        raw.append(0)
        raw.extend(row)

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    with open(path, "wb") as handle:
        handle.write(b"\x89PNG\r\n\x1a\n")
        handle.write(chunk(b"IHDR", header))
        handle.write(chunk(b"IDAT", zlib.compress(bytes(raw), 6)))
        handle.write(chunk(b"IEND", b""))


def render_preview(path, parts, width=1100, height=680,
                   azimuth=38.0, elevation=20.0, zoom=0.90,
                   bg=(0.08, 0.09, 0.11)):
    """正交投影加单方向光的 Z-buffer 光栅化预览，无第三方依赖。"""
    az = math.radians(azimuth)
    el = math.radians(elevation)
    fwd = (math.cos(el) * math.sin(az), -math.cos(el) * math.cos(az), -math.sin(el))
    right = vnorm(vcross((0.0, 0.0, 1.0), fwd))
    cam_up = vcross(fwd, right)

    def project(v):
        return (vdot(v, right), vdot(v, cam_up), vdot(v, fwd))

    triangles = []
    for part in parts:
        mesh = part["mesh"]
        color = part.get("_rgb", (0.72, 0.72, 0.74))
        for a, b, c in mesh.tris:
            triangles.append((mesh.verts[a], mesh.verts[b], mesh.verts[c], color))
    if not triangles:
        return None

    projected = [project(v) for tri in triangles for v in tri[:3]]
    xs = [p[0] for p in projected]
    ys = [p[1] for p in projected]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    span_x = (max(xs) - min(xs)) or 1.0
    span_y = (max(ys) - min(ys)) or 1.0
    scale = min(width / span_x, height / span_y) * zoom

    def to_pixel(p):
        return ((p[0] - cx) * scale + width / 2, height / 2 - (p[1] - cy) * scale)

    depth = array.array("f", [1e9]) * (width * height)
    buffer = bytearray(width * height * 3)
    bg_r, bg_g, bg_b = int(bg[0] * 255), int(bg[1] * 255), int(bg[2] * 255)
    for i in range(width * height):
        buffer[i * 3] = bg_r
        buffer[i * 3 + 1] = bg_g
        buffer[i * 3 + 2] = bg_b

    light = vnorm((-0.45, -0.75, 0.70))
    for (p0, p1, p2, color) in triangles:
        a2, b2, c2 = project(p0), project(p1), project(p2)
        normal = vnorm(vcross(vsub(p1, p0), vsub(p2, p0)))
        if normal == (0.0, 0.0, 0.0):
            continue
        if vdot(normal, fwd) > 0:
            normal = vmul(normal, -1.0)
        shade = 0.24 + 0.76 * max(0.0, vdot(normal, light))
        red = min(255, int(color[0] * shade * 255))
        green = min(255, int(color[1] * shade * 255))
        blue = min(255, int(color[2] * shade * 255))
        x0, y0 = to_pixel(a2)
        x1, y1 = to_pixel(b2)
        x2, y2 = to_pixel(c2)
        minx = max(0, int(min(x0, x1, x2)))
        maxx = min(width - 1, int(max(x0, x1, x2)) + 1)
        miny = max(0, int(min(y0, y1, y2)))
        maxy = min(height - 1, int(max(y0, y1, y2)) + 1)
        if maxx < minx or maxy < miny:
            continue
        area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        if abs(area) < 1e-9:
            continue
        for py in range(miny, maxy + 1):
            sy = py + 0.5
            base = py * width
            for px in range(minx, maxx + 1):
                sx = px + 0.5
                w0 = ((x1 - x0) * (sy - y0) - (sx - x0) * (y1 - y0)) / area
                w1 = ((sx - x0) * (y2 - y0) - (x2 - x0) * (sy - y0)) / area
                w2 = 1.0 - w0 - w1
                if w0 < 0.0 or w1 < 0.0 or w2 < 0.0:
                    continue
                z = w2 * a2[2] + w1 * b2[2] + w0 * c2[2]
                k = base + px
                if z < depth[k]:
                    depth[k] = z
                    o = k * 3
                    buffer[o] = red
                    buffer[o + 1] = green
                    buffer[o + 2] = blue

    rows = [bytes(buffer[y * width * 3:(y + 1) * width * 3]) for y in range(height)]
    _png_write(path, width, height, rows)
    return path
