"""构建流水线：参数 -> 几何 -> GLB / STL / 零件清单 / 预览图 / 单文件网页。

用法：
    python tools/build.py            # 全量构建
    python tools/build.py --no-stl   # 跳过逐零件 STL（更快）
"""
from __future__ import annotations

import base64
import datetime
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import car_model  # noqa: E402
import geom as g  # noqa: E402

PARAMS = os.path.join(ROOT, "params", "car.json")
OUT = os.path.join(ROOT, "outputs")
STL_DIR = os.path.join(OUT, "stl")
SITE = os.path.join(ROOT, "site")


def load_params():
    with open(PARAMS, "r", encoding="utf-8") as handle:
        return json.load(handle)


def build_manifest(parts, params, glb_info):
    systems = []
    by_system = {}
    for part in parts:
        by_system.setdefault(part["system"], []).append(part)
    order = {sid: i for i, (sid, _) in enumerate(car_model.SYSTEMS)}
    for sid, label in car_model.SYSTEMS:
        items = by_system.get(sid, [])
        if not items:
            continue
        center = [round(v, 4) for v in part_center(items)]
        entry = {
            "id": sid,
            "label": label,
            "count": len(items),
            "mass_kg": round(sum(p["mass_kg"] for p in items), 2),
            "center": center,
            "parts": [{
                "id": p["id"],
                "name": p["name"],
                "material": p["material"],
                "material_label": car_model.MATERIAL_LABELS.get(p["material"], p["material"]),
                "mass_kg": round(p["mass_kg"], 3),
                "volume_m3": round(p["volume_m3"], 6),
                "fill": round(p.get("fill", 1.0), 3),
                "note": p.get("note", ""),
                "center": [round(v, 4) for v in p["mesh"].center()],
                "size": [round(v, 4) for v in p["mesh"].size()],
                "node": p.get("_node"),
            } for p in sorted(items, key=lambda x: -x["mass_kg"])],
        }
        systems.append(entry)
    systems.sort(key=lambda s: order.get(s["id"], 99))
    total_parts = sum(s["count"] for s in systems)
    total_mass = sum(s["mass_kg"] for s in systems)
    lo, hi = total_bounds(parts)
    return {
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        "source": os.path.basename(PARAMS),
        "systems": systems,
        "totals": {
            "parts": total_parts,
            "mass_kg": round(total_mass, 1),
            "triangles": glb_info["triangles"] if glb_info else 0,
            "bbox_min": [round(v, 3) for v in lo],
            "bbox_max": [round(v, 3) for v in hi],
            "length_m": round(hi[0] - lo[0], 3),
            "width_m": round(hi[1] - lo[1], 3),
            "height_m": round(hi[2] - lo[2], 3),
        },
        "geometry_params": params,
        "disclaimer": "参数为原型占位值，非车队实测数据；质量由几何体积与材料密度估算，不含紧固件与线束。",
    }


def part_center(items):
    n = len(items)
    return (sum(p["mesh"].center()[0] for p in items) / n,
            sum(p["mesh"].center()[1] for p in items) / n,
            sum(p["mesh"].center()[2] for p in items) / n)


def total_bounds(parts):
    los, his = [], []
    for p in parts:
        lo, hi = p["mesh"].bounds()
        los.append(lo)
        his.append(hi)
    return (min(v[0] for v in los), min(v[1] for v in los), min(v[2] for v in los)), \
           (max(v[0] for v in his), max(v[1] for v in his), max(v[2] for v in his))


def write_part_stls(parts):
    os.makedirs(STL_DIR, exist_ok=True)
    count = 0
    for part in parts:
        fname = part["id"] + ".stl"
        g.write_stl(os.path.join(STL_DIR, fname), part["mesh"])
        count += 1
    return count


def publish_stls():
    """把逐零件 STL 复制到 site/stl，供网页里的下载链接使用。"""
    src = STL_DIR
    dst = os.path.join(SITE, "stl")
    if not os.path.isdir(src):
        return 0
    os.makedirs(dst, exist_ok=True)
    count = 0
    for name in os.listdir(src):
        if name.lower().endswith(".stl"):
            with open(os.path.join(src, name), "rb") as r:
                data = r.read()
            with open(os.path.join(dst, name), "wb") as w:
                w.write(data)
            count += 1
    return count


def build_site(manifest, glb_path):
    template = os.path.join(SITE, "template.html")
    if not os.path.exists(template):
        print("  [跳过网页] 缺少 site/template.html")
        return None
    bundle_path = os.path.join(SITE, "vendor", "app.bundle.js")
    if not os.path.exists(bundle_path):
        print("  [跳过网页] 缺少 site/vendor/app.bundle.js，请先运行 node tools/bundle_site.js")
        return None
    with open(template, "r", encoding="utf-8") as handle:
        html = handle.read()
    with open(bundle_path, "r", encoding="utf-8") as handle:
        bundle = handle.read().replace("</script>", "<\\/script>")
    with open(glb_path, "rb") as handle:
        blob = base64.b64encode(handle.read()).decode("ascii")
    html = html.replace("__APP_BUNDLE__", bundle)
    html = html.replace("__CAR_GLB_BASE64__", blob)
    html = html.replace("__PARTS_JSON__", json.dumps(manifest, ensure_ascii=False))
    out = os.path.join(SITE, "index.html")
    with open(out, "w", encoding="utf-8") as handle:
        handle.write(html)
    return out


def main():
    args = set(sys.argv[1:])
    t0 = time.time()
    params = load_params()
    print("参数：%s" % os.path.basename(PARAMS))
    parts = car_model.build(params)
    print("零件：%d 个" % len(parts))

    os.makedirs(OUT, exist_ok=True)
    glb_path = os.path.join(OUT, "car.glb")
    glb_info = g.write_glb(glb_path, parts, car_model.MATERIALS)
    print("GLB：%.2f MB，三角面 %d，节点 %d，材质 %d"
          % (glb_info["bytes"] / 1048576, glb_info["triangles"],
             glb_info["nodes"], glb_info["materials"]))

    manifest = build_manifest(parts, params, glb_info)
    manifest_path = os.path.join(OUT, "parts.json")
    with open(manifest_path, "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=1)

    print("")
    print("分系统质量：")
    for entry in manifest["systems"]:
        print("  %-10s %3d 件  %7.2f kg" % (entry["label"], entry["count"], entry["mass_kg"]))
    t = manifest["totals"]
    print("  %-10s %3d 件  %7.2f kg" % ("合计", t["parts"], t["mass_kg"]))
    print("整车尺寸：长 %.2f m × 宽 %.2f m × 高 %.2f m"
          % (t["length_m"], t["width_m"], t["height_m"]))

    if "--no-stl" not in args:
        n = write_part_stls(parts)
        m = publish_stls()
        print("逐零件 STL：%d 个 -> outputs/stl/（并发布 %d 个到 site/stl/）" % (n, m))

    # 方位角约定：0° 看左侧面，90° 看车头，-90° 看车尾，38° 为前侧三分之四视角
    views = [("preview_3q.png", 38.0, 20.0), ("preview_front.png", 90.0, 6.0),
             ("preview_rear.png", -90.0, 6.0), ("preview_side.png", 0.0, 4.0),
             ("preview_top.png", 0.0, 85.0)]
    for name, az, el in views:
        g.render_preview(os.path.join(OUT, name), parts, azimuth=az, elevation=el)
    print("预览图：5 张 -> outputs/")

    page = build_site(manifest, glb_path)
    if page:
        print("网页：%s（%.2f MB，单文件、离线可开）"
              % (os.path.relpath(page, ROOT), os.path.getsize(page) / 1048576))
    print("耗时 %.1f 秒" % (time.time() - t0))


if __name__ == "__main__":
    main()
