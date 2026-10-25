"""象の鼻テラス (横浜市中区海岸通1, 2009, 設計: 小泉雅生/小泉アトリエ) の建物を Blender で作り、USD に書き出す。

    "C:\\Program Files\\Blender Foundation\\Blender 4.4\\blender.exe" -b --factory-startup ^
        -P tools\\build_zounohana_terrace.py -- venues\\zounohana_terrace

で venues\\zounohana_terrace.blend と venues\\zounohana_terrace.usdc ができる。

座標: 1 単位 = 1 m、+x = 東、+y = 北、+z = 上、z = 0 が前の広場の地面。
原点は北東のガラス面の正面入口から 6 m 手前の広場 (Go2 の初期位置)。入口は原点から見て
yaw = SPAWN_YAW_DEG の向きにある。

寸法の根拠 (2026-10-25 調査):
    平面形   OpenStreetMap way 84838285 の建物輪郭。国土地理院の基盤地図情報の建物線と 1 m 以内で一致し、
             国土地理院の航空写真とも重なることを確認
    延床面積 604 m², 鉄骨造平屋, 天井高 2.2〜5.13 m (公開資料)
    外観     公式サイト・横浜市の公募資料の写真: 北東面はガラス張りで中央に入口とスロープ、
             南東面は小松石積みの壁とガラスの入口、屋根の縁は茶色のレンガタイルの帯、屋上は芝生
推定した値 (写真の比率などから。実測ではない):
    床の高さ +0.3 m、北棟の天井 4.6 m・帯 1.0 m、南西棟の天井 2.9 m・帯 1.0 m、扉の幅と位置
作っていないもの: 机などの家具、屋上につながる公園の芝生の丘と屋上の起伏、厨房・トイレなどの間仕切り
照明: 天井に円盤形のダウンライトを 4.5 m 間隔で並べる。数と明るさは LIGHT_ で始まる値で変える
      (ライトを増やすと Isaac Sim の描画が重くなり、実時間で動かなくなる)
"""

import math
import os
import sys

import bmesh
import bpy

out_base = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else "zounohana_terrace"

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.context.scene.unit_settings.system = "METRIC"
bpy.context.scene.unit_settings.scale_length = 1.0

# ---------------------------------------------------------------------------------------------
# 平面 (OpenStreetMap の輪郭を、頂点 0 を原点とする m 単位に直したもの。反時計回り)
# ---------------------------------------------------------------------------------------------

OSM = {
    0: (0.00, 0.00),     # 南西端
    1: (7.33, -9.06),    # 南端
    4: (34.14, 14.37),   # 東の角 (南東面と北東面の交点)
    6: (24.87, 37.37),   # 北端
    7: (11.12, 32.57),   # 北西端
    9: (17.90, 14.75),   # 内側の角 (2 つの棟の境目)
}
# 北東面 4→6 の中点から外向きに 6 m の点を原点 (Go2 の初期位置) にする
_fx, _fy = OSM[6][0] - OSM[4][0], OSM[6][1] - OSM[4][1]
_fl = math.hypot(_fx, _fy)
_out = (_fy / _fl, -_fx / _fl)  # 反時計回りの輪郭の外向き法線
ENTRANCE_MID = ((OSM[4][0] + OSM[6][0]) / 2, (OSM[4][1] + OSM[6][1]) / 2)
ORIGIN = (ENTRANCE_MID[0] + 6.0 * _out[0], ENTRANCE_MID[1] + 6.0 * _out[1])
SPAWN_YAW_DEG = math.degrees(math.atan2(-_out[1], -_out[0]))
P = {k: (x - ORIGIN[0], y - ORIGIN[1]) for k, (x, y) in OSM.items()}

FOOTPRINT = [P[0], P[1], P[4], P[6], P[7], P[9]]
WING_NORTH = [P[9], P[4], P[6], P[7]]  # ガラスの広間
WING_SOUTHWEST = [P[0], P[1], P[4], P[9]]  # 石積みの面の棟

# 高さ [m] (z = 0 が広場)
FLOOR = 0.30
NORTH_GLASS_TOP = FLOOR + 4.6
NORTH_ROOF_TOP = NORTH_GLASS_TOP + 1.0
SW_WALL_TOP = FLOOR + 2.9
SW_ROOF_TOP = SW_WALL_TOP + 1.0
ROOF_SLAB = 0.4
DOOR_HEIGHT = 2.4
RAIL_HEIGHT = 1.1

WALL_T = 0.25
GLASS_T = 0.08

# 天井のダウンライト (写真では天井に丸いダウンライトが並んでいる)
LIGHT_SPACING = 4.5  # 格子の間隔 [m] (3 m で 60 個にすると Isaac Sim が実時間の 0.8 倍に落ちた)
LIGHT_MARGIN = 1.0  # 壁から離す距離 [m]
LIGHT_RADIUS = 0.15  # 円盤の半径 [m]
# Isaac Sim (RTX) での明るさ。Blender と Isaac Sim では光の単位の扱いが違うので、
# 書き出した USD の値をこれで上書きする (Blender の画面での明るさは LIGHT_BLENDER_WATTS)
# 2026-10-25 に Isaac Sim で撮り比べ、3 m 間隔なら 100,000 がちょうどよかった。部屋の明るさは
# (強さ / 間隔²) に比例するので、間隔を変えたらこの式で強さが合う
LIGHT_INTENSITY_USD = 100000.0 * (LIGHT_SPACING / 3.0) ** 2
LIGHT_BLENDER_WATTS = 60.0
LIGHT_COLOR = (1.0, 0.92, 0.8)  # 少し暖かい白

# ---------------------------------------------------------------------------------------------
# マテリアル
# ---------------------------------------------------------------------------------------------


def material(name, color, alpha=1.0, roughness=0.6):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, alpha)  # Blender のビューポート表示用
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Roughness"].default_value = roughness
    if alpha < 1.0:
        bsdf.inputs["Alpha"].default_value = alpha
        mat.surface_render_method = "BLENDED"
    return mat


M_GLASS = material("Glass", (0.55, 0.7, 0.75), alpha=0.25, roughness=0.05)
M_MULLION = material("Mullion", (0.75, 0.76, 0.78), roughness=0.3)
M_BRICK = material("BrickTile", (0.42, 0.22, 0.15))
M_STONE = material("KomatsuStone", (0.55, 0.52, 0.47), roughness=0.9)
M_CONCRETE = material("Concrete", (0.62, 0.62, 0.6), roughness=0.8)
M_FLOOR = material("WoodFloor", (0.32, 0.17, 0.08), roughness=0.4)
M_LAWN = material("Lawn", (0.27, 0.45, 0.18), roughness=0.9)
M_RAIL = material("Railing", (0.9, 0.9, 0.9), roughness=0.3)
M_RAMP = material("RampPaving", (0.6, 0.58, 0.55), roughness=0.8)

# ---------------------------------------------------------------------------------------------
# 形を作る道具
# ---------------------------------------------------------------------------------------------

collections = {}


def add_object(name, verts, faces, mat, group):
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    # 凹んだ多角形も正しく扱えるよう三角形に分割しておく (物理・高さ計測用)
    bmesh.ops.triangulate(bm, faces=bm.faces, quad_method="BEAUTY", ngon_method="BEAUTY")
    bm.to_mesh(mesh)
    bm.free()
    mesh.materials.append(mat)
    obj = bpy.data.objects.new(name, mesh)
    if group not in collections:
        coll = bpy.data.collections.new(group)
        bpy.context.scene.collection.children.link(coll)
        collections[group] = coll
    collections[group].objects.link(obj)
    return obj


def prism(name, poly, z0, z1, mat, group):
    """平面の多角形 (反時計回り) を z0〜z1 に押し出した立体。"""
    n = len(poly)
    verts = [(x, y, z0) for x, y in poly] + [(x, y, z1) for x, y in poly]
    faces = [list(range(n - 1, -1, -1)), list(range(n, 2 * n))]
    faces += [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    return add_object(name, verts, faces, mat, group)


def box_along(name, a, b, offset0, offset1, z0, z1, mat, group):
    """線分 a→b に沿った直方体。offset は線分の左 (反時計回りの輪郭なら内側) への距離 [m]。"""
    ux, uy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(ux, uy)
    ux, uy = ux / length, uy / length
    nx, ny = -uy, ux
    corners = [
        (a[0] + nx * offset0, a[1] + ny * offset0), (b[0] + nx * offset0, b[1] + ny * offset0),
        (b[0] + nx * offset1, b[1] + ny * offset1), (a[0] + nx * offset1, a[1] + ny * offset1),
    ]
    return prism(name, corners, z0, z1, mat, group)


def lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def wall_with_openings(name, a, b, z0, z1, thickness, mat, group, openings=(), offset=0.0):
    """a→b の壁。openings は (中心の位置 0〜1, 幅 m, 開口の上端 z) 。開口の上は z1 まで壁を残す。"""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    cuts = sorted((c * length - w / 2, c * length + w / 2, top) for c, w, top in openings)
    pos, k = 0.0, 0
    for s0, s1, top in cuts:
        if s0 > pos:
            box_along(f"{name}_{k}", lerp(a, b, pos / length), lerp(a, b, s0 / length),
                      offset, offset + thickness, z0, z1, mat, group)
            k += 1
        if top < z1:
            box_along(f"{name}_{k}", lerp(a, b, s0 / length), lerp(a, b, s1 / length),
                      offset, offset + thickness, top, z1, mat, group)
            k += 1
        pos = s1
    if pos < length:
        box_along(f"{name}_{k}", lerp(a, b, pos / length), b, offset, offset + thickness, z0, z1, mat, group)


def mullions(name, a, b, z0, z1, spacing, group, skip=()):
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    n = int(length // spacing)
    for i in range(1, n + 1):
        t = i * spacing / length
        if any(abs(t - c) * length < w / 2 + 0.1 for c, w in skip) or t >= 1:
            continue
        p = lerp(a, b, t)
        q = lerp(a, b, min(1.0, t + 0.06 / length))
        box_along(f"{name}_{i}", p, q, -0.06, GLASS_T + 0.06, z0, z1, M_MULLION, group)


def handrail(name, a, b, z_a, z_b, height, group, post_spacing=1.5):
    """支柱と上の横棒だけの手すり。z_a / z_b は a / b の地面の高さ。"""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(length // post_spacing))
    for i in range(n + 1):
        t = i / n
        p = lerp(a, b, t)
        q = lerp(a, b, min(1.0, t + 0.05 / length)) if t < 1 else p
        if t >= 1:
            p = lerp(a, b, 1.0 - 0.05 / length)
            q = b
        z = z_a + (z_b - z_a) * t
        box_along(f"{name}_post{i}", p, q, -0.025, 0.025, z, z + height, M_RAIL, group)
    # 上の横棒 (傾いた手すりは両端の高さを結ぶ)
    ux, uy = b[0] - a[0], b[1] - a[1]
    nx, ny = -uy / length * 0.03, ux / length * 0.03
    top = [(a[0] + nx, a[1] + ny, z_a + height), (b[0] + nx, b[1] + ny, z_b + height),
           (b[0] - nx, b[1] - ny, z_b + height), (a[0] - nx, a[1] - ny, z_a + height)]
    verts = top + [(x, y, z - 0.05) for x, y, z in top]
    faces = [[0, 1, 2, 3], [7, 6, 5, 4], [0, 4, 5, 1], [1, 5, 6, 2], [2, 6, 7, 3], [3, 7, 4, 0]]
    add_object(f"{name}_bar", verts, faces, M_RAIL, group)


def ramp(name, door_center, outward, width, rise, slope, group):
    """扉の前のスロープ。door_center は壁の外面上の点、outward は外向き単位ベクトル。"""
    run = rise / slope
    tx, ty = -outward[1], outward[0]
    hw = width / 2
    p0 = (door_center[0] + tx * hw, door_center[1] + ty * hw)
    p1 = (door_center[0] - tx * hw, door_center[1] - ty * hw)
    q0 = (p0[0] + outward[0] * run, p0[1] + outward[1] * run)
    q1 = (p1[0] + outward[0] * run, p1[1] + outward[1] * run)
    verts = [(*p0, 0), (*p1, 0), (*q1, 0), (*q0, 0), (*p0, rise), (*p1, rise)]
    faces = [[0, 3, 2, 1], [4, 5, 2, 3], [0, 1, 5, 4], [0, 4, 3], [1, 2, 5]]
    add_object(name, verts, faces, M_RAMP, group)
    for side, (s, e) in (("L", (p0, q0)), ("R", (p1, q1))):  # 手すり (高さ 0.85 m)
        handrail(f"{name}_rail{side}", s, e, rise, 0.0, 0.85, group, post_spacing=1.2)


def roof_railing(name, a, b, z, group):
    # 屋根の縁から少し内側に置く
    ux, uy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(ux, uy)
    nx, ny = -uy / length * 0.15, ux / length * 0.15
    handrail(name, (a[0] + nx, a[1] + ny), (b[0] + nx, b[1] + ny), z, z, RAIL_HEIGHT, group)


def point_in_polygon(pt, poly):
    x, y = pt
    inside = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def distance_to_polygon_edges(pt, poly):
    best = float("inf")
    for a, b in zip(poly, poly[1:] + poly[:1]):
        ux, uy = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((pt[0] - a[0]) * ux + (pt[1] - a[1]) * uy) / (ux * ux + uy * uy)))
        best = min(best, math.hypot(pt[0] - a[0] - t * ux, pt[1] - a[1] - t * uy))
    return best


def ceiling_lights(name, poly, z, group):
    """多角形の中に、壁から LIGHT_MARGIN 以上離して LIGHT_SPACING 間隔で下向きの円盤ライトを並べる。"""
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    if group not in collections:
        coll = bpy.data.collections.new(group)
        bpy.context.scene.collection.children.link(coll)
        collections[group] = coll
    count = 0
    x = min(xs) + LIGHT_SPACING / 2
    while x < max(xs):
        y = min(ys) + LIGHT_SPACING / 2
        while y < max(ys):
            if point_in_polygon((x, y), poly) and distance_to_polygon_edges((x, y), poly) >= LIGHT_MARGIN:
                light = bpy.data.lights.new(f"{name}_{count:02d}", type="AREA")
                light.shape = "DISK"
                light.size = LIGHT_RADIUS * 2
                light.energy = LIGHT_BLENDER_WATTS
                light.color = LIGHT_COLOR
                obj = bpy.data.objects.new(f"{name}_{count:02d}", light)
                obj.location = (x, y, z)  # エリアライトは既定で真下 (-Z) を向く
                collections[group].objects.link(obj)
                count += 1
            y += LIGHT_SPACING
        x += LIGHT_SPACING
    return count


def outward_normal(a, b):
    ux, uy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(ux, uy)
    return (uy / length, -ux / length)


# ---------------------------------------------------------------------------------------------
# 建物
# ---------------------------------------------------------------------------------------------

# 床 (広場から +0.3 m)
prism("Floor", FOOTPRINT, 0.0, FLOOR, M_FLOOR, "Floor")

# 北棟: 北東面 (4→6) と北端 (6→7) はガラス、内側 (7→9) はコンクリート
E_DOOR = (0.5, 2.4, FLOOR + DOOR_HEIGHT)  # 北東面の中央の入口
wall_with_openings("NE_Glass", P[4], P[6], FLOOR, NORTH_GLASS_TOP, GLASS_T, M_GLASS, "Glass", [E_DOOR])
mullions("NE_Mullion", P[4], P[6], FLOOR, NORTH_GLASS_TOP, 2.4, "Glass", skip=[E_DOOR[:2]])
wall_with_openings("N_Glass", P[6], P[7], FLOOR, NORTH_GLASS_TOP, GLASS_T, M_GLASS, "Glass")
mullions("N_Mullion", P[6], P[7], FLOOR, NORTH_GLASS_TOP, 2.4, "Glass")
box_along("NW_Wall", P[7], P[9], 0.0, WALL_T, 0.0, NORTH_ROOF_TOP, M_CONCRETE, "Walls")
# 屋根の縁の帯 (レンガタイル)
box_along("NE_Fascia", P[4], P[6], -0.05, WALL_T, NORTH_GLASS_TOP, NORTH_ROOF_TOP, M_BRICK, "Fascia")
box_along("N_Fascia", P[6], P[7], -0.05, WALL_T, NORTH_GLASS_TOP, NORTH_ROOF_TOP, M_BRICK, "Fascia")
# 屋根 (芝生)
prism("NorthRoof", WING_NORTH, NORTH_ROOF_TOP - ROOF_SLAB, NORTH_ROOF_TOP, M_LAWN, "Roof")

# 南西棟: 南東面 (1→4) は石積み + ガラスの入口、内側 (9→0) と南西端 (0→1) はコンクリート
SE_DOOR = (0.8, 2.0, FLOOR + DOOR_HEIGHT)  # 写真で右寄り (東の角寄り) にある入口
SE_GLASS = (0.8, 5.0)  # 入口の両側のガラス
wall_with_openings("SE_Stone", P[1], P[4], FLOOR, SW_WALL_TOP, WALL_T, M_STONE, "Walls",
                   [(SE_GLASS[0], SE_GLASS[1], SW_WALL_TOP)])
_se_len = math.hypot(P[4][0] - P[1][0], P[4][1] - P[1][1])
_g0 = lerp(P[1], P[4], SE_GLASS[0] - SE_GLASS[1] / 2 / _se_len)
_g1 = lerp(P[1], P[4], SE_GLASS[0] + SE_GLASS[1] / 2 / _se_len)
wall_with_openings("SE_Glass", _g0, _g1, FLOOR, SW_WALL_TOP, GLASS_T, M_GLASS, "Glass", [(0.5, SE_DOOR[1], SE_DOOR[2])])
box_along("SE_Plinth", P[1], P[4], 0.0, WALL_T, 0.0, FLOOR, M_STONE, "Walls")
box_along("SW_Wall", P[9], P[0], 0.0, WALL_T, 0.0, SW_ROOF_TOP, M_CONCRETE, "Walls")
box_along("S_End", P[0], P[1], 0.0, WALL_T, 0.0, SW_ROOF_TOP, M_CONCRETE, "Walls")
box_along("SE_Fascia", P[1], P[4], -0.05, WALL_T, SW_WALL_TOP, SW_ROOF_TOP, M_BRICK, "Fascia")
prism("SouthwestRoof", WING_SOUTHWEST, SW_ROOF_TOP - ROOF_SLAB, SW_ROOF_TOP, M_LAWN, "Roof")
# 2 つの棟の境目: 南西棟の天井から北棟の屋根までの壁
box_along("Junction_Wall", P[4], P[9], -WALL_T / 2, WALL_T / 2, SW_WALL_TOP, NORTH_ROOF_TOP, M_CONCRETE, "Walls")

# 屋上の手すり
roof_railing("Rail_NE", P[4], P[6], NORTH_ROOF_TOP, "Railings")
roof_railing("Rail_N", P[6], P[7], NORTH_ROOF_TOP, "Railings")
roof_railing("Rail_SE", P[1], P[4], SW_ROOF_TOP, "Railings")

# 入口のスロープ (勾配 1/12)
ramp("Ramp_NE", lerp(P[4], P[6], E_DOOR[0]), outward_normal(P[4], P[6]), 2.8, FLOOR, 1 / 12, "Ramps")
ramp("Ramp_SE", lerp(P[1], P[4], SE_DOOR[0]), outward_normal(P[1], P[4]), 2.4, FLOOR, 1 / 12, "Ramps")

# 天井のダウンライト (屋根の下面のすぐ下)
n_lights = ceiling_lights("Light_North", WING_NORTH, NORTH_ROOF_TOP - ROOF_SLAB - 0.02, "Lights")
n_lights += ceiling_lights("Light_Southwest", WING_SOUTHWEST, SW_ROOF_TOP - ROOF_SLAB - 0.02, "Lights")

# ---------------------------------------------------------------------------------------------
# 保存と書き出し
# ---------------------------------------------------------------------------------------------

out_dir = os.path.dirname(os.path.abspath(out_base))
os.makedirs(out_dir, exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(out_base + ".blend"))
bpy.ops.wm.usd_export(filepath=os.path.abspath(out_base + ".usdc"))

# ライトの明るさを Isaac Sim 向けに上書きする (Blender 付属の USD ライブラリを使う)
from pxr import Usd, UsdLux  # noqa: E402

stage = Usd.Stage.Open(os.path.abspath(out_base + ".usdc"))
n_usd_lights = 0
for prim in stage.Traverse():
    if prim.IsA(UsdLux.DiskLight):
        light = UsdLux.DiskLight(prim)
        light.GetIntensityAttr().Set(LIGHT_INTENSITY_USD)
        light.GetExposureAttr().Set(0.0)
        light.GetNormalizeAttr().Set(False)  # Omniverse の標準どおり、明るさを光る面の面積に比例させる
        light.GetRadiusAttr().Set(LIGHT_RADIUS)
        light.GetColorAttr().Set(LIGHT_COLOR)
        n_usd_lights += 1
stage.GetRootLayer().Save()
print(f"LIGHTS blender {n_lights}  usd disk lights {n_usd_lights}")
area =0.5 * abs(sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(FOOTPRINT, FOOTPRINT[1:] + FOOTPRINT[:1])))
print(f"EXPORTED {out_base}.usdc  footprint {area:.0f} m2  spawn yaw {SPAWN_YAW_DEG:.1f} deg  "
      f"NE facade {math.hypot(_fx, _fy):.1f} m")
