"""障害物 (ブロック・階段・グリッド状の凹凸) をステージに置く。

置いたものはすべて /World/Terrain の下に、動かない衝突物として作られる。
不整地用ポリシーの高さ計測は、ロボット自身以外のあらゆる衝突物を地面として測る。

会場モデルなど USD ファイル (Blender から書き出したものなど) を置く場合は add_usd() を使う。
衝突判定が無いファイルには自動で付ける (Blender の USD 書き出しは物理の情報を含まない)。
"""

from __future__ import annotations

import numpy as np
from isaacsim.core.experimental.utils.stage import get_current_stage
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

TERRAIN_ROOT = "/World/Terrain"
_counter = 0


def _root(stage: object) -> None:
    """親を型なしのまま子を作ると物理の読み込みで落ちることがあるので、先に Xform で作る。"""
    if not stage.GetPrimAtPath(TERRAIN_ROOT).IsValid():
        UsdGeom.Xform.Define(stage, TERRAIN_ROOT)


def _material(stage: object) -> UsdShade.Material:
    """学習時と同じ摩擦 1.0 の物理マテリアル。"""
    path = f"{TERRAIN_ROOT}/PhysicsMaterial"
    material = UsdShade.Material.Get(stage, path)
    if not material:
        material = UsdShade.Material.Define(stage, path)
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr().Set(1.0)
        api.CreateDynamicFrictionAttr().Set(1.0)
        api.CreateRestitutionAttr().Set(0.0)
    return material


def add_box(center: tuple[float, float, float], size: tuple[float, float, float],
            color: tuple[float, float, float] = (0.55, 0.55, 0.6), name: str | None = None) -> str:
    """直方体を 1 つ置く。center は中心 [m]、size は (x, y, z) の辺の長さ [m]。"""
    global _counter
    stage = get_current_stage()
    _root(stage)
    _counter += 1
    path = f"{TERRAIN_ROOT}/{name or f'box_{_counter:04d}'}"
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    xform.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
        _material(stage), UsdShade.Tokens.weakerThanDescendants, "physics"
    )
    return path


def add_stairs(start: tuple[float, float], num_steps: int = 6, step_height: float = 0.1,
               step_depth: float = 0.3, width: float = 1.5, top_depth: float = 1.0) -> None:
    """+x 方向に上る階段。start は 1 段目の手前の端の (x, y)。最上段の先に踊り場を付ける。"""
    x0, y0 = start
    for i in range(num_steps):
        depth = top_depth if i == num_steps - 1 else step_depth
        h = step_height * (i + 1)
        add_box((x0 + i * step_depth + depth / 2, y0, h / 2), (depth, width, h), color=(0.6, 0.5, 0.4))


def add_pyramid_stairs(center: tuple[float, float], num_steps: int = 5, step_height: float = 0.1,
                       step_depth: float = 0.3, top_size: float = 1.0) -> None:
    """四方から上って下りられるピラミッド状の階段 (Isaac Lab の pyramid_stairs に似た形)。"""
    for i in range(num_steps):
        side = top_size + 2 * (num_steps - 1 - i) * step_depth
        h = step_height * (i + 1)
        add_box((center[0], center[1], h / 2), (side, side, h), color=(0.6, 0.5, 0.4))


def add_random_grid(center: tuple[float, float], size: tuple[float, float] = (3.0, 3.0),
                    cell: float = 0.45, height_range: tuple[float, float] = (0.025, 0.1),
                    seed: int = 0) -> None:
    """Isaac Lab の random_grid 地形に似た、マス目ごとに高さが違う凹凸。"""
    rng = np.random.default_rng(seed)
    nx, ny = int(round(size[0] / cell)), int(round(size[1] / cell))
    x0 = center[0] - nx * cell / 2
    y0 = center[1] - ny * cell / 2
    for i in range(nx):
        for j in range(ny):
            h = float(rng.uniform(*height_range))
            add_box((x0 + (i + 0.5) * cell, y0 + (j + 0.5) * cell, h / 2), (cell, cell, h), color=(0.45, 0.55, 0.5))


def build_demo_course() -> None:
    """Go2 の初期位置 (原点, +x 向き) の周りに障害物を置く。
    正面: ピラミッド階段 (10 cm × 5 段)、左: グリッド状の凹凸、右: 高さの違うブロック。
    """
    add_pyramid_stairs(center=(4.0, 0.0), num_steps=5, step_height=0.1, step_depth=0.3, top_size=1.0)
    add_random_grid(center=(1.5, 4.0), size=(3.15, 3.15), height_range=(0.025, 0.1), seed=1)
    for k, (x, h) in enumerate([(0.0, 0.05), (1.0, 0.08), (2.0, 0.12), (3.0, 0.15)]):
        add_box((x, -3.5, h / 2), (0.6, 1.2, h), color=(0.5, 0.45, 0.6), name=f"block_{k}")


def add_usd(usd_path: str, prim_path: str = f"{TERRAIN_ROOT}/Venue",
            translation: tuple[float, float, float] = (0, 0, 0), add_collision: bool | None = None) -> int:
    """会場モデルなどの USD (Blender から書き出したものなど) を置く。

    ファイルの単位 (metersPerUnit) と上方向 (upAxis) を読んで、m 単位・Z 上向きに直して置く。
    add_collision が None のときは、ファイルに衝突判定が 1 つも無ければ (Blender から書き出したままの
    状態) すべてのメッシュに衝突判定と摩擦 1.0 を付け、すでにあればファイルの設定をそのまま使う。
    高さ計測に映すため /World/Terrain の下に置くこと。

    Returns:
        衝突判定を付けたメッシュの数 (ファイルの設定を使ったときは 0)。
    """
    source = Usd.Stage.Open(usd_path, load=Usd.Stage.LoadAll)
    if source is None:
        raise FileNotFoundError(usd_path)
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(source)
    up_axis = UsdGeom.GetStageUpAxis(source)
    has_collision = any(p.HasAPI(UsdPhysics.CollisionAPI) for p in source.Traverse())

    stage = get_current_stage()
    _root(stage)
    # ファイルのルートが持つ変換とぶつからないよう、位置・単位・向きの変換は親に、ファイルは子に置く
    xform = UsdGeom.Xform.Define(stage, prim_path)
    prim = stage.DefinePrim(f"{prim_path}/model", "Xform")
    prim.GetReferences().AddReference(usd_path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*map(float, translation)))
    if up_axis == UsdGeom.Tokens.y:
        xform.AddRotateXOp().Set(90.0)  # Y 上向き → Z 上向き
    if abs(meters_per_unit - 1.0) > 1e-9:
        xform.AddScaleOp().Set(Gf.Vec3f(float(meters_per_unit)))  # 例: cm 単位なら 0.01 倍

    if add_collision is None:
        add_collision = not has_collision
    if not add_collision:
        return 0
    # インスタンスの中身は書き換えられないので、インスタンスを解除してから衝突判定を付ける
    instances = [p for p in Usd.PrimRange(prim) if p.IsInstance()]
    while instances:
        for p in instances:
            p.SetInstanceable(False)
        instances = [p for p in Usd.PrimRange(prim) if p.IsInstance()]
    material = _material(stage)
    count = 0
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Mesh):
            continue
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set(UsdPhysics.Tokens.none)  # 三角形そのまま
        UsdShade.MaterialBindingAPI.Apply(p).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
        count += 1
    return count
