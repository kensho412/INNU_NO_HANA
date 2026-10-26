"""8×8 のマス目の足場。マスごとの高さを外から (Max から OSC で) 一括で変えられ、どのマスが踏まれているかを返す。

各マスは物理演算つきの柱 (動く剛体)。Go2 が踏むと少し揺れたり押されたりするが、外周の囲いと隣の柱に
挟まれているので、基本的には蹴とばされない。

高さの変え方:
    再生中に剛体の大きさ (USD の scale) を変えると PhysX が剛体を作り直し、Go2 を含むすべての
    物理ビューが無効になる (Isaac Sim 6.2 で確認)。そこで柱の長さは固定 (PILLAR_LENGTH) にして、
    地面より下に沈めて使う。柱は地面・地形とは衝突しない設定にして、地下に置いた動かない台
    (キネマティックの板) の上に載せる。高さを変えるときは、板と柱をいっしょにテレポートさせる。
    囲いも地下まで伸ばしてあるので、柱は横にずれたり倒れたりしない。

高さはすべて base_z (足場を置く床の高さ。建物の中なら床の上面) からの高さ。

マスの番号 (64 個のリストの並び):
    index = row × 8 + col。row 0 が Go2 の初期位置に近い列、col 0 が Go2 から見て左端。
"""

from __future__ import annotations

import numpy as np
import warp as wp
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.experimental.utils.stage import get_current_stage
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

import terrain

GRID_ROOT = "/World/Grid"
FENCE_ROOT = f"{GRID_ROOT}/Fence"  # 高さ計測用の地形メッシュに入れる (動かない)
GRID_N = 8
GRID_CELL = 0.25  # マスの間隔 [m] (8 マスで 2 m 四方)
PILLAR_GAP = 0.004  # 隣の柱とのすき間 [m] (ぴったりだと押し合ってはじける)
MAX_HEIGHT = 0.30  # 柱の高さの上限 [m] (不整地用ポリシーが越えられるのは 10〜20 cm 程度)
MIN_HEIGHT = 0.01  # 下限 [m] (0 だと地面と重なってちらつく)
PILLAR_LENGTH = MAX_HEIGHT + 0.30  # 柱の長さ。いちばん高くしても 30 cm は地下で囲いに挟まれる
PLATE_THICKNESS = 0.02
FENCE_HEIGHT = 0.05  # 囲いの地面からの高さ [m] (Go2 がまたいで入れる高さ)
FENCE_THICKNESS = 0.05
PILLAR_DENSITY = 600.0  # kg/m³ (木材くらい。長さ 0.6 m の柱 1 本で約 22 kg、Go2 は 15 kg)
INITIAL_HEIGHT = 0.05  # 起動時の高さ [m]
STEP_FORCE_N = 1.0  # 足の力がこれより大きければ接地とみなす [N]
GLOW_COLOR = (1.0, 0.75, 0.25)  # 踏まれているマスを光らせる色
GLOW_INTENSITY = 8.0  # 発光の強さ (emissiveColor に掛ける)
GLOW_LIFT = 0.003  # 光る板を柱の上面からどれだけ浮かせるか [m] (柱は踏まれて数 mm 沈むことがある)


class SteppingGrid:
    """再生前に作る。apply_pending_heights() は物理コールバックの中で呼ぶ。

    Args:
        center: 足場の中心 (x, y) [m]。
        yaw: 足場の向き [rad]。row が増える向き (Go2 の進む向き) が +x から反時計回りに yaw。
        base_z: 足場を置く床の高さ [m]。マスの高さはここから測る。
    """

    def __init__(self, center: tuple[float, float], yaw: float = 0.0, base_z: float = 0.0,
                 device: str = "cuda:0") -> None:
        stage = get_current_stage()
        UsdGeom.Xform.Define(stage, GRID_ROOT)
        terrain._root(stage)
        material = terrain._material(stage)
        self.center = np.asarray(center, dtype=np.float64)
        self.yaw = yaw
        self.base_z = base_z
        self.forward = np.array([np.cos(yaw), np.sin(yaw)])
        self.left = np.array([-np.sin(yaw), np.cos(yaw)])
        self._quat = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        self._device = device
        self.size = GRID_N * GRID_CELL
        self.heights = np.full(GRID_N * GRID_N, INITIAL_HEIGHT)
        self._pending: np.ndarray | None = None

        side = GRID_CELL - PILLAR_GAP
        self._cell_xy = self.cell_centers(self.center, yaw)
        pillar_paths, plate_paths = [], []
        for i, (x, y) in enumerate(self._cell_xy):
            row, col = divmod(i, GRID_N)
            tint = 0.62 if (row + col) % 2 == 0 else 0.72  # 市松模様にしてマスを見分けやすくする
            path = f"{GRID_ROOT}/Pillars/p_{i:02d}"
            self._add_cube(stage, path, (x, y, self._pillar_z(self.heights[i])), (side, side, PILLAR_LENGTH),
                           (tint, tint, tint * 1.05), material, body="dynamic")
            pillar_paths.append(path)
            path = f"{GRID_ROOT}/Plates/b_{i:02d}"
            self._add_cube(stage, path, (x, y, self._plate_z(self.heights[i])), (side, side, PLATE_THICKNESS),
                           (0.3, 0.3, 0.3), material, body="kinematic")
            plate_paths.append(path)

        # 踏まれているマスを光らせる板 (見た目だけ。物理演算・高さ計測には入らない)
        glow_material = self._glow_material(stage)
        self._glow_caps = []
        for i, (x, y) in enumerate(self._cell_xy):
            cap = UsdGeom.Cube.Define(stage, f"{GRID_ROOT}/Glow/g_{i:02d}")
            cap.CreateSizeAttr(1.0)
            xform = UsdGeom.Xformable(cap.GetPrim())
            translate = xform.AddTranslateOp()
            translate.Set(Gf.Vec3d(float(x), float(y), self._glow_z(self.heights[i])))
            xform.AddOrientOp().Set(Gf.Quatf(float(self._quat[0]), 0.0, 0.0, float(self._quat[3])))
            xform.AddScaleOp().Set(Gf.Vec3f(side, side, 0.002))
            UsdShade.MaterialBindingAPI.Apply(cap.GetPrim()).Bind(glow_material)
            cap.MakeInvisible()
            self._glow_caps.append((cap, translate))
        self._glowing = np.zeros(GRID_N * GRID_N, dtype=bool)
        self._glow_heights_changed = False

        # 囲い: 柱を外から押さえる 4 枚の壁。地下 (柱の下端) から地上 FENCE_HEIGHT まで
        bottom = base_z + MIN_HEIGHT - PILLAR_LENGTH - PLATE_THICKNESS - 0.05
        height = base_z + FENCE_HEIGHT - bottom
        half = self.size / 2 + FENCE_THICKNESS / 2
        length = self.size + 2 * FENCE_THICKNESS
        for k, (u, v, along_u) in enumerate([(half, 0, False), (-half, 0, False), (0, half, True), (0, -half, True)]):
            x, y = self.center + u * self.forward + v * self.left
            size = (length, FENCE_THICKNESS, height) if along_u else (FENCE_THICKNESS, length, height)
            self._add_cube(stage, f"{FENCE_ROOT}/wall_{k}", (x, y, bottom + height / 2), size,
                           (0.35, 0.33, 0.3), material, body=None)

        # 柱は地面・地形 (/World/Ground, /World/Terrain) とは当たらない (地下に沈めるため)
        pillars = UsdPhysics.CollisionGroup.Define(stage, f"{GRID_ROOT}/PillarGroup")
        pillars.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(f"{GRID_ROOT}/Pillars")
        ground = UsdPhysics.CollisionGroup.Define(stage, f"{GRID_ROOT}/GroundGroup")
        includes = ground.GetCollidersCollectionAPI().CreateIncludesRel()
        includes.AddTarget("/World/Ground")
        includes.AddTarget(terrain.TERRAIN_ROOT)
        pillars.CreateFilteredGroupsRel().AddTarget(ground.GetPath())

        self.pillars = RigidPrim(pillar_paths)
        self.plates = RigidPrim(plate_paths)
        self.half_extents = wp.array(np.tile([side / 2, side / 2, PILLAR_LENGTH / 2], (GRID_N * GRID_N, 1)).astype(np.float32),
                                     dtype=wp.vec3, device=device)

    def _add_cube(self, stage: object, path: str, center: tuple, size: tuple, color: tuple, material: object,
                  body: str | None) -> None:
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        xform.AddOrientOp().Set(Gf.Quatf(float(self._quat[0]), 0.0, 0.0, float(self._quat[3])))
        xform.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        prim = cube.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
        if body is not None:
            rigid = UsdPhysics.RigidBodyAPI.Apply(prim)
            if body == "kinematic":
                rigid.CreateKinematicEnabledAttr().Set(True)
            else:
                UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(float(PILLAR_DENSITY * np.prod(size)))

    def _pillar_z(self, height: float) -> float:
        return self.base_z + height - PILLAR_LENGTH / 2

    def _plate_z(self, height: float) -> float:
        return self.base_z + height - PILLAR_LENGTH - PLATE_THICKNESS / 2

    def cell_center(self, index: int) -> np.ndarray:
        return self._cell_xy[index]

    def _glow_z(self, height: float) -> float:
        return self.base_z + height + GLOW_LIFT

    @staticmethod
    def _glow_material(stage: object) -> UsdShade.Material:
        material = UsdShade.Material.Define(stage, f"{GRID_ROOT}/Looks/Glow")
        shader = UsdShade.Shader.Define(stage, f"{GRID_ROOT}/Looks/Glow/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*GLOW_COLOR))
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*(c * GLOW_INTENSITY for c in GLOW_COLOR)))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return material

    @staticmethod
    def cell_centers(center: tuple[float, float], yaw: float) -> np.ndarray:
        """足場を作る前でも使える: 64 マスの中心 (64, 2)。"""
        row, col = np.divmod(np.arange(GRID_N * GRID_N), GRID_N)
        size = GRID_N * GRID_CELL
        u = (row + 0.5) * GRID_CELL - size / 2
        v = size / 2 - (col + 0.5) * GRID_CELL
        forward = np.array([np.cos(yaw), np.sin(yaw)])
        left = np.array([-np.sin(yaw), np.cos(yaw)])
        return np.asarray(center, dtype=np.float64) + u[:, None] * forward + v[:, None] * left

    def cell_of(self, xy: np.ndarray) -> np.ndarray:
        """xy (N, 2) がどのマスの上か。足場の外は -1。"""
        d = np.asarray(xy, dtype=np.float64) - self.center
        u = d @ self.forward + self.size / 2
        v = self.size / 2 - d @ self.left
        row = np.floor(u / GRID_CELL).astype(int)
        col = np.floor(v / GRID_CELL).astype(int)
        inside = (row >= 0) & (row < GRID_N) & (col >= 0) & (col < GRID_N)
        return np.where(inside, row * GRID_N + col, -1)

    # -- 高さ -------------------------------------------------------------------------------

    def request_heights(self, heights_m: np.ndarray) -> None:
        """64 マスの高さ [m] を予約する。次の物理ステップで一括で反映する。"""
        self._pending = np.clip(np.asarray(heights_m, dtype=np.float64), MIN_HEIGHT, MAX_HEIGHT)

    def apply_pending_heights(self) -> bool:
        """物理コールバックの中で呼ぶ: 予約された高さに、板と柱をテレポートさせる。"""
        if self._pending is None:
            return False
        self.heights, self._pending = self._pending, None
        n = len(self.heights)
        quats = [self._quat] * n
        self.plates.set_world_poses(np.column_stack([self._cell_xy, self._plate_z(self.heights)]), quats)
        self.pillars.set_world_poses(np.column_stack([self._cell_xy, self._pillar_z(self.heights)]), quats)
        self.pillars.set_velocities(np.zeros((n, 3)), np.zeros((n, 3)))
        self._glow_heights_changed = True
        return True

    def show_stepped(self, state: np.ndarray) -> None:
        """踏まれているマス (stepped() の結果) を光らせる。USD を書き換えるので物理コールバックの外で呼ぶ。"""
        if self._glow_heights_changed:
            self._glow_heights_changed = False
            for i, (_, translate) in enumerate(self._glow_caps):
                x, y = self._cell_xy[i]
                translate.Set(Gf.Vec3d(float(x), float(y), self._glow_z(self.heights[i])))
        on = np.asarray(state) > 0
        for i in np.flatnonzero(on != self._glowing):
            cap = self._glow_caps[i][0]
            cap.MakeVisible() if on[i] else cap.MakeInvisible()
        self._glowing = on

    # -- 踏まれているか -----------------------------------------------------------------------

    def foot_cells(self, foot_xy: np.ndarray, foot_force_n: np.ndarray) -> np.ndarray:
        """足先の位置 (4, 2) と力 [N] から、足ごとに着いているマスの番号。浮いている・足場の外なら -1。"""
        cells = self.cell_of(foot_xy).astype(np.int32)
        return np.where(np.asarray(foot_force_n) > STEP_FORCE_N, cells, -1).astype(np.int32)

    @staticmethod
    def stepped(foot_cells: np.ndarray) -> np.ndarray:
        """foot_cells() の結果から、64 マスそれぞれが踏まれているか (0/1)。"""
        state = np.zeros(GRID_N * GRID_N, dtype=np.int32)
        state[foot_cells[foot_cells >= 0]] = 1
        return state

    def boxes(self) -> tuple[wp.array, wp.array, wp.array] | None:
        """高さ計測用: 柱のいまの (位置, 向き wxyz, 辺の半分)。再生前は None。"""
        if not self.pillars.is_physics_tensor_entity_valid():
            return None
        positions, orientations = self.pillars.get_world_poses()
        return positions.to(self._device), orientations.to(self._device), self.half_extents
