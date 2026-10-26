"""キーを押すと Go2 の前に落ちてくる、物理演算つきのブロック (蹴ると動く・積める)。

GPU で物理計算しているときは、再生中に剛体を新しく作れない (Isaac Sim 6.2 の制限)。
そこで再生前に種類ごとにブロックを用意して遠くに置いておき、出すときにテレポートさせる。
用意した数を超えて出すと、その種類でいちばん古いブロックを持ってくる。

ブロックは /World/Terrain の外 (/World/Blocks) に置く。動くので高さ計測用の地形メッシュには
入れず、TerrainRaycaster.add_boxes() で毎回いまの位置を渡して光線を当てる。
"""

from __future__ import annotations

import numpy as np
import warp as wp
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.experimental.utils.stage import get_current_stage
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

import terrain

BLOCK_ROOT = "/World/Blocks"
# 種類: 辺の長さ (x, y, z) [m] と色。長いブロックは x が長辺
BLOCK_KINDS = {
    "long": ((0.2, 0.1, 0.1), (0.78, 0.62, 0.42)),  # 10 × 10 × 20 cm
    "cube": ((0.1, 0.1, 0.1), (0.82, 0.48, 0.34)),  # 10 × 10 × 10 cm
}
BLOCKS_PER_KIND = {"long": 20, "cube": 40}  # 種類ごとの数。10 cm 角はピラミッド (P) の 30 個 + 単品の分
BLOCK_DENSITY = 600.0  # kg/m³ (木材くらい。10×10×20 cm で 1.2 kg、10 cm 角で 0.6 kg)
PARK_ORIGIN = (-300.0, -300.0)  # 使っていないブロックを置いておく場所 (会場から離れた地面の上)


def _yaw_quat(yaw: float) -> np.ndarray:
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


class BlockPool:
    """再生前に作る。spawn() / clear() は物理コールバックの中で呼ぶ。"""

    def __init__(self, per_kind: dict[str, int] = BLOCKS_PER_KIND, density: float = BLOCK_DENSITY,
                 device: str = "cuda:0") -> None:
        stage = get_current_stage()
        UsdGeom.Xform.Define(stage, BLOCK_ROOT)
        terrain._root(stage)
        material = terrain._material(stage)  # 地形と同じ摩擦 1.0
        paths, sizes, self._kind_indices = [], [], {}
        for kind, (size, color) in BLOCK_KINDS.items():
            self._kind_indices[kind] = []
            for k in range(per_kind[kind]):
                index = len(paths)
                path = f"{BLOCK_ROOT}/{kind}_{k:02d}"
                cube = UsdGeom.Cube.Define(stage, path)
                cube.CreateSizeAttr(1.0)
                cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
                xform = UsdGeom.Xformable(cube.GetPrim())
                xform.AddTranslateOp().Set(Gf.Vec3d(*self._park_position(index, size)))
                xform.AddOrientOp().Set(Gf.Quatf(1.0))
                xform.AddScaleOp().Set(Gf.Vec3f(*size))
                prim = cube.GetPrim()
                UsdPhysics.RigidBodyAPI.Apply(prim)
                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(float(density * np.prod(size)))
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
                paths.append(path)
                sizes.append(size)
                self._kind_indices[kind].append(index)
        self._sizes = np.array(sizes)
        self._next = {kind: 0 for kind in BLOCK_KINDS}
        self._device = device
        self.half_extents = wp.array((self._sizes / 2).astype(np.float32), dtype=wp.vec3, device=device)
        self.view = RigidPrim(paths)

    @staticmethod
    def _park_position(index: int, size: tuple[float, float, float]) -> tuple[float, float, float]:
        return PARK_ORIGIN[0] - 0.5 * (index % 10), PARK_ORIGIN[1] - 0.5 * (index // 10), size[2] / 2

    def spawn(self, kind: str, position: np.ndarray, yaw: float, upright: bool = False) -> None:
        """kind のブロックを 1 つ、position (中心 [m]) に置く。

        yaw は長辺の向き [rad]。upright なら長辺を縦にして立てる。
        """
        index = self._kind_indices[kind][self._next[kind]]
        self._next[kind] = (self._next[kind] + 1) % len(self._kind_indices[kind])
        q = _yaw_quat(yaw)
        if upright:  # y 軸まわりに 90 度倒して長辺 (x) を縦にする
            q = _quat_mul(q, np.array([np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0]))
        self.view.set_world_poses([list(position)], [list(q)], indices=[index])
        self.view.set_velocities([[0.0, 0.0, 0.0]], [[0.0, 0.0, 0.0]], indices=[index])

    def clear(self) -> None:
        """すべてのブロックを片付ける (置き場に戻す)。"""
        n = len(self._sizes)
        self.view.set_world_poses([self._park_position(i, self._sizes[i]) for i in range(n)], [[1.0, 0.0, 0.0, 0.0]] * n)
        self.view.set_velocities(np.zeros((n, 3)), np.zeros((n, 3)))
        self._next = {kind: 0 for kind in BLOCK_KINDS}

    def size(self, kind: str) -> np.ndarray:
        return self._sizes[self._kind_indices[kind][0]]

    def boxes(self) -> tuple[wp.array, wp.array, wp.array] | None:
        """高さ計測用: いまの (位置 (N, 3), 向き wxyz (N, 4), 辺の半分 (N,) vec3)。再生前は None。"""
        if not self.view.is_physics_tensor_entity_valid():
            return None
        positions, orientations = self.view.get_world_poses()
        return positions.to(self._device), orientations.to(self._device), self.half_extents
