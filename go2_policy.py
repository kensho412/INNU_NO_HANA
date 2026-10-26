"""Isaac Lab で学習した Go2 歩行ポリシー (rsl_rl の actor) を Isaac Sim 6.2 で動かす。

Isaac Sim 付属の RobotPolicyRunner は高さ計測 (height_scan) に対応していないので、
不整地用ポリシー (Isaac-Velocity-Rough-Unitree-Go2) を動かすためにここで自前で実装する。
RobotPolicyRunner は spawn() (USD の読み込みと物理プロパティ) だけに使い、関節の初期設定・
観測の組み立て・ポリシーの推論・DC モータモデルによるトルク計算はここで行う。

重みは NVIDIA の配布サーバーにある Isaac Lab の学習済みチェックポイントから actor だけを取り出したもの:
    Assets/Isaac/5.1/Isaac/IsaacLab/PretrainedCheckpoints/rsl_rl/Isaac-Velocity-{Rough,Flat}-Unitree-Go2-v0/checkpoint.pt

学習時の設定 (Isaac Lab UnitreeGo2RoughEnvCfg / FlatEnvCfg) に合わせている値:
    観測  base_lin_vel(3) base_ang_vel(3) projected_gravity(3) velocity_commands(3)
          joint_pos_rel(12) joint_vel_rel(12) last_action(12) [height_scan(187)]   スケールなし
    行動  目標角度 = 標準姿勢 + 0.25 × action、50 Hz (物理 200 Hz の 4 ステップごと)
    モータ DCMotor  kp 25, kd 0.5, 最大トルク 23.5 Nm, 速度上限 30 rad/s
    高さ計測 真下へ光線を飛ばし、胴体の向き (yaw) に合わせた 1.6 m × 1.0 m の
          グリッド (0.1 m 間隔, x が先に変わる順) 。値 = 胴体の高さ - 地面の高さ - 0.5 を ±1 に丸める
          (Isaac Lab は 20 m 上から飛ばすが、屋内で屋根に当たらないよう胴体の 0.5 m 上から飛ばす)
"""

from __future__ import annotations

import carb
import numpy as np
import torch
import warp as wp
from isaacsim.core.experimental.utils.stage import get_current_stage
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

# 関節の並び (Isaac Sim / Isaac Lab の PhysX の並びと同じ)
DOF_NAMES = [
    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
]
DEFAULT_JOINT_POS = np.array([0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5], dtype=np.float32)
ACTION_SCALE = 0.25
DECIMATION = 4

KP, KD = 25.0, 0.5
EFFORT_LIMIT = 23.5
SATURATION_EFFORT = 23.5
VELOCITY_LIMIT = 30.0

SCAN_RESOLUTION = 0.1
SCAN_SIZE = (1.6, 1.0)
SCAN_OFFSET = 0.5
# Isaac Lab は胴体の 20 m 上から光線を飛ばすが、それだと建物の中では屋根に当たってしまう。
# 計測値 (胴体の高さ - 地面の高さ - 0.5) は ±1 で切られるので、胴体の 0.5 m より上の物は
# もともと区別されない。そこで胴体の 0.5 m 上から飛ばす (屋外では Isaac Lab と同じ値になる)。
SCAN_START_ABOVE_BASE = SCAN_OFFSET + 0.01


# ---------------------------------------------------------------------------------------------
# 高さ計測用の地形メッシュ
#
# PhysX のレイキャスト (scene query) は GPU で物理計算しているシーンの箱形状に当てると
# 次の物理ステップで落ちる (Isaac Sim 6.2 で確認) ので、Isaac Lab の RayCaster と同じく
# 地形を Warp のメッシュにまとめて、そこに光線を当てる。
# ---------------------------------------------------------------------------------------------

_CUBE_CORNERS = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])
_CUBE_TRIANGLES = np.array(
    [[0, 2, 3], [0, 3, 1], [4, 5, 7], [4, 7, 6], [0, 1, 5], [0, 5, 4],
     [2, 6, 7], [2, 7, 3], [0, 4, 6], [0, 6, 2], [1, 3, 7], [1, 7, 5]]
)


def _outward(points: np.ndarray, tris: np.ndarray) -> np.ndarray:
    """凸な立体の三角形の向きを外向き (表 = 外側) にそろえる。"""
    center = points.mean(axis=0)
    a, b, c = points[tris[:, 0]], points[tris[:, 1]], points[tris[:, 2]]
    inward = np.einsum("ij,ij->i", np.cross(b - a, c - a), (a + b + c) / 3 - center) < 0
    out = tris.copy()
    out[inward] = out[inward][:, [0, 2, 1]]
    return out


_CUBE_TRIANGLES_OUTWARD = _outward(_CUBE_CORNERS, _CUBE_TRIANGLES)


@wp.kernel
def _cast_down_kernel(mesh: wp.uint64, starts: wp.array(dtype=wp.vec3), hit_z: wp.array(dtype=float)):
    i = wp.tid()
    q = wp.mesh_query_ray(mesh, starts[i], wp.vec3(0.0, 0.0, -1.0), 1.0e6)
    if q.result:
        if q.sign < 0.0:
            # 最初に面の裏側に当たった = 出発点が壁などの中にある。出発点の高さまで物があるとみなす
            hit_z[i] = starts[i][2]
        else:
            hit_z[i] = starts[i][2] - q.t
    else:
        hit_z[i] = -1.0e6


@wp.kernel
def _cast_down_boxes_kernel(starts: wp.array(dtype=wp.vec3), positions: wp.array2d(dtype=float),
                            orientations: wp.array2d(dtype=float), half_extents: wp.array(dtype=wp.vec3),
                            hit_z: wp.array(dtype=float)):
    """動く直方体 (blocks.py のブロック) にも光線を当て、地形より高く当たればそちらを採る。"""
    i = wp.tid()
    start = starts[i]
    best = hit_z[i]
    for b in range(half_extents.shape[0]):
        center = wp.vec3(positions[b, 0], positions[b, 1], positions[b, 2])
        rot = wp.quat(orientations[b, 1], orientations[b, 2], orientations[b, 3], orientations[b, 0])  # wxyz → xyzw
        o = wp.quat_rotate_inv(rot, start - center)  # 箱の座標系での出発点と向き
        d = wp.quat_rotate_inv(rot, wp.vec3(0.0, 0.0, -1.0))
        h = half_extents[b]
        t_in = float(-1.0e6)
        t_out = float(1.0e6)
        inside_slabs = True
        for k in range(3):
            if wp.abs(d[k]) < 1.0e-8:
                if wp.abs(o[k]) > h[k]:
                    inside_slabs = False
            else:
                ta = (-h[k] - o[k]) / d[k]
                tb = (h[k] - o[k]) / d[k]
                t_in = wp.max(t_in, wp.min(ta, tb))
                t_out = wp.min(t_out, wp.max(ta, tb))
        if inside_slabs and t_out >= t_in and t_out >= 0.0:
            best = wp.max(best, start[2] - wp.max(t_in, 0.0))  # 出発点が箱の中なら出発点の高さ
    hit_z[i] = best


class TerrainRaycaster:
    """ステージの衝突物 (Mesh / Cube) と z=0 の地面を 1 つのメッシュにして、真下への光線を当てる。

    地形を置き直したら rebuild() を呼ぶ。Sphere や Cylinder など他の形状は高さ計測に映らない。
    動く物 (blocks.py のブロック、grid.py の柱) は add_boxes() で渡すと、毎回その時点の位置で光線を当てる。
    """

    def __init__(self, root_paths: tuple[str, ...] = ("/World/Terrain",), ground_z: float | None = 0.0,
                 device: str = "cuda:0") -> None:
        self._root_paths = root_paths
        self._ground_z = ground_z
        self._device = device
        self._box_sources: list = []
        self.rebuild()

    def add_boxes(self, boxes: object) -> None:
        """boxes() が (位置 (N, 3), 向き wxyz (N, 4), 辺の半分 (N,) vec3) の Warp 配列か None を返す物。"""
        self._box_sources.append(boxes)
        z = wp.zeros(1, dtype=float, device=self._device)  # カーネルのコンパイルを物理コールバックの外で済ませる
        empty2d = wp.zeros((0, 3), dtype=float, device=self._device)
        wp.launch(_cast_down_boxes_kernel, dim=1, device=self._device,
                  inputs=[wp.zeros(1, dtype=wp.vec3, device=self._device), empty2d,
                          wp.zeros((0, 4), dtype=float, device=self._device),
                          wp.zeros(0, dtype=wp.vec3, device=self._device), z])
        z.numpy()

    def rebuild(self) -> None:
        stage = get_current_stage()
        vertices: list[np.ndarray] = []
        triangles: list[np.ndarray] = []

        def add(points: np.ndarray, tris: np.ndarray) -> None:
            triangles.append(tris + sum(len(v) for v in vertices))
            vertices.append(points)

        if self._ground_z is not None:
            g = 500.0
            add(np.array([[-g, -g, self._ground_z], [g, -g, self._ground_z], [g, g, self._ground_z], [-g, g, self._ground_z]]),
                np.array([[0, 1, 2], [0, 2, 3]]))
        skipped = 0
        for root in self._root_paths:
            root_prim = stage.GetPrimAtPath(root)
            if not root_prim.IsValid():
                continue
            for prim in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies()):
                if not prim.HasAPI(UsdPhysics.CollisionAPI):
                    continue
                world = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
                if prim.IsA(UsdGeom.Cube):
                    local = _CUBE_CORNERS * UsdGeom.Cube(prim).GetSizeAttr().Get()
                    tris = _CUBE_TRIANGLES_OUTWARD
                elif prim.IsA(UsdGeom.Mesh):
                    mesh = UsdGeom.Mesh(prim)
                    local = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
                    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
                    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
                    tris_list, offset = [], 0
                    for n in counts:  # 多角形を扇形に三角形分割
                        face = indices[offset:offset + n]
                        tris_list.extend([face[0], face[k], face[k + 1]] for k in range(1, n - 1))
                        offset += n
                    tris = np.asarray(tris_list, dtype=np.int64).reshape(-1, 3)
                else:
                    skipped += 1
                    continue
                points_h = np.hstack([local, np.ones((len(local), 1))]) @ world  # USD は行ベクトル × 行列
                add(points_h[:, :3], tris)
        if skipped:
            carb.log_warn(f"TerrainRaycaster: {skipped} collision prims are not Mesh/Cube and are invisible to the height scan")
        self.num_triangles = sum(len(t) for t in triangles)
        self._points = wp.array(np.vstack(vertices).astype(np.float32), dtype=wp.vec3, device=self._device)
        self._indices = wp.array(np.vstack(triangles).astype(np.int32).ravel(), dtype=wp.int32, device=self._device)
        self._mesh = wp.Mesh(points=self._points, indices=self._indices)
        self.cast_down(np.zeros((1, 3)), with_boxes=False)  # カーネルのコンパイルを物理コールバックの外で済ませる

    def cast_down(self, starts: np.ndarray, with_boxes: bool = True) -> np.ndarray:
        """starts (N, 3) から真下に光線を飛ばし、当たった点の z を返す (当たらなければ -1e6)。"""
        starts_wp = wp.array(starts.astype(np.float32), dtype=wp.vec3, device=self._device)
        hit_z = wp.empty(len(starts), dtype=float, device=self._device)
        wp.launch(_cast_down_kernel, dim=len(starts), inputs=[self._mesh.id, starts_wp, hit_z], device=self._device)
        for source in self._box_sources if with_boxes else []:
            boxes = source()
            if boxes is not None:
                wp.launch(_cast_down_boxes_kernel, dim=len(starts), inputs=[starts_wp, *boxes, hit_z], device=self._device)
        return hit_z.numpy()


def _scan_grid() -> np.ndarray:
    x = np.round(np.arange(-SCAN_SIZE[0] / 2, SCAN_SIZE[0] / 2 + 1e-9, SCAN_RESOLUTION), 6)
    y = np.round(np.arange(-SCAN_SIZE[1] / 2, SCAN_SIZE[1] / 2 + 1e-9, SCAN_RESOLUTION), 6)
    grid_x, grid_y = np.meshgrid(x, y, indexing="xy")
    return np.stack([grid_x.ravel(), grid_y.ravel()], axis=1)  # (187, 2)


def _rotation_matrix_wxyz(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def _to_numpy(x: object) -> np.ndarray:
    return x.numpy() if hasattr(x, "numpy") else np.asarray(x)


def configure_robot(articulation: object, position: tuple[float, float, float], yaw_deg: float = 0.0) -> None:
    """再生前に呼ぶ: 関節を力指令にし、リセット時に戻る姿勢 (標準姿勢・指定位置・向き) を登録する。

    yaw_deg は真上から見た向き (+x から反時計回り、度)。

    RobotPolicyRunner は spawn() (USD の読み込みと物理プロパティ) だけに使う。
    restart_from_default_state() / step() を呼ぶとランナー側のモータモデルが物理ステップ前の
    コールバックとして登録され、こちらのトルクを上書きする。そのコールバックを物理コールバックの
    中から解除すると毎ステップ例外が出て最後は落ちる (Isaac Sim 6.2 で確認) ので、使わない。
    """
    articulation.set_dof_drive_types("force")
    # 静止したときに PhysX がロボットを眠らせると、トルクを書いても起きなくなることがあるので眠らせない
    stage = get_current_stage()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(articulation.paths[0])):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateSleepThresholdAttr().Set(0.0)
    articulation.set_default_state(
        positions=[list(position)],
        orientations=[[np.cos(np.radians(yaw_deg) / 2), 0.0, 0.0, np.sin(np.radians(yaw_deg) / 2)]],
        linear_velocities=[[0.0, 0.0, 0.0]],
        angular_velocities=[[0.0, 0.0, 0.0]],
        dof_positions=DEFAULT_JOINT_POS.reshape(1, -1),
        dof_velocities=np.zeros((1, 12), dtype=np.float32),
    )


def reset_robot(articulation: object) -> None:
    """物理ステップ中に呼ぶ: PD ゲインを 0 (トルクはこちらで計算) にして登録した姿勢へ戻す。"""
    articulation.switch_dof_control_mode("effort")
    articulation.reset_to_default_state()
    articulation.set_dof_efforts(np.zeros((1, 12), dtype=np.float32))


def load_actor(path: str) -> torch.nn.Sequential:
    """rsl_rl の actor (Linear と ELU の繰り返し) を重みだけのファイルから組み立てる。"""
    state = torch.load(path, map_location="cpu", weights_only=True)
    n_linear = len([k for k in state if k.endswith(".weight")])
    layers: list[torch.nn.Module] = []
    for i in range(n_linear):
        w = state[f"actor.{2 * i}.weight"]
        layers.append(torch.nn.Linear(w.shape[1], w.shape[0]))
        if i < n_linear - 1:
            layers.append(torch.nn.ELU())
    actor = torch.nn.Sequential(*layers)
    actor.load_state_dict({k.removeprefix("actor."): v for k, v in state.items()})
    return actor.eval()


class IsaacLabGo2Controller:
    """物理ステップごとに呼び、観測 → ポリシー → DC モータのトルクを関節に書き込む。

    Args:
        articulation: RobotPolicyRunner.spawn() で作った Go2 の Articulation。
        actor_path: load_actor で読める重みファイル。入力 235 なら高さ計測を使う。
        raycaster: 高さ計測に使う地形。None なら作る (入力 235 のときだけ使う)。
    """

    def __init__(self, articulation: object, actor_path: str, raycaster: TerrainRaycaster | None = None) -> None:
        self._articulation = articulation
        self._actor = load_actor(actor_path)
        self.num_obs = self._actor[0].in_features
        self.uses_height_scan = self.num_obs == 48 + 187
        if self.num_obs not in (48, 235):
            raise ValueError(f"unexpected policy input size {self.num_obs} (expected 48 or 235)")
        self.raycaster = raycaster if raycaster is not None or not self.uses_height_scan else TerrainRaycaster()
        self._scan_points = _scan_grid()
        self.last_height_scan = np.zeros(len(self._scan_points), dtype=np.float32)
        self.last_base_pose: tuple[np.ndarray, np.ndarray] | None = None  # (位置 xyz, 向き wxyz)、50 Hz で更新
        self.reset()

    def reset(self) -> None:
        self._tick = 0
        self._action = np.zeros(12, dtype=np.float32)
        self._target = DEFAULT_JOINT_POS.copy()

    # -- 1 物理ステップ分 ------------------------------------------------------------------

    def step(self, command: np.ndarray) -> None:
        q = _to_numpy(self._articulation.get_dof_positions()).reshape(-1)
        qd = _to_numpy(self._articulation.get_dof_velocities()).reshape(-1)
        if self._tick % DECIMATION == 0:
            obs = self._observation(q, qd, np.asarray(command, dtype=np.float32))
            with torch.no_grad():
                self._action = self._actor(torch.from_numpy(obs).unsqueeze(0))[0].numpy()
            self._target = DEFAULT_JOINT_POS + ACTION_SCALE * self._action
        self._tick += 1
        self._articulation.set_dof_efforts(self._dc_motor_torque(q, qd).reshape(1, -1))

    def _dc_motor_torque(self, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        torque = KP * (self._target - q) - KD * qd
        max_torque = np.clip(SATURATION_EFFORT * (1.0 - qd / VELOCITY_LIMIT), 0.0, EFFORT_LIMIT)
        min_torque = np.clip(SATURATION_EFFORT * (-1.0 - qd / VELOCITY_LIMIT), -EFFORT_LIMIT, 0.0)
        return np.clip(torque, min_torque, max_torque).astype(np.float32)

    # -- 観測 -------------------------------------------------------------------------------

    def _observation(self, q: np.ndarray, qd: np.ndarray, command: np.ndarray) -> np.ndarray:
        positions, orientations = self._articulation.get_world_poses()
        base_pos = _to_numpy(positions).reshape(-1)
        base_quat = _to_numpy(orientations).reshape(-1)
        # GPU からの読み出しは遅いので、追従カメラなどはこの値を使い回す
        self.last_base_pose = (base_pos.copy(), base_quat.copy())
        rot = _rotation_matrix_wxyz(base_quat)
        lin_vel, ang_vel = self._articulation.get_velocities()
        parts = [
            rot.T @ _to_numpy(lin_vel).reshape(-1),
            rot.T @ _to_numpy(ang_vel).reshape(-1),
            rot.T @ np.array([0.0, 0.0, -1.0], dtype=np.float32),
            command,
            q - DEFAULT_JOINT_POS,
            qd,
            self._action,
        ]
        if self.uses_height_scan:
            self.last_height_scan = self._height_scan(base_pos, rot)
            parts.append(self.last_height_scan)
        return np.concatenate(parts).astype(np.float32)

    def _height_scan(self, base_pos: np.ndarray, rot: np.ndarray) -> np.ndarray:
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        c, s = np.cos(yaw), np.sin(yaw)
        xy = self._scan_points @ np.array([[c, s], [-s, c]], dtype=np.float32) + base_pos[:2]
        starts = np.hstack([xy, np.full((len(xy), 1), base_pos[2] + SCAN_START_ABOVE_BASE)])
        hit_z = self.raycaster.cast_down(starts)
        return np.clip(base_pos[2] - hit_z - SCAN_OFFSET, -1.0, 1.0).astype(np.float32)
