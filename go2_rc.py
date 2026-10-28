"""Go2 をキーボードでラジコン操作し、足先の接触力を OSC /gait/force で送るシミュレータ。

innu.maxpat の [udpreceive 9871] → [route /gait/force] がそのまま受け取れる形式で送る:
    /gait/force i i i i   (FR FL RR RL, 実機 LowState.foot_force 相当の整数)

歩行ポリシー (--policy):
    rough   Isaac Lab の不整地用 (足元の高さ計測つき)。階段・ブロック・凹凸を踏破できる。既定
    flat    Isaac Lab の平地用。低い姿勢のすり足になる
    bundled Isaac Sim 付属の Go2 ポリシー (平地用)。低い姿勢のすり足になる

操作 (ビューポートをクリックしてフォーカスしてから):
    ↑ / ↓ (W / S)   前進 / 後退
    ← / → (A / D)   左 / 右へ横歩き
    N / M (Q / E)   左旋回 / 右旋回
    R               初期位置に戻す (転んだときも)
    T               障害物を動かした・足したあとに、高さ計測用の地形を作り直す
    C               Go2 の後ろを追従するカメラ / 自由に動かせる通常のカメラ を切り替える (起動時は追従)
    F               足場を手前 (象の鼻テラスでは入口) から少し見下ろす固定カメラ / 元のカメラ を切り替える (--grid)
    B               10×10×20 cm のブロックを Go2 の前に寝かせて置く (進む向きに対して横向き)
    G               10×10×20 cm のブロックを Go2 の前に立てて置く
    V               10 cm 角のブロックを Go2 の前に置く
    P               10 cm 角のブロックを 4 段のピラミッド (4×4, 3×3, 2×2, 1 の 30 個) に積んで Go2 の前に置く
    X               置いたブロックをすべて片付ける
    1〜5            速さを 20 / 40 / 60 / 80 / 100 % にする (前後・横・旋回のすべて)
    - / = (テンキーの - / +)   速さを 10 % ずつ下げる / 上げる (10〜100 %)

障害物は /World/Terrain の下に、衝突判定 (Collision) 付きで置く (terrain.py を参照)。
キーで置くブロックは物理演算つきで、蹴ると動き、積むこともできる (blocks.py を参照)。

胴体の IMU (/gait/force と同じ送信先へ 50 Hz。実機の LowState.imu_state と同じ値・単位):
    /imu/rpy    f f f  姿勢 roll pitch yaw [rad] (roll + = 右に傾く、pitch + = 頭が下がる、yaw + = 左を向く)
    /imu/gyro   f f f  胴体の x (前) y (左) z (上) 軸まわりの回転の速さ [rad/s]
    /imu/accel  f f f  胴体の x y z 方向の加速度 [m/s²]。重力を含む (水平に止まっていると z ≒ +9.8)

記録 (--record 記録.csv): 50 Hz で 1 行ずつ CSV に書く
    time_s (シミュレーションの時刻 [s])、wall_time (PC の時計)、FR_N FL_N RR_N RL_N (足の荷重 [N])、
    FR FL RR RL (/gait/force で送った値)、roll pitch yaw (/imu/rpy と同じ [rad])、x y z (胴体の位置 [m])

8×8 の足場 (--grid、grid.py を参照):
    Max → シミュレータ  /grid/height  64 個の数 (整数でも小数でも)。マスごとの高さ [cm]、受け取ったときに一括で反映
                        送り先はこの PC の --listen-port (既定 9872)
    シミュレータ → Max  /grid/step  i × 64。マスごとに今踏まれていれば 1。/gait/force と同じ送信先へ 50 Hz で送る
    シミュレータ → Max  /grid/foot  i × 4 (FR FL RR RL)。足ごとに着いているマスの番号、浮いている・足場の外なら -1
    並び順は row × 8 + col (row 0 が Go2 の初期位置に近い列、col 0 が Go2 から見て左端)

起動:
    run_go2.bat をダブルクリック、または
    .\\env_isaacsim\\Scripts\\activate
    python go2_rc.py [--host 127.0.0.1] [--port 9871] [--policy rough] [--course demo] [--usd 会場.usd] [--spawn-yaw 度]

会場モデル:
    venues\\zounohana_terrace.usdc   象の鼻テラス (tools\\build_zounohana_terrace.py で Blender から生成)。
                                    原点は北東の正面入口の 6 m 手前で、入口は --spawn-yaw -158 の向き
"""

import argparse
import csv
import datetime
import os
import socket
import struct
import time

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Go2 keyboard RC + /gait/force OSC sender")
parser.add_argument("--host", default="127.0.0.1", help="OSC 送信先 (Mac の Max なら Mac の IP)")
parser.add_argument("--port", type=int, default=9871, help="OSC 送信先ポート (innu.maxpat は 9871)")
parser.add_argument("--policy", choices=["rough", "flat", "bundled"], default="rough", help="歩行ポリシー")
parser.add_argument("--course", choices=["demo", "none"], default="demo", help="demo: 階段・凹凸・ブロックを置く")
parser.add_argument("--usd", default=None, help="会場モデルなどの USD ファイル (/World/Terrain/Venue に置く)")
parser.add_argument("--spawn-yaw", type=float, default=0.0,
                    help="Go2 の初期の向き [度] (+x から反時計回り。bundled では無視)")
parser.add_argument("--speed", type=float, default=1.0,
                    help="起動時の速さ (0.1〜1.0。1.0 で前後 1.0 m/s・横 0.6 m/s・旋回 1.0 rad/s)")
parser.add_argument("--print-force", action="store_true", help="送信値を 0.5 秒ごとに表示する")
parser.add_argument("--grid", action="store_true", help="8×8 の足場を置く (高さは Max から /grid/height で変える)")
parser.add_argument("--grid-center", type=float, nargs=2, default=None, metavar=("X", "Y"),
                    help="足場の中心 [m]。省略すると Go2 の初期位置の --grid-ahead 先から始まるように置く")
parser.add_argument("--grid-ahead", type=float, default=1.0,
                    help="Go2 の初期位置から足場の手前の端までの距離 [m] (--grid-center を省略したとき)")
parser.add_argument("--listen-port", type=int, default=9872, help="Max から /grid/height を受け取るポート")
parser.add_argument("--physics-device", choices=["cpu", "cuda"], default="cpu",
                    help="物理計算の場所。ロボット 1 台とブロック程度なら CPU のほうが速い")
parser.add_argument("--fixed-camera", action="store_true", help="起動時から足場を見る固定カメラにする (F キーで切り替え可)")
parser.add_argument("--record", default=None, metavar="CSV",
                    help="足の荷重と /imu/rpy を時刻つきで CSV に書き出す (50 Hz)")
parser.add_argument("--no-chase-camera", action="store_true", help="起動時は追従カメラを使わない (C キーで切り替え可)")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp(
    {
        "headless": False,
        "extra_args": [
            "--enable", "isaacsim.sensors.experimental.physics",
            "--/app/runLoops/main/rateLimitEnabled=false",  # 実時間合わせはメインループで行う
        ],
    }
)

import carb
import numpy as np
import omni.appwindow
import omni.timeline
from isaacsim.core.experimental.utils.stage import define_prim, get_current_stage, set_stage_units, set_stage_up_axis
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples import RobotPolicyRunner, get_go2_spec
from isaacsim.sensors.experimental.physics import Contact, ContactSensor
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, UsdGeom

import blocks
import grid
import terrain
from go2_policy import IsaacLabGo2Controller, TerrainRaycaster, _rotation_matrix_wxyz, configure_robot, reset_robot

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------------------------
# 設定
# ---------------------------------------------------------------------------------------------

LEGS = ["FR", "FL", "RR", "RL"]  # innu.maxpat の並び順
SPAWN_POSITION = (0.0, 0.0, 0.5)
PHYSICS_DT = 0.005  # 200 Hz (学習時と同じ)
FRAME_DT = 0.02  # 描画・ポリシー 50 Hz

# 力の変換: 生の値 = 力[N] × (実機の静止立ちの値 / 静止時に 1 本の足にかかる力[N])
# 実機の値は innu.maxpat のメッセージボックスに残っていた値 (仮)。土曜に実測して差し替える。
REAL_STANDING = np.array([23.0, 17.0, 36.0, 26.0])
# シミュレータの静止立ちは 4 本の合計が 147 N (≒ 15 kg × g) で一定だが、足ごとの配分は
# 直前の動きやポリシーで大きく変わる (2026-10-25 測定) 。
# そのため足ごとの実測ではなく、均等に 1/4 ずつかかっていると仮定して倍率を決める。
SIM_STANDING_PER_FOOT_N = 147.3 / 4
FORCE_GAIN = REAL_STANDING / SIM_STANDING_PER_FOOT_N

# キー → [vx, vy, yaw_rate] (押している間だけ加算)。学習時の指令範囲は各 ±1.0
VX, VY, YAW = 1.0, 0.6, 1.0  # m/s, m/s, rad/s (速さ 100 % のとき)
# 速さ: キーで決めた指令にこの倍率を掛ける。学習時の範囲を超えないよう 1.0 まで
SPEED_MIN, SPEED_MAX, SPEED_STEP = 0.1, 1.0, 0.1
SPEED_KEYS = {"KEY_1": 0.2, "KEY_2": 0.4, "KEY_3": 0.6, "KEY_4": 0.8, "KEY_5": 1.0}
SPEED_STEP_KEYS = {"MINUS": -1, "NUMPAD_SUBTRACT": -1, "EQUAL": 1, "NUMPAD_ADD": 1}
KEY_COMMANDS = {
    "UP": [VX, 0, 0], "W": [VX, 0, 0], "NUMPAD_8": [VX, 0, 0],
    "DOWN": [-VX, 0, 0], "S": [-VX, 0, 0], "NUMPAD_2": [-VX, 0, 0],
    "LEFT": [0, VY, 0], "A": [0, VY, 0], "NUMPAD_4": [0, VY, 0],
    "RIGHT": [0, -VY, 0], "D": [0, -VY, 0], "NUMPAD_6": [0, -VY, 0],
    "N": [0, 0, YAW], "Q": [0, 0, YAW], "NUMPAD_7": [0, 0, YAW],
    "M": [0, 0, -YAW], "E": [0, 0, -YAW], "NUMPAD_9": [0, 0, -YAW],
}

# ブロック: キー → (種類, 立てるか)。Go2 の胴体の BLOCK_AHEAD 前の、地面 (や他のブロック) の
# BLOCK_DROP_HEIGHT 上から落とす
BLOCK_KEYS = {"B": ("long", False), "G": ("long", True), "V": ("cube", False)}
BLOCK_AHEAD = 0.7  # m
BLOCK_DROP_HEIGHT = 0.05  # m
# P: 10 cm 角のブロックのピラミッド。下から PYRAMID_LEVELS × PYRAMID_LEVELS, …, 1 個ずつ積む
PYRAMID_LEVELS = 4  # 4×4, 3×3, 2×2, 1 の 30 個 (blocks.BLOCKS_PER_KIND["cube"] 以下にする)
PYRAMID_AHEAD = 0.8  # Go2 の胴体からピラミッドの中心まで [m]


# ---------------------------------------------------------------------------------------------
# OSC (python-osc を使わず、必要な分だけを自前で組み立て・読み取る)
# ---------------------------------------------------------------------------------------------


def _osc_string(s: str) -> bytes:
    b = s.encode("ascii") + b"\0"
    return b + b"\0" * (-len(b) % 4)


OSC_PREFIX = _osc_string("/gait/force") + _osc_string(",iiii")
osc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
osc_target = (args.host, args.port)


def send_gait_force(values: np.ndarray) -> None:
    try:
        osc_socket.sendto(OSC_PREFIX + struct.pack(">4i", *values), osc_target)
    except OSError as error:  # 送信先が無くてもシミュレーションは止めない
        carb.log_warn(f"OSC send failed: {error}")


def send_osc_ints(address: str, values: np.ndarray) -> None:
    packet = _osc_string(address) + _osc_string("," + "i" * len(values)) + struct.pack(f">{len(values)}i", *values)
    try:
        osc_socket.sendto(packet, osc_target)
    except OSError as error:
        carb.log_warn(f"OSC send failed: {error}")


def send_osc_floats(address: str, values: np.ndarray) -> None:
    packet = _osc_string(address) + _osc_string("," + "f" * len(values)) + struct.pack(f">{len(values)}f", *values)
    try:
        osc_socket.sendto(packet, osc_target)
    except OSError as error:
        carb.log_warn(f"OSC send failed: {error}")


def _read_osc_string(data: bytes, offset: int) -> tuple[str, int]:
    end = data.index(b"\0", offset)
    return data[offset:end].decode("ascii", "replace"), end + 1 + (-(end + 1 - offset) % 4)


def parse_osc(data: bytes) -> list[tuple[str, list[float]]]:
    """OSC のメッセージ (またはバンドル) を [(アドレス, 数の引数), ...] にする。数以外の引数は飛ばす。"""
    if data.startswith(b"#bundle\0"):
        messages, offset = [], 16  # "#bundle\0" + タイムタグ 8 バイト
        while offset + 4 <= len(data):
            (size,) = struct.unpack_from(">i", data, offset)
            messages += parse_osc(data[offset + 4:offset + 4 + size])
            offset += 4 + size
        return messages
    address, offset = _read_osc_string(data, 0)
    tags, offset = _read_osc_string(data, offset) if offset < len(data) else (",", offset)
    values: list[float] = []
    for tag in tags[1:]:
        if tag in "if":
            values.append(struct.unpack_from(">i" if tag == "i" else ">f", data, offset)[0])
            offset += 4
        elif tag in "hd":
            values.append(struct.unpack_from(">q" if tag == "h" else ">d", data, offset)[0])
            offset += 8
        elif tag == "s":
            offset = _read_osc_string(data, offset)[1]
        elif tag == "b":
            (size,) = struct.unpack_from(">i", data, offset)
            offset += 4 + size + (-size % 4)
    return [(address, values)]


# ---------------------------------------------------------------------------------------------
# シーン
# ---------------------------------------------------------------------------------------------

set_stage_up_axis("Z")
set_stage_units(meters_per_unit=1.0)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    raise RuntimeError("Could not find Isaac Sim assets folder")

define_prim("/World/Ground", "Xform").GetReferences().AddReference(
    assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
)
define_prim("/World/PhysicsScene", "PhysicsScene")
if args.course == "demo":
    terrain.build_demo_course()
if args.usd:
    n_collision = terrain.add_usd(os.path.abspath(args.usd))
    print(f"go2_rc: loaded {args.usd} ({n_collision} meshes given collision)" if n_collision
          else f"go2_rc: loaded {args.usd} (using the file's own collision settings)")
# GPU の物理計算は、動く剛体 (ブロック) があると 1 ステップごとの読み書きが重くなり実時間の 0.6 倍程度に
# 落ちる。ロボット 1 台の小さなシーンでは CPU のほうが速い (2026-10-25 測定: CPU 約 1.0 倍)
SimulationManager.set_physics_sim_device(args.physics_device)

# 8×8 の足場 (高さ計測用の地形メッシュを作る前に置く。囲いは地形として、柱は動く箱として測る)
stepping_grid = None
terrain_roots = (terrain.TERRAIN_ROOT,)
if args.grid:
    grid_yaw = np.radians(args.spawn_yaw if args.policy != "bundled" else 0.0)
    if args.grid_center is not None:
        grid_center = np.array(args.grid_center)
    else:
        grid_center = np.array(SPAWN_POSITION[:2]) + (args.grid_ahead + grid.GRID_N * grid.GRID_CELL / 2) * np.array(
            [np.cos(grid_yaw), np.sin(grid_yaw)])
    # 足場を置く床の高さを測る (建物の中なら床の上面)。屋根に当てないよう、床から 1 m ほどの高さから下へ
    floor_raycaster = TerrainRaycaster(terrain_roots)
    probe = grid.SteppingGrid.cell_centers(grid_center, grid_yaw)
    floor_z = float(floor_raycaster.cast_down(np.column_stack([probe, np.full(len(probe), 1.0)])).max())
    del floor_raycaster
    stepping_grid = grid.SteppingGrid(tuple(grid_center), grid_yaw, base_z=floor_z)
    terrain_roots += (grid.FENCE_ROOT,)
    print(f"go2_rc: 8x8 grid at ({grid_center[0]:.2f}, {grid_center[1]:.2f}, floor {floor_z:.2f}), "
          f"listening for /grid/height on UDP {args.listen_port}")

# Go2 の読み込みは付属のランナーに任せる (bundled のときは歩行もランナー)
go2 = RobotPolicyRunner(get_go2_spec(), prim_path="/World/Go2", position=list(SPAWN_POSITION))
go2.spawn()
RenderingManager.set_dt(FRAME_DT)
SimulationManager.set_physics_dt(PHYSICS_DT)

controller = None
if args.policy != "bundled":
    configure_robot(go2.articulation, SPAWN_POSITION, args.spawn_yaw)
    actor_path = os.path.join(PROJECT_DIR, "policies", f"go2_{args.policy}_actor.pt")
    raycaster = TerrainRaycaster(terrain_roots) if args.policy == "rough" else None
    controller = IsaacLabGo2Controller(go2.articulation, actor_path, raycaster)

# 物理演算つきのブロック (再生前に作っておく)。高さ計測にも映す
block_pool = blocks.BlockPool()
if controller is not None and controller.raycaster is not None:
    block_raycaster = controller.raycaster
else:  # 平地用ポリシーでも、ブロックを床や他のブロックの上に置くために地面の高さは測る
    block_raycaster = TerrainRaycaster(terrain_roots)
block_raycaster.add_boxes(block_pool.boxes)
if stepping_grid is not None:
    block_raycaster.add_boxes(stepping_grid.boxes)

foot_sensors = [
    ContactSensor(Contact.create(f"/World/Go2/{leg}_foot/contact_sensor", min_threshold=0, max_threshold=1e7, radius=-1))
    for leg in LEGS
]
# 足場のどのマスを踏んでいるかを調べるための、足先の位置
foot_links = RigidPrim([f"/World/Go2/{leg}_foot" for leg in LEGS]) if stepping_grid is not None else None
GRID_SEND_EVERY = 4  # /grid/step と /grid/foot は物理 4 ステップごと (50 Hz) に送る
IMU_SEND_EVERY = 4  # /imu/* も 50 Hz
GRAVITY = np.array([0.0, 0.0, -9.81])
imu_previous_velocity: np.ndarray | None = None  # 加速度を速度の差から求めるための、前回の胴体の速度 (ワールド座標)


def send_imu() -> tuple[np.ndarray, np.ndarray]:
    """胴体の姿勢・角速度・加速度を、実機の IMU (LowState.imu_state) と同じ形で送る。胴体の (位置, rpy) を返す。"""
    global imu_previous_velocity
    positions, orientations = go2.articulation.get_world_poses()
    lin_vel, ang_vel = go2.articulation.get_velocities()
    w, x, y, z = orientations.numpy().reshape(-1)
    lin_vel = lin_vel.numpy().reshape(-1).astype(np.float64)
    rot = _rotation_matrix_wxyz(np.array([w, x, y, z]))
    rpy = [np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
           np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0)),
           np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))]
    gyro = rot.T @ ang_vel.numpy().reshape(-1)
    # 加速度計は「加速度 − 重力」を胴体の向きで測る (止まっていると上向きに +9.8)
    world_accel = np.zeros(3) if imu_previous_velocity is None else (
        (lin_vel - imu_previous_velocity) / (IMU_SEND_EVERY * PHYSICS_DT))
    imu_previous_velocity = lin_vel
    accel = rot.T @ (world_accel - GRAVITY)
    send_osc_floats("/imu/rpy", np.array(rpy))
    send_osc_floats("/imu/gyro", gyro)
    send_osc_floats("/imu/accel", accel)
    return positions.numpy().reshape(-1), np.array(rpy)


record_file = None
record_writer = None
if args.record:
    record_file = open(args.record, "w", newline="", encoding="utf-8")
    record_writer = csv.writer(record_file)
    record_writer.writerow(["time_s", "wall_time", *(f"{leg}_N" for leg in LEGS), *LEGS,
                            "roll", "pitch", "yaw", "x", "y", "z"])
    print(f"go2_rc: recording foot force and /imu/rpy to {os.path.abspath(args.record)}")


def record_row(force_n: np.ndarray, base_position: np.ndarray, rpy: np.ndarray) -> None:
    record_writer.writerow([f"{physics_steps * PHYSICS_DT:.3f}", datetime.datetime.now().isoformat(timespec="milliseconds"),
                            *(f"{v:.2f}" for v in force_n), *last_sent.tolist(),
                            *(f"{v:.4f}" for v in rpy), *(f"{v:.3f}" for v in base_position)])
last_grid_step = np.zeros(grid.GRID_N * grid.GRID_N, dtype=np.int32)

# ---------------------------------------------------------------------------------------------
# キーボード
# ---------------------------------------------------------------------------------------------

base_command = np.zeros(3, dtype=np.float32)
speed_scale = float(np.clip(args.speed, SPEED_MIN, SPEED_MAX))


def set_speed(value: float) -> None:
    global speed_scale
    speed_scale = float(np.clip(round(value, 2), SPEED_MIN, SPEED_MAX))
    print(f"go2_rc: speed {speed_scale * 100:.0f} % (forward {VX * speed_scale:.2f} m/s, "
          f"side {VY * speed_scale:.2f} m/s, turn {YAW * speed_scale:.2f} rad/s)", flush=True)


def command() -> np.ndarray:
    return base_command * speed_scale
reset_requested = False
terrain_rebuild_requested = False
camera_toggle_requested = False
fixed_camera_toggle_requested = False
block_requests: list[tuple[str, bool] | str | None] = []  # "pyramid" = ピラミッド、None = すべて片付ける


def on_keyboard_event(event: object, *args: object, **kwargs: object) -> bool:
    global reset_requested, terrain_rebuild_requested, camera_toggle_requested, fixed_camera_toggle_requested
    name = event.input.name
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if name == "R":
            reset_requested = True
        elif name == "T":
            terrain_rebuild_requested = True
        elif name == "C":
            camera_toggle_requested = True
        elif name == "F":
            fixed_camera_toggle_requested = True
        elif name in BLOCK_KEYS:
            block_requests.append(BLOCK_KEYS[name])
        elif name == "P":
            block_requests.append("pyramid")
        elif name == "X":
            block_requests.append(None)
        elif name in SPEED_KEYS:
            set_speed(SPEED_KEYS[name])
        elif name in SPEED_STEP_KEYS:
            set_speed(speed_scale + SPEED_STEP_KEYS[name] * SPEED_STEP)
    increment = KEY_COMMANDS.get(name)
    if increment is not None:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            base_command[:] += increment
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            base_command[:] -= increment
    return True


input_interface = carb.input.acquire_input_interface()
keyboard = omni.appwindow.get_default_app_window().get_keyboard()
keyboard_sub = input_interface.subscribe_to_keyboard_events(keyboard, on_keyboard_event)

# ---------------------------------------------------------------------------------------------
# 追従カメラ: Go2 の後ろ上方から、少し遅れて滑らかについていく
# ---------------------------------------------------------------------------------------------

CHASE_CAMERA_PATH = "/World/ChaseCam"
CHASE_DISTANCE = 4.4  # Go2 の後ろ [m]
CHASE_HEIGHT = 1.0  # Go2 の胴体からの高さ [m]
CHASE_LOOK_AHEAD = 1.2  # Go2 の少し前を見る [m]
CHASE_TIME_CONSTANT = 0.3  # 追従の遅れ [s] (小さいほどぴったりつく)
# 足場を見る固定カメラ: 足場の中心から Go2 の初期の向きと逆へ FIXED_BACK、床から FIXED_HEIGHT の所に置き、
# 足場の中心を見る。象の鼻テラス (--grid-ahead 7.6) では入口のガラス面の 0.5 m 手前 (スロープの上) になる
FIXED_CAMERA_PATH = "/World/GridCam"
FIXED_BACK = 3.1  # [m]
FIXED_HEIGHT = 1.2  # [m] (足場の中心を約 20° 見下ろす)
FIXED_FOCAL_LENGTH = 18.0  # [mm] (足場全体と Go2 が入る画角)


class ChaseCamera:
    def __init__(self) -> None:
        camera =UsdGeom.Camera.Define(get_current_stage(), CHASE_CAMERA_PATH)
        camera.CreateFocalLengthAttr(18.0)
        camera.CreateClippingRangeAttr().Set((0.05, 1000.0))
        self._op = UsdGeom.Xformable(camera.GetPrim()).AddTransformOp()
        self._viewport = get_active_viewport()
        self._free_camera_path = str(self._viewport.camera_path)
        self._eye = None
        self.enabled = False

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled
        self._eye = None  # 切り替えた直後は飛ばして合わせる
        self._viewport.camera_path = CHASE_CAMERA_PATH if enabled else self._free_camera_path

    def update(self, pose: tuple[np.ndarray, np.ndarray] | None, dt: float) -> None:
        """pose は Go2 の胴体の (位置 xyz, 向き wxyz)。"""
        if not self.enabled or pose is None:
            return
        p, (w, x, y, z) = pose
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        forward = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        eye = p - forward * CHASE_DISTANCE + np.array([0.0, 0.0, CHASE_HEIGHT])
        if self._eye is None:
            self._eye = eye
        else:
            self._eye += (eye - self._eye) * (1.0 - np.exp(-dt / CHASE_TIME_CONSTANT))
        target = p + forward * CHASE_LOOK_AHEAD + np.array([0.0, 0.0, 0.1])
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*self._eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
        self._op.Set(view.GetInverse())


chase_camera = ChaseCamera()
chase_camera.set_enabled(not args.no_chase_camera)

fixed_camera_on = False
chase_before_fixed = chase_camera.enabled  # 固定カメラから戻るときに、追従カメラに戻すか
if stepping_grid is not None:
    _camera = UsdGeom.Camera.Define(get_current_stage(), FIXED_CAMERA_PATH)
    _camera.CreateFocalLengthAttr(FIXED_FOCAL_LENGTH)
    _camera.CreateClippingRangeAttr().Set((0.05, 1000.0))
    _center = np.array([*stepping_grid.center, stepping_grid.base_z])
    _back = np.array([*stepping_grid.forward, 0.0])
    _eye = _center - FIXED_BACK * _back + np.array([0.0, 0.0, FIXED_HEIGHT])
    UsdGeom.Xformable(_camera.GetPrim()).AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(Gf.Vec3d(*_eye), Gf.Vec3d(*_center), Gf.Vec3d(0, 0, 1)).GetInverse())


def set_fixed_camera(on: bool) -> None:
    """固定カメラにする / 切り替える前のカメラ (追従か通常) に戻す。"""
    global fixed_camera_on, chase_before_fixed
    if stepping_grid is None or on == fixed_camera_on:
        return
    fixed_camera_on = on
    if on:
        chase_before_fixed = chase_camera.enabled
        chase_camera.set_enabled(False)
        get_active_viewport().camera_path = FIXED_CAMERA_PATH
    else:
        chase_camera.set_enabled(chase_before_fixed)


set_fixed_camera(args.fixed_camera)

# ---------------------------------------------------------------------------------------------
# 物理ステップごと: 歩行ポリシーを進め、足の力を送る
# ---------------------------------------------------------------------------------------------

first_step = True
policy_failed = False
last_sent = np.zeros(4, dtype=np.int32)
physics_steps = 0


def handle_block_requests() -> None:
    """物理コールバックの中で呼ぶ: キーで頼まれたブロックを置く・片付ける。"""
    while block_requests:
        request = block_requests.pop(0)
        if request is None:
            block_pool.clear()
            continue
        if controller is not None and controller.last_base_pose is not None:
            base_pos, (w, x, y, z) = controller.last_base_pose
        else:
            positions, orientations = go2.articulation.get_world_poses()
            base_pos, (w, x, y, z) = positions.numpy().reshape(-1), orientations.numpy().reshape(-1)
        heading = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        if request == "pyramid":
            spawn_pyramid(base_pos, heading)
            continue
        kind, upright = request
        xy = base_pos[:2] + BLOCK_AHEAD * np.array([np.cos(heading), np.sin(heading)])
        ground_z = block_raycaster.cast_down(np.array([[xy[0], xy[1], base_pos[2] + 0.5]]))[0]
        size = block_pool.size(kind)
        height = size[0] if upright else size[2]
        position = np.array([xy[0], xy[1], max(ground_z, -1.0) + height / 2 + BLOCK_DROP_HEIGHT])
        # 寝かせるときは長辺を進む向きに対して横に、立てるときは面を Go2 に向ける
        yaw = heading if upright or kind == "cube" else heading + np.pi / 2
        block_pool.spawn(kind, position, yaw, upright)


def spawn_pyramid(base_pos: np.ndarray, heading: float) -> None:
    """10 cm 角のブロックを、面を Go2 に向けたピラミッドに積んだ状態で置く (落とさない)。"""
    size = block_pool.size("cube")[0]
    pitch = size * 1.01  # ぴったり並べると隣どうしがめり込んではじけるので、少しすき間をあける
    forward = np.array([np.cos(heading), np.sin(heading)])
    left = np.array([-forward[1], forward[0]])
    center = base_pos[:2] + PYRAMID_AHEAD * forward
    # 古いピラミッドのブロックを使い回すことがあるので、ブロックは無視して地面の高さを測る
    ground_z = block_raycaster.cast_down(np.array([[center[0], center[1], base_pos[2] + 0.5]]), with_boxes=False)[0]
    for level in range(PYRAMID_LEVELS):
        n = PYRAMID_LEVELS - level
        z = max(ground_z, -1.0) + size * (level + 0.5) + 0.002 * (level + 1)
        for i in range(n):
            for j in range(n):
                u, v = (np.array([i, j]) - (n - 1) / 2) * pitch
                xy = center + u * forward + v * left
                block_pool.spawn("cube", np.array([xy[0], xy[1], z]), heading)


def on_physics_step(step_size: float, context: object) -> None:
    global first_step, reset_requested, policy_failed, last_sent, physics_steps, last_grid_step, imu_previous_velocity
    physics_steps += 1
    if policy_failed:
        return
    try:
        if block_requests and not first_step:
            handle_block_requests()
        if stepping_grid is not None and stepping_grid.apply_pending_heights():
            print(f"go2_rc: grid heights set (cm) {np.round(stepping_grid.heights * 100).astype(int).tolist()}", flush=True)
        if first_step or reset_requested:
            if controller is None:
                go2.restart_from_default_state(command().tolist())
            else:
                reset_robot(go2.articulation)
                controller.reset()
            first_step = reset_requested = False
            imu_previous_velocity = None
        elif controller is None:
            go2.step(step_size, command().tolist())
        else:
            controller.step(command())
    except Exception as error:  # noqa: BLE001 - a physics callback must not raise
        policy_failed = True
        carb.log_error(f"go2_rc: policy failed, stopping: {error}")
        return

    force_n = np.array([s.get_sensor_reading().value for s in foot_sensors])
    last_sent = np.rint(np.maximum(force_n, 0.0) * FORCE_GAIN).astype(np.int32)
    send_gait_force(last_sent)
    if physics_steps % IMU_SEND_EVERY == 0:
        base_position, rpy = send_imu()
        if record_writer is not None:
            record_row(force_n, base_position, rpy)
    if stepping_grid is not None and physics_steps % GRID_SEND_EVERY == 0:
        positions, _ = foot_links.get_world_poses()
        foot_cells = stepping_grid.foot_cells(positions.numpy()[:, :2], force_n)
        last_grid_step = stepping_grid.stepped(foot_cells)
        send_osc_ints("/grid/step", last_grid_step)
        send_osc_ints("/grid/foot", foot_cells)


physics_callback_id = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)

# ---------------------------------------------------------------------------------------------
# メインループ (実時間に合わせる)
# ---------------------------------------------------------------------------------------------

print(f"go2_rc: policy '{args.policy}', sending /gait/force (FR FL RR RL) to {args.host}:{args.port}")
print("go2_rc: arrows/WASD = move, N/M or Q/E = turn, R = reset, T = rebuild terrain scan, C = chase camera on/off"
      + (", F = fixed grid camera on/off" if stepping_grid is not None else ""))
print("go2_rc: B / G = 10x10x20 cm block lying / standing, V = 10 cm cube, P = cube pyramid, X = clear blocks")
print(f"go2_rc: 1-5 = speed 20-100 %, - / = = slower / faster (now {speed_scale * 100:.0f} %)")

# Max からの /grid/height (メインループで受け取り、次の物理ステップで反映する)
listen_socket = None
if stepping_grid is not None:
    listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listen_socket.bind(("0.0.0.0", args.listen_port))
    listen_socket.setblocking(False)


def poll_osc_input() -> None:
    while True:
        try:
            data = listen_socket.recv(65536)
        except (BlockingIOError, ConnectionResetError):
            return
        try:
            messages = parse_osc(data)
        except (ValueError, struct.error, IndexError) as error:
            carb.log_warn(f"go2_rc: could not read an OSC packet: {error}")
            continue
        for address, values in messages:
            if address != "/grid/height":
                continue
            if len(values) != grid.GRID_N * grid.GRID_N:
                carb.log_warn(f"go2_rc: /grid/height needs {grid.GRID_N * grid.GRID_N} values, got {len(values)}")
                continue
            stepping_grid.request_heights(np.array(values, dtype=np.float64) / 100.0)  # cm → m


timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

next_frame = time.perf_counter()
next_print, steps_at_print = next_frame, physics_steps
while simulation_app.is_running():
    simulation_app.update()
    if not SimulationManager.is_simulating():
        first_step = True  # 停止 → 再生したら初期化からやり直す
    if listen_socket is not None:
        poll_osc_input()
    if stepping_grid is not None:
        stepping_grid.show_stepped(last_grid_step)
    if camera_toggle_requested:
        camera_toggle_requested = False
        if fixed_camera_on:  # 固定カメラのときに C を押したら追従カメラへ
            fixed_camera_on = False
            chase_camera.set_enabled(True)
        else:
            chase_camera.set_enabled(not chase_camera.enabled)
    if fixed_camera_toggle_requested:
        fixed_camera_toggle_requested = False
        set_fixed_camera(not fixed_camera_on)
    if SimulationManager.is_simulating() and chase_camera.enabled:
        if controller is not None:
            chase_camera.update(controller.last_base_pose, FRAME_DT)
        else:  # bundled: ランナーが位置を公開していないので読み出す (少し重い)
            positions, orientations = go2.articulation.get_world_poses()
            chase_camera.update((positions.numpy().reshape(-1), orientations.numpy().reshape(-1)), FRAME_DT)
    if terrain_rebuild_requested:
        terrain_rebuild_requested = False
        if controller is not None and controller.raycaster is not None:
            controller.raycaster.rebuild()  # 物理コールバックの外で作り直す
            print(f"go2_rc: terrain scan rebuilt ({controller.raycaster.num_triangles} triangles)", flush=True)
    now = time.perf_counter()
    if args.print_force and now >= next_print:
        realtime = (physics_steps - steps_at_print) * PHYSICS_DT / (now - next_print + 0.5)
        print(
            f"/gait/force {' '.join(map(str, last_sent))}   cmd {np.round(command(), 2).tolist()}"
            f"   realtime x{realtime:.2f}",
            flush=True,
        )
        next_print, steps_at_print = now + 0.5, physics_steps
    next_frame += FRAME_DT
    if next_frame > now:
        time.sleep(next_frame - now)
    else:
        next_frame = now  # 描画が追いつかないときは遅れを溜めない

timeline.stop()
input_interface.unsubscribe_to_keyboard_events(keyboard, keyboard_sub)
SimulationManager.deregister_callback(physics_callback_id)
go2.close()
osc_socket.close()
if record_file is not None:
    record_file.close()
if listen_socket is not None:
    listen_socket.close()
simulation_app.close()
