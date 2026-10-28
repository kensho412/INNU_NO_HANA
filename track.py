"""足場を横切って周回する、陸上のトラックのような形の道と、その上を自動で歩かせる指令。

道 (上から見た形):
    足場の手前寄り (中心から Go2 の初期の向きと逆へ TRACK_RADIUS) を右から左へ横切り、
    足場の左の外で半円を描いて回り、奥寄り (中心から TRACK_RADIUS 先) を左から右へ横切り、
    右の外で半円を描いて回って、また手前寄りを右から左へ。上から見て時計回り。
    直線は足場の端から TRACK_MARGIN だけ外まで伸ばし、回るのは足場の外にする。

歩かせ方: 道の上で今いる所から TRACK_LOOK_AHEAD 先の点を目指して、前進しながら旋回する
(pure pursuit)。旋回が YAW の上限を超えるカーブでは前進を遅くする。
道から TRACK_JOIN_DISTANCE より離れているとき (会場の外から入っていくときなど) は、
道のいちばん近い点へまっすぐ向かう。このときは向きのずれに比例して旋回する (pure pursuit だと
目指す点が遠いほど旋回が弱くなり、遅いときに Go2 の横へずれる癖に負けて道をそれる)。

床の線: draw() で、障害物がないときに歩く道を床に線で描く (見た目だけ。衝突も高さ計測もしない)。
足場の中は柱の下になるので見えない。

スタックしたとき:
    STUCK_TIME の間に、指令の速さで進むはずの距離の STUCK_RATIO も進んでいなければスタックとみなし、
    BACK_TIME だけ後ろへ下がってからやり直す。STUCK_RESET_COUNT 回続けてスタックしたとき
    (間に STUCK_CLEAR_TIME 以上ふつうに歩けていればそれまでの回数は数えない) と、
    胴体が FALLEN_UP_Z より傾いた (転んだ) 状態が FALLEN_TIME 続いたときは、初期位置に戻す。
"""

from __future__ import annotations

from collections import deque

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdShade

TRACK_RADIUS = 0.6  # 半円の半径 [m]。2 本の直線は足場の中心から ±この距離
TRACK_MARGIN = 0.8  # 直線を足場の端から外へ伸ばす長さ [m]
TRACK_LOOK_AHEAD = 0.5  # 道の上で目指す点までの距離 [m]
TRACK_JOIN_DISTANCE = 1.0  # [m]
JOIN_TURN_GAIN = 2.0  # 道へ向かうときの旋回の強さ [rad/s / rad]
TRACK_STEP = 0.02  # 道を点の列にするときの間隔 [m]
STUCK_TIME = 4.0  # [s]
STUCK_RATIO = 0.25
BACK_TIME = 1.5  # [s]
BACK_SPEED = 0.4  # 後ろへ下がる速さ [m/s]
STUCK_RESET_COUNT = 3
STUCK_CLEAR_TIME = 20.0  # [s]
FALLEN_UP_Z = 0.5  # 胴体の上向きの z 成分 (1 = 水平、0 = 横倒し)
FALLEN_TIME = 1.0  # [s]
LINE_PATH = "/World/TrackLine"
LINE_WIDTH = 0.03  # 床の線の幅 [m]
LINE_LIFT = 0.002  # 床からの浮かせ [m] (床と重なってちらつかないように)
LINE_COLOR = (0.2, 0.8, 1.0)
LINE_INTENSITY = 2.0  # 暗い建物の中でも見えるように、少し光らせる


class Track:
    def __init__(self, center: np.ndarray, forward: np.ndarray, left: np.ndarray, grid_size: float) -> None:
        half = grid_size / 2 + TRACK_MARGIN  # 直線の半分の長さ
        r = TRACK_RADIUS
        # 足場の座標 (u = 前、s = 左) で道を作る
        n_line = int(2 * half / TRACK_STEP)
        n_arc = int(np.pi * r / TRACK_STEP)
        line = np.linspace(-half, half, n_line, endpoint=False)
        arc = np.linspace(0.0, np.pi, n_arc, endpoint=False)
        near = np.column_stack([np.full(n_line, -r), line])  # 手前寄り、右 → 左
        left_turn = np.column_stack([-r * np.cos(arc), half + r * np.sin(arc)])  # 左の外を回る
        far = np.column_stack([np.full(n_line, r), -line])  # 奥寄り、左 → 右
        right_turn = np.column_stack([r * np.cos(arc), -half - r * np.sin(arc)])  # 右の外を回る
        us = np.vstack([near, left_turn, far, right_turn])
        self.points = np.asarray(center)[None, :] + us[:, :1] * forward[None, :] + us[:, 1:] * left[None, :]
        self.restart()

    def draw(self, stage: object, floor_z: float) -> None:
        """道を床 (高さ floor_z) に線で描く。"""
        p = self.points
        tangent = np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0)
        tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
        side = np.column_stack([-tangent[:, 1], tangent[:, 0]]) * (LINE_WIDTH / 2)
        n = len(p)
        z = floor_z + LINE_LIFT
        vertices = [Gf.Vec3f(float(x), float(y), z) for x, y in np.vstack([p + side, p - side])]
        faces = []
        for i in range(n):
            j = (i + 1) % n
            faces += [i, j, n + j, n + i]
        mesh = UsdGeom.Mesh.Define(stage, LINE_PATH)
        mesh.CreatePointsAttr(vertices)
        mesh.CreateFaceVertexCountsAttr([4] * n)
        mesh.CreateFaceVertexIndicesAttr(faces)
        mesh.CreateDoubleSidedAttr(True)
        material = UsdShade.Material.Define(stage, f"{LINE_PATH}/Look")
        shader = UsdShade.Shader.Define(stage, f"{LINE_PATH}/Look/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*LINE_COLOR))
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*(c * LINE_INTENSITY for c in LINE_COLOR)))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

    def restart(self) -> None:
        self._index: int | None = None  # 前回いた点 (道を戻らないように、ここから先だけを探す)
        self._history: deque[tuple[float, np.ndarray]] = deque()  # (時刻, 位置 xy)。スタックの判定用
        self._back_until = -1.0  # この時刻までは後ろへ下がる
        self._stuck_count = 0
        self._last_stuck_time = -np.inf
        self._fallen_since: float | None = None

    def supervise(self, xy: np.ndarray, up_z: float, t: float, vx: float) -> str | None:
        """スタックや転倒を調べる。"back" (下がり始める) / "reset" (初期位置に戻す) / None を返す。"""
        if up_z < FALLEN_UP_Z:
            if self._fallen_since is None:
                self._fallen_since = t
            if t - self._fallen_since >= FALLEN_TIME:
                return "reset"
            return None
        self._fallen_since = None
        if t < self._back_until:
            return None
        self._history.append((t, np.array(xy, dtype=np.float64)))
        while self._history and t - self._history[0][0] > STUCK_TIME:
            self._history.popleft()
        if t - self._history[0][0] < STUCK_TIME * 0.98:  # 下がり終えてから STUCK_TIME たつまでは判定しない
            return None
        if np.linalg.norm(xy - self._history[0][1]) >= STUCK_RATIO * vx * STUCK_TIME:
            return None
        if t - self._last_stuck_time > STUCK_TIME + BACK_TIME + STUCK_CLEAR_TIME:
            self._stuck_count = 0
        self._stuck_count += 1
        self._last_stuck_time = t
        if self._stuck_count >= STUCK_RESET_COUNT:
            return "reset"
        self._back_until = t + BACK_TIME
        self._history.clear()
        self._index = None  # 下がったあとは、いちばん近い点を探し直す
        return "back"

    def command(self, xy: np.ndarray, heading: float, vx: float, yaw_max: float, t: float = 0.0) -> np.ndarray:
        """胴体の位置 xy と向き heading [rad] から、[vx, vy, yaw_rate] を返す。vx は直線での前進の速さ。"""
        if t < self._back_until:
            return np.array([-BACK_SPEED, 0.0, 0.0], dtype=np.float32)
        n = len(self.points)
        if self._index is None:
            candidates = np.arange(n)
        else:  # 今いる所の少し後ろから先を探す
            candidates = (self._index + np.arange(-10, int(2 * TRACK_LOOK_AHEAD / TRACK_STEP))) % n
        distances = np.linalg.norm(self.points[candidates] - xy, axis=1)
        self._index = int(candidates[np.argmin(distances)])
        joining = distances.min() > TRACK_JOIN_DISTANCE
        if joining:
            target = self.points[self._index]
            self._index = None  # 道に戻るまでは毎回いちばん近い点を探し直す
        else:
            target = self.points[(self._index + int(TRACK_LOOK_AHEAD / TRACK_STEP)) % n]
        d = target - xy
        alpha = np.arctan2(d[1], d[0]) - heading  # 目指す点の、胴体から見た向き
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
        if abs(alpha) > np.pi / 2:  # 後ろにあるときはその場で向きを変える
            return np.array([0.0, 0.0, np.sign(alpha) * yaw_max], dtype=np.float32)
        if joining:
            yaw_rate = JOIN_TURN_GAIN * alpha
        else:
            yaw_rate = 2 * vx * np.sin(alpha) / TRACK_LOOK_AHEAD
        if abs(yaw_rate) > yaw_max:  # 曲がりきれないときは前進を遅くする
            vx *= yaw_max / abs(yaw_rate)
            yaw_rate = np.sign(yaw_rate) * yaw_max
        return np.array([vx, 0.0, yaw_rate], dtype=np.float32)
