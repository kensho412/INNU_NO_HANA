# INNU NO HANA — Go2 サウンドアート用シミュレータ

Unitree Go2 を使ったサウンドアートのための、Isaac Sim 上の仮想 Go2 です。
実機に触れられるのが週 1 日だけなので、平日はこのシミュレータで Go2 をラジコン操作し、
足の接地を Max/MSP に送って音の側を作り込めるようにしています。

- Go2 をキーボードでラジコン操作する (前後・左右・旋回の速度指令)
- 4 本の足先の接触力を、実機と同じ形式の OSC `/gait/force` で Max (`innu.maxpat`) に送る
- 階段・ブロック・凹凸や、象の鼻テラスの会場モデルの上を歩かせる
- キーを押すと、物理演算つきの小さなブロック (10×10×20 cm / 10 cm 角) が Go2 の前に落ちてくる
- 8×8 のマス目の足場を置き、マスごとの高さを Max から変え、どのマスが踏まれているかを Max に返す

学習 (機械学習) はしません。NVIDIA が配布している学習済みの歩行ポリシーを動かすだけです。

## データの流れ

```
キーボード ─▶ Isaac Sim (Go2 + 歩行ポリシー) ─▶ 足先の接触力 [N]
                                                   │ 足ごとの倍率で実機の生の値に変換
                                                   ▼
                              OSC /gait/force i i i i (FR FL RR RL) ─▶ Max (innu.maxpat, udp 9871)
                                                                          │ 接地判定
                                                                          ▼
                                                                Ableton (/note/*, /vel/*)
```

実機では Go2 本体から Wi-Fi で同じ `/gait/force` が Max に届きます。`innu.maxpat` は変更せずに、
実機とシミュレータのどちらからでも受け取れます。

## 必要な環境

- Windows 11、NVIDIA GPU (開発機は RTX 4090 24 GB、ドライバ 610)
- Python 3.12
- Isaac Sim 6.2.0 (pip 版) と PyTorch 2.11 (CUDA 12.8)
- 会場モデルを作り直す場合のみ: Blender 4.4

## セットアップ

```powershell
git clone https://github.com/kensho412/INNU_NO_HANA.git
cd INNU_NO_HANA
py -3.12 -m venv env_isaacsim
.\env_isaacsim\Scripts\activate
python -m pip install --upgrade pip
# Isaac Sim 6.2 (詳しくは Isaac Sim 公式ドキュメントの pip インストールの手順を参照)
pip install "isaacsim[all,extscache]==6.2.0" --extra-index-url https://pypi.nvidia.com
# PyTorch (CUDA 12.8 版)
pip install torch --index-url https://download.pytorch.org/whl/cu128
```

初回起動時は Isaac Sim の EULA への同意が必要です (`run_go2.bat` は `OMNI_KIT_ACCEPT_EULA=YES` を設定します)。
Go2 のモデルや床などの素材は、起動時に NVIDIA の配布サーバーから読み込まれます (インターネット接続が必要)。

## 起動

`run_go2.bat` をダブルクリックします。20〜30 秒ほどで Isaac Sim の画面が開き、
象の鼻テラスの前の広場に Go2 が現れます。ビューポートを 1 回クリックしてから操作してください。

- Max のある Mac に送るときは、`run_go2.bat` の `set HOST=127.0.0.1` を Mac の IP アドレスに書き換えます
- USD ファイルを `run_go2.bat` にドラッグ&ドロップすると、その会場で起動します
- デモ用の障害物コース (階段・凹凸・ブロック) にするときは、`run_go2.bat` の `set VENUE=` を空にします

### 操作

| キー | 動き |
|---|---|
| ↑ / ↓ (W / S) | 前進 / 後退 (速さ 100 % で 1.0 m/s) |
| ← / → (A / D) | 左 / 右へ横歩き (速さ 100 % で 0.6 m/s) |
| N / M (Q / E) | 左旋回 / 右旋回 (速さ 100 % で 1.0 rad/s) |
| 1〜5 | 速さを 20 / 40 / 60 / 80 / 100 % にする (前後・横・旋回のすべて) |
| - / = (テンキーの - / +) | 速さを 10 % ずつ下げる / 上げる (10〜100 %) |
| R | 初期位置に戻す (転んだときも) |
| T | 障害物を動かした・足したあとに、足元の高さ計測用の地形を作り直す |
| C | Go2 の後ろを追従するカメラ / 自由に動かせる通常のカメラ の切り替え |
| F | 足場を手前から少し (約 20°) 見下ろす固定カメラ / 元のカメラ の切り替え (`--grid` のとき。象の鼻テラスでは入口から見る) |
| L | 自動歩行 / 手動 の切り替え (`--grid` のとき)。足場の手前寄りを右から左へ横切り、左の外で回って奥寄りを左から右へ横切り、右の外で回る、陸上のトラックの形を周回する (1 周約 11 m、速さ 100 % で約 13 秒)。速さは 1〜5 や - / = キーで変えられる。形は `track.py` の `TRACK_RADIUS` (半円の半径、既定 0.6 m) と `TRACK_MARGIN` (直線を足場の外へ伸ばす長さ、既定 0.8 m) で変えられる。この道は床に水色の線で描いてある (見た目だけ。足場の中は柱の下になって見えない)。進めなくなったら (4 秒で指令の 1/4 も進まない) 1.5 秒下がってやり直し、3 回続けて進めないときと転んだときは初期位置に戻してやり直す |
| B | 10×10×20 cm のブロックを Go2 の前に寝かせて置く (進む向きに対して横向き) |
| G | 10×10×20 cm のブロックを Go2 の前に立てて置く |
| V | 10 cm 角のブロックを Go2 の前に置く |
| P | 10 cm 角のブロックを 4 段のピラミッド (下から 4×4、3×3、2×2、1 の 30 個、幅 40 cm・高さ 40 cm) に積んで Go2 の前に置く |
| X | 置いたブロックをすべて片付ける |

### ブロック

B / G / V で置くブロックは物理演算つきです。Go2 の 0.7 m 前の、地面のすぐ上から落ちてきて、
蹴ると転がり、同じ場所で続けて押すと上に積み重なります。不整地用ポリシーの足元の高さ計測にも映るので、
Go2 は動いたあとの位置を見て踏み越えます。

- 重さは木材くらい (10×10×20 cm で 1.2 kg、10 cm 角で 0.6 kg)。`blocks.py` の `BLOCK_DENSITY` で変えられます
- 10×10×20 cm は 20 個、10 cm 角は 40 個まで。それより多く置くと、いちばん古いものから順に持ってきます (`BLOCKS_PER_KIND`)
- 大きさや色は `blocks.py` の `BLOCK_KINDS`、落とす位置は `go2_rc.py` の `BLOCK_AHEAD` / `BLOCK_DROP_HEIGHT`
- ピラミッドの段数と位置は `go2_rc.py` の `PYRAMID_LEVELS` / `PYRAMID_AHEAD`。P を続けて押すと前のピラミッドのブロックを
  使い回します。5 段 (55 個) 以上にするときは `blocks.py` の `BLOCKS_PER_KIND["cube"]` も増やしてください
- R (Go2 を初期位置に戻す) ではブロックは残ります

### 8×8 の足場 (`--grid`)

Go2 の初期位置の 1 m 先に、25 cm 角のマスが 8×8 (2 m 四方) 並んだ足場を置きます (`run_go2.bat` では最初から置く。
置かないときは `set GRID=` にする)。象の鼻テラスでは、正面入口から 1.5 m 入った広間の床の上に置きます
(`run_go2.bat` の `--grid-ahead 7.6`。Go2 はスロープを上って入口から入る)。マスはそれぞれ物理演算つきの柱で、踏むと少し揺れますが、外周の囲いと
隣の柱に挟まれているので、基本的には蹴とばされません。

| 向き | アドレス | 中身 |
|---|---|---|
| Max → シミュレータ | `/grid/height` | 64 個の数 (整数でも小数でも)。マスごとの高さ [cm]。受け取った瞬間に 64 マスまとめて変わる |
| シミュレータ → Max | `/grid/step` | 64 個の整数。マスごとに、いま足が着いていれば 1、いなければ 0。50 Hz |
| シミュレータ → Max | `/grid/foot` | 4 個の整数 (FR FL RR RL)。足ごとに着いているマスの番号 (0〜63)。浮いている・足場の外なら -1。50 Hz |

- `/grid/height` はこの PC の UDP **9872** に送ります (Max なら `[prepend /grid/height]` → `[udpsend <この PC の IP> 9872]`)。
  ポートは `--listen-port` で変えられます。Windows のファイアウォールの許可を求められたら許可してください
- `/grid/step` と `/grid/foot` は `/gait/force` と同じ送信先 (`run_go2.bat` の `HOST` とポート) に送ります。`[route /gait/force /grid/step /grid/foot]` で分けられます
- 64 個の並びは `row × 8 + col`。row 0 が Go2 の初期位置に近い列、col 0 が Go2 から見て左端です
- 高さは 1〜30 cm に丸めます (0 は 1 cm)。試したところ、2 cm ずつ高くなる段は 14 cm まで上れ、25 cm の列は越えられませんでした
- 64 個ちょうどでないメッセージは無視します
- 踏まれているマスは光ります (`/grid/step` が 1 のマス。色と強さは `grid.py` の `GLOW_COLOR` / `GLOW_INTENSITY`)
- 高さは足場を置いた床から測ります (建物の中なら床の上面。起動時に自動で測る)
- 起動時はすべて 5 cm。Go2 が乗っているときに高さを変えると、Go2 は下から突き上げられます
- マスの大きさ・囲いの高さ・柱の重さなどは `grid.py` の冒頭、置く位置は `--grid-ahead 距離` か `--grid-center X Y`

### 起動オプション (`go2_rc.py`)

```
python go2_rc.py [--host 127.0.0.1] [--port 9871] [--policy rough|flat|bundled]
                 [--course demo|none] [--usd 会場.usd] [--spawn-yaw 度]
                 [--print-force] [--physics-device cpu|cuda] [--no-chase-camera]
                 [--grid] [--grid-ahead 1.0] [--grid-center X Y] [--listen-port 9872]
                 [--speed 1.0] [--fixed-camera] [--auto-walk] [--record 記録.csv]
```

| オプション | 意味 |
|---|---|
| `--host` / `--port` | OSC の送信先 (既定 127.0.0.1:9871) |
| `--policy` | 歩行ポリシー。`rough` (既定) は Isaac Lab の不整地用で、足元の高さを測って階段や段差を越えられる。`flat` と `bundled` は平地用で、低い姿勢のすり足になる |
| `--course` | `demo` で階段・凹凸・ブロックを置く。`none` で何も置かない |
| `--usd` | 会場モデルの USD を読み込む |
| `--spawn-yaw` | Go2 の初期の向き [度] (+x から反時計回り) |
| `--print-force` | 送っている値と、実時間に対する速度を 0.5 秒ごとに表示する |
| `--physics-device` | 物理計算を CPU (既定) と GPU (`cuda`) のどちらでするか。ロボット 1 台の小さなシーンでは CPU のほうが速い (GPU は動くブロックがあると実時間の約 0.6 倍に落ちる) |
| `--no-chase-camera` | 起動時に追従カメラを使わない |
| `--grid` | 8×8 の足場を置く |
| `--grid-ahead` | Go2 の初期位置から足場の手前の端までの距離 [m] (既定 1.0) |
| `--grid-center` | 足場の中心 [m]。省略すると Go2 の初期位置の `--grid-ahead` 先から始まる |
| `--listen-port` | `/grid/height` を受け取るポート (既定 9872) |
| `--fixed-camera` | 起動時から足場を見る固定カメラにする (F キーで切り替え) |
| `--auto-walk` | 起動時から足場を周回する自動歩行にする (L キーで切り替え) |
| `--record` | 足の荷重 [N]・送った /gait/force の値・/imu/rpy・胴体の位置を、時刻つきで 50 Hz の CSV に書き出す |
| `--speed` | 起動時の速さ (0.1〜1.0、既定 1.0)。歩いている途中でも 1〜5 や - / = キーで変えられる |

## OSC `/gait/force`

- 形式: `/gait/force i i i i`、並び順は **FR FL RR RL** (実機 `LowState.foot_force` と同じ)
- 送信頻度: 物理ステップごと (200 Hz、描画 1 フレームに 4 回まとめて届く)
- 値: 足の接触力 [N] × 足ごとの倍率 (整数)

倍率は「静止時に体重 (147 N) が 4 本に均等にかかる」と仮定し、
`実機の静止立ちの値 ÷ 36.8 N` で決めています (`go2_rc.py` の `REAL_STANDING`)。
実機の値 `23 17 36 26` は `innu.maxpat` に残っていた値で、実機で測り直して差し替える予定です。

## OSC `/imu/*` (胴体の姿勢・ジャイロ・加速度)

`/gait/force` と同じ送信先に 50 Hz で送ります。値は小数 (float) で、名前と単位は実機の `LowState.imu_state` に合わせています。
Max では `[route /gait/force /imu/rpy /imu/gyro /imu/accel]` などで分けられます。

| アドレス | 中身 | 実機 |
|---|---|---|
| `/imu/rpy f f f` | 姿勢 roll pitch yaw [rad]。roll + = 右に傾く、pitch + = 頭が下がる、yaw + = 左を向く | `imu_state.rpy` |
| `/imu/gyro f f f` | 胴体の x (前)・y (左)・z (上) 軸まわりの回転の速さ [rad/s] | `imu_state.gyroscope` |
| `/imu/accel f f f` | 胴体の x・y・z 方向の加速度 [m/s²]。重力を含む (水平に止まっていると z ≒ +9.8) | `imu_state.accelerometer` |

- yaw は起動時の向きが基準ではなく、シミュレータの +x 方向が 0 です (象の鼻テラスでは起動時に約 -2.76 rad)
- 加速度は胴体の速度の差から求めているので、足が着くたびに大きく揺れます
- シミュレータの不整地用ポリシーは、止まっていると頭を少し上げた姿勢 (pitch ≒ -0.18 rad) で立ちます

## 会場

### 象の鼻テラス (`venues/zounohana_terrace.usdc`)

横浜・象の鼻パークの象の鼻テラス (2009、設計: 小泉雅生/小泉アトリエ) の建物を、Go2 に対して実寸で作ったモデルです。

- 平面形は OpenStreetMap の建物輪郭から作り、国土地理院の建物データ・航空写真と照合済み
- 高さ (天井・屋根の帯・床の段差) と扉の位置は写真からの推定です
- 家具、屋上につながる公園の芝生の丘、厨房などの間仕切りは作っていません
- 天井にダウンライトがあります
- 原点は北東の正面入口の 6 m 手前の広場で、入口は `--spawn-yaw -158` の向きにあります

寸法を変えるときは `tools/build_zounohana_terrace.py` 冒頭の値を書き換えて、Blender で作り直します。

```powershell
& "C:\Program Files\Blender Foundation\Blender 4.4\blender.exe" -b --factory-startup `
    -P tools\build_zounohana_terrace.py -- venues\zounohana_terrace
```

### 自分で作った会場を使う

Blender で作って USD (`.usd` / `.usdc`) に書き出し、`run_go2.bat` にドラッグ&ドロップします。

- 衝突判定が無いファイル (Blender の書き出しそのまま) には、すべてのメッシュに自動で衝突判定を付けます
- 単位 (`metersPerUnit`) と上方向 (`upAxis`) は自動で m 単位・Z 上向きに直します
- 書き出す前に Blender で「すべてのトランスフォームを適用」(`Ctrl+A`) してください
- 不整地用ポリシーが確実に越えられる段差は 10 cm 前後までです (学習時の上限は 23 cm 程度)

## ファイル構成

| ファイル | 内容 |
|---|---|
| `run_go2.bat` | ダブルクリックで起動 |
| `go2_rc.py` | シミュレータ本体 (シーン、キーボード操作、OSC 送信、追従カメラ) |
| `go2_policy.py` | Isaac Lab の Go2 歩行ポリシーを動かす部分 (観測の組み立て、足元の高さ計測、DC モータのモデル) |
| `terrain.py` | 障害物と会場モデルの配置 |
| `blocks.py` | キーで置く、物理演算つきのブロック |
| `grid.py` | 8×8 の足場 (高さの変更と、踏まれているマスの判定) |
| `tools/build_zounohana_terrace.py` | 象の鼻テラスを Blender で作るスクリプト |
| `venues/` | 会場モデル (`.blend` は Blender で編集する元データ) |
| `policies/` | 歩行ポリシーの重み |
| `innu.maxpat` | 受け取り側の Max パッチ (実機と共通) |
| `CLAUDE.md` | 開発メモ |

## 既知の制限

- シミュレータの歩き方は、実機の Unitree 内蔵の歩行制御 (sport mode) とは違います。リズムの最終調整は実機で行います
- 不整地用ポリシーは、止まっているときに体重のかけ方が偏ることがあります (片足が浮くなど)
- 物理計算が CPU (既定) のとき、歩いたあとに止まると 2 秒ほどしてよろけることがあります (胴体が一瞬 10 cm ほど沈んで立て直す)。
  GPU (`--physics-device cuda`) では起きませんが、ブロックを使うと実時間の約 0.6 倍に遅くなります
- 実時間に追いつかないとシミュレーションがゆっくり進みます。`--print-force` の表示の `realtime x` が 1.0 より小さいときは、
  C キーで通常のカメラに切り替えると軽くなります (開発機では追従カメラありで約 0.97 倍)
- 速さは学習時の指令の範囲 (1.0 まで) に収めています。20 % 以下だと歩みが不安定で、ほとんど進まない時間があります
- 足元の高さ計測は、衝突判定の付いた Mesh と Cube しか見ません (球や円柱は見えない)

## 出典

- 歩行ポリシーの重み (`policies/`): NVIDIA の配布サーバーにある Isaac Lab の学習済みチェックポイント
  (`Isaac-Velocity-Rough-Unitree-Go2-v0`, `Isaac-Velocity-Flat-Unitree-Go2-v0`) から actor の部分だけを取り出したもの
- Go2 のモデル: Isaac Sim / Isaac Lab が配布しているもの (起動時に読み込み、このリポジトリには含まない)
- 象の鼻テラスの平面形: © OpenStreetMap contributors (ODbL 1.0) — https://www.openstreetmap.org/way/84838285
- 照合に使った航空写真・建物データ: 国土地理院 地理院タイル
- 外観の参考: 象の鼻テラス公式サイト、横浜市の公開資料の写真 (このリポジトリには含まない)
