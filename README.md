# INNU NO HANA — Go2 サウンドアート用シミュレータ

Unitree Go2 を使ったサウンドアートのための、Isaac Sim 上の仮想 Go2 です。
実機に触れられるのが週 1 日だけなので、平日はこのシミュレータで Go2 をラジコン操作し、
足の接地を Max/MSP に送って音の側を作り込めるようにしています。

- Go2 をキーボードでラジコン操作する (前後・左右・旋回の速度指令)
- 4 本の足先の接触力を、実機と同じ形式の OSC `/gait/force` で Max (`innu.maxpat`) に送る
- 階段・ブロック・凹凸や、象の鼻テラスの会場モデルの上を歩かせる

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
| ↑ / ↓ (W / S) | 前進 / 後退 (1.0 m/s) |
| ← / → (A / D) | 左 / 右へ横歩き (0.6 m/s) |
| N / M (Q / E) | 左旋回 / 右旋回 (1.0 rad/s) |
| R | 初期位置に戻す (転んだときも) |
| T | 障害物を動かした・足したあとに、足元の高さ計測用の地形を作り直す |
| C | Go2 の後ろを追従するカメラ / 自由に動かせる通常のカメラ の切り替え |

### 起動オプション (`go2_rc.py`)

```
python go2_rc.py [--host 127.0.0.1] [--port 9871] [--policy rough|flat|bundled]
                 [--course demo|none] [--usd 会場.usd] [--spawn-yaw 度]
                 [--print-force] [--no-chase-camera]
```

| オプション | 意味 |
|---|---|
| `--host` / `--port` | OSC の送信先 (既定 127.0.0.1:9871) |
| `--policy` | 歩行ポリシー。`rough` (既定) は Isaac Lab の不整地用で、足元の高さを測って階段や段差を越えられる。`flat` と `bundled` は平地用で、低い姿勢のすり足になる |
| `--course` | `demo` で階段・凹凸・ブロックを置く。`none` で何も置かない |
| `--usd` | 会場モデルの USD を読み込む |
| `--spawn-yaw` | Go2 の初期の向き [度] (+x から反時計回り) |
| `--print-force` | 送っている値と、実時間に対する速度を 0.5 秒ごとに表示する |
| `--no-chase-camera` | 起動時に追従カメラを使わない |

## OSC `/gait/force`

- 形式: `/gait/force i i i i`、並び順は **FR FL RR RL** (実機 `LowState.foot_force` と同じ)
- 送信頻度: 物理ステップごと (200 Hz、描画 1 フレームに 4 回まとめて届く)
- 値: 足の接触力 [N] × 足ごとの倍率 (整数)

倍率は「静止時に体重 (147 N) が 4 本に均等にかかる」と仮定し、
`実機の静止立ちの値 ÷ 36.8 N` で決めています (`go2_rc.py` の `REAL_STANDING`)。
実機の値 `23 17 36 26` は `innu.maxpat` に残っていた値で、実機で測り直して差し替える予定です。

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
| `tools/build_zounohana_terrace.py` | 象の鼻テラスを Blender で作るスクリプト |
| `venues/` | 会場モデル (`.blend` は Blender で編集する元データ) |
| `policies/` | 歩行ポリシーの重み |
| `innu.maxpat` | 受け取り側の Max パッチ (実機と共通) |
| `CLAUDE.md` | 開発メモ |

## 既知の制限

- シミュレータの歩き方は、実機の Unitree 内蔵の歩行制御 (sport mode) とは違います。リズムの最終調整は実機で行います
- 不整地用ポリシーは、止まっているときに体重のかけ方が偏ることがあります (片足が浮くなど)
- 追従カメラを使っていると描画が重くなり、実時間の約 0.9 倍の速さになります。リズムを確かめるときは C キーで通常のカメラに切り替えてください
- 足元の高さ計測は、衝突判定の付いた Mesh と Cube しか見ません (球や円柱は見えない)

## 出典

- 歩行ポリシーの重み (`policies/`): NVIDIA の配布サーバーにある Isaac Lab の学習済みチェックポイント
  (`Isaac-Velocity-Rough-Unitree-Go2-v0`, `Isaac-Velocity-Flat-Unitree-Go2-v0`) から actor の部分だけを取り出したもの
- Go2 のモデル: Isaac Sim / Isaac Lab が配布しているもの (起動時に読み込み、このリポジトリには含まない)
- 象の鼻テラスの平面形: © OpenStreetMap contributors (ODbL 1.0) — https://www.openstreetmap.org/way/84838285
- 照合に使った航空写真・建物データ: 国土地理院 地理院タイル
- 外観の参考: 象の鼻テラス公式サイト、横浜市の公開資料の写真 (このリポジトリには含まない)
