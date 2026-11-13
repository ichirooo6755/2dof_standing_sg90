# 2dof_standing / SG90・XIAO nRF54 移植版

元: https://github.com/ggoonnzzaallo/2dof_standing

固定コミット: `3a9cb54fc3088ccf10322d8de4ceb0128a317056`（controller 0.9 / C3）。
元データ一式は `../../upstream/2dof_standing/` に保存。

## 使うコード

元の `firmware/policy_control/policy_control.ino` から移植。
`RecoveryStrategy.h`、`GravityFilter.h` は元と同一。
現在の `policy_upright.h` は指定3MF・SG90・1000mAh LiPo・PowerBoost 500条件で追加学習したモデル。
262,144ステップの学習から、開発評価で選択した65,536ステップ時点のactorを採用した。
10入力の学習制御、50Hz・0.10radの目標更新、転倒継続時の復帰パターン変更、
足が平らなときの中央戻しを使用する。以前の4入力方策や手前用固定軌道は使用しない。
15°以上の傾きを検出し、ジャイロ・加速度でほぼ静止と判定した状態が連続0.3秒になったら学習制御を開始する。
待ち中は目標角を保持し、再び動いたら計時をやり直す。8°未満へ戻した場合は待ちを解除する。
復帰中は既存actor・再試行処理を続け、8°以内で中央付近・静止が0.3秒続くと転倒監視へ戻る。
ブロックする待機は使用せず、待機中もIMU監視、USR停止、シリアル停止が有効。
新基板での信号・計算・実機確認の範囲は [LM20A確認記録](../../docs/lm20a_recovery_20261113.md) を参照。

## この機体への変更

| 項目 | 設定 |
| --- | --- |
| ボード | XIAO nRF54LM20A Sense または nRF54L15 Sense、lolren core 1.0.23 |
| 下段・上段信号 | D0・D1（配線変更不要） |
| SG90 | 下段中央1500µs、上段中央1464µs、正角度でパルス減少。指令範囲500〜2500µs |
| PWM | 前景ループで2軸のパルスを順に生成し、50 Hzに合わせる。推論やログ処理の影響を受けるため、周期を監視する |
| IMU | 基板内LSM6DS3TR-C、Wire1、208Hz、±8g、±2000°/s |
| センサー軸 | 同じUSB方向・表裏で取り付けた場合、L15は`+X,+Y,+Z`、LM20Aは`−X,−Y,+Z`。新基板の左右方向は実機で要確認 |
| 自動開始 | USB・電池とも、静止したセンサーが準備できた後、中央を1秒保持して開始。起動時に準備に15秒を超えてもPWMを停止したまま待ち、初期化失敗時は2秒ごとに再試行する。運転中の異常停止は自動解除しない |
| 無動作停止 | ユーザーの常時運転指定に合わせて1分の停止を無効化 |
| 起き上がり開始 | 15°以上の傾き後、ほぼ静止が連続300ms。角速度0.2rad/s未満、加速度0.9〜1.1g、加速度変動0.04g未満で判定 |
| 停止 | USRボタン、`off`、`!`。USRをもう一度押すと再開 |
| 異常停止 | IMUの古い・無効なデータ、制御遅延、数値異常でPWM停止。理由を保持 |

`status` / `state` / `sensors` / `config` は動作を停止せず読み取る。
`STATE,4,...` が自動運転、`STATE,0,...` が停止。`D,` 行は元と同じ固定小数点ログ。
指令角は実角度の読み戻しではない。
`gate` は待ち状態を読み取る。`phase=0` が転倒監視、`1` が静止待ち、`2` が復帰中。
`D,` 行の末尾に `fall_gate,still_ms` を追加。`still_ms` は静止待ち中の連続静止時間。

### 電池電圧とモデルの確認範囲

元のnRF52840用ADC・充電GPIO操作はnRF54L15へ流用しない。
この移植版では電池電圧による開始・停止の判定は未実装。
`battery_mv=0` は取得できていないことを示し、0Vの測定値ではない。
nRF54L15の分圧回路・ADCを使った電圧校正が必要。

現在の重みは元のPTK・C3用actorを初期値に、指定3MFとSG90条件で追加学習したもの。
学習とは別の48条件では48/48成功、元モデルも48/48成功。平均判定時間2.241秒→2.208秒。
一部重量、電装位置、SG90外形・動特性、ホーン位置は推定で、実機の成功を保証する結果ではない。
詳細は `../../sim/sg90_1000mah/README.md` と同ディレクトリの `configuration.json` を参照。
書き込み・起動状態と、実物が起き上がれることは別に確認する。
3Dモデルの取付部も元のPTK・nRF52840用。SG90とnRF54L15の取り付け寸法は未確認で、合わない部位は調整する。
新しい筐体へ基板を移したら `MountCalibration.h` の軸対応を再確認する。

## ビルドと書き込み

リポジトリのルートから実行する。LM20Aでは128 MHzを選ぶ。観測した倒れ姿勢の計算時間が64 MHzの最大約13.4 msから、128 MHzでは最大約6.8 msになったため、20 ms周期に対する余裕が増える。

```sh
./tools/build_standing_lm20a.sh
./tools/build_standing_lm20a.sh --upload /dev/cu.usbmodemYOUR_PORT
```

書き込みを明示した場合は、ビルド後に指定ポートへ書き込む。起動後にセンサーが正常なら自動監視が始まる。USBのポート名は接続機器に合わせて変更する。

以前の XIAO nRF54L15 Sense（`/dev/cu.usbmodem6E0C2EEC3`）向け:

```sh
arduino-cli compile --fqbn nrf54l15clean:nrf54l15clean:xiao_nrf54l15:clean_ble=off,clean_vpr=off --build-property build.power_flags= --build-path build/standing_xiao_fall_gate firmware/standing_xiao
arduino-cli upload --fqbn nrf54l15clean:nrf54l15clean:xiao_nrf54l15:clean_ble=off,clean_vpr=off --port /dev/cu.usbmodem6E0C2EEC3 --input-dir build/standing_xiao_fall_gate firmware/standing_xiao
```

ログ例は `python3 tools/standing_monitor.py --port /dev/cu.usbmodemYOUR_PORT --seconds 20 --out output/lm20_status`。

## 3Dモデル

- 現在の学習に使用した指定3MF: `../../sim/sg90_1000mah/source/ボディ1.3mf`
- 全体組立: `../../upstream/2dof_standing/cad/assembly.step`
- 台座: `../../upstream/2dof_standing/cad/electronics_pod/base_modified.stl`
- 電装ケース: `../../upstream/2dof_standing/cad/electronics_pod/electronics_cup.stl`
- 元の印刷パッケージ: `../../upstream/2dof_standing/cad/electronics_pod/rev_c3_print_fit.zip`

## 過去の書き込み: 静止0.5秒後の復帰

2026-11-11、`sg90-nrf54-4` を書き込み。使用量66,868 bytes、静的RAM7,584 bytes。
学習済みactorはport 3と同一。開始条件のみ `FallRecoveryGate.h` で追加し、`RecoveryStrategy.h` の内容は維持。
新たな動きが入った場合は500msを最初から計時する。初期姿勢が転倒している場合も、自動運転の開始後に静止を計時する。
元リポジトリの5秒という値は、初回の開始待ちではなく、復帰が進まないときに別の再試行パターンへ切り替える条件。
記録は `deployment_fall_gate_20261111.json`、起動ログは `../../output/fall_gate_deployed_20261111.txt`。
学習時の48条件の成績は開始待ちを追加する前のactor評価であり、今回の待ち時間を含んだ実機成績ではない。

## SG90・1000mAh追加学習版（port 3の履歴）

2026-11-11、`sg90-nrf54-3` を書き込み済み。使用量65,492 bytes、静的RAM7,544 bytes。
採用ヘッダーSHA256は `c06e47976d8fbad37e788be4233775b7bd453ac499cb76b78d6ec1e4d9bcaabb`。
接続先はXIAOシリアル `6E0C2EEC`。`config` の `policy=sg90_1000mah actor=c06e4797 step=65536` を実機から取得した。
10秒間・503行の受信でmode 4（自動運転）、フレーム976→1478、I2Cエラー0→0、停止メッセージなし。
最終目標0°/0°、PWM1500/1500µs。最大推論12,503µs、遅れ1,720µs、サンプル年齢16,640µs。
これは書き込みと受動的な起動ログの確認で、実機の転倒・起き上がり試行はしていない。
記録は `deployment_sg90_1000mah_20261111.json`、ログは `../../output/sg90_1000mah_deployed_20261111.txt`。
変更前のソースは `deployments/before_sg90_1000mah_20261111/` に保存。

## 初回移植版の書き込み結果（履歴）

`sg90-nrf54-2` を書き込み済み。使用量65,436 bytes、静的RAM7,544 bytes。
初版は中央保持へ入った直後にIMU鮮度の停止条件へ入ったため、
PWMをコアの各ピン専用TIMER/GPIOTE出力へ変更した。
修正版は開始指令なしでSTANDALONE（mode 4）へ入り、408行の記録で
フレーム531→938、両目標0°・1500µsを確認した。記録中の停止メッセージなし。
最大推論12,523µs、最大遅れ1,727µs、最大サンプル年齢16,545µs。
I2Cエラーは0→1回であり、通信エラー0とはしていない。
新しいプログラムでの無支持の起き上がりはまだ試していない。
詳細は `deployment_20261111.json`、ログは `../../output/2dof_port2_deployed.txt`。
