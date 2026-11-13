# SelfRisingRobot SG90 / XIAO nRF54

**現在の基板**は XIAO nRF54LM20A Sense。起き上がりファーム `sg90-lm20a-3` は128 MHzでビルドし、15°以上傾いて静止してから0.3秒後に復帰を開始する。下段・上段の中央は1500/1464 µs。新基板のIMU電源・端子と取付方向に対応している。[ビルド・書き込み](firmware/standing_xiao/README.md) · [実機と計算の確認記録](docs/lm20a_recovery_20261113.md)

サーボを動かさない倒れ姿勢の試験では、静止待ち中は指令を保持し、待機後は大きな目標角が計算された。一方、実際の起き上がり試験では開始付近でUSB通信が切れて少しだけ動いた。基板が再起動した理由と、電源・配線の影響は未確定。停止中の測定はXIAO 3.3 V、PowerBoost 5.2 Vだが、動作時の瞬間的な電圧は測定していない。

以前の XIAO nRF54L15 Sense への最終書き込みは `sg90-nrf54-9`。USB・電池のどちらでも起動後に転倒を監視し、15°以上の傾きで静止が連続0.3秒になってから起き上がる設定。復帰後は次の転倒を監視する。下段D0の中央1500µs・上段D1の中央1464µsを、起動時と復帰中の両方に反映。USBを手前に見て下段＋は手前、上段＋は右を実機で確認した。[書き込み・PWM確認記録](firmware/standing_xiao/deployment_software_pwm_20261111.json)

以前の基板では2026-11-12に「また動かない」との報告があり、シリアル無応答・SWD接続失敗を記録した。使用者の測定で、外部配線を外したXIAO単体でも3V3–GNDがほぼ0Vと判明。以前の上段だけの停止との関係は未確定。[浅い傾きと停止の調査記録](docs/shallow_recovery_freeze_20261111.md)。

## 起き上がり制御の2dof_standing 移植版

2026-11-11の指定により [ggoonnzzaallo/2dof_standing](https://github.com/ggoonnzzaallo/2dof_standing) のcontroller 0.9を移植して本体へ書き込んだ。自動運転に対応。
移植コードは [firmware/standing_xiao](firmware/standing_xiao/README.md)、3D原本は [upstream/2dof_standing/cad](upstream/2dof_standing/cad/README.md)。
指定3MF・SG90・1000mAh LiPo・PowerBoost 500条件で追加学習したactorを使う `sg90-nrf54-4` を中央出し前に使用。転倒検知後、静止が連続0.5秒になってから復帰を開始する。待ち中も停止操作・センサー監視を続ける。転倒からの起き上がりはこの版では未確認。
以下は以前の `xiao_getup` の説明・記録で、今回の移植版の動作仕様は上のREADMEを参照。

指定3MF・SG90・1000mAh電池・PowerBoost 500でのMuJoCo追加学習は [sim/sg90_1000mah](sim/sg90_1000mah/README.md)。入力形状、重量の根拠、モデル、学習ログとチェックポイントを保存しています。

USB側への移動と台座の一辺での保持を、同じ機体でMuJoCo学習しました。[評価結果と比較動画](sim/sg90_skills/RESULTS.md)。

[SelfRisingRobot](https://github.com/homemadegarbage/SelfRisingRobot) を、TowerPro SG90 2個と Seeed Studio XIAO nRF54L15 Sense で起き上がらせるためのSTL、MuJoCo学習、ファームウェアです。

XIAO nRF54L15 Sense の LSM6DS3TR-C で姿勢を見ます。電源投入後、正常なセンサーデータを受け取ると自動で転倒監視を開始します。ボタン操作やPCからの開始指令は不要です。起き上がり後も監視を続けます。ロールまたはピッチが 0.75 rad（約43°）を 300 ms 超えると転倒と判定します。直立（0.25 rad 以内）に加え、両サーボの目標角が中央付近（0.12 rad以内）を 700 ms 維持すると完了します。SG90には角度読み取り機能がないため、実際にアームが伸びたかは実物で確認します。

配線・中央位置・回転方向の確認手順は [実機確認](docs/実機確認.md) にあります。

## 印刷

| ファイル | 数量 | 内容 |
| --- | ---: | --- |
| `stl/foot_xiao_nrf54l15_sense.stl` | 1 | 足、下段サーボ固定、XIAOの位置決め枠 |
| `stl/arm1.stl` | 1 | 中間アーム。公開STL `arm1P.stl` と同一 |
| `stl/arm2.stl` | 1 | 上段アーム。公開STL `arm2P.stl` と同一 |
| `stl/horn_adapter_sg90.stl` | 2 | SG90付属ホーンと各アームを締結 |

単位は mm です。

Fusion で開くときは `step/` のファイルを使ってください。STL は Fusion ではメッシュボディになります。`step/foot_xiao_nrf54l15_sense.step`、`step/arm1.step`、`step/arm2.step` は平面と円柱の解析ソリッドで、座標は対応する STL と同じです。Fusion ではソリッドボディとして開けます。

| ファイル | 対応するSTL |
| --- | --- |
| `step/foot_xiao_nrf54l15_sense.step` | `stl/foot_xiao_nrf54l15_sense.stl` |
| `step/arm1.step` | `stl/arm1.stl` |
| `step/arm2.step` | `stl/arm2.stl` |
| `step/horn_adapter_sg90.step` | `stl/horn_adapter_sg90.stl` |

サーボ受けは高さ 24.0 mm、幅 12.5 mm で背面が開いています。SG90 の本体断面 22.7 × 12.2 mm が入り、取付穴間隔 27.8 mm と関節位置は公開モデルのままです。XIAO の枠は内寸 21.6 × 18.4 mm、高さ 1.5 mm で、USB-C を足の縁へ向けます。基板は薄い両面テープで固定します。ホーンアダプターは厚さ 2.5 mm、幅 10.5 mm です。サーボ軸側の面は元の位置のまま、反対側の端を 1 mm 短くしています。アーム側の2穴（間隔 6.5 mm、軸から 18.6 mm）と、軸から 8〜14 mm の幅 1.6 mm の長穴はそのままです。

薄くする前の厚さ 5 mm、幅 12.5 mm は `step/horn_adapter_sg90_5mm.step` です。

## 配線

Arduino コアは [lolren/nrf54-arduino-core](https://github.com/lolren/nrf54-arduino-core) です。ボードマネージャに次を追加し、`XIAO nRF54L15 / Sense` を選び、`firmware/xiao_getup/xiao_getup.ino` を書き込みます。

```
https://raw.githubusercontent.com/lolren/nrf54-arduino-core/main/package_nrf54l15clean_index.json
```

| 接続 | ピン |
| --- | --- |
| 下段サーボ（足側）の信号 | D0 |
| 上段サーボの信号 | D1 |
| サーボ電源 | 5V |
| サーボと基板のグランド | GND |

サーボを 3V3 から給電しないでください。5V ピンは USB 給電で約 4.7 V、電池ではセル電圧です。サーボ2個が同時に止まると基板がリセットすることがあるので、その場合はサーボ用の 5 V を別に用意し、グランドだけ共通にします。SG90 の公称電圧は 4.8〜6 V です。

通常は電源投入だけで自動復帰が有効になります。基板上のUSRボタンで出力と自動復帰を停止し、もう一度押すと再開します。シリアルは115200 bpsで、モード、自動復帰のON/OFF、IMU状態、チップ座標・足座標の加速度、角度、サーボ指令、通信エラー数、停止理由と停止時刻を出します。`off` か `!` でも出力を停止でき、`run` で再開できます。センサー通信異常や動作タイムアウト後は自動で動作を再開しません。

サーボ中央は1500 µsです。`hardware_config.h` で中央補正・回転方向・センサー軸を設定します。この機体ではD0→下段、D1→上段の動作と、センサー軸 `{1,2,3}` の対応を実機で確認しました。`center 1` / `center 2` と `jog 1 5` / `jog 2 5` で1個ずつ確認できます。ジョグは整数の角度で中央から±10°以内に制限しています。

前後（roll主体）の転倒では `sim/getup_reference.py` の既存軌道を使い、開始時の転倒方向を保持します。手前側は元の `GETUP_SEQUENCE`、奥側は `roll_neg` の軌道です。左右（pitch主体）は従来の学習方策を使います。前後については実機と学習姿勢の差で方策が停止・逆向きの指令を出す入力が確認されたため、2026-11-11に変更しました。手前側の修正版は実機での成功確認が未完了です。経緯は [2026-11-11ログ確認](docs/2026-11-11ログ確認.md) にあります。

足元が安定した後に方策が曲がった目標で止まる実機挙動があったため、安定500ms後は両アームをゆっくり中央へ戻す最終段階を設けています。再び大きく傾いたら、その姿勢に対応する復帰制御へ戻ります。起き上がり段階は14秒、中央戻しは開始から6秒、全体で最大20秒を上限とし、中央戻し中に旧14秒制限で切れないようにしています。足が平らな状態で手動再開した場合は中央戻しから始めます。

## 学習

`sim/` で実行します。モデルは公開版を、SG90 のストールトルク 0.16 N·m、サーボ 9 g、XIAO 3 g に変えたものです。

```
pip install -r requirements.txt
python pretrain_robo1_from_scripted.py
python eval_robo1_policy.py
python export_policy_header.py robo1_getup_ppo.zip -o ../firmware/xiao_getup/policy_network.h
```

`pretrain_robo1_from_scripted.py` は、4つの倒れ姿勢の教師軌道から方策を学習します。ロール正方向の軌道は、軽い機体でも最後まで立っていられるものを探索して `getup_reference.py` に入れています。同梱の `robo1_getup_ppo.zip` は、その学習結果です。

評価では `roll_pos`、`roll_neg`、`pitch_pos`、`pitch_neg` のすべてで、直立かつサーボが中央付近の状態を 50 ステップ維持しました。この判定は `success_count >= 50` です。

同じチェックポイントから既定の学習率で PPO を続けると、立っていられる姿勢が減りました。同梱しているのは、4姿勢とも立った behavior cloning の方策です。

この機体は脚で歩くロボットではありません。足の上に2軸の腕があり、起き上がる機構です。2つの回転軸は平行ではなく、下段は足の X 軸、上段は Y 軸です。周期入力を入れると、前後（X）も左右（Y）もシミュレーション上は動けます。左右が作れないのは軸が平行な場合です。

起き上がり方策は、SG90 のトルクを 0.16 / 0.15 / 0.10 N·m にしても4姿勢とも直立を 50 ステップ維持しました。1S 駆動でトルクがさらに落ちる分は実機で測っていません。

動くときは直立のまま歩くのではなく、腕を振って水平を越え、起き直る動きです。トルク 0.15 N·m の周期入力でも、8 秒で前 +0.27 m、後ろ +0.12 m、左 +0.10 m、右 +0.23 m 動きます。そのあいだの半分以上は水平より下です。前後も左右も、軸が直交しているのでシミュレーション上は作れます。

指令は出した瞬間の機体の前後左右です。ヨーを積分すると磁力計がない IMU では方位がずれます。世界の北は使いません。

`train_walk.py` は、その周期入力と転倒からの起き上がり軌道へ方策を合わせます。Actor の入力は重力、ジャイロ、前回の行動、指令角、コマンドだけです。真の速度は critic だけが見ます。報酬は、指令方向へ進んだ距離、行動の変化、サーボの力、水平より下にいる時間です。摩擦、質量、サーボの kp・トルク・遅れ、IMU ノイズは環境に入っていて、`--timesteps` で PPO を足すときに使います。

32 ユニットでは 4 方向と起き上がりが同時に残りませんでした。同梱の方策は 64 ユニット 2 層です。直立から 8 秒で、前 +0.11 m、後ろ +0.13 m、左 +0.09 m、右 +0.06 m。4 つの転倒姿勢からは 1.8 秒で直立に戻ります。1S の実トルクと、実機の起き上がり・移動はまだ見ていません。

```
python train_walk.py
```

## テスト

```
gcc -Wall -Wextra -O2 -o /tmp/test_getup firmware/test_getup.c -lm && /tmp/test_getup
gcc -Wall -Wextra -O2 -o /tmp/test_policy firmware/test_policy.c -lm && /tmp/test_policy
c++ -std=c++17 -Wall -Wextra -Werror -I firmware/test_support -o /tmp/test_firmware firmware/test_firmware.cpp && /tmp/test_firmware
```

転倒判定、復帰、中央パルス、横倒し・反転を含む姿勢角、曲がったままで完了しない条件、安定後の中央戻し、再び傾いたときの方策への復帰、タイムアウト後の停止をCテストで確認しています。方策のC実装は記録済みのPython出力との一致を確認しています。MuJoCo評価の記載は元チェックポイントについての過去の結果です。

2026-11-10に実機でサーボの接続、中央出し、ホーンの取り付け、上下の回転方向、IMU軸を確認しました。修正版では、USB接続で奥側へ倒した状態から手で支えずに起き上がり、両アームが伸びた状態で立つことをユーザーの目視とログで確認できました。他の3方向と電池単独での一連の動作は未確認です。その後、ユーザーの希望に合わせて電源投入時の自動監視を有効にしました。

## 出典

- 元モデル: `homemadegarbage/SelfRisingRobot` commit `c344c2a4a1a51562247528d8aadcc0a527b6b103`
- SG90: TowerPro 公称寸法
- XIAO nRF54L15 Sense: Seeed Studio 公称基板外形 21.0 × 17.8 mm、IMU は LSM6DS3TR-C

元リポジトリにはライセンス表記がありません。再配布や商用利用の前に、権利者へ条件を確認してください。
