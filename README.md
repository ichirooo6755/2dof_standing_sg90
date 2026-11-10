# SelfRisingRobot SG90 / XIAO nRF54L15 Sense

[SelfRisingRobot](https://github.com/homemadegarbage/SelfRisingRobot) を、TowerPro SG90 2個と Seeed Studio XIAO nRF54L15 Sense で起き上がらせるためのSTL、MuJoCo学習、ファームウェアです。

XIAO nRF54L15 Sense の LSM6DS3TR-C で姿勢を見ます。ロールまたはピッチが 0.75 rad（約43°）を 300 ms 超えると転倒と判定し、学習した方策で起き上がります。直立（0.25 rad 以内）を 700 ms 維持すると止まります。

## 印刷

| ファイル | 数量 | 内容 |
| --- | ---: | --- |
| `stl/foot_xiao_nrf54l15_sense.stl` | 1 | 足、下段サーボ固定、XIAOの位置決め枠 |
| `stl/arm1.stl` | 1 | 中間アーム。公開STL `arm1P.stl` と同一 |
| `stl/arm2.stl` | 1 | 上段アーム。公開STL `arm2P.stl` と同一 |
| `stl/horn_adapter_sg90.stl` | 2 | SG90付属ホーンと各アームを締結 |

単位は mm です。

Fusion で開くときは `step/` のファイルを使ってください。STL は Fusion ではメッシュボディになります。STEP は同じ形状のソリッドです。

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

基板上のボタンで起き上がりを開始します。動作中にもう一度押すと停止し、自動再開も切ります。シリアルは 115200 bps で、`GETUP` / `MANUAL`、`fallen` / `upright`、roll、pitch、サーボ目標を出します。平らな台で `upright` かつ roll と pitch がほぼ 0 にならない場合は、`xiao_getup.ino` の `kAxisSign` を変えます。平坦時にチップの +Z が上、+X が USB 方向という実装時の仮定です。実機では未測定です。

サーボの中央は 1500 µs です。`kServoTrimUs` で個体差を吸収します。

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

## テスト

```
gcc -Wall -Wextra -O2 -o /tmp/test_getup firmware/test_getup.c -lm && /tmp/test_getup
gcc -Wall -Wextra -O2 -o /tmp/test_policy firmware/test_policy.c -lm && /tmp/test_policy
```

転倒判定、復帰、サーボ中央パルス、そして方策の C 実装が Python の出力と一致することを確認しています。MuJoCo 上の起き上がりは `sim/eval_robo1_policy.py` で確認しています。実機への書き込みと転倒からの起き上がりは未確認です。

## 出典

- 元モデル: `homemadegarbage/SelfRisingRobot` commit `c344c2a4a1a51562247528d8aadcc0a527b6b103`
- SG90: TowerPro 公称寸法
- XIAO nRF54L15 Sense: Seeed Studio 公称基板外形 21.0 × 17.8 mm、IMU は LSM6DS3TR-C

元リポジトリにはライセンス表記がありません。再配布や商用利用の前に、権利者へ条件を確認してください。
