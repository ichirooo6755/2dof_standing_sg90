# SG90の方向移動と台座の辺立ち

指定の `ボディ1.3mf`、SG90 2個、1000mAh LiPo、PowerBoost 500、XIAOを含む95.1gのMuJoCoモデルを使った。実機重量と電子部品の位置は推定。学習では、質量、重心、床の摩擦、サーボ速度・トルクと指令遅延を変動させた。重量の内訳、仮定、形状は[起き上がり学習のモデル資料](../sg90_1000mah/README.md)。

## 結果

[集計と動画](RESULTS.md)に、学習に使っていない条件での評価と、サーボを中央に保つ比較を掲載した。今回の作業範囲はシミュレーション。ファームウェアへの組み込みと書き込みは行っていない。

## 課題と採点

- **方向移動**は試行開始時のUSB開口側（CAD −X）へ進む設定。平らな台座から12秒で6cm以上、後半6秒で2cm以上進み、最終横ずれ4cm未満を成功とした。横倒しから開始する評価にも同じ距離条件を使った。
- **辺立ち**はUSB側の底辺で支持し、角速度を抑えた状態が連続3秒で成功。底辺付近の接点が2cm以上に広がり、そこが機体重量の55%以上の垂直荷重を受け、他の接点の合計は3.5%未満。平らな台座からの開始と、辺で釣り合う角度の近くからの開始を評価した。後者では初期姿勢だけを設定してから解放した。
- 方策は20msごとに両サーボの目標角を最大0.1radずつ変更する。入力は既存ファームウェアのフィルタを通したIMU値とサーボ指令履歴。移動には開始時の方位を基準にジャイロで積分した方向と経過時間も与えた。床の接触や位置の真値は採点に使った。関節角の真値を使う辺立ち診断は別に記録した。

## 再現

プロジェクトのルートで、[既存MuJoCo環境と同じ依存バージョン](../sg90_1000mah/requirements-lock.txt)を入れたPythonを使用する。以下は平らな状態からの移動を例にした手順。

```sh
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1
.venv-sim/bin/python sim/sg90_skills/search.py move --family kick \
  --iterations 32 --population 24 --out sim/sg90_skills/results/reproduction/search
.venv-sim/bin/python sim/sg90_skills/train_ppo.py move --steps 262144 --scale .65 \
  --seed-policy sim/sg90_skills/results/reproduction/search/best_policy.json \
  --out sim/sg90_skills/results/reproduction/ppo
.venv-sim/bin/python sim/sg90_skills/replay.py move \
  sim/sg90_skills/results/reproduction/ppo/policy_bundle.json \
  sim/sg90_skills/results/reproduction/replay --video-seed 96000
```

探索・学習時に使った引数と選択結果は各実行フォルダの `search_report.json`、`training_report.json` を参照。PPO方策は `policy_bundle.json` にある探索パラメータと、同じフォルダの `best_residual.zip` を組み合わせて動く。辺立ちではチェックポイント選択で探索時点の方策が残った。
