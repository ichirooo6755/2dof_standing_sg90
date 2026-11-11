# MuJoCoでの移動・辺立ちの結果

USB開口側への前進は、平らな台座から平均2.45cm／12秒を確認した。台座を自分で倒して進む尺取虫動作と、底辺だけで3秒立ち続ける動作は未達。下の評価は指定3MF・SG90・1000mAh LiPo・PowerBoost 500のモデルを使い、質量・摩擦・サーボ特性を変えた24条件ずつで行った。

| 開始姿勢と課題 | 学習方策 | サーボを中央に保つ比較 | 達成数 | 動画・ログ |
|---|---:|---:|---:|---|
| 平らな台座からUSB側へ移動 | 前進2.45cm、横ずれ0.40cm | 前進0.015cm | 0/24 | [動画](results/move/flat_comparison/comparison.mp4) · [評価](results/move/flat_comparison/paired_evaluation.json) |
| USB側に横倒しの姿勢から移動 | 前進4.98cm、横ずれ6.99cm | 前進0.72cm | 1/24 | [動画](results/move/side_comparison/comparison.mp4) · [評価](results/move/side_comparison/paired_evaluation.json) |
| 平らな台座から辺立ち | 最長0.00秒 | 最長0.00秒 | 0/24 | [動画](results/edge/flat_comparison/comparison.mp4) · [評価](results/edge/flat_comparison/paired_evaluation.json) |
| 辺で釣り合う角度の近くから保持 | 最長0.64秒、平均0.25秒 | 最長0.36秒、平均0.18秒 | 0/24 | [動画](results/edge/near_comparison/comparison.mp4) · [評価](results/edge/near_comparison/paired_evaluation.json) |

距離は試行終了時の平均。横ずれは左右の符号を除いた平均。移動の達成条件は12秒で前進6cm、後半6秒で前進2cm、最終横ずれ4cm未満。辺立ちは、USB側底辺の接触・荷重・角速度条件を満たした最長連続時間で、3秒を達成条件にした。[実装と採点](README.md)

## 動きの内訳

- 平らな台座からの方策は、後半6秒にも平均1.27cm前進した。台座の最大傾きは全24試行で約1.65°。物理計算を1msから0.5ms刻みにして同じ条件を再実行すると、平均1.94cm進んだ。床との接触や摩擦の近似に依存するため、実機での移動量は未確認。[計算刻みを変えた評価](results/move/timestep_05ms/paired_evaluation.json)
- 横倒し開始では、サーボで床に接触する姿勢を変えて大きく移動した。横ずれが大きく、後半6秒の前進量は平均−0.95cmだった。進行方向に沿って繰り返し前進できた試行は少数だった。[横倒し開始の評価](results/move/side_comparison/paired_evaluation.json)
- 台座を平らに固定した形状計算で、両関節の±90°を4.5°刻みで1,681通り調べた。アームと床の最小隙間は14.5mm。機体重心の床面への投影は全姿勢で約70mm四方の台座内だった。モデル上で床を腕で押すには、まず台座を傾ける動きが必要になる。[形状計算ログ](results/flat_geometry.json)
- 辺立ちは、シミュレーション上の関節角・角速度も使う制御を別に調整したが、釣り合い付近からの別24条件で最長0.64秒だった。[関節状態を使った診断](results/edge/state_feedback/diagnostic.json)

## 保存した方策と学習ログ

| 課題 | PPOの実行ステップ | 採用したチェックポイント | ファイル |
|---|---:|---:|---|
| 平らな台座から移動 | 262,144 | 229,376 | [方策構成](results/move/ppo/policy_bundle.json) · [学習ログ](results/move/ppo/training_report.json) |
| 横倒しから移動 | 131,072 | 98,304 | [方策構成](results/move/ppo_side/policy_bundle.json) · [学習ログ](results/move/ppo_side/training_report.json) |
| 辺立ち | 524,288 | 0（探索で選んだ方策） | [方策構成](results/edge/ppo/policy_bundle.json) · [学習ログ](results/edge/ppo/training_report.json) |

PPOの前に、動作パラメータを報酬で選ぶ交差エントロピー法を実行した。探索・評価を合わせて1,667,142環境ステップ、PPOによる学習は合計917,504ステップ。各実行フォルダに評価条件とチェックポイントを保存した。実機への書き込みは行っていない。
