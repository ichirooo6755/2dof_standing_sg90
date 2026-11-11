# supplied 3MF + SG90 + 1000mAh MuJoCo/PPO

元実装: [ggoonnzzaallo/2dof_standing](https://github.com/ggoonnzzaallo/2dof_standing)、固定コミット `3a9cb54fc3088ccf10322d8de4ceb0128a317056`。
元リポジトリの10入力→64→64→2出力、20ms周期、PPO、GravityFilter、RecoveryStrategy、直立判定と報酬を再利用。元チェックポイントのactorを初期値にして追加学習する。元リポジトリと実機ファームウェアはこの処理では変更しない。

2026-11-11に262,144ステップを実行済み。[学習結果](results/run_20261111/RESULTS.md)、[4方向の動画](results/run_20261111/recovery_four_directions.mp4)、[学習済みモデル](results/run_20261111/best_trained.zip)。別48条件は追加学習・元モデルとも48/48成功。平均判定時間は2.241秒→2.208秒。実機では未確認。

## 物理条件

| 項目 | 基準 | 根拠・扱い |
|---|---:|---|
| 印刷部品6点 | 47.113g | 指定3MFをBambuStudio 02.08.02.61で再スライス。PETG密度1.37g/cm³、充填45%、外壁2周、0.2mm。ラベル付きG-codeの押出量からサポート・ブリムを除く |
| SG90 | 9g ×2 | TowerProメーカー公称。実機個体未計量 |
| DTP603450(PHR)推定 | 20g | ユーザー表記 DTP60 3450PHR FL。メーカーセル仕様20g未満。保護回路・配線の差を含め18〜22gで変動 |
| PowerBoost 500 | 3.5g | Basic 3g、500C 4g。型の差を含め3〜4.5gで変動 |
| XIAO | 3g | ピン等を含む仮重量。未計量 |
| 配線・ネジ等 | 3.5g | 仮の追加量。SG90メーカー付属リードとの重複の可能性を含め不確かさあり |
| **合計** | **95.113g** | 実測重量ではない |

電池、昇圧基板、XIAOは台座のケース内に仮配置。各部品質量から重心・慣性を合成。印刷物の慣性はCADの一様密度分布をスライス質量へ換算した近似で、実際の外壁・充填分布そのものではない。印刷物質量±10%、SG90±6%、本体重心±2mm、摩擦0.25〜1.25を変動させる。

SG90公称ストールトルク1.8kgf·cm = 0.17652N·m、無負荷速度60°/0.1秒（4.8V）。PowerBoostは公称5.2V。公称値を実機の連続使用トルクとみなさず、ストールトルク0.075〜0.17652N·m、速度5.5〜10.47rad/s、指令遅延0〜60msで学習する。トルクは速度に応じて減少。2軸同時負荷に対する共有トルク上限0.10〜0.25N·mも変動させる。この上限は未校正の電源低下の近似であり、電流・電圧・保護回路・マイコンのリセットを解く電気回路モデルではない。

## 形状と取り付けの前提

- 入力は `source/ボディ1.3mf`。元一時ファイルからコピー済み。SHA256は `configuration.json` に記録。
- プリント台上の配置を組立位置と混同しない。Bambuの `source_offset` から台座・中間・上段・ケースのCAD位置を復元。
- 関節軸は元モデルの下段+X／上段+Y、原点Z=24.4mm／61.4mmを引き継ぐ。ホーン2点は拡張されたフォーク形状と元関節軸から位置を再構成。3MFには組立ジョイント拘束がないため、実機の穴位置・中立角と照合が必要。
- サーボ外形は箱の近似。印刷物は指定メッシュを表示し、CoACDで最大16凸形状/部品に分割。ケースは外側の凸包。隣接する関節部品同士はMuJoCo標準の親子接触除外を使用し、床と非隣接リンクの接触は有効。
- IMUは既存ファームウェアと同じXYZ取付を仮定。新ケースで回転させる場合は変換行列の更新が必要。
- `model_preview.png` はモデルの静止画像。実機の写真ではない。

## 再実行

プロジェクトのルートで実行:

```sh
python3.12 -m venv .venv-sim
.venv-sim/bin/python -m pip install -r sim/sg90_1000mah/requirements-lock.txt
```

元3MFの設定でスライス（印刷は行わない）:

```sh
/Applications/BambuStudio.app/Contents/MacOS/BambuStudio --debug 2 --slice 1 --export-3mf sliced.3mf --outputdir "$PWD/sim/sg90_1000mah/source/sliced" "$PWD/sim/sg90_1000mah/source/ボディ1.3mf"
OMP_NUM_THREADS=2 .venv-sim/bin/python sim/sg90_1000mah/build_model.py
```

追加学習（CPU 2環境、Torchは各1スレッド）:

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 .venv-sim/bin/python -u sim/sg90_1000mah/train.py --steps 262144 --envs 2 --out sim/sg90_1000mah/results/run_20261111
```

`progress.json` は4096ステップごとの進捗。32768ステップごとにモデルと報酬正規化の状態を保存し、学習と分離した開発用12条件で選択。最後にチェックポイント選択に使わない48条件で追加学習モデルと元モデルを比較する。`best_trained.zip` は追加学習後のモデル、`best.zip` は初期モデルも含む開発評価最良モデル。

成功条件は「床に接して台座が8°以内、角速度0.4rad/s未満、両関節角0.12rad未満、目標角0.05rad未満を1秒継続」。シミュレーション成功は実機成功を意味しない。

学習後の書き出し・動画・集計:

```sh
MPLCONFIGDIR="$PWD/sim/sg90_1000mah/results/mpl-cache" .venv-sim/bin/python upstream/2dof_standing/sim/export_actor.py sim/sg90_1000mah/results/run_20261111/best_trained.zip sim/sg90_1000mah/results/run_20261111/policy_sg90_1000mah.h
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv-sim/bin/python sim/sg90_1000mah/render_results.py
.venv-sim/bin/python sim/sg90_1000mah/summarize.py
```

## 出典

- [TowerPro SG90](https://towerpro.com.tw/product/sg90-7/)
- [Data Power DTP603450(PHR)メーカー仕様書](https://www.sengoku.co.jp/item/pdf/SPE-DTP603450%28PHR%29-3.7V-1000mAh-en-2.0ver.pdf)
- [Adafruit PowerBoost 500 Basic: 3g / 5.2V](https://www.adafruit.com/product/1903)
- [Adafruit PowerBoost 500C: 4g / 5.2V](https://www.adafruit.com/product/1944)

`model_report.json` に各部品の重量・重心・慣性、`configuration.json` に仮定、`results/.../training_report.json` に学習・評価と入力ハッシュを保存。実機への指令・書き込みは行わない。

## 学習後の実機運用

学習処理とは別に、ユーザーの指示でactorを `firmware/standing_xiao` に組み込み書き込み済み。
現在のport 5は、中央出しで確認した下段1500µs・上段1464µsを使い、「転倒後、静止が連続300ms」で復帰を開始する。
元の学習・48条件評価にはこの開始待ちを入れていない。実機の現在の記録は [ファームウェアREADME](../../firmware/standing_xiao/README.md) を参照。

静止待ちを含めて転倒から評価する環境と、0.3秒・0.5秒の比較結果をまとめた。[静止待ちを含む追加学習](gated/README.md)

## 方向移動と台座の一辺でのバランス

同じ機体モデルでUSB側への移動と、USB側底辺の上でのバランスを追加学習した。採点条件、学習ログ、サーボを中央に保つ場合との比較動画を[方向移動・辺立ちの結果](../sg90_skills/RESULTS.md)に掲載。実機への書き込みは行っていない。
