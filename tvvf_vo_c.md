# TVVF-VO C++ 実装 徹底解説

このドキュメントは `tvvf_vo_c` パッケージ（`navigation_ws/src/tvvf_vo_cpp`）のアルゴリズムと主要パラメータを、初心者でも追えるようにできるだけ噛み砕いて整理したものです。実装のソースへの参照や、調整時の考え方も合わせてまとめています。

---

## 1. TVVF（Time-Varying Vector Field）とは何か
- **ベクトル場**: マップ上の各点に「ゴールへ進むべき速度ベクトル」を事前に敷き詰めた地図。ロボットは現在位置のベクトルを読むだけで進行方向が分かる。
- **時間変化 (Time-Varying)**: 動的障害物や最新のマップ情報を取り込み、ベクトル場を逐次更新・ブレンドすることで「流れ」を変化させる。
- **狙い**: 経路計画を都度解くのではなく、ベクトル場を参照するだけでリアルタイムに滑らかで安全な指令を生成すること。

---

## 2. 全体処理フロー（1サイクル）
1. **マップ入力**: `OccupancyGrid` を購読。動的障害物マスクも受け取る（`tvvf_vo_node.cpp`）。
2. **コストマップ生成**: 障害物からの距離（クリアランス）を計算し、「走行可能速度レイヤ」を作る（`CostMapBuilder`）。
3. **Fast Marching Method (FMM)**: ゴールから到達時間を解き、勾配をベクトルとして静的ベクトル場を構築（`FastMarching`）。
4. **コストを反映**: OccupancyGridと障害物マスクを統合したコストマップから距離に応じた速度制限を行い、静的ベクトル場を生成・更新。
5. **速度最適化**: 目標ベクトル・前回速度・回避制約・加速度制限を重み付けし、滑らかな並進速度ベクトルを算出（`SmoothVelocityOptimizer`）。
6. **差動二輪への変換**: 並進速度ベクトルをロボット姿勢に合わせて線形速度・角速度 (`cmd_vel`) に変換し、パブリッシュ（`tvvf_vo_control.cpp`）。
7. **可視化・ログ**: ベクトル場やコストマップをRVizに出し、性能計測を実施（`tvvf_vo_visualization.cpp`）。

---

## 3. 主要コンポーネントと役割

### 3.1 コストマップ生成 `CostMapBuilder`
- 入力: OccupancyGrid。
- 出力: `speed_layer`（そのセルで許可する速度の上限）と `clearance_layer`（障害物までの距離）。
- 仕組み: 多源Dijkstraで全障害物からの最近傍距離を計算し、距離が小さいほど速度を落とす単調減衰関数で `speed_layer` を作る。
- 参照: `src/core/cost_map_builder.cpp`, `include/tvvf_vo_c/core/cost_map_builder.hpp`。

### 3.2 Fast Marching Method (FMM) `FastMarching`
- 目的: ゴールからの到達時間場 `T(x,y)` を解き、その勾配 `-∇T` をベクトル場として使う。
- 仕組み: Eikonal方程式 `|∇T| = 1/F(x,y)` をナローバンド法で解く。`F` は `speed_layer`（移動速度）に相当。
- ベクトル取得: 勾配を正規化して「ゴールへ最短時間で進む方向」を各セルに格納。
- 参照: `src/core/fast_marching.cpp`, `include/tvvf_vo_c/core/fast_marching.hpp`。

### 3.3 グローバルフィールド生成 `GlobalFieldGenerator`
- 静的場の生成: FMMで作ったベクトル場を保持。必要に応じてサブマップにクロップして計算コストを削減。
- 動的障害物の反映: 現在はマスクやコストマップによる速度制限のみを利用。
- 出力: 最新ベクトル場と計算時間。
- 参照: `src/core/global_field_generator.cpp`, `include/tvvf_vo_c/core/global_field_generator.hpp`。

### 3.4 速度最適化 `SmoothVelocityOptimizer`
- 入力: 望ましい速度ベクトル（ベクトル場の方向）、前回速度、障害物リスト、制約（速度上限・加速度上限）。
- 処理: 目標方向・滑らかさ・障害物回避を重み付きで合成し、加速度制限でクリップ。
- 出力: 並進速度ベクトル（ロボット座標系で使う前に姿勢変換される）。
- 参照: `src/core/smooth_velocity_optimizer.cpp`, `include/tvvf_vo_c/core/smooth_velocity_optimizer.hpp`。

### 3.5 ROS ノード `TVVFVONode`
- 役割: 入出力のハブ。TFから自己位置取得、マップ購読、ゴール受信、制御ループ実行、可視化送信。
- 制御周期: `control_loop_rate` でタイマー駆動。ゴール到達判定・停止処理を含む。
- 参照: `src/ros/tvvf_vo_node.cpp`, `src/ros/tvvf_vo_control.cpp`, `src/ros/tvvf_vo_visualization.cpp`。

---

## 4. 数式レベルの動き（ざっくり）
- **コストマップ**: クリアランス `d` に対し `speed = clamp(1 / (1 + α/d), min_speed, max_speed)`。障害物セルは `speed=0`。
- **FMM**: `|∇T| = 1/F` を解き、到達時間 `T` の勾配を正規化してベクトル `v_static = -∇T/|∇T|`。
- （斥力項は削除済み）
- **ベクトルブレンド**: `v_total = normalize((1-β) * v_static + β * f_rep_total)`（`β`はブレンド重み）。
- **速度平滑化**: `v_opt = clamp_accel(weighted(goal, smooth) , max_speed, max_acc * dt)` で前回速度との差分を加速度制限。
- **差動二輪変換**: ロボット姿勢角 `θ` に合わせ、ベースリンク座標へ回転させて `v` を `linear`, `angular` に変換。

---

## 5. パラメータ詳細とチューニング指針
主に `navigation_ws/src/tvvf_vo_cpp/config/tvvf_vo_c_params.yaml`（ローカルプランナ）と `navigation_ws/src/tvvf_vo_cpp/config/tvvf_vo_global_params.yaml`（グローバルフィールド版）、環境別の上書き例として `navigation_ws/src/emcl2_ros2/config/tsukuba.yaml` を参照。

### 5.1 ゴール・ベクトル場関連
- `k_attraction`: ゴールへの引力強度。大きいほどゴール指向が強いが、障害物付近で無理をしやすい。
- （斥力パラメータは削除済み）
- `min_distance`: 0割防止の最小距離。通常デフォルトのままでOK。
- `vector_field_path_width`: パス追従を有効にするときの許容幅。広げると経路から多少外れても許容。

### 5.2 経路統合（パス追従）
- `k_path_attraction`: 既知パスへの引力。大きいほどパス中心に吸い寄せる。
- `path_influence_radius`: パスが効く距離。広いとパス外でも引かれる。
- `lookahead_distance`: 先読み距離。小さいとカーブに弱く、大きいとオーバーシュートしやすい。

### 5.3 障害物回避（コストマップベース）
- `safety_margin`: ロボット外形に足す安全オフセット。接触リスクが高いときに増やす。
- `occupancy_clear_radius`（tsukuba.yaml）: ロボット足元を強制クリアにする半径。直下のマップ誤差を無視する。
- `obstacle_mask_topic`: 障害物マスクを購読するトピック。マスクはOccupancyGridに統合して扱う。

### 5.4 速度・加速度制約
- `max_linear_velocity`, `max_angular_velocity`: 実機に合わせた上限。まずは低めで安定性を確保。
- `max_acceleration`, `optimizer_max_linear_acceleration`: 加速度上限。振動が出る場合は下げる。
- `direction_weight` / `safety_weight` / `efficiency_weight`（tvvf_vo_c_params.yaml コメント欄）: 速度最適化の重み。安全を優先するなら `safety_weight` を上げる。

### 5.5 コストマップ
- `costmap_occupied_threshold` / `free_threshold`: 占有判定。センサノイズが多いときは閾値を上げて「障害物」に寄せる。
- `costmap_alpha`: クリアランス→速度の変換カーブの傾き。大きいほど障害物近傍で速度を強く落とす。
- `costmap_min_speed` / `costmap_max_speed`: 速度レイヤの上下限。遅すぎて止まる場合は `min_speed` を上げる。
- `costmap_clearance_epsilon`: クリアランス0近傍のオフセット。小さくしすぎると急減速が出る。
- `costmap_max_clearance`: それ以上離れても速度が頭打ちになる距離。広げると遠くの障害物影響を無視しやすい。

### 5.6 ロボットモデル・フレーム
- `robot_radius`, `wheel_base`: 機体サイズと車輪幅。経路許容幅に影響するので実寸に合わせる。
- `base_frame`, `global_frame`, `laser_frame`: TF名は実機と一致させること。ズレると位置推定が崩れる。
- `orientation_tolerance`, `goal_tolerance`: ゴール判定の緩さ。動作が止まらない場合は少し広げる。

### 5.7 制御ループ・可視化
- `max_computation_time`: 1周期あたりの許容計算時間目安。超える場合は解像度や領域を狭める。
- `viz_update_rate`, `vector_field_resolution`, `vector_field_range`, `max_vector_points`: RViz負荷と情報量のトレードオフ。重いときは解像度を粗くする。

### 5.8 グローバルフィールド特有（`tvvf_vo_global_params.yaml`）
- `global_field.blend_weight`: 動的障害物マスクとのブレンド度合い。
- `tVVF` セクションの `time_horizon`, `prediction_steps`: 予測時間とステップ数。多いほど先読みするが計算負荷が増える。
- `obstacle_avoidance` の `emergency_stop_distance`: 緊急停止距離。誤検知が多い場合はやや小さく。
- `robot` の `footprint_padding`: 実寸よりどれだけ余白を取るか。

---

## 6. 調整の進め方（ステップ例）
1. **安全優先の初期値**: `max_linear_velocity` を低め（0.3〜0.5m/s）、`optimizer_smooth_weight` を高め（4〜6）にして振動を抑える。
2. **障害物回避を強める**: `costmap_alpha` を上げて近傍セルで速度を強く落とすか、`occupancy_clear_radius` を調整して足元の誤検知を除去する。
3. **パス追従の精度向上**: `k_path_attraction` を上げ、`vector_field_path_width` を環境幅に合わせて調整。カーブで膨らむなら `lookahead_distance` を短めに。
4. **速度感の調整**: 直進で遅いときは `costmap_min_speed` を上げ、カーブでの減速を弱めたいときは `costmap_alpha` を下げる。
5. **動的障害物が多い場合**: マスク生成側でOccupancyGridを高頻度更新し、`costmap_max_clearance` や `costmap_alpha` を調整して減速を早める。
6. **発散・振動対策**: `optimizer_max_linear_acceleration` を下げ、`safety_weight` を上げる。`orientation_tolerance` を少し緩めて停止判定を入りやすくする。

---

## 7. よくある症状と対処
- **障害物手前で立ち往生**: `costmap_alpha` を下げて減速しすぎないようにし、`costmap_min_speed` を上げて停止を防ぐ。
- **細い通路を抜けられない**: `vector_field_path_width` を狭めて通路中心に誘導するか、`robot_radius` を実寸に合わせて過大評価を避ける。
- **カーブでオーバーシュート**: `lookahead_distance` を短く、`optimizer_smooth_weight` を上げ、`max_angular_velocity` を少し上げる。
- **急ブレーキ・振動**: `optimizer_max_linear_acceleration` を下げ、`costmap_min_speed` を上げて停止と発進の差を小さくする。
- **RVizが重い**: `vector_field_resolution` を大きく（粗く）し、`max_vector_points` を減らす。

---
