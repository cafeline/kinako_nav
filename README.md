# kinako_nav 立ち上げ手順
## 1. リポジトリ取得とサブモジュール更新

```bash
git clone git@github.com:cafeline/kinako_nav.git
cd kinako_nav
git submodule update --init --recursive
```

## 2. Docker イメージのビルドとコンテナ起動

> **GPUを使う場合**は以下のコマンドの前に
> `export COMPOSE_FILE="docker-compose.yml:docker-compose.gpu.yml"`
> を実行しておくと、以降の `docker compose` コマンドが自動的にGPU設定を取り込みます。
> また、ホストと同じ `ROS_DOMAIN_ID` を使いたい場合はホスト側で
> `export ROS_DOMAIN_ID=<値>`
> をセットしてから `docker compose` を実行すると、同じ値がコンテナへ自動的に渡されます（未設定時は0）。

```bash
# X11 転送を許可（必要に応じて）
xhost +local:docker

# イメージをビルド
docker compose build

# コンテナをバックグラウンド起動
docker compose up -d
```

### CPU / GPU の切り替え

- **CPU版（デフォルト）**: 何も設定せずに上記の `docker compose` コマンドを実行すれば CPU のみで動作します。
- **GPU版**: NVIDIA GPU を使いたいターミナルで先に
  `export COMPOSE_FILE="docker-compose.yml:docker-compose.gpu.yml"`
  を実行してから `docker compose build` / `docker compose up -d` を行います（`xhost +local:docker` と NVIDIA Container Toolkit のセットアップが必要）。

## 3. コンテナに入ってワークスペースをビルド

```bash
# シェルに入る
docker compose exec ros2 bash

# 以降はコンテナ内
cd ${ROS_WS}            # /home/user/navigation_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 4. 統合 bringup を起動（推奨）

```bash
ros2 launch kinako_nav_bringup navigation.launch.py scenario:=tsukuba use_rviz:=false
```

### シナリオ切り替え

```bash
# つくば
ros2 launch kinako_nav_bringup navigation.launch.py scenario:=tsukuba

# 津田沼
ros2 launch kinako_nav_bringup navigation.launch.py scenario:=tsudanuma

# 19f
ros2 launch kinako_nav_bringup navigation.launch.py scenario:=19f
```

### 主な launch 引数

- `scenario`: `tsukuba` / `tsudanuma` / `19f`
- `use_sim_time`: `true` / `false`
- `use_rviz`: `true` / `false`
- `auto_start`: `true` / `false`（ウェイポイント巡航の自動開始）
- `map_hdf5_file`: 自己位置推定用 3D マップ (`.h5`) の上書き
- `vq_map_file`: 可視化用 3D マップ (`.h5`) の上書き
- `map_yaml_file`: 2D マップ (`.yaml`) の上書き
- `waypoint_csv_file`: ウェイポイント (`.csv`) の上書き
- `regions_config_file`: pointcloud 切り出し領域 (`.yaml`) の上書き

例:

```bash
ros2 launch kinako_nav_bringup navigation.launch.py \
  scenario:=tsukuba \
  use_rviz:=true \
  auto_start:=false
```

### 互換 launch（既存呼び出し）

既存の起動コマンドも互換ラッパーとして使えます（内部で `kinako_nav_bringup` を呼び出します）。

```bash
ros2 launch emcl2 emcl2.launch.py scenario:=tsukuba
ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py scenario:=tsukuba
```

## 5. 終了

```bash
exit                        # コンテナシェルを抜ける
docker compose down         # コンテナを停止・削除
```
