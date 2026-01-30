# kinako_nav 立ち上げ手順
## 1. リポジトリ取得とサブモジュール更新

```bash
git clone git@github.com:cafeline/tsukuba2025_container.git
cd tsukuba2025_container
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

## 4. emcl2.launch.py を起動

```bash
ros2 launch emcl2 emcl2.launch.py
```

必要に応じて別ターミナルから `docker compose exec ros2 bash` でログを確認したり、GPU構成を有効にしている場合は `nvidia-smi` で使用状況を監視してください。

## 5. 後片付け

```bash
exit                        # コンテナシェルを抜ける
docker compose down         # コンテナを停止・削除
```

以上で `emcl2.launch.py` 実行までの一連の操作は完了です。
