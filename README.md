# tsukuba2025_container 立ち上げ手順
## 1. リポジトリ取得とサブモジュール更新

```bash
git clone git@github.com:cafeline/tsukuba2025_container.git
cd tsukuba2025_container
git submodule update --init --recursive
```

## 2. Docker イメージのビルドとコンテナ起動

```bash
# X11 転送を許可（必要に応じて）
xhost +local:docker

# イメージをビルド
docker compose build

# コンテナをバックグラウンド起動
docker compose up -d
```

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

必要に応じて別ターミナルから `docker compose exec ros2 bash` でログを確認したり、`nvidia-smi` で GPU 使用状況を監視してください。

## 5. 後片付け

```bash
exit                        # コンテナシェルを抜ける
docker compose down         # コンテナを停止・削除
```

以上で `emcl2.launch.py` 実行までの一連の操作は完了です。
