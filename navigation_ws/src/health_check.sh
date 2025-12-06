#!/usr/bin/env bash
set -euo pipefail

# チェック対象トピック
topics=(
  "/livox/imu"
  "/livox/lidar"
  "/scan"
  "/mcl_pose"
  "/obstacles"
  "/obstacle_mask"
  "/cmd_vel_raw"
  "/cmd_vel"
)

# チェックするTFペア（parent child）
tf_pairs=(
  "map lidar_link"
)

# タイムアウト秒（環境変数で上書き可能）
TIMEOUT="${TIMEOUT:-2}"
TF_TIMEOUT="${TF_TIMEOUT:-2}"
POSE_TIMEOUT="${POSE_TIMEOUT:-2}"

# 現在のトピック一覧を取得
if ! topic_list=$(ros2 topic list 2>/dev/null); then
  echo "ros2 topic list に失敗しました。環境を source 済みか、ROS_DOMAIN_ID などを確認してください。" >&2
  exit 1
fi
readarray -t available_topics <<<"$topic_list"

topic_exists() {
  local needle="$1"
  for t in "${available_topics[@]}"; do
    [[ "$t" == "$needle" ]] && return 0
  done
  return 1
}

check_topic() {
  local topic="$1"
  local info pub_count sub_count
  local ok=1
  local qos_presets=()
  local rel_choice dur_choice
  local pub_rels pub_durs

  if ! topic_exists "$topic"; then
    printf "checking %s ... NOT FOUND (ros2 topic list に存在しません)\n" "$topic"
    return 1
  fi

  if ! info=$(ros2 topic info -v "$topic" 2>/dev/null); then
    printf "checking %s ... ros2 topic info に失敗しました\n" "$topic"
    return 1
  fi
  pub_count=$(echo "$info" | awk '$1=="Publisher" && $2=="count:" {print $3}')
  sub_count=$(echo "$info" | awk '$1=="Subscription" && $2=="count:" {print $3}')

  # publisherのQoSを抽出
  mapfile -t pub_rels < <(echo "$info" | awk '/Endpoint type: PUBLISHER/{p=1;next}/Endpoint type: SUBSCRIPTION/{p=0} p && /Reliability:/{print tolower($2)}')
  mapfile -t pub_durs < <(echo "$info" | awk '/Endpoint type: PUBLISHER/{p=1;next}/Endpoint type: SUBSCRIPTION/{p=0} p && /Durability:/{print tolower($2)}')

  # 配列をユニーク化する簡易ヘルパ
  uniq_array() {
    local -n in_arr=$1
    local out=()
    local seen=""
    for v in "${in_arr[@]}"; do
      [[ -z "$v" ]] && continue
      if [[ " $seen " != *" $v "* ]]; then
        out+=("$v")
        seen+=" $v"
      fi
    done
    echo "${out[@]}"
  }

  read -r -a pub_rels <<<"$(uniq_array pub_rels)"
  read -r -a pub_durs <<<"$(uniq_array pub_durs)"

  # リライアビリティ: best_effort があればそれを優先（reliable publisherとも互換）
  if printf '%s\n' "${pub_rels[@]}" | grep -q "best_effort"; then
    rel_choice="best_effort"
  elif printf '%s\n' "${pub_rels[@]}" | grep -q "reliable"; then
    rel_choice="reliable"
  else
    rel_choice="best_effort"  # 情報なしのときは安全側
  fi

  # デュラビリティ: volatile が一つでもあれば volatile（transient_local publisherとも互換）
  if printf '%s\n' "${pub_durs[@]}" | grep -q "volatile"; then
    dur_choice="volatile"
  elif printf '%s\n' "${pub_durs[@]}" | grep -q "transient_local"; then
    dur_choice="transient_local"
  else
    dur_choice="volatile"
  fi

  qos_presets+=("--qos-reliability ${rel_choice} --qos-durability ${dur_choice}")
  qos_presets+=("")  # 念のためデフォルトも試す

  printf "checking %s ... " "$topic"

  for preset in "${qos_presets[@]}"; do
    read -r -a qos_args <<< "$preset"
    if timeout "$TIMEOUT" ros2 topic echo --once "${qos_args[@]}" "$topic" >/dev/null 2>&1; then
      preset_label="${preset:-default}"
      echo "OK (${preset_label}, pub=${pub_count}, sub=${sub_count})"
      return 0
    fi
  done

  echo "NO DATA (pub=${pub_count}, sub=${sub_count}): QoSを追加調整するかpublisherが実際に送信しているか確認してください"
  return 1
}

overall=0
for t in "${topics[@]}"; do
  if ! check_topic "$t"; then
    overall=1
  fi
done

# TFチェック
check_tf() {
  local parent="$1"
  local child="$2"
  printf "checking TF %s -> %s ... " "$parent" "$child"
  if python3 - "$parent" "$child" "$TF_TIMEOUT" 2>/dev/null <<'PY'
import sys
import time
import os
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import Buffer, TransformListener

parent, child, timeout = sys.argv[1], sys.argv[2], float(sys.argv[3])
os.environ.setdefault("RCUTILS_LOGGING_SEVERITY_THRESHOLD", "ERROR")
rclpy.init(args=None)
node = rclpy.create_node('tf_check_temp')
buffer = Buffer(cache_time=Duration(seconds=10.0))
listener = TransformListener(buffer, node, spin_thread=False)
executor = SingleThreadedExecutor()
executor.add_node(node)

start = time.time()
success = False
try:
    while rclpy.ok() and time.time() - start < timeout:
        executor.spin_once(timeout_sec=0.1)
        if buffer.can_transform(parent, child, Time(), timeout=Duration(seconds=0.0)):
            try:
                buffer.lookup_transform(parent, child, Time())
                success = True
                break
            except Exception:
                pass
finally:
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

sys.exit(0 if success else 1)
PY
  then
    echo "OK"
  else
    echo "NO TF"
    overall=1
  fi
}

for pair in "${tf_pairs[@]}"; do
  # shellcheck disable=SC2086
  check_tf $pair
done

# /mcl_pose から姿勢を取得（x,y,yaw）
check_pose() {
  local topic="/mcl_pose"
  printf "checking %s pose ... " "$topic"
  set +e
  pose_output=$(python3 - "$topic" "$POSE_TIMEOUT" <<'PY'
import sys
import time
import math
import os
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped

topic, timeout = sys.argv[1], float(sys.argv[2])
os.environ.setdefault("RCUTILS_LOGGING_SEVERITY_THRESHOLD", "ERROR")

rclpy.init(args=None)
node = rclpy.create_node('pose_check_temp')
executor = SingleThreadedExecutor()
executor.add_node(node)

msg_box = {"msg": None}

def cb(msg):
    msg_box["msg"] = msg

node.create_subscription(PoseWithCovarianceStamped, topic, cb, 10)

start = time.time()
try:
    while rclpy.ok() and msg_box["msg"] is None and time.time() - start < timeout:
        executor.spin_once(timeout_sec=0.1)
finally:
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if msg_box["msg"] is None:
    print("NO POSE")
    sys.exit(1)

m = msg_box["msg"].pose.pose
q = m.orientation
sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
yaw = math.atan2(sin_yaw, cos_yaw)
y_min, y_max = 1.0, 2.2
x_min, x_max = 10.0, 11.0
yaw_min, yaw_max = -0.22, -0.02

in_range = (x_min <= m.position.x <= x_max) and (y_min <= m.position.y <= y_max) and (yaw_min <= yaw <= yaw_max)
if in_range:
    print(f"OK (x={m.position.x:.3f}, y={m.position.y:.3f}, yaw={yaw:.3f} rad)")
    sys.exit(0)
else:
    print(f"OUT_OF_RANGE (x={m.position.x:.3f} [{x_min:.1f}-{x_max:.1f}], "
          f"y={m.position.y:.3f} [{y_min:.1f}-{y_max:.1f}], "
          f"yaw={yaw:.3f} [{yaw_min:.2f}-{yaw_max:.2f}])")
    sys.exit(2)
PY
)
  pose_status=$?
  set -e
  echo "$pose_output"
  if [[ $pose_status -ne 0 ]]; then
    overall=1
  fi
}

check_pose

exit "$overall"
