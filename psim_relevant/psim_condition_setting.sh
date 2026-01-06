#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_CONFIG="${SCRIPT_DIR}/teleport_to_miraikan.yaml"

CONFIG=""

usage() {
  echo "Usage: $0 [-c <config|config.yaml|config.yml>]"
  echo "  default: ${DEFAULT_CONFIG}"
  exit 2
}

# -c が無ければ DEFAULT_CONFIG
while getopts ":c:h" opt; do
  case "$opt" in
    c) CONFIG="$OPTARG" ;;
    h) usage ;;
    *) usage ;;
  esac
done
CONFIG="${CONFIG:-$DEFAULT_CONFIG}"

resolve_config() {
  local in="$1"

  # 1) 指定されたものがそのまま存在
  [[ -f "$in" ]] && { echo "$in"; return 0; }

  # 2) 拡張子なしっぽいなら補完して探す
  if [[ "$in" != *.* ]]; then
    [[ -f "${in}.yaml" ]] && { echo "${in}.yaml"; return 0; }
    [[ -f "${in}.yml"  ]] && { echo "${in}.yml";  return 0; }

    # 3) スクリプト dir 基準でも探す（任意）
    [[ -f "${SCRIPT_DIR}/${in}.yaml" ]] && { echo "${SCRIPT_DIR}/${in}.yaml"; return 0; }
    [[ -f "${SCRIPT_DIR}/${in}.yml"  ]] && { echo "${SCRIPT_DIR}/${in}.yml";  return 0; }
  fi

  # 4) デフォルト名だけは最後に拾う（任意）
  [[ -f "$DEFAULT_CONFIG" ]] && { echo "$DEFAULT_CONFIG"; return 0; }

  echo "Config not found: $in" >&2
  exit 1
}

CONFIG="$(resolve_config "$CONFIG")"
echo "Using config: $CONFIG"

get() { yq -r "$1" "$CONFIG"; }

FRAME_ID="$(get '.frame_id // "map"')"
SLEEP_SEC="$(get '.sleep_sec // 1')"
USE_SIM_TIME="$(get '.use_sim_time // false')"

INIT_X="$(get '.initialpose.position.x')"
INIT_Y="$(get '.initialpose.position.y')"
INIT_Z="$(get '.initialpose.position.z')"
INIT_QX="$(get '.initialpose.orientation.x')"
INIT_QY="$(get '.initialpose.orientation.y')"
INIT_QZ="$(get '.initialpose.orientation.z')"
INIT_QW="$(get '.initialpose.orientation.w')"

GOAL_X="$(get '.goal.position.x')"
GOAL_Y="$(get '.goal.position.y')"
GOAL_Z="$(get '.goal.position.z')"
GOAL_QX="$(get '.goal.orientation.x')"
GOAL_QY="$(get '.goal.orientation.y')"
GOAL_QZ="$(get '.goal.orientation.z')"
GOAL_QW="$(get '.goal.orientation.w')"

now_sec_nsec() {
  if [[ "${USE_SIM_TIME}" == "true" ]]; then
    local clock sec nsec
    clock="$(ros2 topic echo /clock --once --field clock 2>/dev/null || true)"
    sec="$(awk '/sec:/ {print $2}' <<<"$clock" | head -n1)"
    nsec="$(awk '/nanosec:/ {print $2}' <<<"$clock" | head -n1)"
    if [[ -n "${sec}" && -n "${nsec}" ]]; then
      echo "${sec} ${nsec}"; return 0
    fi
  fi
  echo "$(date +%s) $(date +%N)"
}

read SEC NSEC < <(now_sec_nsec)
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$(cat <<EOF
header: {frame_id: ${FRAME_ID}, stamp: {sec: ${SEC}, nanosec: ${NSEC}}}
pose:
  pose:
    position: {x: ${INIT_X}, y: ${INIT_Y}, z: ${INIT_Z}}
    orientation: {x: ${INIT_QX}, y: ${INIT_QY}, z: ${INIT_QZ}, w: ${INIT_QW}}
  covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06853891909122467]
EOF
)"

sleep "${SLEEP_SEC}"

read SEC NSEC < <(now_sec_nsec)
ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/msg/PoseStamped "$(cat <<EOF
header: {frame_id: ${FRAME_ID}, stamp: {sec: ${SEC}, nanosec: ${NSEC}}}
pose:
  position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: ${GOAL_Z}}
  orientation: {x: ${GOAL_QX}, y: ${GOAL_QY}, z: ${GOAL_QZ}, w: ${GOAL_QW}}
EOF
)"