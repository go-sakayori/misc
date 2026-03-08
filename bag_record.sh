#!/usr/bin/env bash
set -euo pipefail

# Edit this path when changing the ROS 2 setup file to source.
SOURCE_SETUP_PATH="${HOME}/pilot-auto/install/setup.bash"
OUTPUT_DIR="${HOME}/rosbag"
FILE_PREFIX="rosbag"

if [[ "$#" -gt 1 ]]; then
  echo "Usage: $0 [prefix]" >&2
  exit 1
fi

if [[ "$#" -eq 1 ]]; then
  FILE_PREFIX="$1"
fi

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_PATH="${OUTPUT_DIR}/${FILE_PREFIX}_${TIMESTAMP}"

if [[ ! -f "${SOURCE_SETUP_PATH}" ]]; then
  echo "Error: setup file not found: ${SOURCE_SETUP_PATH}" >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "${SOURCE_SETUP_PATH}"
set -u

mkdir -p "${OUTPUT_DIR}"

echo "Starting rosbag record:"

exec ros2 bag record -a --storage mcap -o "${OUTPUT_PATH}"
