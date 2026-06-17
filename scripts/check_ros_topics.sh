#!/usr/bin/env bash
set -euo pipefail

required_topics=(
  "/scan"
  "/odom"
  "/tf"
  "/tf_static"
  "/camera/color/image_raw"
  "/camera/depth/image_raw"
  "/camera/color/camera_info"
)

available="$(ros2 topic list || true)"

for topic in "${required_topics[@]}"; do
  if grep -qx "$topic" <<<"$available"; then
    echo "OK $topic"
  else
    echo "MISSING $topic"
  fi
done
