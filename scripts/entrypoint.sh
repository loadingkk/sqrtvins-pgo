#!/usr/bin/env bash
set -eo pipefail
arguments=("$@")
set --
source /opt/ros/noetic/setup.bash
source /opt/ws/devel/setup.bash
set -- "${arguments[@]}"
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
export GEOMETRY_MODEL=gravity4
case "${1:-}" in
  test) exec /opt/ws/devel/lib/loop_fusion/test_gravity ;;
  *) exec python3 /opt/sqrtvins-pgo/scripts/pipeline.py "$@" ;;
esac
