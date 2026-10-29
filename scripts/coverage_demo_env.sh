#!/usr/bin/env bash
# Source this file from the demo scripts; it does not modify the user's shell.

coverage_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
coverage_demo_dir="$coverage_repo_dir/.coverage_demo/humble"

if [[ ! -f /.dockerenv && ! -f /run/.containerenv ]]; then
  echo '请先执行 /home/hucheng/shell/navigation_sim.sh，在 Docker 容器内编译和运行。' >&2
  return 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '容器内没有找到 /opt/ros/humble/setup.bash。' >&2
  return 1
fi
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ]]; then
  echo "当前环境 ROS_DISTRO=$ROS_DISTRO，请在新的 Humble Docker shell 中运行。" >&2
  return 1
fi

coverage_source_setup() {
  # ROS setup scripts may read unset variables, so temporarily disable nounset.
  local coverage_had_nounset=false
  [[ $- == *u* ]] && coverage_had_nounset=true
  set +u
  source "$1"
  if "$coverage_had_nounset"; then set -u; fi
}

coverage_source_setup /opt/ros/humble/setup.bash

# Interactive container shells select Cyclone DDS, while noninteractive shells
# may leave RMW_IMPLEMENTATION unset. Keep the launcher and service client on
# the same middleware in both cases.
if [[ -n "${COVERAGE_RMW_IMPLEMENTATION:-}" ]]; then
  export RMW_IMPLEMENTATION="$COVERAGE_RMW_IMPLEMENTATION"
elif [[ -z "${RMW_IMPLEMENTATION:-}" ]]; then
  if ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  else
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  fi
fi

if [[ -n "${COVERAGE_NAV2_SETUP:-}" ]]; then
  if [[ ! -f "$COVERAGE_NAV2_SETUP" ]]; then
    echo "Nav2 setup not found: $COVERAGE_NAV2_SETUP" >&2
    return 1
  fi
  coverage_source_setup "$COVERAGE_NAV2_SETUP"
fi

if [[ -f "$coverage_repo_dir/.deps/fields2cover-env.bash" ]]; then
  if grep -Eq '^export OPENNAV_COVERAGE_FIELDS2COVER_ROS_DISTRO="?humble"?$' \
      "$coverage_repo_dir/.deps/fields2cover-env.bash"; then
    coverage_source_setup "$coverage_repo_dir/.deps/fields2cover-env.bash"
  else
    echo '本地 Fields2Cover 不是 Humble 版本，请在容器内运行 scripts/setup_fields2cover.sh --ros-distro humble。' >&2
    return 1
  fi
fi

if [[ -f "$coverage_demo_dir/install/setup.bash" ]]; then
  coverage_source_setup "$coverage_demo_dir/install/setup.bash"
fi

# Keep ROS CLI/launch logs in the writable demo directory.
export ROS_LOG_DIR="$coverage_demo_dir/ros_log"
mkdir -p "$ROS_LOG_DIR"
