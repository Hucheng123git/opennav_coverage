#!/usr/bin/env bash
set -euo pipefail

coverage_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
coverage_repo_dir="$(cd -- "$coverage_script_dir/.." && pwd)"
coverage_map_dir="$coverage_repo_dir/maps/coverage_map"
coverage_build=true
coverage_build_only=false
coverage_rviz=true
coverage_jobs=2
coverage_launch_args=()

usage() {
  cat <<'EOF'
Usage: scripts/run_coverage_map_demo.sh [options] [-- launch_argument:=value ...]
  --map-dir PATH    Map directory (default: maps/coverage_map in this checkout)
  --underlay FILE   Source an additional Nav2 install/setup.bash
  --no-rviz        Start without RViz2
  --no-build       Use the existing .coverage_demo/humble/install
  --build-only     Build the three planning demo packages and exit
  --jobs N         Limit build parallelism (default: 2)

Examples:
  scripts/run_coverage_map_demo.sh
  scripts/run_coverage_map_demo.sh -- route_type:=BOUSTROPHEDON
  scripts/run_coverage_map_demo.sh -- route_type:=SNAKE auto_plan:=false
EOF
}

while (($#)); do
  case "$1" in
    --map-dir|--underlay|--jobs)
      if (($# < 2)); then echo "Missing value for $1" >&2; exit 2; fi
      case "$1" in
        --map-dir) coverage_map_dir="$2" ;;
        --underlay) export COVERAGE_NAV2_SETUP="$2" ;;
        --jobs) coverage_jobs="$2" ;;
      esac
      shift 2 ;;
    --no-rviz) coverage_rviz=false; shift ;;
    --no-build) coverage_build=false; shift ;;
    --build-only) coverage_build_only=true; shift ;;
    --help|-h) usage; exit 0 ;;
    --) shift; coverage_launch_args=("$@"); break ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done
if [[ ! "$coverage_jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo '--jobs must be a positive integer' >&2; exit 2
fi

source "$coverage_script_dir/coverage_demo_env.sh"
for coverage_package in nav2_util nav2_map_server nav2_lifecycle_manager; do
  if ! ros2 pkg prefix "$coverage_package" >/dev/null 2>&1; then
    echo "Missing ROS package $coverage_package. Source a Nav2 workspace with --underlay FILE." >&2
    exit 1
  fi
done

if "$coverage_build"; then
  coverage_cmake_args=(-DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release)
  if [[ -n "${Fields2Cover_DIR:-}" ]]; then
    if [[ ! -f "$Fields2Cover_DIR/Fields2CoverConfig.cmake" ]]; then
      echo "Missing $Fields2Cover_DIR/Fields2CoverConfig.cmake. Run scripts/setup_fields2cover.sh --ros-distro humble." >&2
      exit 1
    fi
  else
    echo 'Fields2Cover has no local installation. If CMake cannot find an existing installation,' >&2
    echo 'run scripts/setup_fields2cover.sh --ros-distro humble first (requires v1.2.1).' >&2
  fi
  export CMAKE_BUILD_PARALLEL_LEVEL="$coverage_jobs"
  colcon --log-base "$coverage_demo_dir/log" build \
    --base-paths "$coverage_repo_dir/opennav_coverage_msgs" \
      "$coverage_repo_dir/opennav_coverage" "$coverage_repo_dir/opennav_coverage_demo" \
    --build-base "$coverage_demo_dir/build" --install-base "$coverage_demo_dir/install" \
    --executor sequential --symlink-install \
    --packages-select opennav_coverage_msgs opennav_coverage opennav_coverage_demo \
    --cmake-args "${coverage_cmake_args[@]}"
  coverage_source_setup "$coverage_demo_dir/install/setup.bash"
fi
if "$coverage_build_only"; then exit 0; fi

coverage_map_dir="$(realpath -m -- "$coverage_map_dir")"
if [[ ! -f "$coverage_map_dir/field.json" ]]; then
  python3 "$coverage_script_dir/edit_coverage_map.py" --output "$coverage_map_dir" --example
fi
for coverage_file in map.yaml map.pgm; do
  if [[ ! -f "$coverage_map_dir/$coverage_file" ]]; then
    echo "Missing $coverage_map_dir/$coverage_file. Save this map again in the Python editor." >&2
    exit 1
  fi
done

echo "Map: $coverage_map_dir"
echo 'After saving edits: scripts/replan_coverage_map.sh'
exec ros2 launch opennav_coverage_demo coverage_map_launch.py \
  "map_directory:=$coverage_map_dir" "rviz:=$coverage_rviz" "${coverage_launch_args[@]}"
