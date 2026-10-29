#!/usr/bin/env bash
set -euo pipefail
coverage_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$coverage_script_dir/coverage_demo_env.sh"
exec ros2 service call /coverage_map/plan std_srvs/srv/Trigger '{}'
