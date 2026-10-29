#!/usr/bin/env python3
# Copyright (c) 2026 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run the map editor directly from the checkout, before building ROS packages."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'opennav_coverage_demo'))

from opennav_coverage_demo.edit_coverage_map import main  # noqa: E402


if __name__ == '__main__':
    sys.exit(main([
        '--output', str(Path(__file__).resolve().parents[1] / 'maps' / 'coverage_map'),
        *sys.argv[1:],
    ]))
