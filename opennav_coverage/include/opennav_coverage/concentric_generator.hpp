// Copyright (c) 2026 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OPENNAV_COVERAGE__CONCENTRIC_GENERATOR_HPP_
#define OPENNAV_COVERAGE__CONCENTRIC_GENERATOR_HPP_

#include <cstddef>
#include <functional>

#include "opennav_coverage/types.hpp"

namespace opennav_coverage
{

struct ConcentricPlan
{
  Swaths swaths;
  Path path;
  size_t contour_count{0};
};

/**
 * @brief Generate successively inset contours and collision-checked connections.
 *
 * Distances are in the field's Cartesian units.  The result is a geometric
 * polyline for a robot that can turn in place; it does not enforce a minimum
 * turning radius.  Every contour edge is a separate work swath, and TURN states
 * separate work edges, including edges sharing a contour vertex.
 */
class ConcentricGenerator
{
public:
  /**
   * @brief Generate all outer and inner contours of every offset component.
   * @param field Valid polygon, including its holes, in a Cartesian frame.
   * @param spacing Positive distance between consecutive contour levels.
   * @param initial_inset Distance of the first level from the original boundary.
   * @param clearance Distance to retain from all original field boundaries.
   * @param point_distance Maximum spacing of points on connection polylines.
   * @param canceled Optional callback; a true result interrupts planning.
   * @param connect Generate a continuous path, or only the contour swaths.
   * @throws std::invalid_argument Invalid geometry, limits, or unreachable path.
   * @throws std::runtime_error Planning was canceled.
   */
  ConcentricPlan generate(
    const Field & field, double spacing, double initial_inset,
    double clearance, double point_distance,
    std::function<bool()> canceled = {}, bool connect = true) const;
};

}  // namespace opennav_coverage

#endif  // OPENNAV_COVERAGE__CONCENTRIC_GENERATOR_HPP_
