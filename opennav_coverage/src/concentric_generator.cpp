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

#include "opennav_coverage/concentric_generator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "geos_c.h"

namespace opennav_coverage
{
namespace
{

constexpr double kPointEpsilon = 1e-9;
constexpr size_t kMaxLayers = 4096;
constexpr size_t kMaxContours = 16384;
constexpr size_t kMaxSwaths = 100000;
constexpr size_t kMaxPathStates = 1000000;
constexpr size_t kMaxVisibilityVertices = 2048;

struct Position
{
  double x;
  double y;
};

using Contour = std::vector<Position>;

double distance(const Position & first, const Position & second)
{
  return std::hypot(second.x - first.x, second.y - first.y);
}

void checkCanceled(const std::function<bool()> & canceled)
{
  if (canceled && canceled()) {
    throw std::runtime_error("Concentric planning canceled");
  }
}

void validateField(const Field & field)
{
  if (field.isEmpty() || field.size() == 0) {
    throw std::invalid_argument("Concentric coverage requires a nonempty polygon");
  }
  if (field.size() > kMaxContours) {
    throw std::invalid_argument("Concentric polygon exceeds the ring limit");
  }
  size_t total_vertices = 0;
  for (size_t ring_index = 0; ring_index < field.size(); ++ring_index) {
    const auto ring = field.getGeometry(ring_index);
    if (ring.size() < 4) {
      throw std::invalid_argument("Concentric polygon rings need at least four closed points");
    }
    if (ring.size() > kMaxSwaths - total_vertices) {
      throw std::invalid_argument("Concentric polygon exceeds the vertex limit");
    }
    total_vertices += ring.size();
    for (size_t point_index = 0; point_index < ring.size(); ++point_index) {
      if (!std::isfinite(ring.getX(point_index)) || !std::isfinite(ring.getY(point_index))) {
        throw std::invalid_argument("Concentric polygon contains a non-finite coordinate");
      }
    }
  }
  const double area = field.get()->get_Area();
  if (!std::isfinite(area) || area <= 0.0 || !field.get()->IsValid()) {
    throw std::invalid_argument("Concentric coverage requires a valid polygon with positive area");
  }
}

Contour readContour(const Polygon & ring, bool hole)
{
  Contour contour;
  for (size_t point_index = 0; point_index < ring.size(); ++point_index) {
    const Position point{ring.getX(point_index), ring.getY(point_index)};
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("Concentric offset produced a non-finite coordinate");
    }
    if (contour.empty() || distance(contour.back(), point) > kPointEpsilon) {
      contour.push_back(point);
    }
    if (contour.size() > kMaxSwaths) {
      throw std::invalid_argument("Concentric contour exceeds the vertex limit");
    }
  }
  if (contour.size() > 1 && distance(contour.front(), contour.back()) <= kPointEpsilon) {
    contour.pop_back();
  }
  if (contour.size() < 3) {
    return {};
  }
  long double twice_area = 0.0;
  for (size_t point_index = 0; point_index < contour.size(); ++point_index) {
    const auto & first = contour[point_index];
    const auto & second = contour[(point_index + 1) % contour.size()];
    const long double first_x = static_cast<long double>(first.x) - contour.front().x;
    const long double first_y = static_cast<long double>(first.y) - contour.front().y;
    const long double second_x = static_cast<long double>(second.x) - contour.front().x;
    const long double second_y = static_cast<long double>(second.y) - contour.front().y;
    twice_area += first_x * second_y - second_x * first_y;
  }
  if (std::abs(twice_area) <= kPointEpsilon * kPointEpsilon) {
    return {};
  }
  // Traverse exterior rings clockwise and hole rings counterclockwise.
  // The usable field remains to the right of either kind of contour.
  if ((!hole && twice_area > 0.0) || (hole && twice_area < 0.0)) {
    std::reverse(contour.begin(), contour.end());
  }
  return contour;
}

std::vector<Contour> readContours(const Fields & fields)
{
  std::vector<Contour> contours;
  for (size_t cell_index = 0; cell_index < fields.size(); ++cell_index) {
    const auto cell = fields.getGeometry(cell_index);
    for (size_t ring_index = 0; ring_index < cell.size(); ++ring_index) {
      auto contour = readContour(cell.getGeometry(ring_index), ring_index != 0);
      if (!contour.empty()) {
        contours.push_back(std::move(contour));
      }
    }
  }
  return contours;
}

struct GeosResources
{
  GEOSContextHandle_t context{GEOS_init_r()};
  GEOSGeometry * geometry{nullptr};
  const GEOSPreparedGeometry * prepared{nullptr};

  ~GeosResources()
  {
    if (prepared != nullptr) {
      GEOSPreparedGeom_destroy_r(context, prepared);
    }
    if (geometry != nullptr) {
      GEOSGeom_destroy_r(context, geometry);
    }
    if (context != nullptr) {
      GEOS_finish_r(context);
    }
  }
};

class ConnectorPlanner
{
public:
  explicit ConnectorPlanner(const Fields & free_space)
  : free_space_(free_space)
  {
    if (geos_.context == nullptr) {
      throw std::invalid_argument("Could not create the concentric geometry context");
    }
    geos_.geometry = free_space_.get()->exportToGEOS(geos_.context);
    if (geos_.geometry == nullptr) {
      throw std::invalid_argument("Could not prepare concentric free space");
    }
    geos_.prepared = GEOSPrepare_r(geos_.context, geos_.geometry);
    if (geos_.prepared == nullptr) {
      throw std::invalid_argument("Could not index concentric free space");
    }
  }

  bool covered(const Position & start, const Position & finish) const
  {
    OGRLineString line;
    line.addPoint(start.x, start.y);
    line.addPoint(finish.x, finish.y);
    GEOSGeometry * geometry = line.exportToGEOS(geos_.context);
    if (geometry == nullptr) {
      throw std::invalid_argument("Could not validate a concentric path segment");
    }
    const char result = GEOSPreparedCovers_r(geos_.context, geos_.prepared, geometry);
    GEOSGeom_destroy_r(geos_.context, geometry);
    if (result == 2) {
      throw std::invalid_argument("Concentric path containment check failed");
    }
    return result == 1;
  }

  std::vector<Position> route(
    const Position & start, const Position & finish,
    const std::function<bool()> & canceled)
  {
    if (distance(start, finish) <= kPointEpsilon) {
      return {start};
    }
    if (covered(start, finish)) {
      return {start, finish};
    }
    prepareVertices();
    std::vector<Position> positions = vertices_;
    const size_t start_index = positions.size();
    positions.push_back(start);
    const size_t finish_index = positions.size();
    positions.push_back(finish);
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> costs(positions.size(), infinity);
    std::vector<size_t> previous(positions.size(), positions.size());
    std::vector<bool> visited(positions.size(), false);
    using QueueEntry = std::pair<double, size_t>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
    costs[start_index] = 0.0;
    queue.emplace(0.0, start_index);

    while (!queue.empty()) {
      checkCanceled(canceled);
      const auto entry = queue.top();
      queue.pop();
      const size_t current_index = entry.second;
      if (visited[current_index]) {
        continue;
      }
      visited[current_index] = true;
      if (current_index == finish_index) {
        break;
      }
      for (size_t next_index = 0; next_index < positions.size(); ++next_index) {
        if (next_index % 64 == 0) {
          checkCanceled(canceled);
        }
        if (visited[next_index]) {
          continue;
        }
        const double candidate_cost = costs[current_index] +
          distance(positions[current_index], positions[next_index]);
        if (candidate_cost >= costs[next_index]) {
          continue;
        }
        bool visible;
        if (current_index < vertices_.size() && next_index < vertices_.size()) {
          auto & cached = visibility_[current_index * vertices_.size() + next_index];
          if (cached < 0) {
            cached = covered(positions[current_index], positions[next_index]) ? 1 : 0;
            visibility_[next_index * vertices_.size() + current_index] = cached;
          }
          visible = cached != 0;
        } else {
          visible = covered(positions[current_index], positions[next_index]);
        }
        if (visible) {
          costs[next_index] = candidate_cost;
          previous[next_index] = current_index;
          queue.emplace(candidate_cost, next_index);
        }
      }
    }
    if (!visited[finish_index]) {
      throw std::invalid_argument(
              "Concentric contours cannot be connected inside clearance free space");
    }
    std::vector<Position> connector;
    for (size_t index = finish_index; ; index = previous[index]) {
      connector.push_back(positions[index]);
      if (index == start_index) {
        break;
      }
    }
    std::reverse(connector.begin(), connector.end());
    return connector;
  }

private:
  void prepareVertices()
  {
    if (!vertices_.empty()) {
      return;
    }
    std::set<std::pair<double, double>> unique;
    for (const auto & contour : readContours(free_space_)) {
      for (const auto & position : contour) {
        if (unique.emplace(position.x, position.y).second) {
          vertices_.push_back(position);
          if (vertices_.size() > kMaxVisibilityVertices) {
            throw std::invalid_argument("Concentric visibility graph exceeds the vertex limit");
          }
        }
      }
    }
    visibility_.assign(vertices_.size() * vertices_.size(), -1);
  }

  Fields free_space_;
  GeosResources geos_;
  std::vector<Position> vertices_;
  std::vector<int8_t> visibility_;
};

void appendState(Path & path, const Position & position, double angle, bool work)
{
  if (path.states.size() >= kMaxPathStates) {
    throw std::invalid_argument("Concentric path exceeds the point limit");
  }
  PathState state{};
  state.point = Point(position.x, position.y);
  state.angle = angle;
  state.velocity = 0.0;
  state.duration = 0.0;
  state.dir = f2c::types::PathDirection::FORWARD;
  state.type = work ? f2c::types::PathSectionType::SWATH : f2c::types::PathSectionType::TURN;
  path.states.push_back(std::move(state));
}

void appendConnector(
  Path & path, const std::vector<Position> & connector, double point_distance,
  double following_angle, const std::function<bool()> & canceled)
{
  std::vector<Position> points{connector.front()};
  for (size_t segment_index = 1; segment_index < connector.size(); ++segment_index) {
    checkCanceled(canceled);
    const auto & start = connector[segment_index - 1];
    const auto & finish = connector[segment_index];
    const double segment_length = distance(start, finish);
    if (segment_length <= kPointEpsilon) {
      continue;
    }
    const double required = std::ceil(segment_length / point_distance);
    if (!std::isfinite(required) || required >
      static_cast<double>(kMaxPathStates - points.size()))
    {
      throw std::invalid_argument("Concentric connector exceeds the point limit");
    }
    const size_t subdivisions = std::max<size_t>(1, static_cast<size_t>(required));
    for (size_t sample_index = 1; sample_index <= subdivisions; ++sample_index) {
      if (sample_index % 256 == 0) {
        checkCanceled(canceled);
      }
      const double fraction = static_cast<double>(sample_index) / subdivisions;
      points.push_back({
        start.x + fraction * (finish.x - start.x),
        start.y + fraction * (finish.y - start.y)});
    }
  }
  if (points.size() > kMaxPathStates - path.states.size()) {
    throw std::invalid_argument("Concentric path exceeds the point limit");
  }
  for (size_t point_index = 0; point_index < points.size(); ++point_index) {
    double angle = following_angle;
    if (point_index + 1 < points.size()) {
      angle = std::atan2(
        points[point_index + 1].y - points[point_index].y,
        points[point_index + 1].x - points[point_index].x);
    }
    appendState(path, points[point_index], angle, false);
  }
}

std::pair<size_t, size_t> chooseEntry(
  const std::vector<Contour> & contours, const Position & current, bool have_current)
{
  size_t contour_index = 0;
  size_t entry_index = 0;
  if (!have_current) {
    const auto & first_contour = contours.front();
    for (size_t index = 1; index < first_contour.size(); ++index) {
      if (std::make_pair(first_contour[index].x, first_contour[index].y) <
        std::make_pair(first_contour[entry_index].x, first_contour[entry_index].y))
      {
        entry_index = index;
      }
    }
    return {contour_index, entry_index};
  }
  double nearest = std::numeric_limits<double>::infinity();
  for (size_t ring_index = 0; ring_index < contours.size(); ++ring_index) {
    for (size_t point_index = 0; point_index < contours[ring_index].size(); ++point_index) {
      const double candidate = distance(current, contours[ring_index][point_index]);
      if (candidate < nearest) {
        nearest = candidate;
        contour_index = ring_index;
        entry_index = point_index;
      }
    }
  }
  return {contour_index, entry_index};
}

}  // namespace

ConcentricPlan ConcentricGenerator::generate(
  const Field & field, double spacing, double initial_inset,
  double clearance, double point_distance,
  std::function<bool()> canceled, bool connect) const
{
  if (!std::isfinite(spacing) || spacing <= 0.0 ||
    !std::isfinite(initial_inset) || initial_inset < 0.0 ||
    !std::isfinite(clearance) || clearance < 0.0 ||
    !std::isfinite(point_distance) || point_distance <= 0.0)
  {
    throw std::invalid_argument("Concentric distances must be finite and positive or nonnegative");
  }
  if (initial_inset < clearance) {
    throw std::invalid_argument("Concentric initial inset must be at least the robot clearance");
  }
  checkCanceled(canceled);
  validateField(field);
  std::unique_ptr<ConnectorPlanner> connector_planner;
  if (connect) {
    const Fields free_space = Fields::Buffer(field, -clearance);
    if (free_space.isEmpty() || free_space.size() == 0) {
      throw std::invalid_argument("Robot clearance leaves no concentric free space");
    }
    if (free_space.size() > 1) {
      throw std::invalid_argument(
              "Robot clearance splits the field into disconnected concentric free spaces");
    }
    connector_planner = std::make_unique<ConnectorPlanner>(free_space);
  }

  ConcentricPlan plan;
  Position current{0.0, 0.0};
  bool have_current = false;
  bool finished = false;
  for (size_t level = 0; level < kMaxLayers; ++level) {
    checkCanceled(canceled);
    const double inset = initial_inset + static_cast<double>(level) * spacing;
    if (!std::isfinite(inset)) {
      throw std::invalid_argument("Concentric inset distance overflowed");
    }
    const Fields level_fields = Fields::Buffer(field, -inset);
    auto contours = readContours(level_fields);
    if (contours.empty()) {
      finished = true;
      break;
    }
    if (contours.size() > kMaxContours - plan.contour_count) {
      throw std::invalid_argument("Concentric coverage exceeds the contour limit");
    }
    while (!contours.empty()) {
      checkCanceled(canceled);
      const auto entry = chooseEntry(contours, current, have_current);
      auto contour = std::move(contours[entry.first]);
      contours.erase(contours.begin() + entry.first);
      std::rotate(contour.begin(), contour.begin() + entry.second, contour.end());
      ++plan.contour_count;
      for (size_t edge_index = 0; edge_index < contour.size(); ++edge_index) {
        checkCanceled(canceled);
        const auto & start = contour[edge_index];
        const auto & finish = contour[(edge_index + 1) % contour.size()];
        if (plan.swaths.size() >= kMaxSwaths) {
          throw std::invalid_argument("Concentric coverage exceeds the work-edge limit");
        }
        const double angle = std::atan2(finish.y - start.y, finish.x - start.x);
        if (connect) {
          if (!connector_planner->covered(start, finish)) {
            throw std::invalid_argument("Concentric work edge exceeds clearance free space");
          }
          if (have_current) {
            const auto connector = connector_planner->route(current, start, canceled);
            appendConnector(plan.path, connector, point_distance, angle, canceled);
          }
          appendState(plan.path, start, angle, true);
          appendState(plan.path, finish, angle, true);
        }
        LineString edge;
        edge.addPoint(start.x, start.y);
        edge.addPoint(finish.x, finish.y);
        plan.swaths.emplace_back(edge, spacing, static_cast<int>(plan.swaths.size()));
        current = finish;
        have_current = true;
      }
    }
  }
  if (!finished) {
    throw std::invalid_argument("Concentric coverage exceeds the offset-layer limit");
  }
  if (plan.contour_count == 0) {
    throw std::invalid_argument("Initial inset leaves no concentric coverage contours");
  }
  return plan;
}

}  // namespace opennav_coverage
