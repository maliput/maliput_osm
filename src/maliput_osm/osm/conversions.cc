// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "maliput_osm/osm/conversions.h"

#include <algorithm>
#include <optional>
#include <string>

using maliput_sparse::geometry::LineString3d;

namespace maliput_osm {
namespace osm {

LineString3d ToMaliput(const lanelet::ConstLineString3d& line_string) {
  std::vector<maliput::math::Vector3> points;
  std::transform(line_string.begin(), line_string.end(), std::back_inserter(points),
                 [](const auto& point) { return maliput::math::Vector3(point.x(), point.y(), point.z()); });
  return LineString3d{points};
}

Lane ToMaliput(const lanelet::Lanelet& lanelet, const lanelet::LaneletLayer& map_layer) {
  // Get Id.
  const std::string id = std::to_string(lanelet.id());
  // Get left boundary.
  const LineString3d left_bound = ToMaliput(lanelet.leftBound());
  // Get right boundary.
  const LineString3d right_bound = ToMaliput(lanelet.rightBound());

  // Obtains the lanelets that uses same @p bound linestring.
  auto find_usage_of_lane_bounds = [&lanelet, &map_layer](const lanelet::ConstLineString3d& bound) {
    const auto lanelets = map_layer.findUsages(bound);
    MALIPUT_THROW_UNLESS(lanelets.size() >= 1);
    MALIPUT_THROW_UNLESS(lanelets.size() <= 2);
    const auto lanelet_it = std::find_if(lanelets.begin(), lanelets.end(), [&lanelet](const auto& lanelet_usage) {
      return lanelet_usage.id() != lanelet.id();
    });
    return lanelet_it != lanelets.end() ? std::make_optional<std::string>(std::to_string(lanelet_it->id()))
                                        : std::nullopt;
  };
  // Get left lane id.
  const std::optional<std::string> left_lane_id = find_usage_of_lane_bounds(lanelet.leftBound());
  // Get left right id.
  const std::optional<std::string> right_lane_id = find_usage_of_lane_bounds(lanelet.rightBound());

  // TODO(#26): Find the successor and predecessor lanelets.
  return {id, left_bound, right_bound, left_lane_id, right_lane_id, {} /* successors */, {} /* predecessors */};
}

}  // namespace osm
}  // namespace maliput_osm
