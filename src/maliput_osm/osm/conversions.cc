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
#include <unordered_set>

using maliput_sparse::geometry::LineString3d;

namespace maliput_osm {
namespace osm {
namespace {

static constexpr bool kLeftBound{true};
static constexpr bool kRightBound{!kLeftBound};

// Gets the LaneEnd type of @p connected_ls linestring that connects to @ls linestring.
// @param ls The line string to get the maliput::api::LaneEnd::Which from.
// @param connected_ls The line string that shares the initial or end point with ls.
// @returns The LaneEnd::Which of the connected_ls linestring. If no shared point is found, returns std::nullopt.
std::optional<maliput::api::LaneEnd::Which> GetLaneEndWhich(const lanelet::ConstLineString3d& ls,
                                                            const lanelet::ConstLineString3d& connected_ls) {
  if (ls.front() == connected_ls.front() || ls.back() == connected_ls.front()) {
    return maliput::api::LaneEnd::Which::kStart;
  } else if (ls.front() == connected_ls.back() || ls.back() == connected_ls.back()) {
    return maliput::api::LaneEnd::Which::kFinish;
  }
  return std::nullopt;
}

// Returns the id and maliput::api::LaneEnd::Which pair for @p lanelet that is connected to the given @p line_string.
// @p left works as a hint to determine which lanelet is connected to the given @p line_string.
std::pair<Lane::Id, maliput::api::LaneEnd::Which> GetIdLaneEnd(const lanelet::ConstLanelet& lanelet,
                                                               const lanelet::ConstLineString3d& line_string,
                                                               bool left) {
  const Lane::Id lanelet_id = std::to_string(lanelet.id());
  auto lanelet_line_string_bound = left ? lanelet.leftBound() : lanelet.rightBound();
  auto lane_end_which = GetLaneEndWhich(line_string, lanelet_line_string_bound);
  if (!lane_end_which) {
    // If no lane end is found then lets try the opposite bound in case the lanelet is inverted.
    lanelet_line_string_bound = !left ? lanelet.leftBound() : lanelet.rightBound();
    lane_end_which = GetLaneEndWhich(line_string, lanelet_line_string_bound);
  }
  MALIPUT_THROW_UNLESS(lane_end_which.has_value());
  return std::make_pair(lanelet_id, *lane_end_which);
}

// Get Lanelets that share points to given line strings that are not @p lanelet
std::unordered_map<Lane::Id, maliput::api::LaneEnd::Which> GetLaneletsUsingLineStrings(
    const lanelet::Lanelet& lanelet, const lanelet::LineStrings3d& line_strings,
    const lanelet::LaneletLayer& lanelet_layer, bool left) {
  std::unordered_map<Lane::Id, maliput::api::LaneEnd::Which> lanelets_with_share_line_strings;
  for (const auto& line_string : line_strings) {
    if (line_string == lanelet.leftBound() || line_string == lanelet.rightBound()) {
      // Omitting the case where linestring belongs to the lanelet itself.
      continue;
    }
    const auto lanelets = lanelet_layer.findUsages(line_string);
    std::for_each(lanelets.begin(), lanelets.end(),
                  [&left, &lanelet_ = lanelet, &line_string, &lanelets_with_share_line_strings](const auto& lanelet) {
                    if (lanelet.id() != lanelet_.id()) {
                      lanelets_with_share_line_strings.insert(
                          GetIdLaneEnd(lanelet, (left ? lanelet_.leftBound() : lanelet_.rightBound()), left));
                    }
                  });
  }
  return lanelets_with_share_line_strings;
}

// Intersects unordered maps.
// Note: std::set_intersection doesn't seem to be working correctly for a map:
template <typename T, typename U>
std::unordered_map<T, U> IntersectMaps(const std::unordered_map<T, U>& map1, const std::unordered_map<T, U>& map2) {
  std::unordered_map<T, U> intersection;
  for (const auto& [key, value] : map1) {
    if (map2.find(key) != map2.end()) {
      intersection.insert({key, value});
    }
  }
  return intersection;
}

}  // namespace

LineString3d ToMaliput(const lanelet::ConstLineString3d& line_string) {
  std::vector<maliput::math::Vector3> points;
  std::transform(line_string.begin(), line_string.end(), std::back_inserter(points),
                 [](const auto& point) { return maliput::math::Vector3(point.x(), point.y(), point.z()); });
  return LineString3d{points};
}

Lane ToMaliput(const lanelet::Lanelet& lanelet, const lanelet::LaneletMapPtr& map) {
  // Get Id.
  const std::string id = std::to_string(lanelet.id());
  // Get left boundary.
  const LineString3d left_bound = ToMaliput(lanelet.leftBound());
  // Get right boundary.
  const LineString3d right_bound = ToMaliput(lanelet.rightBound());

  const auto map_layer = &(map->laneletLayer);
  // Obtains the lanelets that uses same @p bound linestring.
  auto find_usage_of_lane_bounds = [&lanelet, &map_layer](const lanelet::ConstLineString3d& bound) {
    const auto lanelets = map_layer->findUsages(bound);
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
  // Get right lane id.
  const std::optional<std::string> right_lane_id = find_usage_of_lane_bounds(lanelet.rightBound());

  const auto line_strings_layer = &(map->lineStringLayer);
  // Obtains the lanelets that are connected to @p left_point and @p right_point.
  auto find_usage_of_end_points = [&lanelet, &line_strings_layer, &map_layer](
                                      const lanelet::ConstPoint3d& left_point,
                                      const lanelet::ConstPoint3d& right_point) {
    // Line strings sharing the left point.
    const lanelet::LineStrings3d line_strings_sharing_left_point = line_strings_layer->findUsages(left_point);
    // Line strings sharing the right point.
    const lanelet::LineStrings3d line_strings_sharing_right_point = line_strings_layer->findUsages(right_point);
    MALIPUT_THROW_UNLESS(line_strings_sharing_left_point.size() >= 1);
    MALIPUT_THROW_UNLESS(line_strings_sharing_right_point.size() >= 1);

    // Obtains the lanelets that uses the left_point.
    std::unordered_map<Lane::Id, maliput::api::LaneEnd::Which> lanelets_for_left_point =
        GetLaneletsUsingLineStrings(lanelet, line_strings_sharing_left_point, *map_layer, kLeftBound);

    // Obtains the lanelets that uses the right_point.
    std::unordered_map<Lane::Id, maliput::api::LaneEnd::Which> lanelets_for_right_point =
        GetLaneletsUsingLineStrings(lanelet, line_strings_sharing_right_point, *map_layer, kRightBound);

    // Intersects both sets to obtain the lanelets that uses both points.
    const std::unordered_map<Lane::Id, maliput::api::LaneEnd::Which> lanelets_for_end_points{
        IntersectMaps(lanelets_for_left_point, lanelets_for_right_point)};

    return lanelets_for_end_points;
  };

  const auto predecessor_lanelets = find_usage_of_end_points(lanelet.leftBound().front(), lanelet.rightBound().front());
  const auto successor_lanelets = find_usage_of_end_points(lanelet.leftBound().back(), lanelet.rightBound().back());

  return {id, left_bound, right_bound, left_lane_id, right_lane_id, successor_lanelets, predecessor_lanelets};
}

}  // namespace osm
}  // namespace maliput_osm
