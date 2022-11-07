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
#include "maliput_osm/osm/osm_manager.h"

#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <maliput/common/logger.h>

#include "maliput_osm/osm/conversions.h"

namespace maliput_osm {
namespace osm {
namespace {

// @param[out] connections
void AddToConnections(const Connection& connection, std::vector<Connection>* connections) {
  MALIPUT_THROW_UNLESS(connections);
  if (std::find(connections->begin(), connections->end(), connection) == connections->end()) {
    connections->push_back(connection);
  }
}

}  // namespace
OSMManager::OSMManager(const std::string& osm_file_path, const ParserConfig& config) {
  using namespace lanelet;
  const LaneletMapPtr map = load(osm_file_path, Origin{GPSPoint{config.origin.x(), config.origin.y()}});

  // Obtain lanelet lanes.
  std::unordered_map<Lane::Id, Lane> lanes;
  for (const auto& lanelet : map->laneletLayer) {
    const Lane lane = ToMaliput(lanelet, map);
    lanes.emplace(lane.id, lane);
  }

  // Fill up segment according their adjacency.
  for (const auto& lane : lanes) {
    const std::optional<Segment> segment = CreateSegmentForLane(lane.second, lanes);
    if (!segment.has_value()) {
      continue;
    }
    segments_.emplace(segment->id, std::move(segment.value()));
  }

  // Fill up connections.
  for (const auto& segment : segments_) {
    for (const auto& lane : segment.second.lanes) {
      for (const auto& predecessor : lane.predecessors) {
        const Connection connection{predecessor.second, {lane.id, LaneEnd::Which::kStart}};
        AddToConnections(connection, &connections_);
      }
      for (const auto& successor : lane.successors) {
        const Connection connection{{lane.id, LaneEnd::Which::kFinish}, successor.second};
        AddToConnections(connection, &connections_);
      }
    }
  }
}

OSMManager::~OSMManager() = default;

const std::unordered_map<Segment::Id, Segment>& OSMManager::GetOSMSegments() const { return segments_; }

const std::vector<osm::Connection>& OSMManager::GetOSMConnections() const { return connections_; }

std::optional<Segment> OSMManager::CreateSegmentForLane(const Lane& lane,
                                                        const std::unordered_map<Lane::Id, Lane>& lanes) {
  // If segment for this lane is already added then return.
  if (std::find_if(segments_.begin(), segments_.end(), [&lane](const auto& segment) {
        return std::find_if(segment.second.lanes.begin(), segment.second.lanes.end(),
                            [&lane_id = lane.id](const auto& seg_lane) { return seg_lane.id == lane_id; }) !=
               segment.second.lanes.end();
      }) != segments_.end()) {
    return std::nullopt;
  }

  // Create and populate segment for this lane.
  Segment segment;
  segment.id = segment_id_gen_();
  segment.lanes.emplace_back(lane);

  if (lane.left_lane_id.has_value()) {
    AddLanesToSegment(lane.left_lane_id.value(), lanes, kAdjacentLeft, &segment);
  }
  if (lane.right_lane_id.has_value()) {
    AddLanesToSegment(lane.right_lane_id.value(), lanes, kAdjacentRight, &segment);
  }
  return {segment};
}

void OSMManager::AddLanesToSegment(const Lane::Id& lane_id, const std::unordered_map<Lane::Id, Lane>& lanes, bool left,
                                   Segment* segment) {
  const auto lane = lanes.find(lane_id);
  MALIPUT_THROW_UNLESS(lane != lanes.end());
  // Add lane to segment.
  segment->lanes.insert(left ? segment->lanes.end() : segment->lanes.begin(), lane->second);
  // Check the following adjacent lane.
  const std::optional<Lane::Id>& next_adjacent_lane_id = left ? lane->second.left_lane_id : lane->second.right_lane_id;
  if (next_adjacent_lane_id.has_value()) {
    AddLanesToSegment(next_adjacent_lane_id.value(), lanes, left, segment);
  }
}

}  // namespace osm
}  // namespace maliput_osm
