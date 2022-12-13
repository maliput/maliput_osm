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

#include <unordered_set>

#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <maliput/common/logger.h>

#include "maliput_osm/osm/conversions.h"

namespace maliput_osm {
namespace osm {

using maliput_sparse::parser::Connection;
using maliput_sparse::parser::Junction;
using maliput_sparse::parser::Lane;
using maliput_sparse::parser::LaneEnd;
using maliput_sparse::parser::Segment;

namespace {

// @param[out] connections
void AddToConnections(const Connection& connection, std::vector<Connection>* connections) {
  MALIPUT_THROW_UNLESS(connections);
  if (std::find(connections->begin(), connections->end(), connection) == connections->end()) {
    connections->push_back(connection);
  }
}

bool IsPotentialComplexJunction(const std::unordered_map<Lane::Id, LaneEnd>& connections) {
  return connections.size() >= 2;
}

void AddLanesToCollectionOfSet(const std::unordered_map<Lane::Id, LaneEnd>& connections,
                               std::vector<std::unordered_set<std::string>>* sets) {
  MALIPUT_THROW_UNLESS(sets);
  // Verify if one of the connections is already added to a set. If so, then add the other connection to the same set.
  // Otherwise create a new set.
  std::vector<std::unordered_set<std::string>>::iterator set_it = sets->end();
  for (const auto& connection : connections) {
    set_it = std::find_if(sets->begin(), sets->end(), [&connection](const std::unordered_set<std::string>& set) {
      return set.find(connection.first) != set.end();
    });
  };
  for (const auto& connection : connections) {
    if (set_it == sets->end()) {
      sets->push_back({connection.first});
      set_it = sets->end() - 1;
    } else {
      set_it->emplace(connection.first);
    }
  }
}

// Given a lane find the segment that contains it.
std::pair<Segment::Id, Segment> FindSegmentForLane(const Lane::Id& lane_id,
                                                   const std::unordered_map<Segment::Id, Segment>& segments) {
  const auto segment_it =
      std::find_if(segments.begin(), segments.end(), [&lane_id](const std::pair<Segment::Id, Segment>& segment) {
        return std::find_if(segment.second.lanes.begin(), segment.second.lanes.end(),
                            [&lane_id](const auto& lane) { return lane.id == lane_id; }) != segment.second.lanes.end();
      });
  return *segment_it;
}

// Join the sets that share a common segment.
void JoinSetsIfSharedValues(std::vector<std::unordered_set<std::string>>* sets) {
  auto join_sets = [](std::vector<std::unordered_set<std::string>>* sets) {
    bool joined = false;
    for (auto& set : *sets) {
      auto set_it = std::find_if(sets->begin(), sets->end(), [&set](const std::unordered_set<std::string>& s) {
        return std::find_if(set.begin(), set.end(),
                            [&s](const std::string& value) { return s.find(value) != s.end(); }) != set.end();
      });
      if (set_it != sets->end() && (*set_it != set)) {
        set.insert(set_it->begin(), set_it->end());
        set_it->clear();
        joined = true;
      }
    }
    if (joined) {
      // Remove empty sets.
      sets->erase(std::remove_if(sets->begin(), sets->end(),
                                 [](const std::unordered_set<std::string>& set) { return set.empty(); }),
                  sets->end());
    }
    return joined;
  };

  while (join_sets(sets)) {
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

  std::unordered_map<Segment::Id, Segment> segments{};

  // Fill up segment according their adjacency.
  for (const auto& lane : lanes) {
    const std::optional<Segment> segment = CreateSegmentForLane(lane.second, lanes, segments);
    if (!segment.has_value()) {
      continue;
    }
    segments.emplace(segment->id, std::move(segment.value()));
  }

  // Fill up connections.
  for (const auto& segment : segments) {
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

  // Organize segments into junctions.
  std::vector<std::unordered_set<std::string>> potential_junction_lanes_set;
  // Group lanes that belong to a complex junction.
  for (const auto& segment : segments) {
    for (const auto& lane : segment.second.lanes) {
      if (IsPotentialComplexJunction(lane.predecessors)) {
        // check the lanes in the predecessor and add them to the potential_junction_lanes_set
        AddLanesToCollectionOfSet(lane.predecessors, &potential_junction_lanes_set);
      }
      if (IsPotentialComplexJunction(lane.successors)) {
        // check the lanes in the successor and add them to the potential_junction_lanes_set
        AddLanesToCollectionOfSet(lane.successors, &potential_junction_lanes_set);
      }
    }
  }

  // Join the potential junctions that share segments.
  JoinSetsIfSharedValues(&potential_junction_lanes_set);

  // Create junctions for complex connections.
  for (const auto& set : potential_junction_lanes_set) {
    Junction junction;
    Junction::Id junction_id{""};
    for (const auto& lane_id : set) {
      const std::pair<Segment::Id, Segment> id_segment = FindSegmentForLane(lane_id, segments);  // TODO this method.
      junction.segments.emplace(id_segment);
      junction.id = junction.id + (junction.id.empty() ? "" : "_") + id_segment.first;
    }
    junctions_.insert({junction.id, junction});
  }

  // Create junctions for the rest of the segments.
  for (const auto& segment : segments) {
    if (std::find_if(junctions_.begin(), junctions_.end(),
                     [&segment](const std::pair<Junction::Id, Junction>& junction) {
                       return junction.second.segments.find(segment.first) != junction.second.segments.end();
                     }) == junctions_.end()) {
      junctions_.insert({segment.first, {segment.first, {segment}}});
    }
  }
}

OSMManager::~OSMManager() = default;

const std::unordered_map<Junction::Id, Junction>& OSMManager::DoGetJunctions() const { return junctions_; }

const std::vector<osm::Connection>& OSMManager::DoGetConnections() const { return connections_; }

std::optional<Segment> OSMManager::CreateSegmentForLane(const Lane& lane,
                                                        const std::unordered_map<Lane::Id, Lane>& lanes,
                                                        const std::unordered_map<Segment::Id, Segment>& segments) {
  // If segment for this lane is already added then return.
  if (std::find_if(segments.begin(), segments.end(), [&lane](const auto& segment) {
        return std::find_if(segment.second.lanes.begin(), segment.second.lanes.end(),
                            [&lane_id = lane.id](const auto& seg_lane) { return seg_lane.id == lane_id; }) !=
               segment.second.lanes.end();
      }) != segments.end()) {
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
