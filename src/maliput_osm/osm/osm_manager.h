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
#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <maliput/common/maliput_copyable.h>

#include "maliput_osm/osm/connection.h"
#include "maliput_osm/osm/junction.h"
#include "maliput_osm/osm/lane.h"
#include "maliput_osm/osm/segment.h"
#include "utilities/id_gen.h"

namespace maliput_osm {
namespace osm {

/// Configuration for the OSM parser.
struct ParserConfig {
  /// Lat and lon of the origin of the OSM map.
  maliput::math::Vector2 origin{0., 0.};
};

/// OSMManager is in charge of loading a Lanelet2-OSM map, parsing it, and providing
/// accessors to get the map's important data.
class OSMManager {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(OSMManager)

  /// Constructs a OSMManager object.
  /// @param osm_file_path The path to the OSM file to load.
  /// @param config The parser configuration.
  OSMManager(const std::string& osm_file_path, const ParserConfig& config);

  ~OSMManager();

  /// Gets the map's junctions.
  /// Junction's ids are generated as combination of the contained segment's ids connected by a "_"
  const std::unordered_map<Junction::Id, Junction>& GetOSMJunctions() const;

  /// Gets connections between the map's lanes.
  const std::vector<osm::Connection>& GetOSMConnections() const;

 private:
  // Convenient definitions for the AddLanesToSegment method's left parameter.
  // @{
  static constexpr bool kAdjacentLeft{true};
  static constexpr bool kAdjacentRight{!kAdjacentLeft};
  // @}

  // Recurrent method for finding the left or right adjacent lanes of a given @p lane_id and populate the @p segment.
  //
  // @param lane_id The lane_id to find the adjacent lanes of.
  // @param lanes The lanes to search in.
  // @param left Whether to search for the left or right adjacent lanes.
  // @param[out] segment The segment to be filled. The lanes vector is filled from rightest most lane to leftest most
  // lane.
  //
  // @throws maliput::common::assertion_error When the adjacent lane id doesn't match any lane in the @p lanes.
  static void AddLanesToSegment(const Lane::Id& lane_id, const std::unordered_map<Lane::Id, Lane>& lanes, bool left,
                                Segment* segment);

  // Creates the complete Segment for a given @p lane .
  // The graph is traversed using the lane.left_lane_id and lane.right_lane_id in order to complete the Segment.
  // When a lane already belongs to a segment then no segment is created and std::nullopt is returned.
  // @param lane The lane to create a Segment from.
  // @param lanes The map of all lanes.
  // @param segments The map of all segments to verify if @p lane already belongs to a segment.
  // @returns A populated Segment or std::nullopt when the lane already belongs to a segment.
  std::optional<Segment> CreateSegmentForLane(const Lane& lane, const std::unordered_map<Lane::Id, Lane>& lanes,
                                              const std::unordered_map<Segment::Id, Segment>& segments);

  // Id generator for the segments.
  utilities::IdGen<Segment::Id> segment_id_gen_;

  // Collection of junctions.
  std::unordered_map<Junction::Id, Junction> junctions_{};

  // Collection of connections;
  std::vector<osm::Connection> connections_{};
};

}  // namespace osm
}  // namespace maliput_osm
