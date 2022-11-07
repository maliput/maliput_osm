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

#include <optional>
#include <string>
#include <unordered_set>

#include <maliput_sparse/geometry/line_string.h>

namespace maliput_osm {
namespace osm {

struct LaneEnd {
  /// Labels for the endpoints of a lanelet.
  enum class Which {
    kStart,   //> The start of the lanelet.
    kFinish,  //> The end of the lanelet.
  };
  bool operator==(const LaneEnd& other) const;

  // The lanelet id.
  std::string lane_id;
  // The endpoint of the lanelet.
  Which end;
};

/// A lane in an Lanelet2-OSM road.
struct Lane {
  using Id = std::string;

  /// Equality operator.
  /// @param other The other object to compare against.
  bool operator==(const Lane& other) const;

  /// Id of the lane.
  Id id{};
  /// The lane's left boundary.
  maliput_sparse::geometry::LineString3d left;
  /// The lane's right boundary.
  maliput_sparse::geometry::LineString3d right;
  /// The id of the lane to the left of this lane.
  std::optional<Id> left_lane_id;
  /// The id of the lane to the right of this lane.
  std::optional<Id> right_lane_id;
  /// The ids of the lanes that follow this lane.
  std::unordered_map<Id, LaneEnd> successors;
  /// The ids of the lanes that precede this lane.
  std::unordered_map<Id, LaneEnd> predecessors;
};

}  // namespace osm
}  // namespace maliput_osm
