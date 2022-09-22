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

#include <map>
#include <memory>
#include <string>

#include <maliput/api/type_specific_identifier.h>
#include <maliput/common/assertion_error.h>
#include <maliput/common/maliput_copyable.h>
#include <maliput_sparse/geometry/line_string.h>

#include "maliput_osm/osm/lane.h"

namespace maliput_osm {
namespace osm {

using SegmentId = maliput::api::TypeSpecificIdentifier<class Segment>;

/// Abstraction for a road segment obtained from a osm map.
/// A segment is a collection of lanes added with a strict order, from right to left,
/// similarly to maliput::api::Segment abstraction.
///
/// A lanelet2-osm based map is composed by lanelets that could have adjacent lanes. The collection
/// of adjacent lanes are expected to be grouped in the Segment abstraction.
class Segment {
 public:
  /// @param id Id of the lane.
  /// @param lanes Lanes in the segment added from left to right.
  /// @throws maliput::common::assertion_error When @p lanes is empty.
  Segment(const SegmentId& id, const std::map<LaneId, Lane>& lanes) : id_(id), lanes_(lanes) {
    MALIPUT_THROW_UNLESS(!lanes_.empty());
  }

  /// @param id Id of the lane.
  /// @param lanes Lanes in the segment added from right to left.
  /// @param successors Ids of the lanes that are successors of this lane.
  /// @param predecessors Ids of the lanes that are predecessors of this lane.
  /// @throws maliput::common::assertion_error When @p lanes is empty.
  Segment(const SegmentId& id, const std::map<LaneId, Lane>& lanes, const std::vector<SegmentId>& successors,
          const std::vector<SegmentId>& predecessors)
      : id_(id), lanes_(lanes), successors_(successors), predecessors_(predecessors) {
    MALIPUT_THROW_UNLESS(!lanes_.empty());
  }

  /// Returns the id of the segment.
  const SegmentId& id() const { return id_; }
  /// Returns the lanes of the segment.
  const std::map<LaneId, Lane>& lanes() const { return lanes_; }

  /// Set the ids of the lanes that succeed this lane.
  /// @param successors Successors.
  void set_successors(const std::vector<SegmentId>& successors) { successors_ = successors; }
  /// Set the ids of the lanes that precede this lane.
  /// @param predecessors Predecessors.
  void set_predecessors(const std::vector<SegmentId>& predecessors) { predecessors_ = predecessors; }
  /// @returns The ids of the lanes that succeed this lane.
  const std::vector<SegmentId>& get_successors() const { return successors_; }
  /// @returns The ids of the lanes that precede this lane.
  const std::vector<SegmentId>& get_predecessors() const { return predecessors_; }

  /// Equality operator.
  /// @param other The other object to compare against.
  bool operator==(const Segment& other) const {
    return id_ == other.id_ && lanes_ == other.lanes_ && successors_ == other.successors_ &&
           predecessors_ == other.predecessors_;
  }

 private:
  const SegmentId id_;
  const std::map<LaneId, Lane> lanes_;
  std::vector<SegmentId> successors_;
  std::vector<SegmentId> predecessors_;
};

}  // namespace osm
}  // namespace maliput_osm
