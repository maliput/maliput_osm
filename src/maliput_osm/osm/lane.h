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
#include <optional>
#include <string>
#include <vector>

#include <maliput/api/type_specific_identifier.h>
#include <maliput/common/maliput_copyable.h>
#include <maliput_sparse/geometry/line_string.h>

namespace maliput_osm {
namespace osm {

using LaneId = maliput::api::TypeSpecificIdentifier<class Lane>;

/// A lane in an Lanelet2-OSM road.
class Lane {
 public:
  /// Constructs a Lane
  /// @param id Id of the lane.
  /// @param left Left polyline describing the left boundary of the lane.
  /// @param right Right polyline describing the right boundary of the lane.
  Lane(const LaneId& id, const maliput_sparse::geometry::LineString3d& left,
       const maliput_sparse::geometry::LineString3d& right)
      : id_(id), left_(left), right_(right) {}

  /// Constructs a Lane
  /// @param id Id of the lane.
  /// @param left Left polyline describing the left boundary of the lane.
  /// @param right Right polyline describing the right boundary of the lane.
  /// @param left_lane_id Id of the lane to the left of this lane.
  /// @param right_lane_id Id of the right to the left of this lane.
  /// @param successors Ids of the lanes that are successors of this lane.
  /// @param predecessors Ids of the lanes that are predecessors of this lane.
  Lane(const LaneId& id, const maliput_sparse::geometry::LineString3d& left,
       const maliput_sparse::geometry::LineString3d& right, const std::optional<LaneId>& left_lane_id,
       const std::optional<LaneId>& right_lane_id, const std::vector<LaneId>& successors,
       const std::vector<LaneId>& predecessors)
      : id_(id),
        left_(left),
        right_(right),
        left_lane_id_(left_lane_id),
        right_lane_id_(right_lane_id),
        successors_(successors),
        predecessors_(predecessors) {}

  /// Returns the id of the lane.
  const LaneId& id() const { return id_; }
  /// Returns the left polyline describing the left boundary of the lane.
  const maliput_sparse::geometry::LineString3d& left() const { return left_; }
  /// Returns the right polyline describing the right boundary of the lane.
  const maliput_sparse::geometry::LineString3d& right() const { return right_; }

  /// Set the id of the lane to the left of this lane.
  /// @param left_lane_id Left LaneId.
  void set_left_lane_id(const std::optional<LaneId>& left_lane_id) { left_lane_id_ = left_lane_id; }
  /// Set the id of the lane to the right of this lane.
  /// @param right_lane_id Right LaneId.
  void set_right_lane_id(const std::optional<LaneId>& right_lane_id) { right_lane_id_ = right_lane_id; }
  /// @returns The id of the lane to the left of this lane.
  const std::optional<LaneId>& get_left_lane_id() const { return left_lane_id_; }
  /// @returns The id of the lane to the right of this lane.
  const std::optional<LaneId>& get_right_lane_id() const { return right_lane_id_; }

  /// Set the ids of the lanes that succeed this lane.
  /// @param successors Successors.
  void set_successors(const std::vector<LaneId>& successors) { successors_ = successors; }
  /// Set the ids of the lanes that precede this lane.
  /// @param predecessors Predecessors.
  void set_predecessors(const std::vector<LaneId>& predecessors) { predecessors_ = predecessors; }
  /// @returns The ids of the lanes that succeed this lane.
  const std::vector<LaneId>& get_successors() const { return successors_; }
  /// @returns The ids of the lanes that precede this lane.
  const std::vector<LaneId>& get_predecessors() const { return predecessors_; }

  /// Equality operator.
  /// @param other Other Lane.
  bool operator==(const Lane& other) const {
    return id_ == other.id_ && left_ == other.left_ && right_ == other.right_ && left_lane_id_ == other.left_lane_id_ &&
           right_lane_id_ == other.right_lane_id_ && successors_ == other.successors_ &&
           predecessors_ == other.predecessors_;
  }

 private:
  const LaneId id_;
  const maliput_sparse::geometry::LineString3d left_;
  const maliput_sparse::geometry::LineString3d right_;
  std::optional<LaneId> left_lane_id_;
  std::optional<LaneId> right_lane_id_;
  std::vector<LaneId> successors_;
  std::vector<LaneId> predecessors_;
};

}  // namespace osm
}  // namespace maliput_osm
