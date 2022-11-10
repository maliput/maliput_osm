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
#include "maliput_osm/builder/road_geometry_builder.h"

#include <maliput_sparse/builder/builder.h>

namespace maliput_osm {
namespace builder {
namespace {

maliput::api::LaneEnd::Which ToMaliputLaneEndWhich(const osm::LaneEnd::Which end) {
  switch (end) {
    case osm::LaneEnd::Which::kStart:
      return maliput::api::LaneEnd::Which::kStart;
    case osm::LaneEnd::Which::kFinish:
      return maliput::api::LaneEnd::Which::kFinish;
    default:
      MALIPUT_THROW_MESSAGE("Unknown osm::LaneEnd::Which value: " + static_cast<int>(end));
  }
}

}  // namespace

RoadGeometryBuilder::RoadGeometryBuilder(std::unique_ptr<osm::OSMManager> osm_manager,
                                         const BuilderConfiguration& builder_configuration)
    : osm_manager_(std::move(osm_manager)), builder_configuration_(builder_configuration) {
  MALIPUT_THROW_UNLESS(osm_manager_ != nullptr);
}

std::unique_ptr<const maliput::api::RoadGeometry> RoadGeometryBuilder::operator()() {
  const std::unordered_map<osm::Junction::Id, osm::Junction>& osm_junctions = osm_manager_->GetOSMJunctions();
  const std::vector<osm::Connection>& connections = osm_manager_->GetOSMConnections();

  maliput_sparse::builder::RoadGeometryBuilder rg_builder{};
  rg_builder.Id(builder_configuration_.road_geometry_id)
      .LinearTolerance(builder_configuration_.linear_tolerance)
      .AngularTolerance(builder_configuration_.angular_tolerance)
      .ScaleLength(builder_configuration_.scale_length)
      .InertialToBackendFrameTranslation(builder_configuration_.inertial_to_backend_frame_translation);

  for (const auto& osm_junction : osm_junctions) {
    maliput_sparse::builder::JunctionBuilder junction_builder = rg_builder.StartJunction();
    junction_builder.Id(maliput::api::JunctionId{osm_junction.first});
    for (const auto& osm_segment : osm_junction.second.segments) {
      maliput_sparse::builder::SegmentBuilder segment_builder = junction_builder.StartSegment();
      segment_builder.Id(maliput::api::SegmentId{osm_segment.first});

      for (const osm::Lane& osm_lane : osm_segment.second.lanes) {
        const maliput::api::LaneId lane_id{osm_lane.id};
        segment_builder.StartLane()
            .Id(lane_id)
            .HeightBounds(maliput::api::HBounds{0., 5.})
            .StartLaneGeometry()
            .LeftLineString(osm_lane.left)
            .RightLineString(osm_lane.right)
            .EndLaneGeometry()
            .EndLane();
      }
      segment_builder.EndSegment();
    }
    junction_builder.EndJunction();
  }
  maliput_sparse::builder::BranchPointBuilder bp_builder = rg_builder.StartBranchPoints();
  for (const auto& connection : connections) {
    bp_builder.Connect(maliput::api::LaneId{connection.from.lane_id}, ToMaliputLaneEndWhich(connection.from.end),
                       maliput::api::LaneId{connection.to.lane_id}, ToMaliputLaneEndWhich(connection.to.end));
  }
  bp_builder.EndBranchPoints();

  return rg_builder.Build();
}

}  // namespace builder
}  // namespace maliput_osm
