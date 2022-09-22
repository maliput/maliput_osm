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
#include "maliput_osm/osm/segment.h"

#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <maliput_sparse/geometry/line_string.h>

#include "maliput_osm/osm/lane.h"

namespace maliput_osm {
namespace osm {
namespace test {
namespace {

using maliput_sparse::geometry::LineString3d;

class SegmentTest : public ::testing::Test {
 protected:
  static Lane CreateLane(const LaneId& id) {
    static const LineString3d left{{1., 1., 1.}, {10., 1., 1.}};
    static const LineString3d right{{1., -1., 1.}, {10., -1., 1.}};
    static const std::optional<LaneId> left_lane_id{"left_lane_id"};
    static const std::optional<LaneId> right_lane_id{"right_lane_id"};
    static const std::vector<LaneId> successors{LaneId{"successor_1"}, LaneId{"successor_2"}};
    static const std::vector<LaneId> predecessors{LaneId{"predecessor_1"}, LaneId{"predecessor_2"}};
    return {id, left, right, left_lane_id, right_lane_id, successors, predecessors};
  }

  const SegmentId id_{"segment_id"};
  const std::map<LaneId, Lane> lanes_{{LaneId{"lane_1"}, CreateLane(LaneId{"lane_1"})},
                                      {LaneId{"lane_1"}, CreateLane(LaneId{"lane_2"})}};
  const std::vector<SegmentId> successors_{SegmentId{"successor_1"}, SegmentId{"successor_2"}};
  const std::vector<SegmentId> predecessors_{SegmentId{"predecessor_1"}, SegmentId{"predecessor_2"}};
};

TEST_F(SegmentTest, Test) {
  EXPECT_THROW(Segment(id_, {}), maliput::common::assertion_error);
  EXPECT_NO_THROW(Segment(id_, lanes_));
  EXPECT_NO_THROW(Segment(id_, lanes_, successors_, predecessors_));

  Segment segment{id_, lanes_};
  EXPECT_EQ(id_, segment.id());
  EXPECT_EQ(lanes_, segment.lanes());
  EXPECT_TRUE(segment.get_successors().empty());
  EXPECT_TRUE(segment.get_predecessors().empty());

  const Segment segment_2{id_, lanes_, successors_, predecessors_};
  EXPECT_EQ(segment.id(), segment_2.id());
  EXPECT_EQ(segment.lanes(), segment_2.lanes());
  EXPECT_EQ(successors_, segment_2.get_successors());
  EXPECT_EQ(predecessors_, segment_2.get_predecessors());
  segment.set_successors(successors_);
  segment.set_predecessors(predecessors_);
  EXPECT_EQ(successors_, segment.get_successors());
  EXPECT_EQ(predecessors_, segment.get_predecessors());
  EXPECT_EQ(segment_2, segment);
}

}  // namespace
}  // namespace test
}  // namespace osm
}  // namespace maliput_osm
