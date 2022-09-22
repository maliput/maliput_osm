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
#include "maliput_osm/osm/lane.h"

#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <maliput_sparse/geometry/line_string.h>

namespace maliput_osm {
namespace osm {
namespace test {
namespace {

using maliput_sparse::geometry::LineString3d;

class LaneTest : public ::testing::Test {
 protected:
  const LaneId id{"lane_id"};
  const LineString3d left{{1., 1., 1.}, {10., 1., 1.}};
  const LineString3d right{{1., -1., 1.}, {10., -1., 1.}};
  const std::optional<LaneId> left_lane_id{"left_lane_id"};
  const std::optional<LaneId> right_lane_id{"right_lane_id"};
  const std::vector<LaneId> successors{LaneId{"successor_1"}, LaneId{"successor_2"}};
  const std::vector<LaneId> predecessors{LaneId{"predecessor_1"}, LaneId{"predecessor_2"}};
};

TEST_F(LaneTest, Test) {
  EXPECT_NO_THROW(Lane(id, left, right));
  EXPECT_NO_THROW(Lane(id, left, right, left_lane_id, right_lane_id, successors, predecessors));

  Lane lane{id, left, right};
  EXPECT_EQ(id, lane.id());
  EXPECT_EQ(left, lane.left());
  EXPECT_EQ(right, lane.right());
  EXPECT_EQ(std::nullopt, lane.get_left_lane_id());
  EXPECT_EQ(std::nullopt, lane.get_right_lane_id());

  const Lane lane_2{id, left, right, left_lane_id, right_lane_id, successors, predecessors};
  EXPECT_EQ(lane_2.id(), lane.id());
  EXPECT_EQ(lane_2.left(), lane.left());
  EXPECT_EQ(lane_2.right(), lane.right());
  EXPECT_EQ(left_lane_id, lane_2.get_left_lane_id());
  EXPECT_EQ(right_lane_id, lane_2.get_right_lane_id());
  EXPECT_EQ(successors, lane_2.get_successors());
  EXPECT_EQ(predecessors, lane_2.get_predecessors());

  lane.set_left_lane_id(left_lane_id);
  lane.set_right_lane_id(right_lane_id);
  EXPECT_EQ(left_lane_id, lane.get_left_lane_id());
  EXPECT_EQ(right_lane_id, lane.get_right_lane_id());
  lane.set_successors(successors);
  lane.set_predecessors(predecessors);
  EXPECT_EQ(successors, lane.get_successors());
  EXPECT_EQ(predecessors, lane.get_predecessors());
  EXPECT_EQ(lane_2, lane);
}

}  // namespace
}  // namespace test
}  // namespace osm
}  // namespace maliput_osm
