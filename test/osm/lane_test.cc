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
  const Lane dut{id, left, right, left_lane_id, right_lane_id, successors, predecessors};
  EXPECT_EQ(id, dut.id);
  EXPECT_EQ(left, dut.left);
  EXPECT_EQ(right, dut.right);
  EXPECT_EQ(left_lane_id, dut.left_lane_id);
  EXPECT_EQ(right_lane_id, dut.right_lane_id);
  EXPECT_EQ(successors, dut.successors);
  EXPECT_EQ(predecessors, dut.predecessors);
  const Lane dut2 = dut;
  EXPECT_EQ(dut, dut2);
}

}  // namespace
}  // namespace test
}  // namespace osm
}  // namespace maliput_osm
