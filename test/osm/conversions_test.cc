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
#include "maliput_osm/osm/conversions.h"

#include <string>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <maliput_sparse/geometry/line_string.h>

#include "maliput_osm/osm/lane.h"
#include "osm/utilities/lanelet2.h"
#include "test_utilities/osm_types_compare.h"
#include "utilities/utilities.h"

namespace maliput_osm {
namespace osm {
namespace test {
namespace {

static constexpr double kTolerance{1e-6};

TEST(ToMaliput, LineString) {
  const std::vector<maliput::math::Vector3> expected_points{{1., 1., 1.}, {2., 2., 2.}, {3., 3., 3.}};
  const lanelet::Point3d p1{1, expected_points[0].x(), expected_points[0].y(), expected_points[0].z()};
  const lanelet::Point3d p2{2, expected_points[1].x(), expected_points[1].y(), expected_points[1].z()};
  const lanelet::Point3d p3{3, expected_points[2].x(), expected_points[2].y(), expected_points[2].z()};
  const lanelet::LineString3d ls(4., {p1, p2, p3});
  EXPECT_EQ(maliput_sparse::geometry::LineString3d{expected_points}, ToMaliput(ls));
}

TEST(ToMaliput, Lanelet) {
  const std::vector<maliput::math::Vector3> points_left{{1., 1., 1.}, {2., 2., 2.}, {3., 3., 3.}};
  const std::vector<maliput::math::Vector3> points_right{{3., 3., 3.}, {4., 4., 4.}, {5., 5., 5.}};
  constexpr int kLaneIdNumber{10};
  const Lane::Id lane_id{std::to_string(kLaneIdNumber)};
  const Lane lane{lane_id,
                  maliput_sparse::geometry::LineString3d{points_left},
                  maliput_sparse::geometry::LineString3d{points_right},
                  {} /* no left lane */,
                  {} /* no right lane */,
                  {} /* no successor */,
                  {} /* no predecessor */};

  const lanelet::LineString3d lanelet_left(
      4, {lanelet::Point3d{1, points_left[0].x(), points_left[0].y(), points_left[0].z()},
          lanelet::Point3d{2, points_left[1].x(), points_left[1].y(), points_left[1].z()},
          lanelet::Point3d{3, points_left[2].x(), points_left[2].y(), points_left[2].z()}});
  const lanelet::LineString3d lanelet_right(
      8, {lanelet::Point3d{5, points_right[0].x(), points_right[0].y(), points_right[0].z()},
          lanelet::Point3d{6, points_right[1].x(), points_right[1].y(), points_right[1].z()},
          lanelet::Point3d{7, points_right[2].x(), points_right[2].y(), points_right[2].z()}});
  const lanelet::Lanelet lanelet(kLaneIdNumber, lanelet_left, lanelet_right);
  const lanelet::LaneletMapPtr lanelet_map = lanelet::utils::createMap({lanelet}, {});

  EXPECT_TRUE(test::CompareOSMLane(lane, ToMaliput(lanelet, lanelet_map), kTolerance));
}

// Tests ToMaliput conversion using multi_lanes_road.osm map file.
class MultiLanesRoadMapToMaliputTest : public testing::Test {
 public:
  const std::string kOSMFilePath = ::utilities::FindOSMResource("multi_lanes_road.osm");
  const lanelet::LaneletMapPtr lanelet_map_ = lanelet::load(kOSMFilePath, lanelet::Origin{lanelet::GPSPoint{0., 0.}});
};

TEST_F(MultiLanesRoadMapToMaliputTest, LeftLane) {
  constexpr int kLaneIdNumber{1006};
  constexpr int kRightLaneIdNumber{1010};
  const Lane::Id lane_id{std::to_string(kLaneIdNumber)};
  const std::vector<maliput::math::Vector3> points_left{{0., 10.5, 0.}, {100., 10.5, 0.}};
  const std::vector<maliput::math::Vector3> points_right{{0., 7., 0.}, {100., 7., 0.}};
  const std::optional<Lane::Id> left_lane_id{std::nullopt};
  const std::optional<Lane::Id> right_lane_id{std::to_string(kRightLaneIdNumber)};

  const Lane expected_lane{lane_id,
                           maliput_sparse::geometry::LineString3d{points_left},
                           maliput_sparse::geometry::LineString3d{points_right},
                           left_lane_id,
                           right_lane_id,
                           {} /* no successor */,
                           {} /* no predecessor */};

  const auto lanelet = lanelet_map_->laneletLayer.find(lanelet::Id{kLaneIdNumber});
  ASSERT_NE(lanelet, lanelet_map_->laneletLayer.end());
  EXPECT_TRUE(test::CompareOSMLane(expected_lane, ToMaliput(*lanelet, lanelet_map_), kTolerance));
}

TEST_F(MultiLanesRoadMapToMaliputTest, RightLane) {
  constexpr int kLaneIdNumber{1026};
  constexpr int kLeftLaneIdNumber{1022};
  const Lane::Id lane_id{std::to_string(kLaneIdNumber)};
  const std::vector<maliput::math::Vector3> points_left{{0., -7., 0.}, {100., -7., 0.}};
  const std::vector<maliput::math::Vector3> points_right{{0., -10.5, 0.}, {100., -10.5, 0.}};
  const std::optional<Lane::Id> left_lane_id{std::to_string(kLeftLaneIdNumber)};
  const std::optional<Lane::Id> right_lane_id{std::nullopt};

  const Lane expected_lane{lane_id,
                           maliput_sparse::geometry::LineString3d{points_left},
                           maliput_sparse::geometry::LineString3d{points_right},
                           left_lane_id,
                           right_lane_id,
                           {} /* no successor */,
                           {} /* no predecessor */};

  const auto lanelet = lanelet_map_->laneletLayer.find(lanelet::Id{kLaneIdNumber});
  ASSERT_NE(lanelet, lanelet_map_->laneletLayer.end());
  EXPECT_TRUE(test::CompareOSMLane(expected_lane, ToMaliput(*lanelet, lanelet_map_), kTolerance));
}

TEST_F(MultiLanesRoadMapToMaliputTest, CenterLane) {
  constexpr int kLaneIdNumber{1014};
  constexpr int kLeftLaneIdNumber{1010};
  constexpr int kRightLaneIdNumber{1018};
  const Lane::Id lane_id{std::to_string(kLaneIdNumber)};
  const std::vector<maliput::math::Vector3> points_left{{0., 3.5, 0.}, {100., 3.5, 0.}};
  const std::vector<maliput::math::Vector3> points_right{{0., 0., 0.}, {100., 0., 0.}};
  const std::optional<Lane::Id> left_lane_id{std::to_string(kLeftLaneIdNumber)};
  const std::optional<Lane::Id> right_lane_id{std::to_string(kRightLaneIdNumber)};

  const Lane expected_lane{lane_id,
                           maliput_sparse::geometry::LineString3d{points_left},
                           maliput_sparse::geometry::LineString3d{points_right},
                           left_lane_id,
                           right_lane_id,
                           {} /* no successor */,
                           {} /* no predecessor */};

  const auto lanelet = lanelet_map_->laneletLayer.find(lanelet::Id{kLaneIdNumber});
  ASSERT_NE(lanelet, lanelet_map_->laneletLayer.end());
  EXPECT_TRUE(test::CompareOSMLane(expected_lane, ToMaliput(*lanelet, lanelet_map_), kTolerance));
}

// Tests ToMaliput conversion for predecessors and successors fields using a map that has three consecutive lanes.
class LShapeRoadMapToMaliputPredecessorSuccessorTest : public testing::Test {
 public:
  const std::string kOSMFilePath = ::utilities::FindOSMResource("l_shape_road.osm");
  const lanelet::LaneletMapPtr lanelet_map_ = lanelet::load(kOSMFilePath, lanelet::Origin{lanelet::GPSPoint{0., 0.}});
};

TEST_F(LShapeRoadMapToMaliputPredecessorSuccessorTest, StartLane) {
  constexpr int kLaneIdNumber{1206};
  const Lane::Id lane_id{std::to_string(kLaneIdNumber)};
  const std::unordered_map<Lane::Id, LaneEnd> expected_successors{{"1335", {"1335", LaneEnd::Which::kStart}}};
  const std::unordered_map<Lane::Id, LaneEnd> expected_predecessors{};

  const auto lanelet = lanelet_map_->laneletLayer.find(lanelet::Id{kLaneIdNumber});
  ASSERT_NE(lanelet, lanelet_map_->laneletLayer.end());
  const Lane converted_lane = ToMaliput(*lanelet, lanelet_map_);
  ASSERT_EQ(lane_id, converted_lane.id);
  EXPECT_EQ(expected_predecessors, converted_lane.predecessors);
  EXPECT_EQ(expected_successors, converted_lane.successors);
}

TEST_F(LShapeRoadMapToMaliputPredecessorSuccessorTest, MiddleLane) {
  constexpr int kLaneIdNumber{1335};
  const Lane::Id lane_id{std::to_string(kLaneIdNumber)};
  const std::unordered_map<Lane::Id, LaneEnd> expected_successors{{"1542", {"1542", LaneEnd::Which::kStart}}};
  const std::unordered_map<Lane::Id, LaneEnd> expected_predecessors{{"1206", {"1206", LaneEnd::Which::kFinish}}};

  const auto lanelet = lanelet_map_->laneletLayer.find(lanelet::Id{kLaneIdNumber});
  ASSERT_NE(lanelet, lanelet_map_->laneletLayer.end());
  const Lane converted_lane = ToMaliput(*lanelet, lanelet_map_);
  ASSERT_EQ(lane_id, converted_lane.id);
  EXPECT_EQ(expected_predecessors, converted_lane.predecessors);
  EXPECT_EQ(expected_successors, converted_lane.successors);
}

TEST_F(LShapeRoadMapToMaliputPredecessorSuccessorTest, EndLane) {
  constexpr int kLaneIdNumber{1542};
  const Lane::Id lane_id{std::to_string(kLaneIdNumber)};
  const std::unordered_map<Lane::Id, LaneEnd> expected_successors{};
  const std::unordered_map<Lane::Id, LaneEnd> expected_predecessors{{"1335", {"1335", LaneEnd::Which::kFinish}}};

  const auto lanelet = lanelet_map_->laneletLayer.find(lanelet::Id{kLaneIdNumber});
  ASSERT_NE(lanelet, lanelet_map_->laneletLayer.end());
  const Lane converted_lane = ToMaliput(*lanelet, lanelet_map_);
  ASSERT_EQ(lane_id, converted_lane.id);
  EXPECT_EQ(expected_predecessors, converted_lane.predecessors);
  EXPECT_EQ(expected_successors, converted_lane.successors);
}

struct AdjacentPredecessorSuccessorTest {
  int id;
  std::optional<Lane::Id> left_lane_id;
  std::optional<Lane::Id> right_lane_id;
  std::unordered_map<Lane::Id, LaneEnd> predecessors;
  std::unordered_map<Lane::Id, LaneEnd> successors;
};

std::vector<AdjacentPredecessorSuccessorTest> GetAdjacentPredecessorSuccessorTest() {
  return {
      {
          12,
          {} /* left_lane */,
          "23" /* right_lane */,
          {} /* predecessors */,
          {{"56", {"56", LaneEnd::Which::kStart}}} /* successors */,
      },
      {
          23,
          "12" /* left_lane */,
          "34" /* right_lane */,
          {} /* predecessors */,
          {{"67", {"67", LaneEnd::Which::kStart}}} /* successors */,
      },
      {
          34,
          "23" /* left_lane */,
          {} /* right_lane */,
          {} /* predecessors */,
          {{"78", {"78", LaneEnd::Which::kStart}}} /* successors */,
      },
      {
          56,
          {} /* left_lane */,
          "67" /* right_lane */,
          {{"12", {"12", LaneEnd::Which::kFinish}}} /* predecessors */,
          {} /* successors */,
      },
      {
          67,
          "56" /* left_lane */,
          "78" /* right_lane */,
          {{"23", {"23", LaneEnd::Which::kFinish}}} /* predecessors */,
          {{"910", {"910", LaneEnd::Which::kStart}}} /* successors */,
      },
      {
          78,
          "67" /* left_lane */,
          {} /* right_lane */,
          {{"34", {"34", LaneEnd::Which::kFinish}}} /* predecessors */,
          {} /* successors */,
      },
      {
          910,
          {} /* left_lane */,
          {} /* right_lane */,
          {{"67", {"67", LaneEnd::Which::kFinish}}} /* predecessors */,
          {{"1112", {"1112", LaneEnd::Which::kStart}}, {"1314", {"1314", LaneEnd::Which::kStart}}} /* successors */,
      },
      {
          1112,
          {} /* left_lane */,
          {} /* right_lane */,
          {{"910", {"910", LaneEnd::Which::kFinish}}} /* predecessors */,
          {} /* successors */,
      },
      {
          1314,
          {} /* left_lane */,
          {} /* right_lane */,
          {{"910", {"910", LaneEnd::Which::kFinish}}} /* predecessors */,
          {} /* successors */,
      },
  };
}

// Tests adjacent and predecessors/successors lanes
class MultiLaneMultiSegmentRoadAdjacentPredecessorSuccessorTest
    : public testing::TestWithParam<AdjacentPredecessorSuccessorTest> {
 public:
  const lanelet::LaneletMapPtr lanelet_map_ = osm::test::utilities::CreateLanelet2Map();
  const AdjacentPredecessorSuccessorTest test_case_ = GetParam();
};

TEST_P(MultiLaneMultiSegmentRoadAdjacentPredecessorSuccessorTest, Test) {
  const auto lanelet = lanelet_map_->laneletLayer.find(lanelet::Id{test_case_.id});
  ASSERT_NE(lanelet, lanelet_map_->laneletLayer.end());
  const Lane converted_lane = ToMaliput(*lanelet, lanelet_map_);
  ASSERT_EQ(std::to_string(test_case_.id), converted_lane.id);
  EXPECT_EQ(test_case_.predecessors, converted_lane.predecessors);
  EXPECT_EQ(test_case_.successors, converted_lane.successors);
}

INSTANTIATE_TEST_CASE_P(MultiLaneMultiSegmentRoadAdjacentPredecessorSuccessorTestGroup,
                        MultiLaneMultiSegmentRoadAdjacentPredecessorSuccessorTest,
                        ::testing::ValuesIn(GetAdjacentPredecessorSuccessorTest()));

std::vector<AdjacentPredecessorSuccessorTest> GetAdjacentPredecessorSuccessorWithInversionsTest() {
  return {
      {
          12,
          {} /* left_lane */,
          "23" /* right_lane */,
          {} /* predecessors */,
          {{"65", {"65", LaneEnd::Which::kFinish}}} /* successors */,
      },
      {
          23,
          "12" /* left_lane */,
          "34" /* right_lane */,
          {} /* predecessors */,
          {{"76", {"76", LaneEnd::Which::kFinish}}} /* successors */,
      },
      {
          34,
          "23" /* left_lane */,
          {} /* right_lane */,
          {} /* predecessors */,
          {{"87", {"87", LaneEnd::Which::kFinish}}} /* successors */,
      },
      {
          65,
          "76" /* left_lane */,
          {} /* right_lane */,
          {{"910", {"910", LaneEnd::Which::kStart}}} /* predecessors */,
          {{"12", {"12", LaneEnd::Which::kFinish}}} /* successors */,
      },
      {
          76,
          "87" /* left_lane */,
          "65" /* right_lane */,
          {{"1011", {"1011", LaneEnd::Which::kStart}}} /* predecessors */,
          {{"23", {"23", LaneEnd::Which::kFinish}}} /* successors */,
      },
      {
          87,
          {} /* left_lane */,
          "76" /* right_lane */,
          {{"1112", {"1112", LaneEnd::Which::kStart}}} /* predecessors */,
          {{"34", {"34", LaneEnd::Which::kFinish}}} /* successors */,
      },
      {
          910,
          {} /* left_lane */,
          "1011" /* right_lane */,
          {{"65", {"65", LaneEnd::Which::kStart}}} /* predecessors */,
          {} /* successors */,
      },
      {
          1011,
          "910" /* left_lane */,
          "1112" /* right_lane */,
          {{"76", {"76", LaneEnd::Which::kStart}}} /* predecessors */,
          {} /* successors */,
      },
      {
          1112,
          "1011" /* left_lane */,
          {} /* right_lane */,
          {{"87", {"87", LaneEnd::Which::kStart}}} /* predecessors */,
          {} /* successors */,
      },
  };
}

// Tests adjacent and predecessors/successors lanes when inverted lanes are present.
class InvertedConnectionsAdjacentPredecessorSuccessorTest
    : public testing::TestWithParam<AdjacentPredecessorSuccessorTest> {
 public:
  const lanelet::LaneletMapPtr lanelet_map_ = osm::test::utilities::CreateInvertedLaneMap();
  const AdjacentPredecessorSuccessorTest test_case_ = GetParam();
};

TEST_P(InvertedConnectionsAdjacentPredecessorSuccessorTest, Test) {
  const auto lanelet = lanelet_map_->laneletLayer.find(lanelet::Id{test_case_.id});
  ASSERT_NE(lanelet, lanelet_map_->laneletLayer.end());
  const Lane converted_lane = ToMaliput(*lanelet, lanelet_map_);
  ASSERT_EQ(std::to_string(test_case_.id), converted_lane.id);
  EXPECT_EQ(test_case_.predecessors, converted_lane.predecessors);
  EXPECT_EQ(test_case_.successors, converted_lane.successors);
}

INSTANTIATE_TEST_CASE_P(InvertedConnectionsAdjacentPredecessorSuccessorTestGroup,
                        InvertedConnectionsAdjacentPredecessorSuccessorTest,
                        ::testing::ValuesIn(GetAdjacentPredecessorSuccessorWithInversionsTest()));

}  // namespace
}  // namespace test
}  // namespace osm
}  // namespace maliput_osm
