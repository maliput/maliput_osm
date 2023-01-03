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

#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput_sparse/loader/road_geometry_loader.h>

#include "maliput_osm/osm/osm_manager.h"
#include "test_utilities/builder_configuration_for_osm.h"
#include "utilities/utilities.h"

namespace maliput_osm {
namespace builder {
namespace test {
namespace {

using maliput::api::JunctionId;
using maliput::api::LaneEnd;
using maliput::api::LaneId;
using maliput::api::SegmentId;

class RoadGeometryBuilderTest : public ::testing::Test {
 public:
  const std::string kOSMFilePath = utilities::FindOSMResource("straight_forward.osm");
};

TEST_F(RoadGeometryBuilderTest, Test) {
  BuilderConfiguration builder_config;
  builder_config.osm_file = kOSMFilePath;
  auto osm_manager = std::make_unique<osm::OSMManager>(kOSMFilePath, osm::ParserConfig{});
  std::unique_ptr<maliput_sparse::loader::RoadGeometryLoader> dut;
  ASSERT_NO_THROW(dut = std::make_unique<maliput_sparse::loader::RoadGeometryLoader>(std::move(osm_manager),
                                                                                     builder_config.sparse_config));
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry;
  ASSERT_NO_THROW(road_geometry = (*dut)());
  EXPECT_THROW(maliput_sparse::loader::RoadGeometryLoader(nullptr, builder_config.sparse_config),
               maliput::common::assertion_error);
}

// Holds the parameters to be checked.
struct RoadGeometryBuilderTestParameters {
  std::string osm_file{};
  std::string road_geometry_id{};
  std::unordered_map<JunctionId, std::vector<std::pair<SegmentId, std::vector<LaneId>>>>
      expected_junctions_segments_lanes_ids;
};

// Returns a vector of test parameters for the Builder test.
std::vector<RoadGeometryBuilderTestParameters> InstantiateBuilderParameters() {
  return {
      /*
        straight_forward.osm map has the following structure:
                          (0.,0.,0.)         (0.,1000.,0.)
        Driving   | L: 1  |-------->---------| Width: 3.5m
        Track lane| L: 0  |========>=========| Width: 0m
        Driving   | L: -1 |-------->---------| Width: 3.5m

      */
      {"straight_forward.osm",
       "straight_forward",
       {
           {JunctionId("0"), {{{SegmentId("0")}, {LaneId("1006"), LaneId("1010")}}}},
       }},
      {"arc_lane.osm",
       "arc_lane",
       {
           {JunctionId("0"), {{{SegmentId("0")}, {LaneId("1044"), LaneId("1068")}}}},
       }},
      {"arc_lane_dense.osm",
       "arc_lane_dense",
       {
           {JunctionId("0"), {{{SegmentId("0")}, {LaneId("2956"), LaneId("3985")}}}},
       }},
      {"multi_lanes_road.osm",
       "multi_lanes_road",
       {
           {JunctionId("0"),
            {{{SegmentId("0")},
              {LaneId("1026"), LaneId("1022"), LaneId("1018"), LaneId("1014"), LaneId("1010"), LaneId("1006")}}}},
       }},
      {"l_shape_road.osm",
       "l_shape_road",
       {
           {JunctionId("0"), {{{SegmentId("0")}, {LaneId("1206")}}}},
           {JunctionId("1"), {{{SegmentId("1")}, {LaneId("1335")}}}},
           {JunctionId("2"), {{{SegmentId("2")}, {LaneId("1542")}}}},
       }},
      {"y_shape_road.osm",
       "y_shape_road",
       {
           {JunctionId("0"), {{{SegmentId("0")}, {LaneId("1096")}}}},
           {JunctionId("1_2"), {{{SegmentId("1")}, {LaneId("1117")}}, {{SegmentId("2")}, {LaneId("1239")}}}},
           {JunctionId("3"), {{{SegmentId("3")}, {LaneId("1214")}}}},
           {JunctionId("4"), {{{SegmentId("4")}, {LaneId("1336")}}}},
       }},
      {"t_shape_road.osm",
       "t_shape_road",
       {
           {JunctionId("8_4_1_0_7_5"),
            {{{SegmentId("8")}, {LaneId("2053")}},
             {{SegmentId("4")}, {LaneId("1641")}},
             {{SegmentId("1")}, {LaneId("1968")}},
             {{SegmentId("0")}, {LaneId("1604")}},
             {{SegmentId("7")}, {LaneId("2032")}},
             {{SegmentId("5")}, {LaneId("1989")}}}},
           {JunctionId("2"), {{{SegmentId("2")}, {LaneId("1472"), LaneId("1567")}}}},
           {JunctionId("3"), {{{SegmentId("3")}, {LaneId("1188"), LaneId("1283")}}}},
           {JunctionId("6"), {{{SegmentId("6")}, {LaneId("1830"), LaneId("1925")}}}},
       }},
  };
}

// Builder test that evaluates Junction, Segment and Lane construction.
class RoadGeometryBuilderBaseTest : public ::testing::TestWithParam<RoadGeometryBuilderTestParameters> {
 protected:
  void SetUp() override {
    const auto builder_config = maliput_osm::test::GetBuilderConfigurationFor(case_.osm_file);
    ASSERT_NE(builder_config, std::nullopt);
    builder_config_ = builder_config.value();
    osm_manager_ =
        std::make_unique<osm::OSMManager>(builder_config_.osm_file, osm::ParserConfig{builder_config_.origin});
  }

 public:
  const RoadGeometryBuilderTestParameters case_ = GetParam();
  BuilderConfiguration builder_config_;
  std::unique_ptr<osm::OSMManager> osm_manager_;
};

// Tests Junction, Segments and Lanes graph.
TEST_P(RoadGeometryBuilderBaseTest, TestGraph) {
  std::unique_ptr<const maliput::api::RoadGeometry> dut =
      maliput_sparse::loader::RoadGeometryLoader(std::move(osm_manager_), builder_config_.sparse_config)();
  ASSERT_NE(dut.get(), nullptr);

  // Junctions.
  const int num_junctions = static_cast<int>(case_.expected_junctions_segments_lanes_ids.size());
  EXPECT_EQ(num_junctions, dut->num_junctions());

  for (int i = 0; i < num_junctions; ++i) {
    const maliput::api::Junction* junction = dut->junction(i);
    ASSERT_NE(junction, nullptr);
    EXPECT_EQ(junction->road_geometry(), dut.get());
    EXPECT_NE(case_.expected_junctions_segments_lanes_ids.find(junction->id()),
              case_.expected_junctions_segments_lanes_ids.end())
        << "Unexpected junction id: " << junction->id();

    const int num_segments = static_cast<int>(case_.expected_junctions_segments_lanes_ids.at(junction->id()).size());
    EXPECT_EQ(junction->num_segments(), num_segments);

    // Segments.
    for (int j = 0; j < num_segments; j++) {
      const maliput::api::Segment* segment = junction->segment(j);
      ASSERT_NE(segment, nullptr);
      EXPECT_EQ(junction, segment->junction());

      const auto& expected_segments = case_.expected_junctions_segments_lanes_ids.at(junction->id());
      const auto expected_segment_it =
          std::find_if(expected_segments.begin(), expected_segments.end(),
                       [segment](const auto expected_segment) { return expected_segment.first == segment->id(); });
      EXPECT_NE(expected_segment_it, expected_segments.end()) << "Unexpected segment id: " << segment->id();

      const int num_lanes = static_cast<int>(expected_segment_it->second.size());
      EXPECT_EQ(num_lanes, segment->num_lanes());

      // Lanes.
      for (int k = 0; k < num_lanes; k++) {
        const auto& expected_lanes = expected_segment_it->second;
        const maliput::api::Lane* lane = segment->lane(k);
        ASSERT_NE(lane, nullptr);
        EXPECT_EQ(segment, lane->segment());

        const auto expected_lane_it =
            std::find_if(expected_lanes.begin(), expected_lanes.end(),
                         [lane](const auto expected_lane) { return expected_lane == lane->id(); });
        EXPECT_NE(expected_lane_it, expected_lanes.end()) << "Unexpected lane id: " << lane->id();

        if (k == 0) {
          EXPECT_EQ(nullptr, lane->to_right());
        } else {
          EXPECT_EQ(segment->lane(k - 1), lane->to_right());
        }
        if (k == (num_lanes - 1)) {
          EXPECT_EQ(nullptr, lane->to_left());
        } else {
          EXPECT_EQ(segment->lane(k + 1), lane->to_left());
        }
      }
    }
  }
}

INSTANTIATE_TEST_CASE_P(RoadGeometryBuilderBaseTestGroup, RoadGeometryBuilderBaseTest,
                        ::testing::ValuesIn(InstantiateBuilderParameters()));

// Compare functor between a pair <LaneId, LaneEnd::Which> and a LaneEnd.
// It is convenient for std::find_if().
struct CompareConnectionExpectation {
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CompareConnectionExpectation)

  CompareConnectionExpectation() = delete;

  explicit CompareConnectionExpectation(const LaneEnd _lane_end) : lane_end(_lane_end) {}

  bool operator()(const std::pair<LaneId, LaneEnd::Which>& lane_id_and_end) {
    return (lane_end.lane->id() == lane_id_and_end.first) && (lane_end.end == lane_id_and_end.second);
  }

  LaneEnd lane_end;
};

// Alternative to BranchPoint that instead of using Lane pointers, uses LaneIds
// which are easy to define from the test description point of view.
struct ConnectionExpectation {
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConnectionExpectation)

  ConnectionExpectation() = default;

  // Constructs the expectation based on two collections of LaneIds and
  // LaneEnd::Which pairs.
  //
  // @param x_side_in maps to the A/B side of the BranchPoint to evaluate.
  // @param y_side_in maps to the B/A side of the BranchPoint to evaluate.
  ConnectionExpectation(const std::vector<std::pair<LaneId, LaneEnd::Which>>& x_side_in,
                        const std::vector<std::pair<LaneId, LaneEnd::Which>>& y_side_in)
      : x_side(x_side_in), y_side(y_side_in) {}

  std::vector<std::pair<LaneId, LaneEnd::Which>> x_side{};
  std::vector<std::pair<LaneId, LaneEnd::Which>> y_side{};
};

// Holds the parameters for a BranchPoint test.
struct BuilderBranchPointTestParameters {
  std::string osm_file{};
  std::string road_geometry_id{};
  std::unordered_map<LaneId, std::pair<ConnectionExpectation, ConnectionExpectation>> expected_connections;
};

// Returns a vector of test parameters for the Builder BranchPoint test.
std::vector<BuilderBranchPointTestParameters> InstantiateBuilderBranchPointParameters() {
  return {
      {"l_shape_road.osm",
       "l_shape_road",
       {
           {LaneId("1206"),
            {ConnectionExpectation({{LaneId("1206"), LaneEnd::Which::kStart}}, {}),
             ConnectionExpectation({{LaneId("1206"), LaneEnd::Which::kFinish}},
                                   {{LaneId("1335"), LaneEnd::Which::kStart}})}},
           {LaneId("1335"),
            {ConnectionExpectation({{LaneId("1206"), LaneEnd::Which::kFinish}},
                                   {{LaneId("1335"), LaneEnd::Which::kStart}}),
             ConnectionExpectation({{LaneId("1335"), LaneEnd::Which::kFinish}},
                                   {{LaneId("1542"), LaneEnd::Which::kStart}})}},
           {LaneId("1542"),
            {ConnectionExpectation({{LaneId("1335"), LaneEnd::Which::kFinish}},
                                   {{LaneId("1542"), LaneEnd::Which::kStart}}),
             ConnectionExpectation({{LaneId("1542"), LaneEnd::Which::kFinish}}, {})}},
       }},
  };
}

// Class to setup the RoadGeometry and later test BranchPoint formation.
class BuilderBranchPointTest : public ::testing::TestWithParam<BuilderBranchPointTestParameters> {
 protected:
  //@{  Tolerances set to match the involved geometries and the parser resolution.
  static constexpr double kLinearTolerance{1e-6};
  static constexpr double kAngularTolerance{1e-6};
  static constexpr double kScaleLength{1.};
  //@}

  void SetUp() override {
    const auto builder_config = maliput_osm::test::GetBuilderConfigurationFor(case_.osm_file);
    ASSERT_NE(builder_config, std::nullopt);
    auto osm_manager =
        std::make_unique<osm::OSMManager>(builder_config->osm_file, osm::ParserConfig{builder_config->origin});
    rg_ = maliput_sparse::loader::RoadGeometryLoader(std::move(osm_manager), builder_config->sparse_config)();
    expected_connections = GetParam().expected_connections;
  }

  const BuilderBranchPointTestParameters case_ = GetParam();
  std::unique_ptr<const maliput::api::RoadGeometry> rg_;
  std::unordered_map<LaneId, std::pair<ConnectionExpectation, ConnectionExpectation>> expected_connections;
};

// Tests Lane and BranchPoint formation.
TEST_P(BuilderBranchPointTest, LaneAndBranchPointTest) {
  // Evaluates that LaneEnds on both bp sides are the expected ones.
  auto test_branch_point = [](const maliput::api::BranchPoint* bp,
                              const ConnectionExpectation& connection_expectation) {
    ASSERT_NE(bp->GetASide(), nullptr);
    ASSERT_NE(bp->GetBSide(), nullptr);

    // Evaluates which side belongs to each connection expectation.
    const maliput::api::LaneEndSet* a_side = bp->GetASide();
    const maliput::api::LaneEndSet* b_side = bp->GetBSide();

    ASSERT_TRUE(a_side->size() != 0 || b_side->size() != 0);

    std::pair<const maliput::api::LaneEndSet*, std::vector<std::pair<LaneId, LaneEnd::Which>>> first_side;
    std::pair<const maliput::api::LaneEndSet*, std::vector<std::pair<LaneId, LaneEnd::Which>>> second_side;

    const LaneEnd dut = a_side->size() != 0 ? a_side->get(0) : b_side->get(0);
    const CompareConnectionExpectation cmp(dut);
    if (std::find_if(connection_expectation.x_side.begin(), connection_expectation.x_side.end(), cmp) !=
        connection_expectation.x_side.end()) {
      // a --> x
      first_side.first = a_side;
      first_side.second = connection_expectation.x_side;
      // b --> y
      second_side.first = b_side;
      second_side.second = connection_expectation.y_side;
    } else if (std::find_if(connection_expectation.y_side.begin(), connection_expectation.y_side.end(), cmp) !=
               connection_expectation.y_side.end()) {
      // b --> x
      first_side.first = b_side;
      first_side.second = connection_expectation.x_side;
      // x --> y
      second_side.first = a_side;
      second_side.second = connection_expectation.y_side;
    } else {
      GTEST_FAIL() << "BranchPoint: " << bp->id().string() << " does not match the expectation.";
    }

    for (int lane_end_index = 0; lane_end_index < first_side.first->size(); ++lane_end_index) {
      const LaneEnd dut = first_side.first->get(lane_end_index);
      const CompareConnectionExpectation cmp(dut);
      EXPECT_NE(std::find_if(first_side.second.begin(), first_side.second.end(), cmp), first_side.second.end());
    }

    for (int lane_end_index = 0; lane_end_index < second_side.first->size(); ++lane_end_index) {
      const LaneEnd dut = second_side.first->get(lane_end_index);
      const CompareConnectionExpectation cmp(dut);
      EXPECT_NE(std::find_if(second_side.second.begin(), second_side.second.end(), cmp), second_side.second.end());
    }
  };

  for (const auto& lane_id_connections : expected_connections) {
    const maliput::api::Lane* lane = rg_->ById().GetLane(lane_id_connections.first);
    EXPECT_NE(lane, nullptr);

    // Analyzes A and B sides at the start BranchPoint.
    const maliput::api::BranchPoint* start_bp = lane->GetBranchPoint(LaneEnd::Which::kStart);
    EXPECT_NE(start_bp, nullptr);
    EXPECT_EQ(start_bp->road_geometry(), rg_.get());
    test_branch_point(start_bp, lane_id_connections.second.first);

    // Analyzes A and B sides at the end BranchPoint.
    const maliput::api::BranchPoint* end_bp = lane->GetBranchPoint(LaneEnd::Which::kFinish);
    EXPECT_NE(end_bp, nullptr);
    EXPECT_EQ(end_bp->road_geometry(), rg_.get());
    test_branch_point(end_bp, lane_id_connections.second.second);
  }
}

INSTANTIATE_TEST_CASE_P(BuilderBranchPointTestGroup, BuilderBranchPointTest,
                        ::testing::ValuesIn(InstantiateBuilderBranchPointParameters()));

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace maliput_osm
