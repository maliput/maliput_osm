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

#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput_osm/osm/osm_manager.h"
#include "test_utilities/builder_configuration_for_osm.h"
#include "utilities/utilities.h"

namespace maliput_osm {
namespace builder {
namespace test {
namespace {

using maliput::api::JunctionId;
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
  std::unique_ptr<RoadGeometryBuilder> dut;
  ASSERT_NO_THROW(dut = std::make_unique<RoadGeometryBuilder>(std::move(osm_manager), builder_config));
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry;
  ASSERT_NO_THROW(road_geometry = (*dut)());
  EXPECT_THROW(RoadGeometryBuilder(nullptr, builder_config), maliput::common::assertion_error);
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
      {"multi_lanes_road.osm",
       "multi_lanes_road",
       {
           {JunctionId("0"),
            {{{SegmentId("0")},
              {LaneId("1026"), LaneId("1022"), LaneId("1018"), LaneId("1014"), LaneId("1010"), LaneId("1006")}}}},
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
      builder::RoadGeometryBuilder(std::move(osm_manager_), builder_config_)();
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

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace maliput_osm
