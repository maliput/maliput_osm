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
#include "maliput_osm/builder/road_network_builder.h"

#include <map>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <maliput/api/road_network.h>
#include <maliput/api/road_network_validator.h>
#include <maliput/common/assertion_error.h>

#include "test_utilities/builder_configuration_for_osm.h"
#include "utilities/utilities.h"

namespace maliput_osm {
namespace builder {
namespace test {
namespace {

using maliput::api::rules::Phase;

class RoadNetworkBuilderTest : public ::testing::Test {
 public:
  void SetUp() override { ASSERT_NE(std::nullopt, builder_config_); }
  const std::string kOSMFileName = "straight_forward.osm";
  const std::optional<builder::BuilderConfiguration> builder_config_ =
      maliput_osm::test::GetBuilderConfigurationFor(kOSMFileName);
};

TEST_F(RoadNetworkBuilderTest, Constructor) {
  std::unique_ptr<RoadNetworkBuilder> dut;
  EXPECT_NO_THROW(dut = std::make_unique<RoadNetworkBuilder>(builder_config_.value().ToStringMap()));
  std::unique_ptr<maliput::api::RoadNetwork> rn;
  EXPECT_NO_THROW(rn = (*dut)());
  const auto rg = rn->road_geometry();
  EXPECT_NE(rg, nullptr);
}

// Holds the parameters to be checked.
struct RoadNetworkBuilderTestParameters {
  std::string osm_file{};
  std::string road_geometry_id{};
};

// Returns a vector of test parameters for the Builder test.
std::vector<RoadNetworkBuilderTestParameters> InstantiateBuilderParameters() {
  return {/*
            straight_forward.osm map has the following structure:
                              (0.,0.,0.)         (0.,1000.,0.)
            Driving   | L: 1  |-------->---------| Width: 3.5m
            Track lane| L: 0  |========>=========| Width: 0m
            Driving   | L: -1 |-------->---------| Width: 3.5m

          */
          {
              "straight_forward.osm",
              "straight_forward",
          }};
}

// Builder test that evaluates Junction, Segment and Lane construction.
class RoadNetworkBuilderValidatorTest : public ::testing::TestWithParam<RoadNetworkBuilderTestParameters> {
 protected:
  void SetUp() override {
    const auto builder_config = maliput_osm::test::GetBuilderConfigurationFor(case_.osm_file);
    ASSERT_NE(builder_config, std::nullopt);
    builder_config_ = builder_config.value();
  }

 public:
  const RoadNetworkBuilderTestParameters case_ = GetParam();
  builder::BuilderConfiguration builder_config_;
};

TEST_P(RoadNetworkBuilderValidatorTest, RoadGeometryBuilding) {
  const std::unique_ptr<maliput::api::RoadNetwork> dut = builder::RoadNetworkBuilder(builder_config_.ToStringMap())();
  ASSERT_NE(dut->road_geometry(), nullptr);
  EXPECT_EQ(builder_config_.sparse_config.road_geometry_id, dut->road_geometry()->id());
  EXPECT_NO_THROW(maliput::api::ValidateRoadNetwork(
      *dut, {false, true /* Invariants */, false, false, false, false, false, false}));
}

INSTANTIATE_TEST_CASE_P(RoadNetworkBuilderValidatorTestGroup, RoadNetworkBuilderValidatorTest,
                        ::testing::ValuesIn(InstantiateBuilderParameters()));

// RoadNetworkBuilder constructs a RoadNetwork that holds:
// - RoadGeometry
// - RuleRegistry
// - RoadRulebook
// - TrafficLightBook
// - PhaseRingBook
// - IntersectionBook
// Except the RoadGeometry, the rest of the entities can be loaded via a YAML descriptions provided at runtime.
// This suite of tests guarantees that the RoadNetworkBuilder supports loading all the entities via YAML description.
//
// The map being used is loop_road_pedestrian_crosswalk.osm with its corresponding YAML description.
class RoadNetworkBuilderPopulationTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  const std::string map_id{"loop_road_pedestrian_crosswalk"};
  const std::string osm_file_path{utilities::FindOSMResource(map_id + ".osm")};
  const std::string rule_registry_path{utilities::FindOSMResource(map_id + ".yaml")};
  const std::string road_rulebook_path{utilities::FindOSMResource(map_id + ".yaml")};
  const std::string traffic_light_book_path{utilities::FindOSMResource(map_id + ".yaml")};
  const std::string phase_ring_book_path{utilities::FindOSMResource(map_id + ".yaml")};
  const std::string intersection_book_path{utilities::FindOSMResource(map_id + ".yaml")};
  const builder::BuilderConfiguration builder_config{builder::BuilderConfiguration::FromMap({
      {config::kOsmFile, osm_file_path},
      {config::kRuleRegistry, rule_registry_path},
      {config::kRoadRuleBook, road_rulebook_path},
      {config::kTrafficLightBook, traffic_light_book_path},
      {config::kPhaseRingBook, phase_ring_book_path},
      {config::kIntersectionBook, intersection_book_path},
  })};
};

TEST_F(RoadNetworkBuilderPopulationTest, Builder) {
  std::unique_ptr<maliput::api::RoadNetwork> dut;
  ASSERT_NO_THROW(dut = builder::RoadNetworkBuilder(builder_config.ToStringMap())());

  EXPECT_NE(nullptr, dut->road_geometry());

  EXPECT_FALSE(dut->rule_registry()->DiscreteValueRuleTypes().empty());
  EXPECT_FALSE(dut->rule_registry()->RangeValueRuleTypes().empty());

  const auto rules = dut->rulebook()->Rules();
  EXPECT_FALSE(rules.discrete_value_rules.empty());
  EXPECT_FALSE(rules.range_value_rules.empty());
  // New Rule API is being used therefore old rules aren't populated
  EXPECT_TRUE(rules.direction_usage.empty());
  EXPECT_TRUE(rules.speed_limit.empty());
  EXPECT_TRUE(rules.right_of_way.empty());

  // Traffic Light
  const auto traffic_lights = dut->traffic_light_book()->TrafficLights();
  EXPECT_EQ(4., static_cast<int>(traffic_lights.size()));

  // Phase Ring book
  const auto phase_rings = dut->phase_ring_book()->GetPhaseRings();
  EXPECT_EQ(2., static_cast<int>(phase_rings.size()));
  const auto phase_ring = dut->phase_ring_book()->GetPhaseRing(phase_rings[0]);
  ASSERT_NE(std::nullopt, phase_ring);
  const std::unordered_map<Phase::Id, Phase>& phases = phase_ring->phases();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // DiscreteValueRulesStates of any phases should be greater than zero while
  // RuleStates should be zero.
  EXPECT_TRUE(phases.begin()->second.rule_states().empty());
#pragma GCC diagnostic pop
  EXPECT_FALSE(phases.begin()->second.discrete_value_rule_states().empty());
  ASSERT_NE(std::nullopt, phases.begin()->second.bulb_states());
  EXPECT_FALSE(phases.begin()->second.bulb_states()->empty());

  // Intersection book
  EXPECT_EQ(2., static_cast<int>(dut->intersection_book()->GetIntersections().size()));
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace maliput_osm
