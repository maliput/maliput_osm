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
  EXPECT_EQ(builder_config_.road_geometry_id, dut->road_geometry()->id());
  EXPECT_NO_THROW(maliput::api::ValidateRoadNetwork(
      *dut, {false, true /* Invariants */, false, false, false, false, false, false}));
}

INSTANTIATE_TEST_CASE_P(RoadNetworkBuilderValidatorTestGroup, RoadNetworkBuilderValidatorTest,
                        ::testing::ValuesIn(InstantiateBuilderParameters()));

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace maliput_osm
