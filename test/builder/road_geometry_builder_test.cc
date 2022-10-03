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

#include "maliput_osm/osm/osm_manager.h"
#include "utilities/utilities.h"

namespace maliput_osm {
namespace builder {
namespace test {
namespace {

class RoadGeometryBuilderTest : public ::testing::Test {
 public:
  const std::string kOSMFilePath = utilities::FindOSMResource("straight_forward.osm");
};

TEST_F(RoadGeometryBuilderTest, Test) {
  BuilderConfiguration builder_config;
  builder_config.osm_file = kOSMFilePath;
  auto osm_manager = std::make_unique<osm::OSMManager>(kOSMFilePath, osm::ParserConfig{});
  std::unique_ptr<const maliput::api::RoadGeometry> dut;
  EXPECT_NO_THROW(dut = RoadGeometryBuilder(std::move(osm_manager), builder_config)());
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace maliput_osm
