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
#include "maliput_osm/osm/osm_manager.h"

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#include <lanelet2_io/Io.h>
#include <maliput/common/filesystem.h>

#include "test_utilities/osm_types_compare.h"
#include "utilities/utilities.h"

using maliput_sparse::geometry::LineString3d;

namespace maliput_osm {
namespace osm {
namespace test {
namespace {

struct OSMManagerTestCase {
  std::string osm_file;
  std::unordered_map<Junction::Id, Junction> junctions;
};

std::vector<OSMManagerTestCase> OSMManagerTestCases() {
  return {
      {"straight_forward.osm",
       {{Junction::Id{"0"},
         {Junction::Id{"0"},
          {{Segment::Id{"0"},
            {Segment::Id{"0"},
             {
                 {Lane::Id{"1010"},
                  LineString3d{{0., -500., 0.}, {0., 500., 0.}},
                  LineString3d{{3.5, -500., 0.}, {3.5, 500., 0.}},
                  {std::make_optional<Lane::Id>("1006")},
                  std::nullopt},
                 {Lane::Id{"1006"},
                  LineString3d{{-3.5, -500., 0.}, {-3.5, 500., 0.}},
                  LineString3d{{0., -500., 0.}, {0., 500., 0.}},
                  std::nullopt,
                  {std::make_optional<Lane::Id>("1010")}},
             }}}}}}}},
  };
}

class OSMMangerTest : public ::testing::TestWithParam<OSMManagerTestCase> {
 protected:
  const std::string kOSMFilePath = utilities::FindOSMResource(GetParam().osm_file);
  const ParserConfig kParserConfig = {{0., 0.} /* origin */};
  const OSMManagerTestCase case_ = GetParam();
};

TEST_P(OSMMangerTest, Test) {
  const OSMManager dut{kOSMFilePath, kParserConfig};
  const auto osm_junctions = dut.GetOSMJunctions();
  EXPECT_EQ(case_.junctions.size(), osm_junctions.size());

  for (const auto& osm_junction : osm_junctions) {
    EXPECT_EQ(case_.junctions.at(osm_junction.first).segments.size(), osm_junction.second.segments.size());
    for (const auto& case_segment : case_.junctions.at(osm_junction.first).segments) {
      const auto it = osm_junction.second.segments.find(case_segment.first);
      ASSERT_TRUE(it != osm_junction.second.segments.end());
      EXPECT_TRUE(CompareOSMSegment(case_segment.second, it->second, 1e-5));
    }
  }
}

INSTANTIATE_TEST_CASE_P(OSMMangerTestGroup, OSMMangerTest, ::testing::ValuesIn(OSMManagerTestCases()));

}  // namespace
}  // namespace test
}  // namespace osm
}  // namespace maliput_osm
