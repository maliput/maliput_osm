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
#include "test_utilities/builder_configuration_for_osm.h"

#include <unordered_map>

#include <maliput/api/road_geometry.h>
#include <maliput/math/vector.h>

#include "utilities/utilities.h"

namespace maliput_osm {
namespace test {

std::optional<builder::BuilderConfiguration> GetBuilderConfigurationFor(const std::string& osm_file_name) {
  const static maliput::math::Vector3 kZeroVector{0., 0., 0.};

  const static std::unordered_map<std::string, builder::BuilderConfiguration> kOSMConfigurations{
      {"straight_forward.osm",
       builder::BuilderConfiguration{
           maliput::api::RoadGeometryId{"straight_forward"},
           utilities::FindOSMResource("straight_forward.osm"),
           {0., 0.} /* origin */,
           5e-2 /* linear_tolerance */,
           1e-3 /* angular_tolerance */,
           1. /* scale_length */,
           kZeroVector,
       }},

  };

  return kOSMConfigurations.find(osm_file_name) != kOSMConfigurations.end()
             ? std::make_optional<builder::BuilderConfiguration>(kOSMConfigurations.at(osm_file_name))
             : std::nullopt;
}

}  // namespace test
}  // namespace maliput_osm
