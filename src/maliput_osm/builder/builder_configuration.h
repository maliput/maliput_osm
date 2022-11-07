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
#pragma once

#include <maliput/api/road_geometry.h>
#include <maliput/math/vector.h>

namespace maliput_osm {
namespace builder {

/// Holds the configuration parameters for the builder.
struct BuilderConfiguration {
  /// Create a BuilderConfiguration from a string dictionary.
  static BuilderConfiguration FromMap(const std::map<std::string, std::string>& config);

  /// @details The keys of the map are listed at @ref builder_configuration_keys.
  /// @returns A string-string map containing the builder configuration.
  std::map<std::string, std::string> ToStringMap() const;

  /// Id of the road geometry.
  maliput::api::RoadGeometryId road_geometry_id{"maliput_osm"};
  /// Path to the OSM file.
  std::string osm_file{""};
  /// Origin in lat/lon coordinates of the OSM file description.
  maliput::math::Vector2 origin{0., 0.};
  /// Linear resolution of the road geometry.
  double linear_tolerance{1.e-3};
  /// Angular resolution of the road geometry.
  double angular_tolerance{1.e-3};
  /// Scale factor for road geometry.
  double scale_length{1.};
  /// Translation from maliput to maliput_osm inertial frame.
  maliput::math::Vector3 inertial_to_backend_frame_translation{};
  /// Path to the configuration file to load a RuleRegistry
  std::optional<std::string> rule_registry{std::nullopt};
  /// Path to the configuration file to load a RoadRulebook
  std::optional<std::string> road_rule_book{std::nullopt};
  /// Path to the configuration file to load a TrafficLightBook.
  std::optional<std::string> traffic_light_book{std::nullopt};
  /// Path to the configuration file to load a PhaseRingBook.
  std::optional<std::string> phase_ring_book{std::nullopt};
  /// Path to the configuration file to load an IntersectionBook.
  std::optional<std::string> intersection_book{std::nullopt};
};

}  // namespace builder
}  // namespace maliput_osm
