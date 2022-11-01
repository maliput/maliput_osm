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
#include "maliput_osm/builder/builder_configuration.h"

#include "maliput_osm/builder/road_network_builder.h"

namespace maliput_osm {
namespace builder {

BuilderConfiguration BuilderConfiguration::FromMap(const std::map<std::string, std::string>& config) {
  BuilderConfiguration builder_config;
  auto it = config.find(config::kRoadGeometryId);
  if (it != config.end()) {
    builder_config.road_geometry_id = maliput::api::RoadGeometryId(it->second);
  }

  it = config.find(config::kOsmFile);
  if (it != config.end()) {
    builder_config.osm_file = it->second;
  }

  it = config.find(config::kLinearTolerance);
  if (it != config.end()) {
    builder_config.linear_tolerance = std::stod(it->second);
  }

  it = config.find(config::kAngularTolerance);
  if (it != config.end()) {
    builder_config.angular_tolerance = std::stod(it->second);
  }

  it = config.find(config::kScaleLength);
  if (it != config.end()) {
    builder_config.scale_length = std::stod(it->second);
  }

  it = config.find(config::kOrigin);
  if (it != config.end()) {
    builder_config.origin = maliput::math::Vector2::FromStr(it->second);
  }

  it = config.find(config::kInertialToBackendFrameTranslation);
  if (it != config.end()) {
    builder_config.inertial_to_backend_frame_translation = maliput::math::Vector3::FromStr(it->second);
  }

  it = config.find(config::kRuleRegistry);
  if (it != config.end()) {
    builder_config.rule_registry = std::make_optional(it->second);
  }
  it = config.find(config::kRoadRuleBook);
  if (it != config.end()) {
    builder_config.road_rule_book = std::make_optional(it->second);
  }
  it = config.find(config::kTrafficLightBook);
  if (it != config.end()) {
    builder_config.traffic_light_book = std::make_optional(it->second);
  }
  it = config.find(config::kPhaseRingBook);
  if (it != config.end()) {
    builder_config.phase_ring_book = std::make_optional(it->second);
  }
  it = config.find(config::kIntersectionBook);
  if (it != config.end()) {
    builder_config.intersection_book = std::make_optional(it->second);
  }

  return builder_config;
}

std::map<std::string, std::string> BuilderConfiguration::ToStringMap() const {
  std::map<std::string, std::string> config;
  config.emplace(config::kRoadGeometryId, road_geometry_id.string());
  config.emplace(config::kOsmFile, osm_file);
  config.emplace(config::kOrigin, origin.to_str());
  config.emplace(config::kLinearTolerance, std::to_string(linear_tolerance));
  config.emplace(config::kAngularTolerance, std::to_string(angular_tolerance));
  config.emplace(config::kScaleLength, std::to_string(scale_length));
  config.emplace(config::kInertialToBackendFrameTranslation, inertial_to_backend_frame_translation.to_str());
  if (rule_registry.has_value()) {
    config.emplace(config::kRuleRegistry, rule_registry.value());
  }
  if (road_rule_book.has_value()) {
    config.emplace(config::kRoadRuleBook, road_rule_book.value());
  }
  if (traffic_light_book.has_value()) {
    config.emplace(config::kTrafficLightBook, traffic_light_book.value());
  }
  if (phase_ring_book.has_value()) {
    config.emplace(config::kPhaseRingBook, phase_ring_book.value());
  }
  if (intersection_book.has_value()) {
    config.emplace(config::kIntersectionBook, intersection_book.value());
  }
  return config;
}

}  // namespace builder
}  // namespace maliput_osm
