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

#include <memory>

#include <maliput/base/intersection_book.h>
#include <maliput/base/intersection_book_loader.h>
#include <maliput/base/manual_discrete_value_rule_state_provider.h>
#include <maliput/base/manual_phase_ring_book.h>
#include <maliput/base/manual_range_value_rule_state_provider.h>
#include <maliput/base/manual_rulebook.h>
#include <maliput/base/phase_based_right_of_way_rule_state_provider.h>
#include <maliput/base/phase_ring_book_loader.h>
#include <maliput/base/rule_registry.h>
#include <maliput/base/traffic_light_book.h>
#include <maliput/base/traffic_light_book_loader.h>
#include <maliput/common/logger.h>
#include <maliput/common/maliput_unused.h>

#include "maliput_osm/builder/builder_configuration.h"
#include "maliput_osm/builder/road_geometry_builder.h"
#include "maliput_osm/osm/osm_manager.h"

namespace maliput_osm {
namespace builder {

std::unique_ptr<maliput::api::RoadNetwork> RoadNetworkBuilder::operator()() const {
  const BuilderConfiguration builder_config{BuilderConfiguration::FromMap(builder_config_)};

  maliput::log()->info("Loading database from file: {} ...", builder_config.osm_file);

  std::unique_ptr<osm::OSMManager> osm_manager =
      std::make_unique<osm::OSMManager>(builder_config.osm_file, osm::ParserConfig{builder_config.origin});
  maliput::log()->trace("Building RoadGeometry...");
  std::unique_ptr<const maliput::api::RoadGeometry> rg = RoadGeometryBuilder(std::move(osm_manager), builder_config)();

  maliput::log()->trace("Building TrafficLightBook...");
  auto traffic_light_book = std::make_unique<maliput::TrafficLightBook>();
  maliput::log()->trace("Built TrafficLightBook.");

  maliput::log()->trace("Building RuleRegistry...");
  auto rule_registry = std::make_unique<maliput::api::rules::RuleRegistry>();
  maliput::log()->trace("Built RuleRegistry...");

  maliput::log()->trace("Building RuleRoadBook...");

  auto rule_book = std::make_unique<maliput::ManualRulebook>();
  maliput::log()->trace("Built RuleRoadBook.");

  maliput::log()->trace("Building PhaseRingBook...");
  auto phase_ring_book = std::make_unique<maliput::ManualPhaseRingBook>();
  maliput::log()->trace("Built PhaseRingBook.");

  maliput::log()->trace("Building PhaseProvider...");
  auto manual_phase_provider = std::make_unique<maliput::ManualPhaseProvider>();
  maliput::log()->trace("Built PhaseProvider.");

  maliput::log()->trace("Building DiscreteValueRuleStateProvider...");
  auto discrete_value_rule_state_provider =
      std::make_unique<maliput::ManualDiscreteValueRuleStateProvider>(rule_book.get());
  maliput::log()->trace("Built DiscreteValueRuleStateProvider.");

  maliput::log()->trace("Building RangeValueRuleStateProvider...");
  auto range_value_rule_state_provider = std::make_unique<maliput::ManualRangeValueRuleStateProvider>(rule_book.get());
  maliput::log()->trace("Built RangeValueRuleStateProvider.");

  maliput::log()->trace("Building IntersectionBook...");
  auto intersection_book = std::make_unique<maliput::IntersectionBook>(rg.get());
  maliput::log()->trace("Built IntersectionBook.");

  maliput::log()->trace("Building RuleStateProvider...");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  auto state_provider = std::make_unique<maliput::PhaseBasedRightOfWayRuleStateProvider>(phase_ring_book.get(),
                                                                                         manual_phase_provider.get());
#pragma GCC diagnostic pop
  maliput::log()->trace("Built RuleStateProvider.");

  return std::make_unique<maliput::api::RoadNetwork>(
      std::move(rg), std::move(rule_book), std::move(traffic_light_book), std::move(intersection_book),
      std::move(phase_ring_book), std::move(state_provider), std::move(manual_phase_provider), std::move(rule_registry),
      std::move(discrete_value_rule_state_provider), std::move(range_value_rule_state_provider));
}

}  // namespace builder
}  // namespace maliput_osm
