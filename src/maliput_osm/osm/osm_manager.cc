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

#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <maliput/common/logger.h>

#include "maliput_osm/osm/conversions.h"

namespace maliput_osm {
namespace osm {

OSMManager::OSMManager(const std::string& osm_file_path, const ParserConfig& config) {
  using namespace lanelet;
  const LaneletMapPtr map = load(osm_file_path, Origin{GPSPoint{config.origin.x(), config.origin.y()}});

  for (const auto& lanelet : map->laneletLayer) {
    const Lane lane{ToMaliput(lanelet)};

    // TODO(#6): Support multiple lanes per segment. Organize adjacent lanes within segments.
    maliput::log()->warn("Each lane from {} populates only one segment. Lanes are not organized within segments yet.",
                         osm_file_path);

    const Segment::Id segment_id{"segment_" + std::to_string(lanelet.id())};
    segments_.emplace(segment_id, Segment{segment_id, {lane}});
  }
}

OSMManager::~OSMManager() = default;

const std::unordered_map<Segment::Id, Segment>& OSMManager::GetOSMSegments() const { return segments_; }

}  // namespace osm
}  // namespace maliput_osm
