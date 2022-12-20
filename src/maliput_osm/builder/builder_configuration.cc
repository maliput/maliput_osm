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

#include "maliput_osm/builder/params.h"

namespace maliput_osm {
namespace builder {

BuilderConfiguration BuilderConfiguration::FromMap(const std::map<std::string, std::string>& config) {
  BuilderConfiguration builder_config;
  builder_config.sparse_config = maliput_sparse::loader::BuilderConfiguration::FromMap(config);

  auto it = config.find(params::kOsmFile);
  if (it != config.end()) {
    builder_config.osm_file = it->second;
  }

  it = config.find(params::kOrigin);
  if (it != config.end()) {
    builder_config.origin = maliput::math::Vector2::FromStr(it->second);
  }

  return builder_config;
}

std::map<std::string, std::string> BuilderConfiguration::ToStringMap() const {
  std::map<std::string, std::string> config = sparse_config.ToStringMap();
  config.emplace(params::kOsmFile, osm_file);
  config.emplace(params::kOrigin, origin.to_str());
  return config;
}

}  // namespace builder
}  // namespace maliput_osm
