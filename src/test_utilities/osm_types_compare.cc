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
#include "test_utilities/osm_types_compare.h"

#include <maliput/test_utilities/maliput_math_compare.h>
#include <maliput_sparse/test_utilties/maliput_sparse_types_compare.h>

namespace maliput_osm {
namespace osm {
namespace test {

::testing::AssertionResult CompareOSMLane(const maliput_sparse::parser::Lane& lhs,
                                          const maliput_sparse::parser::Lane& rhs, double tolerance) {
  if (lhs.id != rhs.id) {
    return ::testing::AssertionFailure() << "Lane ids do not match: " << lhs.id << " != " << rhs.id;
  }
  if (!maliput_sparse::test::CompareLineString3d(lhs.left, rhs.left, tolerance)) {
    return ::testing::AssertionFailure() << "Lane left boundaries do not match: " << lhs.id;
  }
  if (!maliput_sparse::test::CompareLineString3d(lhs.right, rhs.right, tolerance)) {
    return ::testing::AssertionFailure() << "Lane right boundaries do not match: " << lhs.id;
  }
  if (lhs.left_lane_id != rhs.left_lane_id) {
    return ::testing::AssertionFailure() << "Lane left lane ids do not match: " << lhs.id;
  }
  if (lhs.right_lane_id != rhs.right_lane_id) {
    return ::testing::AssertionFailure() << "Lane right lane ids do not match: " << lhs.id;
  }
  if (lhs.successors != rhs.successors) {
    return ::testing::AssertionFailure() << "Lane successors do not match: " << lhs.id;
  }
  if (lhs.predecessors != rhs.predecessors) {
    return ::testing::AssertionFailure() << "Lane predecessors do not match: " << lhs.id;
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CompareOSMSegment(const maliput_sparse::parser::Segment& lhs,
                                             const maliput_sparse::parser::Segment& rhs, double tolerance) {
  if (lhs.id != rhs.id) {
    return ::testing::AssertionFailure() << "Segment ids do not match: " << lhs.id << " != " << rhs.id;
  }
  if (lhs.lanes.size() != rhs.lanes.size()) {
    return ::testing::AssertionFailure() << "Segment lane size mismatch: " << lhs.lanes.size() << " vs "
                                         << rhs.lanes.size();
  }
  for (size_t i = 0; i < lhs.lanes.size(); ++i) {
    if (!CompareOSMLane(lhs.lanes[i], rhs.lanes[i], tolerance)) {
      return ::testing::AssertionFailure() << "Segment lane mismatch: " << lhs.lanes[i].id << " != " << rhs.lanes[i].id;
    }
  }
  return ::testing::AssertionSuccess();
}

}  // namespace test
}  // namespace osm
}  // namespace maliput_osm
