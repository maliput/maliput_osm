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
#include "maliput_osm/osm/conversions.h"

#include <string>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <maliput_sparse/geometry/line_string.h>

#include "maliput_osm/osm/lane.h"

namespace maliput_osm {
namespace osm {
namespace test {
namespace {

TEST(ToMaliput, LineString) {
  const std::vector<maliput::math::Vector3> expected_points{{1., 1., 1.}, {2., 2., 2.}, {3., 3., 3.}};
  const lanelet::Point3d p1{1, expected_points[0].x(), expected_points[0].y(), expected_points[0].z()};
  const lanelet::Point3d p2{2, expected_points[1].x(), expected_points[1].y(), expected_points[1].z()};
  const lanelet::Point3d p3{3, expected_points[2].x(), expected_points[2].y(), expected_points[2].z()};
  const lanelet::LineString3d ls(4., {p1, p2, p3});
  EXPECT_EQ(maliput_sparse::geometry::LineString3d{expected_points}, ToMaliput(ls));
}

TEST(ToMaliput, Lanelet) {
  const std::vector<maliput::math::Vector3> points_left{{1., 1., 1.}, {2., 2., 2.}, {3., 3., 3.}};
  const std::vector<maliput::math::Vector3> points_right{{3., 3., 3.}, {4., 4., 4.}, {5., 5., 5.}};
  constexpr int kLaneIdNumber{10.};
  const Lane::Id lane_id{std::to_string(kLaneIdNumber)};
  const Lane lane{lane_id, maliput_sparse::geometry::LineString3d{points_left},
                  maliput_sparse::geometry::LineString3d{points_right}};

  const lanelet::LineString3d lanelet_left(
      4, {lanelet::Point3d{1, points_left[0].x(), points_left[0].y(), points_left[0].z()},
          lanelet::Point3d{2, points_left[1].x(), points_left[1].y(), points_left[1].z()},
          lanelet::Point3d{3, points_left[2].x(), points_left[2].y(), points_left[2].z()}});
  const lanelet::LineString3d lanelet_right(
      8, {lanelet::Point3d{5, points_right[0].x(), points_right[0].y(), points_right[0].z()},
          lanelet::Point3d{6, points_right[1].x(), points_right[1].y(), points_right[1].z()},
          lanelet::Point3d{7, points_right[2].x(), points_right[2].y(), points_right[2].z()}});
  const lanelet::Lanelet lanelet(kLaneIdNumber, lanelet_left, lanelet_right);

  EXPECT_EQ(lane, ToMaliput(lanelet));
}

}  // namespace
}  // namespace test
}  // namespace osm
}  // namespace maliput_osm
