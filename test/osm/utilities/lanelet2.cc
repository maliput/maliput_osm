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
#include "osm/utilities/lanelet2.h"

#include <lanelet2_core/primitives/Lanelet.h>

namespace maliput_osm {
namespace osm {
namespace test {
namespace utilities {

using lanelet::Lanelet;
using lanelet::LineString3d;
using lanelet::Point3d;
using lanelet::Points3d;
using lanelet::utils::getId;

lanelet::LaneletMapPtr CreateLanelet2Map() {
  const Points3d points_1{Point3d{getId(), 0., 0., 0.}, Point3d{getId(), 10., 0., 0.}};
  const Points3d points_2{Point3d{getId(), 0., -2., 0.}, Point3d{getId(), 10., -2., 0.}};
  const Points3d points_3{Point3d{getId(), 0., -4., 0.}, Point3d{getId(), 10., -4., 0.}};
  const Points3d points_4{Point3d{getId(), 0., -6., 0.}, Point3d{getId(), 10., -6., 0.}};
  const LineString3d ls_1{getId(), points_1};
  const LineString3d ls_2{getId(), points_2};
  const LineString3d ls_3{getId(), points_3};
  const LineString3d ls_4{getId(), points_4};
  const Lanelet lanelet_1_2{12, ls_1, ls_2};
  const Lanelet lanelet_2_3{23, ls_2, ls_3};
  const Lanelet lanelet_3_4{34, ls_3, ls_4};
  const Points3d points_5{points_1.back(), Point3d{getId(), 20., 0., 0.}};
  const Points3d points_6{points_2.back(), Point3d{getId(), 20., -2., 0.}};
  const Points3d points_7{points_3.back(), Point3d{getId(), 20., -4., 0.}};
  const Points3d points_8{points_4.back(), Point3d{getId(), 20., -6., 0.}};
  const LineString3d ls_5{getId(), points_5};
  const LineString3d ls_6{getId(), points_6};
  const LineString3d ls_7{getId(), points_7};
  const LineString3d ls_8{getId(), points_8};
  const Lanelet lanelet_5_6{56, ls_5, ls_6};
  const Lanelet lanelet_6_7{67, ls_6, ls_7};
  const Lanelet lanelet_7_8{78, ls_7, ls_8};
  const Points3d points_9{points_6.back(), Point3d{getId(), 30., -2., 0.}};
  const Points3d points_10{points_7.back(), Point3d{getId(), 30., -4., 0.}};
  const LineString3d ls_9{getId(), points_9};
  const LineString3d ls_10{getId(), points_10};
  const Lanelet lanelet_9_10{910, ls_9, ls_10};
  const Points3d points_11{points_9.back(), Point3d{getId(), 40., 0., 0.}};
  const Points3d points_12{points_10.back(), Point3d{getId(), 40., -2., 0.}};
  const LineString3d ls_11{getId(), points_11};
  const LineString3d ls_12{getId(), points_12};
  const Lanelet lanelet_11_12{1112, ls_11, ls_12};
  const Points3d points_13{points_9.back(), Point3d{getId(), 40., -4, 0.}};
  const Points3d points_14{points_10.back(), Point3d{getId(), 40., -6., 0.}};
  const LineString3d ls_13{getId(), points_13};
  const LineString3d ls_14{getId(), points_14};
  const Lanelet lanelet_13_14{1314, ls_13, ls_14};

  return lanelet::utils::createMap({lanelet_1_2, lanelet_2_3, lanelet_3_4, lanelet_5_6, lanelet_6_7, lanelet_7_8,
                                    lanelet_9_10, lanelet_11_12, lanelet_13_14});
}

lanelet::LaneletMapPtr CreateInvertedLaneMap() {
  const Points3d points_1{Point3d{getId(), 0., 0., 0.}, Point3d{getId(), 10., 0., 0.}};
  const Points3d points_2{Point3d{getId(), 0., -2., 0.}, Point3d{getId(), 10., -2., 0.}};
  const Points3d points_3{Point3d{getId(), 0., -4., 0.}, Point3d{getId(), 10., -4., 0.}};
  const Points3d points_4{Point3d{getId(), 0., -6., 0.}, Point3d{getId(), 10., -6., 0.}};
  const Points3d points_5{Point3d{getId(), 20., 0., 0.}, points_1.back()};
  const Points3d points_6{Point3d{getId(), 20., -2., 0.}, points_2.back()};
  const Points3d points_7{Point3d{getId(), 20., -4., 0.}, points_3.back()};
  const Points3d points_8{Point3d{getId(), 20., -6., 0.}, points_4.back()};
  const Points3d points_9{points_5.front(), Point3d{getId(), 30., 0., 0.}};
  const Points3d points_10{points_6.front(), Point3d{getId(), 30., -2., 0.}};
  const Points3d points_11{points_7.front(), Point3d{getId(), 30., -4., 0.}};
  const Points3d points_12{points_8.front(), Point3d{getId(), 30., -6., 0.}};
  const LineString3d ls_1{getId(), points_1};
  const LineString3d ls_2{getId(), points_2};
  const LineString3d ls_3{getId(), points_3};
  const LineString3d ls_4{getId(), points_4};
  const LineString3d ls_5{getId(), points_5};
  const LineString3d ls_6{getId(), points_6};
  const LineString3d ls_7{getId(), points_7};
  const LineString3d ls_8{getId(), points_8};
  const LineString3d ls_9{getId(), points_9};
  const LineString3d ls_10{getId(), points_10};
  const LineString3d ls_11{getId(), points_11};
  const LineString3d ls_12{getId(), points_12};
  const Lanelet lanelet_1_2{12, ls_1, ls_2};
  const Lanelet lanelet_2_3{23, ls_2, ls_3};
  const Lanelet lanelet_3_4{34, ls_3, ls_4};
  const Lanelet lanelet_6_5{65, ls_6, ls_5};
  const Lanelet lanelet_7_6{76, ls_7, ls_6};
  const Lanelet lanelet_8_7{87, ls_8, ls_7};
  const Lanelet lanelet_9_10{910, ls_9, ls_10};
  const Lanelet lanelet_10_11{1011, ls_10, ls_11};
  const Lanelet lanelet_11_12{1112, ls_11, ls_12};

  return lanelet::utils::createMap({lanelet_1_2, lanelet_2_3, lanelet_3_4, lanelet_6_5, lanelet_7_6, lanelet_8_7,
                                    lanelet_9_10, lanelet_10_11, lanelet_11_12});
}

}  // namespace utilities
}  // namespace test
}  // namespace osm
}  // namespace maliput_osm
