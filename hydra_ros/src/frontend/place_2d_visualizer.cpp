/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_ros/frontend/place_2d_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/global_info.h>
#include <hydra_visualizer/drawing.h>
#include <kimera_pgmo/mesh_delta.h>

#include <tf2_eigen/tf2_eigen.hpp>

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<Place2dSegmenter::Sink,
                                   Place2dVisualizer,
                                   Place2dVisualizer::Config>("Place2dVisualizer");

template <typename Scalar>
inline void fillPoint(const Eigen::Matrix<Scalar, 3, 1>& v,
                      geometry_msgs::msg::Point& p) {
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
}

}  // namespace

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void declare_config(Place2dVisualizer::Config& config) {
  using namespace config;
  name("Place2dVisualizer::Config");
  field(config.module_ns, "module_ns");
  field(config.scale, "scale");
  field(config.alpha, "alpha");
}

Place2dVisualizer::Place2dVisualizer(const Config& config)
    : config(config::checkValid(config)),
      nh_(ianvs::NodeHandle::this_node(config.module_ns)),
      pubs_(nh_) {}

std::string Place2dVisualizer::printInfo() const { return config::toString(config); }

struct DeltaPointAdaptor : spark_dsg::BoundingBox::PointAdaptor {
  DeltaPointAdaptor(const kimera_pgmo::MeshDelta& delta,
                    const std::vector<size_t>& indices)
      : delta(delta), indices(indices) {}

  size_t size() const override { return indices.size(); }

  Eigen::Vector3f get(size_t index) const override {
    return delta.getVertex(indices.at(index)).pos;
  }
  const kimera_pgmo::MeshDelta& delta;
  const std::vector<size_t>& indices;
};

void Place2dVisualizer::call(uint64_t timestamp_ns,
                             const kimera_pgmo::MeshDelta& delta,
                             const kimera_pgmo::MeshOffsetInfo& offsets,
                             const Place2dSegmenter::LabelPlaces& label_places) const {
  pubs_.publish("active_places", [&]() {
    auto markers = std::make_unique<MarkerArray>();
    markers->markers.resize(2);

    auto& boundaries = markers->markers[0];
    boundaries.header.stamp = rclcpp::Time(timestamp_ns);
    boundaries.header.frame_id = GlobalInfo::instance().getFrames().odom;
    boundaries.ns = "active_place_boundaries";
    boundaries.id = 0;
    boundaries.type = Marker::LINE_LIST;
    boundaries.action = Marker::ADD;
    boundaries.scale.x = config.scale;
    boundaries.pose.orientation.w = 1.0;

    auto& points = markers->markers[1];
    points.header.stamp = rclcpp::Time(timestamp_ns);
    points.header.frame_id = GlobalInfo::instance().getFrames().odom;
    points.ns = "active_place_points";
    points.id = 0;
    points.type = Marker::CUBE_LIST;
    points.action = Marker::ADD;
    points.scale.x = config.scale;
    points.scale.y = config.scale;
    points.scale.z = config.scale;
    points.pose.orientation.w = 1.0;

    std_msgs::msg::ColorRGBA active;
    active.r = 0.1;
    active.g = 0.6;
    active.b = 0.2;
    active.a = config.alpha;
    std_msgs::msg::ColorRGBA frozen;
    frozen.r = 0.8;
    frozen.g = 0.4;
    frozen.b = 0.35;
    frozen.a = config.alpha;

    for (const auto& [label, places] : label_places) {
      for (const auto& place : places) {
        const auto is_frozen = place.min_mesh_index < offsets.archived_vertices;
        for (size_t i = 1; i < place.boundary.size(); ++i) {
          auto& p1 = boundaries.points.emplace_back();
          fillPoint(place.boundary[i - 1], p1);
          boundaries.colors.push_back(is_frozen ? frozen : active);
          auto& p2 = boundaries.points.emplace_back();
          fillPoint(place.boundary[i], p2);
          boundaries.colors.push_back(is_frozen ? frozen : active);
        }

        for (const auto idx : place.indices) {
          if (idx < offsets.archived_vertices) {
            continue;
          }

          auto& p = points.points.emplace_back();
          fillPoint(delta.getVertex(offsets.toLocalVertex(idx)).pos, p);
          points.colors.push_back(is_frozen ? frozen : active);
        }
      }
    }

    return markers;
  });
}

}  // namespace hydra
