#include "hydra_visualizer/adapters/mesh_color.h"
#include "hydra_visualizer/color/color_parsing.h"
#include <spark_dsg/colormaps.h>
#include <config_utilities/config.h>

namespace hydra {

using spark_dsg::Color;
using spark_dsg::Mesh;

void declare_config(UniformMeshColoring::Config& config) {
  using namespace config;
  name("UniformMeshColoring::Config");
  field(config.color, "color");
}

UniformMeshColoring::UniformMeshColoring(const spark_dsg::Color& color)
    : UniformMeshColoring(Config{color}) {}

UniformMeshColoring::UniformMeshColoring(const Config& config) : config(config) {}

void declare_config(SemanticMeshColoring::Config& config) {
  using namespace config;
  name("SemanticMeshColoring::Config");
  field(config.colormap, "colormap");
}

SemanticMeshColoring::SemanticMeshColoring(const Config& config)
    : config(config), colormap_(config.colormap) {}

Color SemanticMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  if (!mesh.has_labels) {
    return Color::black();
  }

  return colormap_.getColor(mesh.label(i));
}

void declare_config(FirstSeenMeshColoring::Config&) {
  config::name("FirstSeenMeshColoring::Config");
}

FirstSeenMeshColoring::FirstSeenMeshColoring() : FirstSeenMeshColoring(Config()) {}

FirstSeenMeshColoring::FirstSeenMeshColoring(const Config&) {}

void FirstSeenMeshColoring::setMesh(const Mesh& mesh) {
  min_ = std::numeric_limits<Mesh::Timestamp>::max();
  max_ = std::numeric_limits<Mesh::Timestamp>::min();
  for (const auto t : mesh.first_seen_stamps) {
    if (t < min_ && t > 0) {
      min_ = t;
    }
    if (t > max_) {
      max_ = t;
    }
  }
}

Color FirstSeenMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  return colorFromTime(mesh.first_seen_stamps[i], min_, max_);
}

void FirstSeenMeshColoring::setBounds(Mesh::Timestamp min, Mesh::Timestamp max) {
  min_ = min;
  max_ = max;
}

void declare_config(LastSeenMeshColoring::Config&) {
  config::name("LastSeenMeshColoring::Config");
}

LastSeenMeshColoring::LastSeenMeshColoring() : LastSeenMeshColoring(Config()) {}

LastSeenMeshColoring::LastSeenMeshColoring(const Config&) {}

void LastSeenMeshColoring::setMesh(const Mesh& mesh) {
  min_ = std::numeric_limits<Mesh::Timestamp>::max();
  max_ = std::numeric_limits<Mesh::Timestamp>::min();
  for (const auto t : mesh.stamps) {
    if (t < min_ && t > 0) {
      min_ = t;
    }
    if (t > max_) {
      max_ = t;
    }
  }
}

Color LastSeenMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  return colorFromTime(mesh.stamps[i], min_, max_);
}

void LastSeenMeshColoring::setBounds(Mesh::Timestamp min, Mesh::Timestamp max) {
  min_ = min;
  max_ = max;
}

void declare_config(SeenDurationMeshColoring::Config&) {
  config::name("SeenDurationMeshColoring::Config");
}

SeenDurationMeshColoring::SeenDurationMeshColoring()
    : SeenDurationMeshColoring(Config()) {}

SeenDurationMeshColoring::SeenDurationMeshColoring(const Config&) {}

void SeenDurationMeshColoring::setMesh(const Mesh& mesh) {
  max_ = 0;
  for (size_t i = 0; i < mesh.stamps.size(); ++i) {
    const auto duration = mesh.stamps[i] - mesh.first_seen_stamps[i];
    if (duration > max_) {
      max_ = duration;
    }
  }
}

Color SeenDurationMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  return colorFromTime(mesh.stamps[i] - mesh.first_seen_stamps[i], 0, max_);
}

void SeenDurationMeshColoring::setMaxDuration(spark_dsg::Mesh::Timestamp max) {
  max_ = max;
}

void declare_config(FusionCountMeshColoring::Config&) {
  config::name("FusionCountMeshColoring::Config");
}

FusionCountMeshColoring::FusionCountMeshColoring() : FusionCountMeshColoring(Config()) {}

FusionCountMeshColoring::FusionCountMeshColoring(const Config&) {}

void FusionCountMeshColoring::setMesh(const Mesh& mesh) {
  if (!mesh.has_fusion_counts || mesh.fusion_counts.empty()) {
    min_count_ = 2;
    max_count_ = 2;
    return;
  }

  // Calculate range for counts > 1 (count=0 is gray, count=1 is green)
  min_count_ = std::numeric_limits<uint32_t>::max();
  max_count_ = 0;
  for (const auto count : mesh.fusion_counts) {
    if (count > 1) {  // Only consider multi-fused vertices for palette range
      if (count < min_count_) {
        min_count_ = count;
      }
      if (count > max_count_) {
        max_count_ = count;
      }
    }
  }
  if (min_count_ > max_count_) {
    min_count_ = 2;
    max_count_ = 2;
  }
}

Color FusionCountMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  static const visualizer::RainbowPalette palette;

  if (!mesh.has_fusion_counts || i >= mesh.fusion_counts.size()) {
    return Color::gray();
  }

  const uint32_t count = mesh.fusion_counts[i];
  if (count == 0) {
    return Color::gray();  // Never fused
  }
  if (count == 1) {
    return Color::green();  // Fused once
  }

  // Fused multiple times: use rainbow palette (red=low to violet=high)
  if (count <= min_count_) {
    return palette(0);
  } else if (count >= max_count_) {
    return palette(1);
  }

  const float normalized =
      static_cast<float>(count - min_count_) / static_cast<float>(max_count_ - min_count_);
  return palette(normalized);
}

void declare_config(TemporalIslandMeshColoring::Config&) {
  config::name("TemporalIslandMeshColoring::Config");
}

TemporalIslandMeshColoring::TemporalIslandMeshColoring()
    : TemporalIslandMeshColoring(Config()) {}

TemporalIslandMeshColoring::TemporalIslandMeshColoring(const Config&) {}

Color TemporalIslandMeshColoring::getVertexColor(const Mesh& mesh, size_t i) const {
  if (!mesh.has_temporal_island_ids || i >= mesh.temporal_island_ids.size()) {
    return Color::gray();  // No island data available
  }

  const uint32_t island_id = mesh.temporal_island_ids[i];
  if (island_id == 0) {
    return Color::gray();  // No island assignment
  }

  return spark_dsg::colormaps::distinct150Id(island_id);
}

MeshColorAdapter::MeshColorAdapter(const Mesh& mesh, MeshColoring::ConstPtr coloring)
    : mesh(mesh), coloring_(std::move(coloring)) {
  if (coloring_) {
    const_cast<MeshColoring&>(*coloring_).setMesh(mesh);
    getVertexColor = [this](size_t i) {
      return coloring_->getVertexColor(this->mesh, i);
    };
  } else {
    getVertexColor = [this](size_t i) { return this->mesh.color(i); };
  }
}

Color colorFromTime(Mesh::Timestamp time, Mesh::Timestamp min, Mesh::Timestamp max) {
  static const visualizer::IronbowPalette palette;
  if (time == 0) {
    return Color::green();
  }

  if (time <= min) {
    return palette(0);
  } else if (time >= max) {
    return palette(1);
  }

  return palette(static_cast<double>(time - min) / static_cast<double>(max - min));
}

}  // namespace hydra
