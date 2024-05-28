#include "drake/examples/glider/glider_geometry.h"

#include <memory>
#include <utility>

#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace examples {
namespace glider {

using Eigen::Vector3d;
using Eigen::Vector4d;
const GliderGeometry& GliderGeometry::AddToBuilder(
    systems::DiagramBuilder<double>* builder,
    const Glider<double>& glider,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  auto glider_geometry = builder->AddSystem(
      std::unique_ptr<GliderGeometry>(new GliderGeometry(
          scene_graph, glider.num_particles(),
          glider.h())));
  builder->Connect(glider.get_output_port(0),
                   glider_geometry->get_input_port(0));
  builder->Connect(glider_geometry->get_output_port(0),
                   scene_graph->get_source_pose_port(
                       glider_geometry->source_id_));

  return *glider_geometry;
}

GliderGeometry::GliderGeometry(
    geometry::SceneGraph<double>* scene_graph, int num_particles, double h)
    : num_particles_(num_particles),
      // We set particle radius to be 80% of the gap between particles for
      // pleasing visual effects.
      particle_radius_(h * 0.8) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource();

  this->DeclareInputPort("particle_positions", systems::kVectorValued,
                         num_particles * 3);
  this->DeclareAbstractOutputPort(
      "geometry_pose", &GliderGeometry::OutputGeometryPose);

  frame_ids_.resize(num_particles);
  for (int i = 0; i < num_particles; ++i) {
    frame_ids_[i] = scene_graph->RegisterFrame(
        source_id_, geometry::GeometryFrame("particle" + std::to_string(i)));
    // Attach a sphere to the frame for simple visualization of the particles.
    const geometry::GeometryId id = scene_graph->RegisterGeometry(
        source_id_, frame_ids_[i],
        std::make_unique<geometry::GeometryInstance>(
            math::RigidTransformd::Identity(),
            std::make_unique<geometry::Sphere>(particle_radius_),
            "sphere_visual"));
    scene_graph->AssignRole(
        source_id_, id,
        geometry::MakePhongIllustrationProperties(Vector4d(1, 0, 1, 1)));
  }
}

void GliderGeometry::OutputGeometryPose(
    const systems::Context<double>& context,
    geometry::FramePoseVector<double>* poses) const {
  for (int i = 0; i < num_particles_; ++i) {
    DRAKE_DEMAND(frame_ids_[i].is_valid());
  }
  const auto& input = get_input_port(0).Eval(context);
  poses->clear();
  // Set the frames to the positions of the particles.
  for (int i = 0; i < static_cast<int>(frame_ids_.size()); ++i) {
    const double x = input(3 * i);
    const double y = input(3 * i + 1);
    const double z = input(3 * i + 2);
    const math::RigidTransformd pose(Vector3d(x, y, z));
    poses->set_value(frame_ids_[i], pose);
  }
}

}  // namespace glider
}  // namespace examples
}  // namespace drake
