#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/examples/glider/glider_state.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake::examples::glider {

template <typename T>
class Glider final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Glider)

  Glider();

 private:
  auto ComputeForces(const systems::Context<T>& context) const -> const std::pair<T, T>;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  auto CopyStateOut(const systems::Context<T>& context, GliderState<T> output);

  auto OutputForces(const systems::Context<T>&context, GliderState<T> output);

  T Sw; //  surface area of wing + fuselage + tail.
  T Se; // surface area of elevator.
  T lw; // horizontal offset of wing center.
  T le; //  elevator aerodynamic center from hinge.
  T lh; //  elevator hinge.
  T inertia; // body inertia.
  T m; //  body mass.
  T rho; // air density (kg/m^3).
  T gravity; // gravity
  // TODO(russt): Declare elevator constraints:
  T elevator_lower_limit;
  T elevator_upper_limit;
};
}  // namespace drake::examples::glider
