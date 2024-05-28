#pragma once

#include <limits>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/glider/glider_params.h"
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
};
}  // namespace drake::examples::glider
