#include "drake/multibody/tree/prismatic_mobilizer.h"

#include <memory>
#include <stdexcept>

#include "drake/common/autodiff.h"
#include "drake/multibody/tree/body_node_impl.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
PrismaticMobilizer<T>::~PrismaticMobilizer() = default;

template <typename T>
std::unique_ptr<BodyNode<T>> PrismaticMobilizer<T>::CreateBodyNode(
    const BodyNode<T>* parent_node, const RigidBody<T>* body,
    const Mobilizer<T>* mobilizer) const {
  return std::make_unique<BodyNodeImpl<T, PrismaticMobilizer>>(parent_node,
                                                               body, mobilizer);
}

template <typename T>
std::string PrismaticMobilizer<T>::position_suffix(
    int position_index_in_mobilizer) const {
  if (position_index_in_mobilizer == 0) {
    return "x";
  }
  throw std::runtime_error("PrismaticMobilizer has only 1 position.");
}

template <typename T>
std::string PrismaticMobilizer<T>::velocity_suffix(
    int velocity_index_in_mobilizer) const {
  if (velocity_index_in_mobilizer == 0) {
    return "v";
  }
  throw std::runtime_error("PrismaticMobilizer has only 1 velocity.");
}

template <typename T>
const T& PrismaticMobilizer<T>::get_translation(
    const systems::Context<T>& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return q.coeffRef(0);
}

template <typename T>
const PrismaticMobilizer<T>& PrismaticMobilizer<T>::SetTranslation(
    systems::Context<T>* context, const T& translation) const {
  auto q = this->GetMutablePositions(context);
  DRAKE_ASSERT(q.size() == kNq);
  q[0] = translation;
  return *this;
}

template <typename T>
const T& PrismaticMobilizer<T>::get_translation_rate(
    const systems::Context<T>& context) const {
  const auto& v = this->get_velocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  return v.coeffRef(0);
}

template <typename T>
const PrismaticMobilizer<T>& PrismaticMobilizer<T>::SetTranslationRate(
    systems::Context<T>* context, const T& translation_dot) const {
  auto v = this->GetMutableVelocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  v[0] = translation_dot;
  return *this;
}

template <typename T>
math::RigidTransform<T> PrismaticMobilizer<T>::CalcAcrossMobilizerTransform(
    const systems::Context<T>& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return calc_X_FM(q.data());
}

template <typename T>
SpatialVelocity<T> PrismaticMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return calc_V_FM(nullptr, v.data());
}

template <typename T>
SpatialAcceleration<T>
PrismaticMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return calc_A_FM(nullptr, nullptr, vdot.data());
}

template <typename T>
void PrismaticMobilizer<T>::ProjectSpatialForce(
    const systems::Context<T>&, const SpatialForce<T>& F_BMo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  calc_tau(nullptr, F_BMo_F, tau.data());
}

template <typename T>
void PrismaticMobilizer<T>::DoCalcNMatrix(const systems::Context<T>&,
                                          EigenPtr<MatrixX<T>> N) const {
  (*N)(0, 0) = 1.0;
}

template <typename T>
void PrismaticMobilizer<T>::DoCalcNplusMatrix(
    const systems::Context<T>&, EigenPtr<MatrixX<T>> Nplus) const {
  (*Nplus)(0, 0) = 1.0;
}

template <typename T>
void PrismaticMobilizer<T>::MapVelocityToQDot(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);
  *qdot = v;
}

template <typename T>
void PrismaticMobilizer<T>::MapQDotToVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  DRAKE_ASSERT(qdot.size() == kNq);
  DRAKE_ASSERT(v != nullptr);
  DRAKE_ASSERT(v->size() == kNv);
  *v = qdot;
}

template <typename T>
void PrismaticMobilizer<T>::MapAccelerationToQDDot(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& vdot,
    EigenPtr<VectorX<T>> qddot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  DRAKE_ASSERT(qddot != nullptr);
  DRAKE_ASSERT(qddot->size() == kNq);
  *qddot = vdot;
}

template <typename T>
void PrismaticMobilizer<T>::MapQDDotToAcceleration(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& qddot,
    EigenPtr<VectorX<T>> vdot) const {
  DRAKE_ASSERT(qddot.size() == kNq);
  DRAKE_ASSERT(vdot != nullptr);
  DRAKE_ASSERT(vdot->size() == kNv);
  *vdot = qddot;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
PrismaticMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->outboard_frame());
  return std::make_unique<PrismaticMobilizer<ToScalar>>(
      tree_clone.get_mobod(this->mobod().index()), inboard_frame_clone,
      outboard_frame_clone, this->translation_axis());
}

template <typename T>
std::unique_ptr<Mobilizer<double>> PrismaticMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>> PrismaticMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<symbolic::Expression>>
PrismaticMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::PrismaticMobilizer);
