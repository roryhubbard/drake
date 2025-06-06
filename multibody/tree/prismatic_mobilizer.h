#pragma once

#include <limits>
#include <memory>
#include <string>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/mobilizer_impl.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

// This Mobilizer allows two frames to translate relative to one another
// along an axis whose direction is constant when measured in either this
// mobilizer's inboard frame or its outboard frame. There is no relative
// rotation between the inboard and outboard frames, just translation.
// To fully specify this mobilizer, a user must provide the inboard frame F,
// the outboard (or "mobilized") frame M and the axis `axis_F` (expressed in
// frame F) along which frame M translates with respect to frame F.
// The single generalized coordinate q introduced by this mobilizer
// corresponds to the translation distance (in meters) of the origin `Mo` of
// frame M with respect to frame F along `axis_F`. When `q = 0`, frames F and M
// are coincident. The translation distance is defined to be positive in the
// direction of `axis_F`.
//
// H_FM₆ₓ₁=[0₃, axis_F]ᵀ     Hdot_FM₆ₓ₁ = 0
//
// @tparam_default_scalar
template <typename T>
class PrismaticMobilizer final : public MobilizerImpl<T, 1, 1> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrismaticMobilizer);
  using MobilizerBase = MobilizerImpl<T, 1, 1>;
  using MobilizerBase::kNq, MobilizerBase::kNv, MobilizerBase::kNx;
  template <typename U>
  using QVector = typename MobilizerBase::template QVector<U>;
  template <typename U>
  using VVector = typename MobilizerBase::template VVector<U>;
  template <typename U>
  using HMatrix = typename MobilizerBase::template HMatrix<U>;

  // Constructor for a %PrismaticMobilizer between the `inboard_frame_F` and
  // `outboard_frame_M` granting a single translational degree of freedom along
  // `axis_F`, expressed in the `inboard_frame_F`.
  // @pre `axis_F` must be a non-zero vector with norm at least root square of
  // machine epsilon. This vector can have any length (subject to the norm
  // restriction above), only the direction is used.
  // @throws std::exception if the L2 norm of `axis_F` is less than the square
  // root of machine epsilon.
  PrismaticMobilizer(const SpanningForest::Mobod& mobod,
                     const Frame<T>& inboard_frame_F,
                     const Frame<T>& outboard_frame_M,
                     const Vector3<double>& axis_F)
      : MobilizerBase(mobod, inboard_frame_F, outboard_frame_M),
        axis_F_(axis_F) {
    double kEpsilon = std::sqrt(std::numeric_limits<double>::epsilon());
    DRAKE_DEMAND(!axis_F.isZero(kEpsilon));
    axis_F_.normalize();
  }

  ~PrismaticMobilizer() final;

  std::unique_ptr<BodyNode<T>> CreateBodyNode(
      const BodyNode<T>* parent_node, const RigidBody<T>* body,
      const Mobilizer<T>* mobilizer) const final;

  // Overloads to define the suffix names for the position and velocity
  // elements.
  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  bool can_rotate() const final { return false; }
  bool can_translate() const final { return true; }

  // @retval axis_F The translation axis as a unit vector expressed in the
  // inboard frame F.
  const Vector3<double>& translation_axis() const { return axis_F_; }

  // Gets the translational distance for `this` mobilizer from `context`. See
  // class documentation for sign convention details.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @returns The translation coordinate of `this` mobilizer in the `context`.
  const T& get_translation(const systems::Context<T>& context) const;

  // Sets `context` so that the generalized coordinate corresponding to the
  // translation for `this` mobilizer equals `translation`.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @param[in] translation The desired translation in meters.
  // @returns a constant reference to `this` mobilizer.
  const PrismaticMobilizer<T>& SetTranslation(systems::Context<T>* context,
                                              const T& translation) const;

  // Gets the rate of change, in meters per second, of `this` mobilizer's
  // translation (see get_translation()) from `context`. See class
  // documentation for the translation sign convention.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @returns The rate of change of `this` mobilizer's translation in the
  // `context`.
  const T& get_translation_rate(const systems::Context<T>& context) const;

  // Sets the rate of change, in meters per second, of `this` mobilizer's
  // translation to `translation_dot`. The new rate of change `translation_dot`
  // gets stored in `context`.
  // See class documentation for the translation sign convention.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @param[in] translation_dot The desired rate of change of `this`
  // mobilizer's translation in meters per second.
  // @returns a constant reference to `this` mobilizer.
  const PrismaticMobilizer<T>& SetTranslationRate(
      systems::Context<T>* context, const T& translation_dot) const;

  // Computes the across-mobilizer transform `X_FM(q)` between the inboard
  // frame F and the outboard frame M as a function of the translation distance
  // along this mobilizer's axis (see translation_axis().)
  math::RigidTransform<T> calc_X_FM(const T* q) const {
    return math::RigidTransform<T>(q[0] * translation_axis());
  }

  /* We're not yet attempting to optimize the X_FM update. */
  // TODO(sherm1) Optimize this.
  void update_X_FM(const T* q, math::RigidTransform<T>* X_FM) const {
    DRAKE_ASSERT(q != nullptr && X_FM != nullptr);
    *X_FM = calc_X_FM(q);
  }

  // Computes the across-mobilizer velocity `V_FM(q, v)` of the outboard frame
  // M measured and expressed in frame F as a function of the input
  // translational velocity v along this mobilizer's axis (see
  // translation_axis()).
  SpatialVelocity<T> calc_V_FM(const T*, const T* v) const {
    return SpatialVelocity<T>(Vector3<T>::Zero(), v[0] * translation_axis());
  }

  SpatialAcceleration<T> calc_A_FM(const T*, const T*, const T* vdot) const {
    return SpatialAcceleration<T>(Vector3<T>::Zero(),
                                  vdot[0] * translation_axis());
  }

  // Returns tau = H_FMᵀ⋅F, where H_FMᵀ = [0₃ᵀ axis_Fᵀ].
  void calc_tau(const T*, const SpatialForce<T>& F_BMo_F, T* tau) const {
    DRAKE_ASSERT(tau != nullptr);
    const Vector3<T>& f_BMo_F = F_BMo_F.translational();
    tau[0] = axis_F_.dot(f_BMo_F);
  }

  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const final;

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const final;

  // Computes the across-mobilizer acceleration `A_FM(q, v, v̇)` of the
  // outboard frame M in the inboard frame F.
  // By definition `A_FM = d_F(V_FM)/dt`. The acceleration `A_FM` will be a
  // function of the translation distance q, its rate of change v for the
  // current state in `context` and of the input generalized acceleration
  // `v̇ = dv/dt`, the rate of change of v.
  // See class documentation for the translation sign convention.
  // This method aborts in Debug builds if `vdot.size()` is not one.
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const final;

  // Projects the spatial force `F_Mo_F` on `this` mobilizer's outboard
  // frame M onto its translation axis (see translation_axis().)
  // Mathematically: <pre>
  //    tau = F_Mo_F.translational().dot(axis_F)
  // </pre>
  // Therefore, the result of this method is the scalar value of the linear
  // force along the axis of `this` mobilizer.
  // This method aborts in Debug builds if `tau.size()` is not one.
  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_Mo_F,
                           Eigen::Ref<VectorX<T>> tau) const final;

  bool is_velocity_equal_to_qdot() const override { return true; }

  // Maps v to qdot, which for this mobilizer is q̇ = v.
  void MapVelocityToQDot(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& v,
                         EigenPtr<VectorX<T>> qdot) const final;

  // Maps qdot to v, which for this mobilizer is v = q̇.
  void MapQDotToVelocity(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         EigenPtr<VectorX<T>> v) const final;

  // Maps vdot to qddot, which for this mobilizer is q̈ = v̇.
  void MapAccelerationToQDDot(const systems::Context<T>& context,
                              const Eigen::Ref<const VectorX<T>>& vdot,
                              EigenPtr<VectorX<T>> qddot) const final;

  // Maps qddot to vdot, which for this mobilizer is v̇ = q̈.
  void MapQDDotToAcceleration(const systems::Context<T>& context,
                              const Eigen::Ref<const VectorX<T>>& qddot,
                              EigenPtr<VectorX<T>> vdot) const final;

 protected:
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const final;

  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const final;

  std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const final;

 private:
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // Default axis expressed in the inboard frame F. It is a unit vector.
  Vector3<double> axis_F_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::PrismaticMobilizer);
