#include "drake/multibody/tree/prismatic_mobilizer.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using math::RigidTransformd;
using std::make_unique;
using std::unique_ptr;
using systems::Context;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

// Fixture to setup a simple MBT model containing a prismatic mobilizer.
class PrismaticMobilizerTest : public MobilizerTester {
 public:
  void SetUp() override {
    slider_ = &AddJointAndFinalize<PrismaticJoint, PrismaticMobilizer>(
        std::make_unique<PrismaticJoint<double>>(
            "joint0", tree().world_body().body_frame(), body_->body_frame(),
            axis_F_));
  }

 protected:
  const PrismaticMobilizer<double>* slider_{nullptr};
  // Prismatic mobilizer axis, expressed in the inboard frame F.
  // It's intentionally left non-normalized to verify the mobilizer properly
  // normalizes it at construction.
  const Vector3d axis_F_{1.0, 2.0, 3.0};
};

TEST_F(PrismaticMobilizerTest, CanRotateOrTranslate) {
  EXPECT_FALSE(slider_->can_rotate());
  EXPECT_TRUE(slider_->can_translate());
}

// Verify that PrismaticMobilizer normalizes its axis on construction.
TEST_F(PrismaticMobilizerTest, AxisIsNormalizedAtConstruction) {
  EXPECT_TRUE(CompareMatrices(slider_->translation_axis(), axis_F_.normalized(),
                              kTolerance, MatrixCompareType::relative));
}

// Verifies method to mutate and access the context.
TEST_F(PrismaticMobilizerTest, StateAccess) {
  const double some_value1 = 1.5;
  const double some_value2 = std::sqrt(2);
  // Verify we can set a prismatic mobilizer position given the model's context.
  slider_->SetTranslation(context_.get(), some_value1);
  EXPECT_EQ(slider_->get_translation(*context_), some_value1);
  slider_->SetTranslation(context_.get(), some_value2);
  EXPECT_EQ(slider_->get_translation(*context_), some_value2);

  // Verify we can set a prismatic mobilizer position rate given the model's
  // context.
  slider_->SetTranslationRate(context_.get(), some_value1);
  EXPECT_EQ(slider_->get_translation_rate(*context_), some_value1);
  slider_->SetTranslationRate(context_.get(), some_value2);
  EXPECT_EQ(slider_->get_translation_rate(*context_), some_value2);
}

TEST_F(PrismaticMobilizerTest, ZeroState) {
  const double some_value1 = 1.5;
  const double some_value2 = std::sqrt(2);
  // Set the state to some arbitrary non-zero value.
  slider_->SetTranslation(context_.get(), some_value1);
  EXPECT_EQ(slider_->get_translation(*context_), some_value1);
  slider_->SetTranslationRate(context_.get(), some_value2);
  EXPECT_EQ(slider_->get_translation_rate(*context_), some_value2);

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // zero position and velocity.
  slider_->SetZeroState(*context_, &context_->get_mutable_state());
  EXPECT_EQ(slider_->get_translation(*context_), 0);
  EXPECT_EQ(slider_->get_translation_rate(*context_), 0);
}

TEST_F(PrismaticMobilizerTest, CalcAcrossMobilizerTransform) {
  const double kTol = 4 * std::numeric_limits<double>::epsilon();
  const double translation = 1.5;
  slider_->SetTranslation(context_.get(), translation);
  math::RigidTransformd X_FM(slider_->CalcAcrossMobilizerTransform(*context_));

  const math::RigidTransformd X_FM_expected(
      Vector3d(axis_F_.normalized() * translation));

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no rotations at all.
  EXPECT_EQ(X_FM.rotation().matrix(), Matrix3d::Identity());
  EXPECT_TRUE(CompareMatrices(X_FM.GetAsMatrix34(),
                              X_FM_expected.GetAsMatrix34(), kTolerance,
                              MatrixCompareType::relative));

  // Now check the fast inline methods.
  RigidTransformd fast_X_FM = slider_->calc_X_FM(&translation);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));
  const double new_translation = 3;
  slider_->SetTranslation(context_.get(), new_translation);
  X_FM = slider_->CalcAcrossMobilizerTransform(*context_);
  slider_->update_X_FM(&new_translation, &fast_X_FM);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));

  TestPrePostMultiplyByX_FM(X_FM, *slider_);
}

TEST_F(PrismaticMobilizerTest, CalcAcrossMobilizerSpatialVeloctiy) {
  const double translation_rate = 1.5;
  const SpatialVelocity<double> V_FM =
      slider_->CalcAcrossMobilizerSpatialVelocity(*context_,
                                                  Vector1d(translation_rate));

  const SpatialVelocity<double> V_FM_expected(
      Vector3d::Zero(), axis_F_.normalized() * translation_rate);

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no rotations at all.
  EXPECT_EQ(V_FM.rotational(), Vector3d::Zero());
  EXPECT_TRUE(V_FM.IsApprox(V_FM_expected, kTolerance));
}

TEST_F(PrismaticMobilizerTest, CalcAcrossMobilizerSpatialAcceleration) {
  const double translational_acceleration = 1.5;
  const SpatialAcceleration<double> A_FM =
      slider_->CalcAcrossMobilizerSpatialAcceleration(
          *context_, Vector1d(translational_acceleration));

  const SpatialAcceleration<double> A_FM_expected(
      Vector3d::Zero(), axis_F_.normalized() * translational_acceleration);

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no rotations at all.
  EXPECT_EQ(A_FM.rotational(), Vector3d::Zero());
  EXPECT_TRUE(A_FM.IsApprox(A_FM_expected, kTolerance));
}

TEST_F(PrismaticMobilizerTest, ProjectSpatialForce) {
  const Vector3d torque_Mo_F(1.0, 2.0, 3.0);
  const Vector3d force_Mo_F(1.0, 2.0, 3.0);
  const SpatialForce<double> F_Mo_F(torque_Mo_F, force_Mo_F);
  Vector1d tau;
  slider_->ProjectSpatialForce(*context_, F_Mo_F, tau);

  // Only the force along axis_F does work.
  const double tau_expected = force_Mo_F.dot(axis_F_.normalized());
  EXPECT_NEAR(tau(0), tau_expected, kTolerance);
}

TEST_F(PrismaticMobilizerTest, MapVelocityToQDotAndBack) {
  EXPECT_TRUE(slider_->is_velocity_equal_to_qdot());

  Vector1d v(1.5);
  Vector1d qdot;
  slider_->MapVelocityToQDot(*context_, v, &qdot);
  EXPECT_NEAR(qdot(0), v(0), kTolerance);

  qdot(0) = -std::sqrt(2);
  slider_->MapQDotToVelocity(*context_, qdot, &v);
  EXPECT_NEAR(v(0), qdot(0), kTolerance);

  // Test relationship between q̈ (2ⁿᵈ time derivatives of generalized positions)
  // and v̇ (1ˢᵗ time derivatives of generalized velocities) and vice-versa.
  Vector1d vdot(1.2345);
  Vector1d qddot;
  slider_->MapAccelerationToQDDot(*context_, vdot, &qddot);
  EXPECT_NEAR(qddot(0), vdot(0), kTolerance);

  qddot(0) = -std::sqrt(5);
  slider_->MapQDDotToAcceleration(*context_, qddot, &vdot);
  EXPECT_NEAR(vdot(0), qddot(0), kTolerance);
}

TEST_F(PrismaticMobilizerTest, KinematicMapping) {
  // For this joint, Nplus = 1 independently of the state. We therefore set the
  // state to NaN in order to verify this.
  tree()
      .GetMutablePositionsAndVelocities(context_.get())
      .setConstant(std::numeric_limits<double>::quiet_NaN());

  // Compute N.
  MatrixX<double> N(1, 1);
  slider_->CalcNMatrix(*context_, &N);
  EXPECT_EQ(N(0, 0), 1.0);

  // Compute Nplus.
  MatrixX<double> Nplus(1, 1);
  slider_->CalcNplusMatrix(*context_, &Nplus);
  EXPECT_EQ(Nplus(0, 0), 1.0);
}

TEST_F(PrismaticMobilizerTest, MapUsesN) {
  // Set an arbitrary "non-zero" state.
  slider_->SetTranslation(context_.get(), 1.5);

  // Set arbitrary v and MapVelocityToQDot.
  Vector1d v(1.5);
  Vector1d qdot;
  slider_->MapVelocityToQDot(*context_, v, &qdot);

  // Compute N.
  MatrixX<double> N(1, 1);
  slider_->CalcNMatrix(*context_, &N);

  // Ensure N(q) is used in `q̇ = N(q)⋅v`
  EXPECT_EQ(qdot, N * v);
}

TEST_F(PrismaticMobilizerTest, MapUsesNplus) {
  // Set an arbitrary "non-zero" state.
  slider_->SetTranslation(context_.get(), 1.5);

  // Set arbitrary qdot and MapQDotToVelocity.
  Vector1d qdot(1.5);
  Vector1d v;
  slider_->MapQDotToVelocity(*context_, qdot, &v);

  // Compute Nplus.
  MatrixX<double> Nplus(1, 1);
  slider_->CalcNplusMatrix(*context_, &Nplus);

  // Ensure N⁺(q) is used in `v = N⁺(q)⋅q̇`
  EXPECT_EQ(v, Nplus * qdot);
}
}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
