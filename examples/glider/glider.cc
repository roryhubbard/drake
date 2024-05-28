#include "drake/examples/glider/glider.h"

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake::examples::glider {

/// Describes the row indices of a AcrobotState.
struct GliderStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 7;

  // The index of each individual coordinate.
  static const int kX = 0;
  static const int kZ = 1;
  static const int kPitch = 2;
  static const int kElevator = 3;
  static const int kXdot = 4;
  static const int kZdot = 5;
  static const int kPitchDot = 6;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `GliderStateIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

template <typename T>
class GliderState final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef AcrobotStateIndices K;

  GliderState() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    T x;
    T z;
    T pitch;
    T elevator;
    T xdot;
    T zdot;
    T pitchdot;
  }
}

template <typename T>
Glider<T>::Glider() : systems::LeafSystem<T>(systems::SystemTypeTag<Glider>{}) {

  // one inputs (elevator_velocity)
  this->DeclareVectorInputPort("elevatordot", 1);
  this->DeclareContinuousState(7);
  // seven outputs (full state)
  this->DeclareVectorOutputPort("state", 7, &Glider::CopyStateOut);
  this->DeclareVectorOutputPort("forces", 2, &Glider::OutputForces);

  // parameters based on Rick Cory's "R1 = no dihedral" model.
  this->Sw = 0.0885; //  surface area of wing + fuselage + tail.
  this->Se = 0.0147; // surface area of elevator.
  this->lw = 0; // horizontal offset of wing center.
  this->le = 0.022; //  elevator aerodynamic center from hinge.
  // this->lh = 0.317 // elevator hinge.
  this->lh = 0.27; //  elevator hinge.
  this->inertia = 0.0015; // body inertia.
  this->m = 0.08; //  body mass.
  this->rho = 1.204; // air density (kg/m^3).
  this->gravity = 9.81; // gravity
  // TODO(russt): Declare elevator constraints:
  this->elevator_lower_limit = -0.9473;
  this->elevator_upper_limit = 0.4463;
}

template <typename T>
auto Glider::ComputeForces(
    const systems::Context<T>& context) const -> const std::pair<T, T> {
  const auto s = GliderState(
      context.get_mutable_continuous_state_vector().CopyToVector()
  )

  const systems::VectorBase<T>& s = context.get_continuous_state_vector();
  const T elevatordot = this->EvalVectorInput(context, 0)[0];

  const eps = 1e-10;
  const T xwdot = s.xdot + this->lw * s.pitchdot * std::sin(s.pitch);
  const T zwdot = s.zdot + this->lw * s.pitchdot * std::cos(s.pitch);
  const T vw = std::sqrt(zwdot**2 + xwdot**2 + eps);
  const T fw = (
      -this->rho
      * this->Sw
      * (std::sin(s.pitch) * xwdot + std::cos(s.pitch) * zwdot)
      * vw
  );

  const T e = s.pitch + s.elevator;
  const T edot = s.pitchdot + elevatordot;
  const T xedot = (
      s.xdot
      + this->lh * s.pitchdot * std::sin(s.pitch)
      + this->le * edot * std::sin(e)
  );
  const T zedot = (
      s.zdot
      + this->lh * s.pitchdot * std::cos(s.pitch)
      + this->le * edot * std::cos(e)
  );
  const T ve = std::sqrt(zedot**2 + xedot**2 + eps);
  const T fe = -this->rho * this->Se * (std::sin(e) * xedot + std::cos(e) * zedot) * ve;

  return std::make_pair(fw, fe);
}

template <typename T>
void Glider<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
}

}  // namespace drake::examples::glider
   //
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::glider::Glider)
