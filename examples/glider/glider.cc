#include "drake/examples/glider/glider.h"

#include <cmath>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

using std::cos;
using std::sin;
using std::pow;
using std::sqrt;

namespace drake::examples::glider {

template <typename T>
Glider<T>::Glider() : systems::LeafSystem<T>(systems::SystemTypeTag<Glider>{}) {
  // one inputs (elevator_velocity)
  this->DeclareVectorInputPort("elevatordot", 1);
  this->DeclareContinuousState(7);
  // seven outputs (full state)
  this->DeclareVectorOutputPort("state", 7, &Glider<T>::CopyStateOut);
  this->DeclareVectorOutputPort("forces", 2, &Glider<T>::OutputForces);

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
auto Glider<T>::ComputeForces(
    const systems::Context<T>& context) const -> const std::pair<T, T> {
  const GliderState<T> s = GliderState<T>(
      context.get_mutable_continuous_state_vector().CopyToVector()
  );

  const T elevatordot = this->EvalVectorInput(context, 0)[0];

  const T eps = 1e-10;
  const T xwdot = s.xdot + this->lw * s.pitchdot * sin(s.pitch);
  const T zwdot = s.zdot + this->lw * s.pitchdot * cos(s.pitch);
  const T vw = sqrt(pow(zwdot, 2) + pow(xwdot, 2) + eps);
  const T fw = (
      -this->rho
      * this->Sw
      * (sin(s.pitch) * xwdot + cos(s.pitch) * zwdot)
      * vw
  );

  const T e = s.pitch + s.elevator;
  const T edot = s.pitchdot + elevatordot;
  const T xedot = (
      s.xdot
      + this->lh * s.pitchdot * sin(s.pitch)
      + this->le * edot * sin(e)
  );
  const T zedot = (
      s.zdot
      + this->lh * s.pitchdot * cos(s.pitch)
      + this->le * edot * cos(e)
  );
  const T ve = sqrt(pow(zedot, 2) + pow(xedot, 2) + eps);
  const T fe = -this->rho * this->Se * (sin(e) * xedot + cos(e) * zedot) * ve;

  return std::make_pair(fw, fe);
}

template <typename T>
void Glider<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const GliderState<T> s = GliderState<T>(
      context.get_mutable_continuous_state_vector().CopyToVector()
  );
  const T elevatordot = this->EvalVectorInput(context, 0)[0];
  const T e = s.pitch + s.elevator;
  const auto [fw, fe] = this->ComputeForces(context);

  GliderState<T> sdot = GliderState(s);
//  sdot[0:3] = s[4:7]
//  sdot.elevator = elevatordot
//  sdot.xdot = (fw * sin(s.pitch) + fe * sin(e)) / this->m
//  sdot.zdot = (fw * cos(s.pitch) + fe * cos(e)) / this->m - this->gravity
//  sdot.pitchdot = (
//      fw * this->lw + fe * (this->lh * cos(s.elevator) + this->le)
//  ) / this->inertia
//  derivatives.get_mutable_vector().SetFromVector(sdot);
}

template <typename T>
auto Glider<T>::CopyStateOut(
  const systems::Context<T>& context, GliderState<T> output)
{
  const VectorX<T> x = context.get_continuous_state_vector().CopyToVector();
  output.SetFromVector(x);
}

template <typename T>
auto Glider<T>::OutputForces(
  const systems::Context<T>&context, GliderState<T> output)
{
  const auto [fw, fe] = this->ComputeForces(context);
  output.SetFromVector(Vector2<T>(fw, fe));
}

}  // namespace drake::examples::glider

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::glider::Glider)
