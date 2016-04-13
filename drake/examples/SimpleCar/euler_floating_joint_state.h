// This file is generated by drake/examples/SimpleCar/lcm_vector_gen.py. Do not edit.
#ifndef DRAKE_EXAMPLES_SIMPLECAR_EULER_FLOATING_JOINT_STATE_H_
#define DRAKE_EXAMPLES_SIMPLECAR_EULER_FLOATING_JOINT_STATE_H_

#include <stdexcept>
#include <Eigen/Core>

#include "lcmtypes/drake/lcmt_euler_floating_joint_state_t.hpp"

namespace Drake {

template <typename ScalarType = double>
class EulerFloatingJointState {  // models the Drake::LCMVector concept
 public:
  typedef drake::lcmt_euler_floating_joint_state_t LCMMessageType;
  static std::string channel() { return "EULER_FLOATING_JOINT_STATE"; }
  static const int RowsAtCompileTime = 6;
  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;

  EulerFloatingJointState() {}

  template <typename Derived>
  EulerFloatingJointState(const Eigen::MatrixBase<Derived>& initial)
      : x(initial(0)),
        y(initial(1)),
        z(initial(2)),
        roll(initial(3)),
        pitch(initial(4)),
        yaw(initial(5)) {}

  template <typename Derived>
  EulerFloatingJointState& operator=(const Eigen::MatrixBase<Derived>& rhs) {
    x = rhs(0);
    y = rhs(1);
    z = rhs(2);
    roll = rhs(3);
    pitch = rhs(4);
    yaw = rhs(5);
    return *this;
  }

  friend EigenType toEigen(const EulerFloatingJointState<ScalarType>& vec) {
    EigenType result;
    result << vec.x, vec.y, vec.z, vec.roll, vec.pitch, vec.yaw;
    return result;
  }

  friend std::string getCoordinateName(const EulerFloatingJointState<ScalarType>& vec,
                                       unsigned int index) {
    switch (index) {
      case 0: return "x";
      case 1: return "y";
      case 2: return "z";
      case 3: return "roll";
      case 4: return "pitch";
      case 5: return "yaw";
    }
    throw std::domain_error("unknown coordinate index");
  }

  ScalarType x = 0;
  ScalarType y = 0;
  ScalarType z = 0;
  ScalarType roll = 0;
  ScalarType pitch = 0;
  ScalarType yaw = 0;
};

template <typename ScalarType>
bool encode(const double& t, const EulerFloatingJointState<ScalarType>& wrap,
            drake::lcmt_euler_floating_joint_state_t& msg) {
  msg.timestamp = static_cast<int64_t>(t * 1000);
  msg.x = wrap.x;
  msg.y = wrap.y;
  msg.z = wrap.z;
  msg.roll = wrap.roll;
  msg.pitch = wrap.pitch;
  msg.yaw = wrap.yaw;
  return true;
}

template <typename ScalarType>
bool decode(const drake::lcmt_euler_floating_joint_state_t& msg, double& t,
            EulerFloatingJointState<ScalarType>& wrap) {
  t = double(msg.timestamp) / 1000.0;
  wrap.x = msg.x;
  wrap.y = msg.y;
  wrap.z = msg.z;
  wrap.roll = msg.roll;
  wrap.pitch = msg.pitch;
  wrap.yaw = msg.yaw;
  return true;
}

}

#endif  // DRAKE_EXAMPLES_SIMPLECAR_EULER_FLOATING_JOINT_STATE_H_
