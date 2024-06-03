#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_bool.h"
#include "drake/common/dummy_value.h"
#include "drake/common/name_value.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic/expression.h"
#include "drake/systems/framework/basic_vector.h"

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
  static const int kPitchdot = 6;

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
  typedef GliderStateIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c theta1 defaults to 0.0 rad.
  /// @arg @c theta2 defaults to 0.0 rad.
  /// @arg @c theta1dot defaults to 0.0 rad/s.
  /// @arg @c theta2dot defaults to 0.0 rad/s.
  GliderState() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_x(0.0);
    this->set_z(0.0);
    this->set_pitch(0.0);
    this->set_elevator(0.0);
    this->set_xdot(0.0);
    this->set_zdot(0.0);
    this->set_pitchdot(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  GliderState(const GliderState& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  GliderState(GliderState&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  GliderState& operator=(const GliderState& other) {
    this->values() = other.values();
    return *this;
  }
  GliderState& operator=(GliderState&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }
  //@}

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if_t<std::is_same_v<U, symbolic::Expression>>
  SetToNamedVariables() {
    this->set_x(symbolic::Variable("x"));
    this->set_z(symbolic::Variable("z"));
    this->set_pitch(symbolic::Variable("pitch"));
    this->set_elevator(symbolic::Variable("elevator"));
    this->set_xdot(symbolic::Variable("xdot"));
    this->set_zdot(symbolic::Variable("ydot"));
    this->set_pitchdot(symbolic::Variable("pitchdot"));
  }

  [[nodiscard]] GliderState<T>* DoClone() const final {
    return new GliderState;
  }

  const T& x() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kX);
  }
  void set_x(const T& x) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kX, x);
  }

  const T& z() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kZ);
  }
  void set_z(const T& z) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kZ, z);
  }

  const T& pitch() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kPitch);
  }
  void set_pitch(const T& pitch) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kPitch, pitch);
  }

  const T& elevator() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kElevator);
  }
  void set_elevator(const T& elevator) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kElevator, elevator);
  }

  const T& xdot() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kXdot);
  }
  void set_xdot(const T& xdot) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kXdot, xdot);
  }

  const T& zdot() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kZdot);
  }
  void set_zdot(const T& zdot) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kZdot, zdot);
  }

  const T& pitchdot() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kPitchdot);
  }
  void set_pitchdot(const T& pitchdot) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kPitchdot, pitchdot);
  }

  /// See GliderStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return GliderStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(x());
    result = result && !isnan(z());
    result = result && !isnan(pitch());
    result = result && !isnan(elevator());
    result = result && !isnan(xdot());
    result = result && !isnan(zdot());
    result = result && !isnan(pitchdot());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The GliderState vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace drake::examples::glider
