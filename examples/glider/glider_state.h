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

/// Describes the row indices of a GliderState.
struct GliderStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kTheta1 = 0;
  static const int kTheta2 = 1;
  static const int kTheta1dot = 2;
  static const int kTheta2dot = 3;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `GliderStateIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
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
    this->set_theta1(0.0);
    this->set_theta2(0.0);
    this->set_theta1dot(0.0);
    this->set_theta2dot(0.0);
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
    this->set_theta1(symbolic::Variable("theta1"));
    this->set_theta2(symbolic::Variable("theta2"));
    this->set_theta1dot(symbolic::Variable("theta1dot"));
    this->set_theta2dot(symbolic::Variable("theta2dot"));
  }

  [[nodiscard]] GliderState<T>* DoClone() const final {
    return new GliderState;
  }

  /// @name Getters and Setters
  //@{
  /// The shoulder joint angle
  /// @note @c theta1 is expressed in units of rad.
  const T& theta1() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kTheta1);
  }
  /// Setter that matches theta1().
  void set_theta1(const T& theta1) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kTheta1, theta1);
  }
  /// Fluent setter that matches theta1().
  /// Returns a copy of `this` with theta1 set to a new value.
  [[nodiscard]] GliderState<T> with_theta1(const T& theta1) const {
    GliderState<T> result(*this);
    result.set_theta1(theta1);
    return result;
  }
  /// The elbow joint angle
  /// @note @c theta2 is expressed in units of rad.
  const T& theta2() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kTheta2);
  }
  /// Setter that matches theta2().
  void set_theta2(const T& theta2) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kTheta2, theta2);
  }
  /// Fluent setter that matches theta2().
  /// Returns a copy of `this` with theta2 set to a new value.
  [[nodiscard]] GliderState<T> with_theta2(const T& theta2) const {
    GliderState<T> result(*this);
    result.set_theta2(theta2);
    return result;
  }
  /// The shoulder joint velocity
  /// @note @c theta1dot is expressed in units of rad/s.
  const T& theta1dot() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kTheta1dot);
  }
  /// Setter that matches theta1dot().
  void set_theta1dot(const T& theta1dot) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kTheta1dot, theta1dot);
  }
  /// Fluent setter that matches theta1dot().
  /// Returns a copy of `this` with theta1dot set to a new value.
  [[nodiscard]] GliderState<T> with_theta1dot(const T& theta1dot) const {
    GliderState<T> result(*this);
    result.set_theta1dot(theta1dot);
    return result;
  }
  /// The elbow joint velocity
  /// @note @c theta2dot is expressed in units of rad/s.
  const T& theta2dot() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kTheta2dot);
  }
  /// Setter that matches theta2dot().
  void set_theta2dot(const T& theta2dot) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kTheta2dot, theta2dot);
  }
  /// Fluent setter that matches theta2dot().
  /// Returns a copy of `this` with theta2dot set to a new value.
  [[nodiscard]] GliderState<T> with_theta2dot(const T& theta2dot) const {
    GliderState<T> result(*this);
    result.set_theta2dot(theta2dot);
    return result;
  }
  //@}

  /// Visit each field of this named vector, passing them (in order) to the
  /// given Archive.  The archive can read and/or write to the vector values.
  /// One common use of Serialize is the //common/yaml tools.
  template <typename Archive>
  void Serialize(Archive* a) {
    T& theta1_ref = this->GetAtIndex(K::kTheta1);
    a->Visit(drake::MakeNameValue("theta1", &theta1_ref));
    T& theta2_ref = this->GetAtIndex(K::kTheta2);
    a->Visit(drake::MakeNameValue("theta2", &theta2_ref));
    T& theta1dot_ref = this->GetAtIndex(K::kTheta1dot);
    a->Visit(drake::MakeNameValue("theta1dot", &theta1dot_ref));
    T& theta2dot_ref = this->GetAtIndex(K::kTheta2dot);
    a->Visit(drake::MakeNameValue("theta2dot", &theta2dot_ref));
  }

  /// See GliderStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return GliderStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(theta1());
    result = result && !isnan(theta2());
    result = result && !isnan(theta1dot());
    result = result && !isnan(theta2dot());
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
