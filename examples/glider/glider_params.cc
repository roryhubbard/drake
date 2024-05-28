#include "drake/examples/glider/glider_params.h"

namespace drake {
namespace examples {
namespace glider {

const int GliderParamsIndices::kNumCoordinates;
const int GliderParamsIndices::kMass;
const int GliderParamsIndices::kK;
const int GliderParamsIndices::kD;
const int GliderParamsIndices::kGravity;

const std::vector<std::string>&
GliderParamsIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "mass",
          "k",
          "d",
          "gravity",
      });
  return coordinates.access();
}

}  // namespace glider
}  // namespace examples
}  // namespace drake
