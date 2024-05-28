#include "drake/examples/glider/glider_params.h"

namespace drake {
namespace examples {
namespace glider {

const int ClothSpringModelParamsIndices::kNumCoordinates;
const int ClothSpringModelParamsIndices::kMass;
const int ClothSpringModelParamsIndices::kK;
const int ClothSpringModelParamsIndices::kD;
const int ClothSpringModelParamsIndices::kGravity;

const std::vector<std::string>&
ClothSpringModelParamsIndices::GetCoordinateNames() {
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
