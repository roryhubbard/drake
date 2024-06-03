#include "drake/examples/glider/glider_state.h"

namespace drake::examples::glider {

const std::vector<std::string>& GliderStateIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "x",
          "z",
          "pitch",
          "elevator",
          "xdot",
          "zdot",
          "pitchdot",
      });
  return coordinates.access();
}

}  // namespace drake::examples::glider
