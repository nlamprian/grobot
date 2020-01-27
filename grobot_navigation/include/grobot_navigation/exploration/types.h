#ifndef GROBOT_NAVIGATION_EXPLORATION_TYPES_H
#define GROBOT_NAVIGATION_EXPLORATION_TYPES_H

#include <string>

namespace exploration {

enum class Strategy : uint8_t { INVALID, MOVE_TO_TARGET, ROTATE };

static Strategy parseStrategy(const std::string &strategy) {
  if (strategy == "move_to_target") {
    return Strategy::MOVE_TO_TARGET;
  } else if (strategy == "rotate") {
    return Strategy::ROTATE;
  } else {
    return Strategy::INVALID;
  }
}

}  // namespace exploration

#endif  // GROBOT_NAVIGATION_EXPLORATION_TYPES_H
