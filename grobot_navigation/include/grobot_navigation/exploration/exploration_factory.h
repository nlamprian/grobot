#ifndef GROBOT_NAVIGATION_EXPLORATION_EXPLORATION_FACTORY_H
#define GROBOT_NAVIGATION_EXPLORATION_EXPLORATION_FACTORY_H

#include <grobot_navigation/exploration/move_to_target_exploration.h>
#include <grobot_navigation/exploration/rotate_exploration.h>

namespace exploration {

class ExplorationFactory {
 public:
  static ExplorationBasePtr get(ros::NodeHandle &nh) {
    std::string strategy;
    if (not nh.getParam("strategy", strategy))
      std::runtime_error("Failed to get strategy");

    ExplorationBasePtr exploration;
    switch (exploration::parseStrategy(strategy)) {
      case Strategy::MOVE_TO_TARGET: {
        exploration = std::make_shared<MoveToTargetExploration>();
      } break;
      case Strategy::ROTATE: {
        exploration = std::make_shared<RotateExploration>();
      } break;
    }

    bool inited = exploration->init(nh);
    if (not inited) std::runtime_error("Failed to initialize exploration");
    return exploration;
  }

 private:
  ExplorationFactory() = delete;
};

}  // namespace exploration

#endif  // GROBOT_NAVIGATION_EXPLORATION_EXPLORATION_FACTORY_H
