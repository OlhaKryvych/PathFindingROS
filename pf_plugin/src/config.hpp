#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace CONFIG {

[[maybe_unused]] static const int MAP_WIDTH                     = 4000; // Default map width
[[maybe_unused]] static const int MAP_HEIGHT                    = 4000; // Default map height
[[maybe_unused]] static const double WORLD_X_MIN                = -100.0; // Gazebo X Axis min value
[[maybe_unused]] static const double WORLD_X_MAX                = 100.0; // Gazebo X Axis max value
[[maybe_unused]] static const double WORLD_Y_MIN                = -100.0; // Gazebo Y Axis min value
[[maybe_unused]] static const double WORLD_Y_MAX                = 100.0; // Gazebo Y Axis max value

[[maybe_unused]] static const int MOVE_TIMER_INTERVAL_MS        = 60000; // Maximum number of ms to reach target goal

[[maybe_unused]] static const char* MAP_TOPIC                   = "map"; // default '/map' topic name
[[maybe_unused]] static const char* TECH_POSES_TOPIC            = "pf_tech_poses"; // position placer topic name
[[maybe_unused]] static const char* ROBOT_POSITION_TOPIC        = "initialpose"; // robot initial position topic name
[[maybe_unused]] static const char* AMCL_POSITION_TOPIC         = "amcl_pose"; // amcl topic name
[[maybe_unused]] static const char* NODES_POSES_TOPIC           = "pf_nodes_poses"; // graph nodes topic name
[[maybe_unused]] static const char* CURRENT_PATH_NODES_TOPIC    = "pf_path_nodes_poses"; // path nodes topic name

} // namespace CONFIG
#endif
