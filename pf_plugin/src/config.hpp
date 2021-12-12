#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace CONFIG {

[[maybe_unused]] static const int MAP_WIDTH                     = 4000;
[[maybe_unused]] static const int MAP_HEIGHT                    = 4000;
[[maybe_unused]] static const double WORLD_X_MIN                = -100.0; 
[[maybe_unused]] static const double WORLD_X_MAX                = 100.0;
[[maybe_unused]] static const double WORLD_Y_MIN                = -100.0; 
[[maybe_unused]] static const double WORLD_Y_MAX                = 100.0;

[[maybe_unused]] static const int MOVE_TIMER_INTERVAL_MS        = 60000;

[[maybe_unused]] static const char* MAP_TOPIC                   = "map";
[[maybe_unused]] static const char* TECH_POSES_TOPIC            = "pf_tech_poses";
[[maybe_unused]] static const char* ROBOT_POSITION_TOPIC        = "initialpose";
[[maybe_unused]] static const char* AMCL_POSITION_TOPIC         = "amcl_pose";
[[maybe_unused]] static const char* NODES_POSES_TOPIC           = "pf_nodes_poses";
[[maybe_unused]] static const char* CURRENT_PATH_NODES_TOPIC    = "pf_path_nodes_poses";

} // namespace CONFIG
#endif
