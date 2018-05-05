#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include "json.hpp"
#include "tools.h"

struct ScanStatus {
    bool carInFrontIsTooClose;
    bool carInFrontIsDangerouslyClose;
    bool gapAtLeftLane;
    bool gapAtRightLane;
};

enum State {
  ADVANCE,
  REDUCE_SPEED,
  CHANGE_TO_LEFT_LANE,
  CHANGE_TO_RIGHT_LANE,
  EMERGENCY_BREAK
};

class PathPlanner {

  public:

    PathPlanner(
      const double &max_speed_mph,
      const double &move_time,
      const double &max_acceleration,
      const double &security_seconds_ahead,
      const double &planning_seconds_ahead,
      const double &action_seconds);

    std::vector<std::vector<double>> updatePath(
      const nlohmann::json &car_data,
      const std::vector<double> &map_waypoints_x,
      const std::vector<double> &map_waypoints_y,
      const std::vector<double> &map_waypoints_s);

  private:

    const double MIN_ACTION_METERS = 8.0;
    const int PREFERRED_LANE = 1;

    Tools tools;

    double max_speed_mph;
    double move_time;
    double max_acceleration;
    double security_seconds_ahead;
    double planning_seconds_ahead;
    double action_seconds;

    double ideal_speed; // Desired speed in m/s
    double ideal_acceleration;
    int no_next_vals;

    bool isInitialized;
    double lane;

    void initialize();

    ScanStatus scanSurroundings(
        const double &current_pos_s, const double &current_speed,
        const double &time_in_future, const double &future_pos_s, const double &future_speed,
        const std::vector<std::vector<double>> &sensor_fusion);

    State getNextState(const ScanStatus &scan_status);

    bool isCarInLane(const double &car_d, const int &lane);

    bool isCarToClose(const double &from_car_s, const double &from_car_speed, const double &to_car_s);
};

#endif /* PATH_PLANNER_H */
