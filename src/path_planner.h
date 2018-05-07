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
      const double &takeover_agressivity_rate,
      const double &action_seconds);

    std::vector<std::vector<double>> updatePath(
      const nlohmann::json &car_data,
      const std::vector<double> &map_waypoints_x,
      const std::vector<double> &map_waypoints_y,
      const std::vector<double> &map_waypoints_s);

  private:

    const double MIN_ACTION_METERS = 10.0;
    const int FIRST_LANE = 0;
    const int LAST_LANE = 2;
    const int PREFERRED_LANE = 1;

    const int X_IDX = 0;
    const int Y_IDX = 1;
    const int YAW_IDX = 2;
    const int SPEED_IDX = 3;
    const int S_IDX = 4;
    const int D_IDX = 5;

    Tools tools;

    double max_speed_mph;
    double move_time;
    double max_acceleration;
    double security_seconds_ahead;
    double planning_seconds_ahead;
    double takeover_agressivity_rate; // 0.0 to 1.0
    double action_seconds;

    double ideal_speed; // Desired speed in m/s
    double ideal_acceleration;
    int no_next_vals;

    bool isInitialized;
    double lane;

    void initialize();

    std::vector<double> getCurrentPosition(const nlohmann::json &car_data);

    std::vector<double> getPlannedPosition(
        const std::vector<double> &current_pos,
        const std::vector<double> &ptr_previous_path_x, const std::vector<double> &ptr_previous_path_y,
        const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y);

    std::vector<std::vector<double>> createPathPoints(
        const std::vector<double> &current_pos,
        const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y);

    void movePreviousPointsToNext(
      const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
      std::vector<double> &next_x, std::vector<double> &next_y);

    void fillPathPoints(
      std::vector<double> &spline_pts_x, std::vector<double> &spline_pts_y,
      const std::vector<double> &planned_pos,
      const std::vector<double> &map_waypoints_x,
      const std::vector<double> &map_waypoints_y,
      const std::vector<double> &map_waypoints_s);

    void addNextPoints(
        const std::vector<double> &planned_pos, const State &state,
        std::vector<double> &spline_pts_x, std::vector<double> &spline_pts_y,
        std::vector<double> &next_x, std::vector<double> &next_y,
        const int &prev_path_size);

    ScanStatus scanSurroundings(
        const std::vector<double> &current_pos,
        const double &time_in_future, const std::vector<double> &future_pos,
        const std::vector<std::vector<double>> &sensor_fusion);

    State getNextState(const ScanStatus &scan_status);

    double adjustSpeed(const double &speed, const State &state);

    bool isCarInLane(const double &car_d, const int &lane);

    bool isCarToClose(const double &from_car_s, const double &from_car_speed, const double &to_car_s);

    bool blocksLaneChange(const double &from_car_s, const double &from_car_speed,
                          const double &target_car_s, const double &target_car_speed);
};

#endif /* PATH_PLANNER_H */
