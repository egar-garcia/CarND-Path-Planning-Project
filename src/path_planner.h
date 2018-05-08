#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include "json.hpp"
#include "tools.h"

struct ScanStatus {
    bool carInFrontIsTooClose;
    bool gapAtLeftLane;
    bool gapAtRightLane;
    std::vector<double> *carAheadInFront;
    std::vector<double> *carAheadAtLeftLane;
    std::vector<double> *carAheadAtRightLane;
};

enum State {
  ADVANCE,
  REDUCE_SPEED,
  CHANGE_TO_LEFT_LANE,
  CHANGE_TO_RIGHT_LANE
};

class PathPlanner {

  public:

    PathPlanner(
      const double &max_speed_mph,
      const double &move_time,
      const double &max_acceleration,
      const double &security_seconds_ahead,
      const double &planning_seconds_ahead,
      const double &action_seconds,
      const double &takeover_agressivity_rate,
      const double &security_reaction_rate);

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

    const int SENSOR_FUSION_ID_IDX = 0;
    const int SENSOR_FUSION_X_IDX = 1;
    const int SENSOR_FUSION_Y_IDX = 2;
    const int SENSOR_FUSION_VX_IDX = 3;
    const int SENSOR_FUSION_VY_IDX = 4;
    const int SENSOR_FUSION_S_IDX = 5;
    const int SENSOR_FUSION_D_IDX = 6;

    Tools tools;

    double max_speed_mph;
    double move_time;
    double max_acceleration;
    double security_seconds_ahead;
    double planning_seconds_ahead;
    double action_seconds;
    double takeover_agressivity_rate; // 0.0 to 1.0
    double security_reaction_rate; // 0.0 to 1.0

    double ideal_speed; // Desired speed in m/s
    double ideal_acceleration;
    int no_next_vals;

    bool isInitialized;
    double lane;

    void initialize();

    std::vector<double> getCurrentPosition(
        const nlohmann::json &car_data);

    std::vector<double> getPlannedPosition(
        const std::vector<double> &current_pos,
        const std::vector<double> &ptr_previous_path_x,
        const std::vector<double> &ptr_previous_path_y,
        const std::vector<double> &map_waypoints_x,
        const std::vector<double> &map_waypoints_y);

    std::vector<std::vector<double>> createPathPoints(
        const std::vector<double> &current_pos,
        const std::vector<double> &previous_path_x,
        const std::vector<double> &previous_path_y);

    void movePreviousPointsToNext(
        const std::vector<double> &previous_path_x,
        const std::vector<double> &previous_path_y,
        std::vector<double> &next_x,
        std::vector<double> &next_y);

    void fillPathPoints(
        std::vector<double> &spline_pts_x,
        std::vector<double> &spline_pts_y,
        const std::vector<double> &planned_pos,
        const std::vector<double> &map_waypoints_x,
        const std::vector<double> &map_waypoints_y,
        const std::vector<double> &map_waypoints_s);

    void addNextPoints(
        const std::vector<double> &planned_pos,
        const State &state,
        std::vector<double> &spline_pts_x,
        std::vector<double> &spline_pts_y,
        std::vector<double> &next_x,
        std::vector<double> &next_y,
        const int &prev_path_size);

    ScanStatus scanSurroundings(
        const std::vector<double> &current_pos,
        const double &time_in_future,
        const std::vector<double> &future_pos,
        std::vector<std::vector<double>> &sensor_fusion);

    State getNextState(
        const ScanStatus &scan_status);

    double adjustSpeed(
        const double &speed,
        const State &state);

    bool isCarInLane(
        const double &car_d,
        const int &lane);

    bool isCarInFrontTooClose(
        const double &from_car_s,
        const double &from_car_speed,
        const double &target_car_s);

    bool isCarInFrontTooClose(
        const double &from_car_s,
        const double &from_car_speed,
        const double &target_car_s,
        const double &security_seconds_ahead_proportion);

    bool doesCarBlockLaneChange(
        const double &from_car_s,
        const double &from_car_speed,
        const double &target_car_s,
        const double &target_car_d,
        const double &target_car_speed,
        const int &target_lane);

    bool doesCarBehindBlockLaneChange(
        const double &from_car_s,
        const double &from_car_speed,
        const double &target_car_s,
        const double &target_car_speed);

    bool isClosestCarAheadInLane(
        std::vector<double> *&tracked_car,
        const double target_lane,
        const double checked_car_s,
        const double checked_car_d,
        const double planned_pos_s);
};

#endif /* PATH_PLANNER_H */
