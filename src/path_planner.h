#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include "json.hpp"
#include "tools.h"

/**
 * Information as a result of scanning the car surroundings. Indicates if a car of lower speed
 * is detected in front, if there are gaps in the left and right lanes to make a safe lane change
 * and idetifies the cars in fron in the current, left and right lines.
 */
struct ScanStatus {
    bool carInFrontIsTooClose;
    bool gapAtLeftLane;
    bool gapAtRightLane;
    std::vector<double> *carAheadInFront;
    std::vector<double> *carAheadAtLeftLane;
    std::vector<double> *carAheadAtRightLane;
};

/**
 * The states to model the car's behavior. Four states are used:
 * ADVANCE: Means the car keeps in the same lane accelating till reach the ideal speed (which is
 *          close to the max speed).
 * REDUCE_SPEED: Keeps in the same lane but reducing the speed.
 * CHANGE_TO_LEFT_LANE: Performs a change to the next lane in the left.
 * CHANGE_TO_RIGHT_LANE: Performs a change to the next lane in the right.
 */
enum State {
  ADVANCE,
  REDUCE_SPEED,
  CHANGE_TO_LEFT_LANE,
  CHANGE_TO_RIGHT_LANE
};

/**
 * Controls the car behavior and generates the motion path.
 */
class PathPlanner {

  public:

    /**
      * @param max_speed_mph              Maximum speed in MPH.
      * @param move_time                  Time in seconds to do a move, p.e. a lane change.
      * @param max_acceleration           Maximum max_acceleration in m/s^2.
      * @param security_seconds_ahead     Seconds ahead to keep distance from the car in front.
      * @param planning_seconds_ahead     Seconds ahead to generate the path.
      * @param action_seconds             Time in seconds of the duration of an action,
      *                                   p.e. a lane change.
      * @param takeover_agressivity_rate  Rate [0,1] of how close in front the car can be of the
      *                                   side cars in terms of the safety distance (in seconds).
      *                                   The colser to 0 the more conservative.
      *                                   0 needs all the distance, 1 no distance.
      * @param security_reaction_rate     Rate [0,1] of the proportion of the safety distance
      *                                   necessary to start braking when a car in front is close.
      */
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

    /*
    State getNextState(
        const ScanStatus &scan_status);
    */

    std::vector<State> getNextStates(
        const ScanStatus &scan_status);

    double calculateCost(
        const State &state,
        const ScanStatus &scan_status);

    State pickBestState(
        const std::vector<State> next_states,
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
