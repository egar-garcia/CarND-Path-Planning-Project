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
      * @param max_speed_mph                 Maximum speed in MPH.
      * @param move_time                     Time in seconds to record a move by the car.
      *                                      P.e. 20 milliseconds given by the simulator.
      * @param max_acceleration              Maximum max_acceleration in m/s^2.
      * @param security_seconds_ahead        Seconds ahead to keep distance from the car in front.
      * @param planning_seconds_ahead        Seconds ahead to generate the path.
      * @param action_seconds                Time in seconds of the duration of an action,
      *                                      p.e. a lane change.
      * @param takeover_agressivity_rate     Rate of how close in front the car can be of the
      *                                      side cars in terms of the safety distance (in seconds).
      *                                      The colser to 0 the more conservative.
      *                                      0 needs all the distance, 1 no distance.
      * @param min_time_between_lane_changes The minimum time to wait between lane changes
      * @param security_reaction_rate        Proportion of the safety distance necessary to start
      *                                      breaking when a car in front is close.
      * @param max_visibily_seconds_ahead    Maximum visibility ahead in seconds.
      */
    PathPlanner(
      const double &max_speed_mph,
      const double &move_time,
      const double &max_acceleration,
      const double &security_seconds_ahead,
      const double &planning_seconds_ahead,
      const double &action_seconds,
      const double &takeover_agressivity_rate,
      const double &min_time_between_lane_changes,
      const double &security_reaction_rate,
      const double &max_visibily_seconds_ahead);

    std::vector<std::vector<double>> updatePath(
      const nlohmann::json &car_data,
      const std::vector<double> &map_waypoints_x,
      const std::vector<double> &map_waypoints_y,
      const std::vector<double> &map_waypoints_s);

  private:

    const double MIN_ACTION_METERS = 10.0;
    const int FIRST_LANE = 0;
    const int LAST_LANE = 2;

    const double MIN_SPEED = 0.001;

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
    double takeover_agressivity_rate;
    double min_time_between_lane_changes;
    double security_reaction_rate; // 0.0 to 1.0
    double max_visibily_seconds_ahead;

    double ideal_speed; // Desired speed in m/s, less but close to the max speed
    double ideal_acceleration; // Ideal accelation, less than the maximum accelation
    int no_next_vals; // Number of next values in the path (path's length in seconds / move time)

    double time_reference; // Keeps the time the car is running based in move_time
    double time_of_last_lane_change;

    bool isInitialized;
    double lane; // Current lane

    void initialize();

    /** Gets the data (x, y, yaw, speed, s, d) of the car's current position */
    std::vector<double> getCurrentPosition(
        const nlohmann::json &car_data);

    /** Gets the data (x, y, yaw, speed, s, d) of car's the last position in the previous path */
    std::vector<double> getPlannedPosition(
        const std::vector<double> &current_pos,
        const std::vector<double> &ptr_previous_path_x,
        const std::vector<double> &ptr_previous_path_y,
        const std::vector<double> &map_waypoints_x,
        const std::vector<double> &map_waypoints_y);

    /** Helper method to create new path points (to fill a spline) based in the previous path */
    std::vector<std::vector<double>> createPathPoints(
        const std::vector<double> &current_pos,
        const std::vector<double> &previous_path_x,
        const std::vector<double> &previous_path_y);

    /** Helper method to moves the previous path into the new one */
    void movePreviousPointsToNext(
        const std::vector<double> &previous_path_x,
        const std::vector<double> &previous_path_y,
        std::vector<double> &next_x,
        std::vector<double> &next_y);

    /** Helper method to fill the path points (to fill a spline) with the path to follow */
    void fillPathPoints(
        std::vector<double> &spline_pts_x,
        std::vector<double> &spline_pts_y,
        const std::vector<double> &planned_pos,
        const std::vector<double> &map_waypoints_x,
        const std::vector<double> &map_waypoints_y,
        const std::vector<double> &map_waypoints_s);

    /** Helper method to add new point to the path to follow, complementing the previous ones */
    void addNextPoints(
        const std::vector<double> &planned_pos,
        const State &state,
        std::vector<double> &spline_pts_x,
        std::vector<double> &spline_pts_y,
        std::vector<double> &next_x,
        std::vector<double> &next_y,
        const int &prev_path_size);

    /**
      * Extracts iformation about other cars in the track,
      * useful to chose the next behavioral state
      */
    ScanStatus scanSurroundings(
        const std::vector<double> &current_pos,
        const double &time_in_future,
        const std::vector<double> &future_pos,
        std::vector<std::vector<double>> &sensor_fusion);

    /** Gets the collection next possible states */
    std::vector<State> getNextStates(
        const ScanStatus &scan_status);

    /** Calculates the cost of a next possible state */
    double calculateCost(
        const State &state,
        const ScanStatus &scan_status);

    /** Picks the state of lower cost from the collection of possible states */
    State pickBestState(
        const std::vector<State> next_states,
        const ScanStatus &scan_status);

    /** Gets the adjusted speed of the car according to a behavioral state */
    double adjustSpeed(
        const double &speed,
        const State &state);

    /** Checks if a car is in a specific lane */
    bool isCarInLane(
        const double &car_d,
        const int &lane);

    /**
      * Helper method to check if a car in front is to close,
      * according to the security deistance in seconds
      */
    bool isCarInFrontTooClose(
        const double &from_car_s,
        const double &from_car_speed,
        const double &target_car_s);

    /**
      * Helper method to check if a car in front is to close,
      * according to a proportion of the security deistance in seconds
      */
    bool isCarInFrontTooClose(
        const double &from_car_s,
        const double &from_car_speed,
        const double &target_car_s,
        const double &security_seconds_ahead_proportion);

    /** Helper method to check if a car in front or behind blocks a change of lane */
    bool doesCarBlockLaneChange(
        const double &from_car_s,
        const double &from_car_speed,
        const double &target_car_s,
        const double &target_car_d,
        const double &target_car_speed,
        const int &target_lane);

    /** Helper method to check if a car behind blocks a change of lane */
    bool doesCarBehindBlockLaneChange(
        const double &from_car_s,
        const double &from_car_speed,
        const double &target_car_s,
        const double &target_car_speed);

    /** Helper method to find the closest car in front in a particular lane */
    bool isClosestCarAheadInLane(
        std::vector<double> *&tracked_car,
        const double target_lane,
        const double checked_car_s,
        const double checked_car_d,
        const double planned_pos_s);
};

#endif /* PATH_PLANNER_H */
