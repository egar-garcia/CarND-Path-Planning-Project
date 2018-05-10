#include "path_planner.h"
#include "tools.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;


PathPlanner::PathPlanner(
    const double &max_speed_mph, const double &move_time, const double &max_acceleration,
    const double &security_seconds_ahead, const double &planning_seconds_ahead,
    const double &action_seconds, const double &takeover_agressivity_rate,
    const double &min_time_between_lane_changes, const double &security_reaction_rate,
    const double &max_visibily_seconds_ahead) {
  this -> max_speed_mph = max_speed_mph;
  this -> move_time = move_time;
  this -> max_acceleration = max_acceleration;
  this -> security_seconds_ahead = security_seconds_ahead;
  this -> planning_seconds_ahead = planning_seconds_ahead;
  this -> action_seconds = action_seconds;
  this -> takeover_agressivity_rate = takeover_agressivity_rate;
  this -> min_time_between_lane_changes = min_time_between_lane_changes;
  this -> security_reaction_rate = security_reaction_rate;
  this -> max_visibily_seconds_ahead = max_visibily_seconds_ahead;
  this -> ideal_speed = tools.mph2ms(max_speed_mph - 1);
  this -> ideal_acceleration = max_acceleration / 2.0;
  this -> no_next_vals = round(planning_seconds_ahead / move_time);
  this -> time_reference = 0.0;
  this -> time_of_last_lane_change = -2.0 * min_time_between_lane_changes - action_seconds;

  this -> isInitialized = false;
}

vector<vector<double>> PathPlanner::updatePath(
    const json &car_data,
    const vector<double> &map_waypoints_x,
    const vector<double> &map_waypoints_y,
    const vector<double> &map_waypoints_s) {
  if (!isInitialized) {
    initialize();
  }

  // Current's car position
  const vector<double> current_pos = getCurrentPosition(car_data);

  // Previous path data
  vector<double> previous_path_x = car_data["previous_path_x"];
  vector<double> previous_path_y = car_data["previous_path_y"];
  int prev_path_size = previous_path_x.size();

  // Last postion in the previous path, or the current one if no previous path
  vector<double> planned_pos = getPlannedPosition(current_pos, previous_path_x, previous_path_y,
                                                  map_waypoints_x, map_waypoints_y);

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  vector<vector<double>> sensor_fusion = car_data["sensor_fusion"];

  // Information about other cars on the track useful to chose a next behavioral state
  const ScanStatus scan_status = scanSurroundings(current_pos, prev_path_size * move_time,
                                                  planned_pos, sensor_fusion);

  // Getting the state with lower cost of the possible next states
  const State state = pickBestState(getNextStates(scan_status), scan_status);

  // Changing lane in case of required by next state
  if (state == CHANGE_TO_LEFT_LANE) {
    lane--;
    time_of_last_lane_change = time_reference;
  } else if (state == CHANGE_TO_RIGHT_LANE) {
    lane++;
    time_of_last_lane_change = time_reference;
  }

  // Filling a spline with to model the planned path,
  // the last planned position is used to continue the path
  vector<vector<double>> spline_pts
      = createPathPoints(current_pos, previous_path_x, previous_path_y);
  vector<double> spline_pts_x = spline_pts[0];
  vector<double> spline_pts_y = spline_pts[1];
  fillPathPoints(spline_pts_x, spline_pts_y, planned_pos,
                 map_waypoints_x, map_waypoints_y, map_waypoints_s);

  // Creating the new next points using the remaining previous ones
  // and generating new ones to continue using the spline
  vector<double> next_x;
  vector<double> next_y;
  movePreviousPointsToNext(previous_path_x, previous_path_y, next_x, next_y);
  addNextPoints(planned_pos, state, spline_pts_x, spline_pts_y, next_x, next_y, prev_path_size);

  time_reference += move_time;

  return {next_x, next_y};
}

void PathPlanner::initialize() {
  lane = 1;
  isInitialized = true;
}

vector<double> PathPlanner::getCurrentPosition(const json &car_data) {
  const double pos_x = car_data["x"];
  const double pos_y = car_data["y"];
  const double yaw = tools.normalizeAngle(tools.deg2rad(car_data["yaw"]));
  const double speed = tools.mph2ms(car_data["speed"]);
  const double pos_s = car_data["s"];
  const double pos_d = car_data["d"];

  return {pos_x, pos_y, yaw, speed, pos_s, pos_d};
}

vector<double> PathPlanner::getPlannedPosition(
    const vector<double> &current_pos,
    const vector<double> &previous_path_x, const vector<double> &previous_path_y,
    const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y) {
  const int prev_path_size = previous_path_x.size();

  if (prev_path_size < 1) {
    return current_pos;
  }

  const double prev_pos_x
      = prev_path_size < 2 ? current_pos[X_IDX] : previous_path_x[prev_path_size - 2];
  const double prev_pos_y
      = prev_path_size < 2 ? current_pos[Y_IDX] : previous_path_y[prev_path_size - 2];
  const double planned_pos_x = previous_path_x[prev_path_size - 1];
  const double planned_pos_y = previous_path_y[prev_path_size - 1];
  const double planned_yaw
      = tools.normalizeAngle(atan2(planned_pos_y - prev_pos_y, planned_pos_x - prev_pos_x));
  const double planned_speed
      = tools.distance(prev_pos_x, prev_pos_y, planned_pos_x, planned_pos_y) / move_time;
  const vector<double> planned_pos_sd = tools.getFrenet(planned_pos_x, planned_pos_y, planned_yaw,
                                                        map_waypoints_x, map_waypoints_y);
  return {planned_pos_x, planned_pos_y, planned_yaw, planned_speed,
          planned_pos_sd[X_IDX], planned_pos_sd[Y_IDX]};
}

vector<vector<double>> PathPlanner::createPathPoints(
    const vector<double> &current_pos,
    const vector<double> &previous_path_x, const vector<double> &previous_path_y) {
  const int prev_path_size = previous_path_x.size();
  vector<double> spline_pts_x;
  vector<double> spline_pts_y;

  if (prev_path_size < 1) {
    spline_pts_x.push_back(current_pos[X_IDX] - cos(current_pos[YAW_IDX]));
    spline_pts_y.push_back(current_pos[Y_IDX] - sin(current_pos[YAW_IDX]));
    spline_pts_x.push_back(current_pos[X_IDX]);
    spline_pts_y.push_back(current_pos[Y_IDX]);
  } else if (prev_path_size < 2) {
    spline_pts_x.push_back(current_pos[X_IDX]);
    spline_pts_y.push_back(current_pos[Y_IDX]);
    spline_pts_x.push_back(previous_path_x[prev_path_size - 1]);
    spline_pts_y.push_back(previous_path_y[prev_path_size - 1]);
  } else {
    spline_pts_x.push_back(previous_path_x[prev_path_size - 2]);
    spline_pts_y.push_back(previous_path_y[prev_path_size - 2]);
    spline_pts_x.push_back(previous_path_x[prev_path_size - 1]);
    spline_pts_y.push_back(previous_path_y[prev_path_size - 1]);
  }

  return {spline_pts_x, spline_pts_y};
}

void PathPlanner::movePreviousPointsToNext(
    const vector<double> &previous_path_x, const vector<double> &previous_path_y,
    vector<double> &next_x, vector<double> &next_y) {
  const int prev_path_size = previous_path_x.size();
  for (int i = 0; i < prev_path_size; i++) {
    next_x.push_back(previous_path_x[i]);
    next_y.push_back(previous_path_y[i]);
  }
}

void PathPlanner::fillPathPoints(
    vector<double> &spline_pts_x, vector<double> &spline_pts_y,
    const vector<double> &planned_pos,
    const vector<double> &map_waypoints_x,
    const vector<double> &map_waypoints_y,
    const vector<double> &map_waypoints_s) {
  for (int i = 0; i < 5; i++) {
    const vector<double> wp = tools.getXY(
        planned_pos[S_IDX] + (i+1) * max(planned_pos[SPEED_IDX] * action_seconds,
                                         MIN_ACTION_METERS),
        lane * 4.0 + 2.0,
        map_waypoints_s, map_waypoints_x, map_waypoints_y);
    spline_pts_x.push_back(wp[0]);
    spline_pts_y.push_back(wp[1]);
  }
}

void PathPlanner::addNextPoints(
    const vector<double> &planned_pos, const State &state,
    vector<double> &spline_pts_x, vector<double> &spline_pts_y,
    vector<double> &next_x, vector<double> &next_y,
    const int &prev_path_size) {
  const double planned_pos_x = planned_pos[X_IDX];
  const double planned_pos_y = planned_pos[Y_IDX];
  const double planned_yaw = planned_pos[YAW_IDX];
  const double planned_speed = planned_pos[SPEED_IDX];

  for (int i = 0; i < spline_pts_x.size(); i++) {
    // Moving points to (0, 0) with angle 0
    tools.translateAndRotate(spline_pts_x[i], spline_pts_y[i],
                             planned_pos_x, planned_pos_y, planned_yaw);
  }

  tk::spline spline;
  spline.set_points(spline_pts_x, spline_pts_y);

  double plan_shift = 0.0;
  double speed = planned_speed;
  for (int i = 0; i < no_next_vals - prev_path_size; i++) {
    speed = adjustSpeed(speed, state);
    plan_shift += speed * move_time;
    double point_x = plan_shift;
    double point_y = spline(point_x);
    // Moving points back to their previous position and angle
    tools.rotateAndTranslate(point_x, point_y, -planned_yaw, -planned_pos_x, -planned_pos_y);
    next_x.push_back(point_x);
    next_y.push_back(point_y);
  }
}

ScanStatus PathPlanner::scanSurroundings(
    const std::vector<double> &current_pos,
    const double &time_in_future, const std::vector<double> &planned_pos,
    vector<vector<double>> &sensor_fusion) {
  const double current_pos_s = current_pos[S_IDX];
  const double current_speed = current_pos[SPEED_IDX];
  const double planned_pos_s = planned_pos[S_IDX];
  const double planned_speed = planned_pos[SPEED_IDX];

  ScanStatus scan_status;
  scan_status.carInFrontIsTooClose = false;
  scan_status.gapAtLeftLane = true;
  scan_status.gapAtRightLane = true;
  scan_status.carAheadInFront = NULL;
  scan_status.carAheadAtLeftLane = NULL;
  scan_status.carAheadAtRightLane = NULL;

  for (int i = 0; i < sensor_fusion.size(); i++) {
    const double checked_car_vx = sensor_fusion[i][SENSOR_FUSION_VX_IDX];
    const double checked_car_vy = sensor_fusion[i][SENSOR_FUSION_VY_IDX];
    const double checked_car_speed
        = sqrt(checked_car_vx * checked_car_vx + checked_car_vy * checked_car_vy);
    double checked_car_s = sensor_fusion[i][SENSOR_FUSION_S_IDX];
    const double checked_car_d = sensor_fusion[i][SENSOR_FUSION_D_IDX];

    if (isCarInLane(checked_car_d, lane) &&
        isCarInFrontTooClose(current_pos_s, current_speed, checked_car_s,
                             security_reaction_rate)) {
      scan_status.carInFrontIsTooClose = true;
    }

    // Estimated position of the car in the future
    checked_car_s += checked_car_speed * time_in_future;

    if (isCarInLane(checked_car_d, lane) &&
        isCarInFrontTooClose(planned_pos_s, planned_speed, checked_car_s)) {
      scan_status.carInFrontIsTooClose = true;
    }

    // Checkig if a change to the left lane is possible/safe
    if (lane > FIRST_LANE) {
      if (doesCarBlockLaneChange(planned_pos_s, planned_speed, checked_car_s,
                                 checked_car_d, checked_car_speed, lane - 1)) {
        scan_status.gapAtLeftLane = false;
      };
    } else {
      scan_status.gapAtLeftLane = false;
    }

    // Checkig if a change to the right lane is possible/safe
    if (lane < LAST_LANE) {
      if (doesCarBlockLaneChange(planned_pos_s, planned_speed, checked_car_s,
                                 checked_car_d, checked_car_speed, lane + 1)) {
        scan_status.gapAtRightLane = false;
      }
    } else {
      scan_status.gapAtRightLane = false;
    }

    // Cars beyond the maximum visibility are ignored
    if (checked_car_s - planned_pos_s <= max_visibily_seconds_ahead * planned_speed) {
      // Getting the closest cars in front for the same, left and right lanes
      if (isClosestCarAheadInLane(scan_status.carAheadInFront, lane,
                                  checked_car_s, checked_car_d, planned_pos_s)) {
        scan_status.carAheadInFront = &sensor_fusion[i];
      }
      if (lane > FIRST_LANE &&
          isClosestCarAheadInLane(scan_status.carAheadAtLeftLane, lane - 1,
                                  checked_car_s, checked_car_d, planned_pos_s)) {
        scan_status.carAheadAtLeftLane = &sensor_fusion[i];
      }
      if (lane < LAST_LANE &&
          isClosestCarAheadInLane(scan_status.carAheadAtRightLane, lane + 1,
                                  checked_car_s, checked_car_d, planned_pos_s)) {
        scan_status.carAheadAtRightLane = &sensor_fusion[i];
      }
    }
  }

  return scan_status;
}

vector<State> PathPlanner::getNextStates(const ScanStatus &scan_status) {
  vector<State> next_states;

  if (!scan_status.carInFrontIsTooClose) {
    next_states.push_back(ADVANCE);
  } else {
    next_states.push_back(REDUCE_SPEED);
  }
  if (scan_status.gapAtLeftLane &&
      time_reference >= time_of_last_lane_change + min_time_between_lane_changes + action_seconds) {
    next_states.push_back(CHANGE_TO_LEFT_LANE);
  }
  if (scan_status.gapAtRightLane &&
      time_reference >= time_of_last_lane_change + min_time_between_lane_changes + action_seconds) {
    next_states.push_back(CHANGE_TO_RIGHT_LANE);
  }

  return next_states;
}

double PathPlanner::calculateCost(const State &state, const ScanStatus &scan_status) {
  // Cost function based in the velocity of the car that would be in front in the intended lane
  // the fastest the car the less the cost, but using the ideal speed as maximum

  bool lane_change = false;
  vector<double> *car_in_front = scan_status.carAheadInFront;
  if (state == CHANGE_TO_LEFT_LANE) {
    car_in_front = scan_status.carAheadAtLeftLane;
    lane_change = true;
  } else if (state == CHANGE_TO_RIGHT_LANE) {
    car_in_front = scan_status.carAheadAtRightLane;
    lane_change = true;
  }

  // If there would be no car in front, minimizing the cost
  if (car_in_front == NULL) {
    if (lane_change) { // Putting some cost to penalize a lane change (if not really necessary)
      return exp(-ideal_speed);
    }
    return 0.0;
  }

  // Using the speed of the potential car in front to calculate the cost, limited to the ideal speed
  const double car_in_front_speed
      = min(sqrt(pow(car_in_front -> at(SENSOR_FUSION_VX_IDX), 2) +
                 pow(car_in_front -> at(SENSOR_FUSION_VY_IDX), 2)),
            ideal_speed);
  return exp(-car_in_front_speed);
}

State PathPlanner::pickBestState(const vector<State> next_states, const ScanStatus &scan_status) {
  const int no_states = next_states.size();
  // It's assumed that there is at least one possible state
  State best_state = next_states[0];
  double best_cost = calculateCost(best_state, scan_status);

  for (int i = 1; i < no_states; i++) {
    const State state = next_states[i];
    const double cost = calculateCost(state, scan_status);
    if (cost < best_cost) {
      best_cost = cost;
      best_state = state;
    }
  }
  return best_state;
}

double PathPlanner::adjustSpeed(const double &speed, const State &state) {
  if (state == REDUCE_SPEED) {
    return max(speed - ideal_acceleration * move_time, MIN_SPEED);
  }
  if (speed > ideal_speed) {
    return max(speed - ideal_acceleration * move_time, ideal_speed);
  }
  if (speed < ideal_speed) {
    return min(speed + ideal_acceleration * move_time, ideal_speed);
  }
  return speed;
}

bool PathPlanner::isCarInLane(const double &car_d, const int &lane) {
  if (car_d >= 2.0 + 4.0 * lane - 2.0 && car_d <= 2.0 + 4.0 * lane + 2.0) {
    return true;
  }
  return false;
}

bool PathPlanner::isCarInFrontTooClose(
    const double &from_car_s, const double &from_car_speed, const double &target_car_s) {
  return isCarInFrontTooClose(from_car_s, from_car_speed, target_car_s, 1.0);
}

bool PathPlanner::isCarInFrontTooClose(
    const double &from_car_s, const double &from_car_speed, const double &target_car_s,
    const double &security_seconds_ahead_proportion) {
  if (from_car_s <= target_car_s &&
      target_car_s <
      from_car_s + from_car_speed * security_seconds_ahead * security_seconds_ahead_proportion) {
    return true;
  }
  return false;
}

bool PathPlanner::doesCarBlockLaneChange(
    const double &car_s, const double &car_speed,
    const double &target_car_s, const double &target_car_d, const double &target_car_speed,
    const int &target_lane) {
  if (!isCarInLane(target_car_d, target_lane)) {
    return false;
  }
  if (isCarInFrontTooClose(car_s, car_speed, target_car_s)) {
    return true;
  }
  if (doesCarBehindBlockLaneChange(car_s, car_speed, target_car_s, target_car_speed)) {
    return true;
  }
  return false;
}

bool PathPlanner::doesCarBehindBlockLaneChange(
    const double &car_s, const double &car_speed,
    const double &target_car_s, const double &target_car_speed) {
  if (!isCarInFrontTooClose(target_car_s, target_car_speed, car_s)) {
    return false;
  }
  if (target_car_speed < car_speed &&
      !isCarInFrontTooClose(target_car_s, target_car_speed, car_s,
                            1.0 - takeover_agressivity_rate)) {
    return false;
  }
  return true;
}

bool PathPlanner::isClosestCarAheadInLane(
    vector<double> *&tracked_car, const double target_lane,
    const double checked_car_s, const double checked_car_d, const double planned_pos_s) {
  if (isCarInLane(checked_car_d, target_lane) &&
      checked_car_s > planned_pos_s &&
      (tracked_car == NULL || checked_car_s < tracked_car->at(SENSOR_FUSION_S_IDX))) {
    return true;
  }
  return false;
}
