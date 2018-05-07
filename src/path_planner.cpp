#include "path_planner.h"
#include "tools.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;


PathPlanner::PathPlanner(
    const double &max_speed_mph,
    const double &move_time,
    const double &max_acceleration,
    const double &security_seconds_ahead,
    const double &planning_seconds_ahead,
    const double &takeover_agressivity_rate,
    const double &action_seconds) {
  this -> max_speed_mph = max_speed_mph;
  this -> move_time = move_time;
  this -> max_acceleration = max_acceleration;
  this -> security_seconds_ahead = security_seconds_ahead;
  this -> planning_seconds_ahead = planning_seconds_ahead;
  this -> takeover_agressivity_rate = takeover_agressivity_rate;
  this -> action_seconds = action_seconds;
  this -> ideal_speed = tools.mph2ms(max_speed_mph - 1);
  this -> ideal_acceleration = max_acceleration / 2.0;
  this -> no_next_vals = round(planning_seconds_ahead / move_time);

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

  const vector<double> current_pos = getCurrentPosition(car_data);
  //double current_speed = current_pos[3];
  //double current_pos_s = current_pos[4];

  // Previous path data given to the Planner
  vector<double> previous_path_x = car_data["previous_path_x"];
  vector<double> previous_path_y = car_data["previous_path_y"];
  int prev_path_size = previous_path_x.size();

  vector<double> planned_pos = getPlannedPosition(current_pos, previous_path_x, previous_path_y,
                                                  map_waypoints_x, map_waypoints_y);
  //double planned_speed = planned_pos[SPEED_IDX];
  //double planned_pos_s = planned_pos[S_IDX];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  const vector<vector<double>> sensor_fusion = car_data["sensor_fusion"];
  const ScanStatus scan_status = scanSurroundings(current_pos, prev_path_size * move_time,
                                                  planned_pos, sensor_fusion);
  cout << "--> " << "LANE: " << lane
       << ", carInFrontIsTooClose: " << scan_status.carInFrontIsTooClose
       << ", carInFrontIsDangerouslyClose: " << scan_status.carInFrontIsDangerouslyClose
       << ", gapAtLeftLane: " << scan_status.gapAtLeftLane
       << ", gapAtRightLane: " << scan_status.gapAtRightLane
       << endl;

  const State state = getNextState(scan_status);
  /*
  if (state == EMERGENCY_BREAK) {
    planned_pos = current_pos;
    ptr_previous_path_x = NULL;
    ptr_previous_path_y = NULL;
    prev_path_size = 0;
  } else
  */
  if (state == CHANGE_TO_LEFT_LANE) {
    lane--;
  } else if (state == CHANGE_TO_RIGHT_LANE) {
    lane++;
  }

  vector<vector<double>> spline_pts = createPathPoints(current_pos, previous_path_x, previous_path_y);
  vector<double> spline_pts_x = spline_pts[0];
  vector<double> spline_pts_y = spline_pts[1];
  fillPathPoints(spline_pts_x, spline_pts_y, planned_pos,
                 map_waypoints_x, map_waypoints_y, map_waypoints_s);

  vector<double> next_x;
  vector<double> next_y;
  movePreviousPointsToNext(previous_path_x, previous_path_y, next_x, next_y);
  addNextPoints(planned_pos, state, spline_pts_x, spline_pts_y, next_x, next_y, prev_path_size);

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

  const double prev_pos_x = prev_path_size < 2 ? current_pos[X_IDX] : previous_path_x[prev_path_size - 2];
  const double prev_pos_y = prev_path_size < 2 ? current_pos[Y_IDX] : previous_path_y[prev_path_size - 2];
  const double planned_pos_x = previous_path_x[prev_path_size - 1];
  const double planned_pos_y = previous_path_y[prev_path_size - 1];
  const double planned_yaw
      = tools.normalizeAngle(atan2(planned_pos_y - prev_pos_y, planned_pos_x - prev_pos_x));
  const double planned_speed
      = tools.distance(prev_pos_x, prev_pos_y, planned_pos_x, planned_pos_y) / move_time;
  const vector<double> planned_pos_sd
      = tools.getFrenet(planned_pos_x, planned_pos_y, planned_yaw, map_waypoints_x, map_waypoints_y);
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
        planned_pos[S_IDX] + (i+1) * max(planned_pos[SPEED_IDX] * action_seconds, MIN_ACTION_METERS),
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
    tools.translateAndRotate(spline_pts_x[i], spline_pts_y[i], planned_pos_x, planned_pos_y, planned_yaw);
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
    // Moving points back to their position
    tools.rotateAndTranslate(point_x, point_y, -planned_yaw, -planned_pos_x, -planned_pos_y);
    next_x.push_back(point_x);
    next_y.push_back(point_y);
  }
}

ScanStatus PathPlanner::scanSurroundings(
    const std::vector<double> &current_pos,
    const double &time_in_future, const std::vector<double> &planned_pos,
    const vector<vector<double>> &sensor_fusion) {
  const double current_pos_s = current_pos[S_IDX];
  const double current_speed = current_pos[SPEED_IDX];
  const double planned_pos_s = planned_pos[S_IDX];
  const double planned_speed = planned_pos[SPEED_IDX];

  ScanStatus scan_status;
  scan_status.carInFrontIsTooClose = false;
  scan_status.carInFrontIsDangerouslyClose = false;
  scan_status.gapAtLeftLane = true;
  scan_status.gapAtRightLane = true;

  for (int i = 0; i < sensor_fusion.size(); i++) {
    const double checked_car_vx = sensor_fusion[i][3];
    const double checked_car_vy = sensor_fusion[i][4];
    const double checked_car_speed = sqrt(checked_car_vx * checked_car_vx + checked_car_vy * checked_car_vy);
    double checked_car_s = sensor_fusion[i][5];
    const double checked_car_d = sensor_fusion[i][6];

    if (isCarInLane(checked_car_d, lane) &&
        checked_car_s >= current_pos_s &&
        checked_car_s < current_pos_s + current_speed * security_seconds_ahead * 0.75) {
      scan_status.carInFrontIsDangerouslyClose = true;
    }

    checked_car_s += checked_car_speed * time_in_future;

    if (isCarInLane(checked_car_d, lane) && isCarToClose(planned_pos_s, planned_speed, checked_car_s)) {
      scan_status.carInFrontIsTooClose = true;
    }

    if (lane > FIRST_LANE) {
      if (isCarInLane(checked_car_d, lane - 1) &&
          (isCarToClose(planned_pos_s, planned_speed, checked_car_s) ||
           blocksLaneChange(planned_pos_s, planned_speed, checked_car_s, checked_car_speed))) {
           //isCarToClose(checked_car_s, checked_car_speed, planned_pos_s))) {
        scan_status.gapAtLeftLane = false;
      }
    } else {
      scan_status.gapAtLeftLane = false;
    }

    if (lane < LAST_LANE) {
      if (isCarInLane(checked_car_d, lane + 1) &&
          (isCarToClose(planned_pos_s, planned_speed, checked_car_s) ||
           //isCarToClose(checked_car_s, checked_car_speed, planned_pos_s))) {
           blocksLaneChange(planned_pos_s, planned_speed, checked_car_s, checked_car_speed))) {
        scan_status.gapAtRightLane = false;
      }
    } else {
      scan_status.gapAtRightLane = false;
    }
  }

  return scan_status;
}

State PathPlanner::getNextState(const ScanStatus &scan_status) {
  if (scan_status.carInFrontIsDangerouslyClose) {
    return EMERGENCY_BREAK;
  }

  if (scan_status.carInFrontIsTooClose) {
    if (scan_status.gapAtLeftLane) {
      return CHANGE_TO_LEFT_LANE;
    }
    if (scan_status.gapAtRightLane) {
      return CHANGE_TO_RIGHT_LANE;
    }
    return REDUCE_SPEED;
  }

  if (lane < PREFERRED_LANE && scan_status.gapAtRightLane) {
    return CHANGE_TO_RIGHT_LANE;
  }
  if (lane > PREFERRED_LANE && scan_status.gapAtLeftLane) {
    return CHANGE_TO_LEFT_LANE;
  }

  return ADVANCE;
}

double PathPlanner::adjustSpeed(const double &speed, const State &state) {
  if (state == EMERGENCY_BREAK) {
    return max(speed - ideal_acceleration * move_time, 0.0);
  }
  if (state == REDUCE_SPEED) {
    return max(speed - ideal_acceleration * move_time, 0.0);
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

bool PathPlanner::isCarToClose(const double &from_car_s, const double &from_car_speed,
                               const double &to_car_s) {
  if (from_car_s <= to_car_s && to_car_s < from_car_s + from_car_speed * security_seconds_ahead) {
    return true;
  }
  return false;
}

bool PathPlanner::blocksLaneChange(const double &car_s, const double &car_speed,
                                   const double &target_car_s, const double &target_car_speed) {
  if (target_car_s <= car_s && car_s < target_car_s + target_car_speed * security_seconds_ahead) {
    if (target_car_speed < car_speed &&
        car_s > target_car_s +
                target_car_speed * security_seconds_ahead * (1.0 - takeover_agressivity_rate)) {
       return false;
    }
    return true;
  }
  return false;
}
