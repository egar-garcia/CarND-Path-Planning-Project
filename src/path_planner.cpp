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
    const double &action_seconds) {
  this -> max_speed_mph = max_speed_mph;
  this -> move_time = move_time;
  this -> max_acceleration = max_acceleration;
  this -> security_seconds_ahead = security_seconds_ahead;
  this -> planning_seconds_ahead = planning_seconds_ahead;
  this -> action_seconds = action_seconds;
  this -> ideal_speed = tools.mph2ms(max_speed_mph - 1);
  this -> ideal_acceleration = max_acceleration / 2.0;
  this -> no_next_vals = round(planning_seconds_ahead / move_time);

  this -> isInitialized = false;
}

void PathPlanner::initialize() {
  lane = 1;
  isInitialized = true;
}

vector<vector<double>> PathPlanner::updatePath(
    const json &car_data,
    const vector<double> &map_waypoints_x,
    const vector<double> &map_waypoints_y,
    const vector<double> &map_waypoints_s) {
  // Main car's localization Data
  double car_x = car_data["x"];
  double car_y = car_data["y"];
  double car_s = car_data["s"];
  double car_d = car_data["d"];
  double car_yaw = car_data["yaw"];
  double car_speed = car_data["speed"];
  // Previous path data given to the Planner
  vector<double> previous_path_x = car_data["previous_path_x"];
  vector<double> previous_path_y = car_data["previous_path_y"];
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  vector<vector<double>> sensor_fusion = car_data["sensor_fusion"];

  if (!isInitialized) {
    initialize();
  }

  double current_pos_x = car_x;
  double current_pos_y = car_y;
  double current_pos_s = car_s;
  double current_yaw = tools.normalizeAngle(tools.deg2rad(car_yaw));
  double current_speed = tools.mph2ms(car_speed);

  vector<double> spline_pts_x;
  vector<double> spline_pts_y;
  double planned_pos_x = current_pos_x;
  double planned_pos_y = current_pos_y;
  double planned_yaw = current_yaw;
  double planned_speed = current_speed;

  int prev_path_size = previous_path_x.size();
  if (prev_path_size < 2) {
    double prev_pos_x = planned_pos_x - cos(planned_yaw);
    double prev_pos_y = planned_pos_y - sin(planned_yaw);
    spline_pts_x.push_back(prev_pos_x);
    spline_pts_y.push_back(prev_pos_y);
    spline_pts_x.push_back(planned_pos_x);
    spline_pts_y.push_back(planned_pos_y);
  } else {
    double prev_pos_x = previous_path_x[prev_path_size - 2];
    double prev_pos_y = previous_path_y[prev_path_size - 2];
    planned_pos_x = previous_path_x[prev_path_size - 1];
    planned_pos_y = previous_path_y[prev_path_size - 1];
    planned_yaw = tools.normalizeAngle(atan2(planned_pos_y - prev_pos_y, planned_pos_x - prev_pos_x));
    planned_speed = tools.distance(prev_pos_x, prev_pos_y, planned_pos_x, planned_pos_y) / move_time;
    spline_pts_x.push_back(prev_pos_x);
    spline_pts_y.push_back(prev_pos_y);
    spline_pts_x.push_back(planned_pos_x);
    spline_pts_y.push_back(planned_pos_y);
  }

  vector<double> planned_pos_sd = tools.getFrenet(planned_pos_x, planned_pos_y, planned_yaw,
                                                  map_waypoints_x, map_waypoints_y);
  double planned_pos_s = planned_pos_sd[0];
  double planned_pos_d = planned_pos_sd[1];

  ScanStatus scan_status = scanSurroundings(current_pos_s, current_speed,
                                            prev_path_size * move_time, planned_pos_s, planned_speed,
                                            sensor_fusion);
  cout << "--> " << "LANE: " << lane
       << ", carInFrontIsTooClose: " << scan_status.carInFrontIsTooClose
       << ", carInFrontIsDangerouslyClose: " << scan_status.carInFrontIsDangerouslyClose
       << ", gapAtLeftLane: " << scan_status.gapAtLeftLane
       << ", gapAtRightLane: " << scan_status.gapAtRightLane
       << endl;

  State state = getNextState(scan_status);
  /*if (state == STOP) {
    cout << "*** EMERGENCY STOP" << endl;
    pos_x = car_x;
    pos_y = car_y;
    yaw = tools.normalizeAngle(tools.deg2rad(car_yaw));
    speed = car_speed;
    if (prev_path_size > 0) {
      double next_pos_x = previous_path_x[0];
      double next_pos_y = previous_path_y[0];
      yaw = tools.normalizeAngle(atan2(next_pos_y - pos_y, next_pos_x - pos_x));
      speed = tools.distance(pos_x, pos_y, next_pos_x, next_pos_y) / move_time;
    }
    prev_path_size = 0;
  } else*/
  if (state == CHANGE_TO_LEFT_LANE) {
    lane--;
  } else if (state == CHANGE_TO_RIGHT_LANE) {
    lane++;
  }

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  for (int i = 0; i < prev_path_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Path points
  for (int i = 0; i < 5; i++) {
    vector<double> wp = tools.getXY(
        planned_pos_s + (i+1) * max(planned_speed * action_seconds, MIN_ACTION_METERS),
        lane * 4.0 + 2.0,
        map_waypoints_s, map_waypoints_x, map_waypoints_y);
    spline_pts_x.push_back(wp[0]);
    spline_pts_y.push_back(wp[1]);
  }

  for (int i = 0; i < spline_pts_x.size(); i++) {
    // Moving points to (0, 0) with angle 0
    double shift_x = spline_pts_x[i] - planned_pos_x;
    double shift_y = spline_pts_y[i] - planned_pos_y;
    spline_pts_x[i] = shift_x * cos(planned_yaw) + shift_y * sin(planned_yaw);
    spline_pts_y[i] = -shift_x * sin(planned_yaw) + shift_y * cos(planned_yaw);
    //tools.translateAndRotate(spline_pts_x[i], spline_pts_y[i],
    //                         planned_pos_x, planned_pos_y, planned_yaw);
  }

  tk::spline spline;
  spline.set_points(spline_pts_x, spline_pts_y);

  double plan_shift = 0.0;
  double speed = planned_speed;
  for (int i = 0; i < no_next_vals - prev_path_size; i++) {
    /*if (state == STOP) {
      speed = max(speed - ideal_acceleration * move_time, 0.0);
    } else*/
    if (state == REDUCE_SPEED) {
      speed = max(speed - ideal_acceleration * move_time, 0.0);
    } else if (speed < ideal_speed) {
      speed = min(speed + ideal_acceleration * move_time, ideal_speed);
    } else if (speed > ideal_speed) {
      speed = max(speed - ideal_acceleration * move_time, ideal_speed);
    }
    plan_shift += speed * move_time;
    //current += ideal_speed * move_time;
    double point_x = plan_shift;
    double point_y = spline(point_x);
    // Moving points back to their position
    next_x_vals.push_back(point_x * cos(planned_yaw) - point_y * sin(planned_yaw) + planned_pos_x);
    next_y_vals.push_back(point_x * sin(planned_yaw) + point_y * cos(planned_yaw) + planned_pos_y);
    //tools.rotateAndTranslate(point_x, point_y,
    //                         -planned_yaw, -planned_pos_x, -planned_pos_y);
  }

  return {next_x_vals, next_y_vals};
}

ScanStatus PathPlanner::scanSurroundings(
    const double &current_pos_s, const double &current_speed,
    const double &time_in_future, const double &future_pos_s, const double &future_speed,
    const vector<vector<double>> &sensor_fusion) {

  ScanStatus scan_status;
  scan_status.carInFrontIsTooClose = false;
  scan_status.carInFrontIsDangerouslyClose = false;
  scan_status.gapAtLeftLane = true;
  scan_status.gapAtRightLane = true;

  for (int i = 0; i < sensor_fusion.size(); i++) {
    double checked_car_vx = sensor_fusion[i][3];
    double checked_car_vy = sensor_fusion[i][4];
    double checked_car_speed = sqrt(checked_car_vx * checked_car_vx + checked_car_vy * checked_car_vy);
    double checked_car_s = sensor_fusion[i][5];
    double checked_car_d = sensor_fusion[i][6];

    /*
    if (isCarInLane(checked_car_d, lane) &&
        checked_car_s >= current_pos_s &&
        checked_car_s <= current_pos_s + current_speed * security_seconds_ahead / 2.0) {
      scan_status.carInFrontIsDangerouslyClose = true;
    }
    */

    checked_car_s += checked_car_speed * time_in_future;

    if (isCarInLane(checked_car_d, lane) && isCarToClose(future_pos_s, future_speed, checked_car_s)) {
      scan_status.carInFrontIsTooClose = true;
    }

    if (lane > 0) {
      if (isCarInLane(checked_car_d, lane - 1) &&
          (isCarToClose(future_pos_s, future_speed, checked_car_s) ||
           isCarToClose(checked_car_s, checked_car_speed, future_pos_s))) {
        scan_status.gapAtLeftLane = false;
      }
    } else {
      scan_status.gapAtLeftLane = false;
    }

    if (lane < 2) {
      if (isCarInLane(checked_car_d, lane + 1) &&
          (isCarToClose(future_pos_s, future_speed, checked_car_s) ||
           isCarToClose(checked_car_s, checked_car_speed, future_pos_s))) {
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
