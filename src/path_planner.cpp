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
  this -> ideal_speed = ((max_speed_mph - 1) * 1609.34) / (60.0 * 60.0);
  this -> no_next_vals = round(planning_seconds_ahead / move_time);

  this -> lane = 1;
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

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  vector<double> pts_x;
  vector<double> pts_y;
  int prev_path_size = previous_path_x.size();
  double pos_x = car_x;
  double pos_y = car_y;
  double yaw = tools.normalizeAngle(tools.deg2rad(car_yaw));
  double speed = car_speed;

  if (prev_path_size < 2) {
    double prev_pos_x = car_x - cos(car_yaw);
    double prev_pos_y = car_y - sin(car_yaw);
    pts_x.push_back(prev_pos_x);
    pts_y.push_back(prev_pos_y);
    pts_x.push_back(pos_x);
    pts_y.push_back(pos_y);
  } else {
    pos_x = previous_path_x[prev_path_size - 1];
    pos_y = previous_path_y[prev_path_size - 1];
    double prev_pos_x = previous_path_x[prev_path_size - 2];
    double prev_pos_y = previous_path_y[prev_path_size - 2];
    yaw = tools.normalizeAngle(atan2(pos_y - prev_pos_y, pos_x - prev_pos_x));
    speed = tools.distance(prev_pos_x, prev_pos_y, pos_x, pos_y) / move_time;
    pts_x.push_back(prev_pos_x);
    pts_y.push_back(prev_pos_y);
    pts_x.push_back(pos_x);
    pts_y.push_back(pos_y);
  }

  vector<double> pos_sd = tools.getFrenet(pos_x, pos_y, yaw, map_waypoints_x, map_waypoints_y);
  double pos_s = pos_sd[0];
  double pos_d = pos_sd[1];

  for (int i = 0; i < prev_path_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  ScanStatus status = scanSurroundings(prev_path_size * move_time, pos_s, speed, sensor_fusion);
  cout << "--> " << "LANE: " << lane
       << ", carInFrontIsTooClose: " << status.carInFrontIsTooClose
       << ", gapAtLeftLane: " << status.gapAtLeftLane
       << ", gapAtRightLane: " << status.gapAtRightLane
       << endl;

  PathPlannerAction action = decideAction(status);
  if (action == CHANGE_TO_LEFT_LANE) {
    lane--;
  } else if (action == CHANGE_TO_RIGHT_LANE) {
    lane++;
  }

  // Path points
  for (int i = 0; i < 5; i++) {
    vector<double> wp = tools.getXY(pos_s + (i+1) * max(speed * action_seconds, MIN_ACTION_METERS),
                                    lane * 4.0 + 2.0,
                                    map_waypoints_s, map_waypoints_x, map_waypoints_y);
    pts_x.push_back(wp[0]);
    pts_y.push_back(wp[1]);
  }

  for (int i = 0; i < pts_x.size(); i++) {
    // Moving points to (0, 0) with angle 0
    double shift_x = pts_x[i] - pos_x;
    double shift_y = pts_y[i] - pos_y;
    pts_x[i] = shift_x * cos(yaw) + shift_y * sin(yaw);
    pts_y[i] = -shift_x * sin(yaw) + shift_y * cos(yaw);
  }

  tk::spline s;
  s.set_points(pts_x, pts_y);

  double current = 0.0;
  for (int i = 0; i < no_next_vals - prev_path_size; i++) {
    if (action == REDUCE_SPEED) {
      speed = max(speed - max_acceleration * move_time, 0.0);
    } else if (speed < ideal_speed) {
      speed = min(speed + max_acceleration * move_time, ideal_speed);
    } else if (speed > ideal_speed) {
      speed = max(speed - max_acceleration * move_time, ideal_speed);
    }
    current += speed * move_time;
    //current += ideal_speed * move_time;
    double point_x = current;
    double point_y = s(point_x);
    // Moving points back to their position
    next_x_vals.push_back(point_x * cos(yaw) - point_y * sin(yaw) + pos_x);
    next_y_vals.push_back(point_x * sin(yaw) + point_y * cos(yaw) + pos_y);
  }

  return {next_x_vals, next_y_vals};
}

ScanStatus PathPlanner::scanSurroundings(
    const int &time_head, const double &future_pos_s, const double &future_speed,
    const vector<vector<double>> &sensor_fusion) {

  ScanStatus status;
  status.carInFrontIsTooClose = false;
  status.gapAtLeftLane = true;
  status.gapAtRightLane = true;

  for (int i = 0; i < sensor_fusion.size(); i++) {
    double checked_car_vx = sensor_fusion[i][3];
    double checked_car_vy = sensor_fusion[i][4];
    double checked_car_speed = sqrt(checked_car_vx * checked_car_vx + checked_car_vy * checked_car_vy);
    double checked_car_s = sensor_fusion[i][5];
    double checked_car_d = sensor_fusion[i][6];

    if (isCarInLane(checked_car_d, lane) && isCarToClose(future_pos_s, future_speed, checked_car_s)) {
      status.carInFrontIsTooClose = true;
    }

    if (lane > 0) {
      if (isCarInLane(checked_car_d, lane - 1) &&
          (isCarToClose(future_pos_s, future_speed, checked_car_s) ||
           isCarToClose(checked_car_s, checked_car_speed, future_pos_s))) {
        status.gapAtLeftLane = false;
      }
    } else {
      status.gapAtLeftLane = false;
    }

    if (lane < 2) {
      if (isCarInLane(checked_car_d, lane + 1) &&
          (isCarToClose(future_pos_s, future_speed, checked_car_s) ||
           isCarToClose(checked_car_s, checked_car_speed, future_pos_s))) {
        status.gapAtRightLane = false;
      }
    } else {
      status.gapAtRightLane = false;
    }
  }

  return status;
}

PathPlannerAction PathPlanner::decideAction(const ScanStatus &status) {
  if (status.carInFrontIsTooClose) {
    if (status.gapAtLeftLane) {
      return CHANGE_TO_LEFT_LANE;
    } else if (status.gapAtRightLane) {
      return CHANGE_TO_RIGHT_LANE;
    } else {
      return REDUCE_SPEED;
    }
  } else {
    if (lane < PREFERRED_LANE && status.gapAtRightLane) {
      return CHANGE_TO_RIGHT_LANE;
    } else if (lane > PREFERRED_LANE && status.gapAtLeftLane) {
      return CHANGE_TO_LEFT_LANE;
    }
  }
  return KEEP_GOING;
}

bool PathPlanner::isCarInLane(const double &car_d, const int &lane) {
  if (car_d > 2.0 + 4.0 * lane - 2.0 && car_d < 2.0 + 4.0 * lane + 2.0) {
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
