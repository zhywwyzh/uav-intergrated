#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>

#include <atomic>
#include <cmath>
#include <mutex>
#include <traj_opt/poly_traj_utils.hpp>

#ifndef TRACK_WARN
#define TRACK_WARN(fmt, ...) ROS_WARN("\033[34m[TRACK]\033[0m " fmt, ##__VA_ARGS__)
#endif

ros::Publisher pos_cmd_pub_;
ros::Time heartbeat_time_;
bool receive_traj_ = false;
bool flight_start_ = false;
quadrotor_msgs::PolyTraj trajMsg_, trajMsg_last_;
Eigen::Vector3d last_p_;
double last_yaw_ = 0;
bool has_last_cmd_ = false;
int active_session_id_ = 0;
bool have_session_ = false;
ros::Time session_update_time_(0);
int expected_owner_mode_ = 2;
bool strict_owner_gate_ = false;
std::atomic_int cmd_owner_mode_{0};
std::atomic_bool have_cmd_owner_{false};
double cmd_period_sec_ = 0.01;
double yaw_rate_max_ = 1.0;  // rad/s, <=0 disables yaw-rate limiting
std::string odom_topic_ = "odom";
std::atomic_bool have_odom_{false};
std::atomic_bool need_yaw_resync_{true};
std::mutex odom_yaw_mutex_;
double odom_yaw_ = 0.0;
ros::Time last_yaw_cmd_stamp_(0);

void reset_runtime_state(bool clear_last_cmd) {
  receive_traj_ = false;
  flight_start_ = false;
  trajMsg_ = quadrotor_msgs::PolyTraj();
  trajMsg_last_ = quadrotor_msgs::PolyTraj();
  need_yaw_resync_.store(true, std::memory_order_relaxed);
  last_yaw_cmd_stamp_ = ros::Time(0);
  if (clear_last_cmd) {
    has_last_cmd_ = false;
  }
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  const auto &q = msg->pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  {
    std::lock_guard<std::mutex> lk(odom_yaw_mutex_);
    odom_yaw_ = yaw;
  }
  have_odom_.store(true, std::memory_order_relaxed);
}

bool try_get_odom_yaw(double &yaw) {
  if (!have_odom_.load(std::memory_order_relaxed)) {
    return false;
  }
  std::lock_guard<std::mutex> lk(odom_yaw_mutex_);
  yaw = odom_yaw_;
  return true;
}

double wrap_pi(double a) {
  if (a >= M_PI) a -= 2.0 * M_PI;
  if (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

void publish_cmd(int traj_id,
                 const Eigen::Vector3d &p,
                 const Eigen::Vector3d &v,
                 const Eigen::Vector3d &a,
                 double y, double yd) {
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub_.publish(cmd);
  last_p_ = p;
  has_last_cmd_ = true;
}

bool exe_traj(const quadrotor_msgs::PolyTraj &trajMsg) {
  double t = (ros::Time::now() - trajMsg.start_time).toSec();
  if (t > 0) {
    const ros::Time now = ros::Time::now();
    double dt = cmd_period_sec_;
    if (last_yaw_cmd_stamp_.toSec() > 1e-6) {
      dt = (now - last_yaw_cmd_stamp_).toSec();
      if (dt < 1e-4) dt = cmd_period_sec_;
    }
    if (dt < 1e-4) dt = 1e-4;

    if (need_yaw_resync_.load(std::memory_order_relaxed)) {
      double yaw_from_odom = 0.0;
      if (try_get_odom_yaw(yaw_from_odom)) {
        last_yaw_ = yaw_from_odom;
        need_yaw_resync_.store(false, std::memory_order_relaxed);
      } else {
        ROS_WARN_THROTTLE(1.0, "[traj_server] Waiting odom yaw for tracker yaw re-sync.");
      }
    }

    if (trajMsg.hover) {
      if (trajMsg.hover_p.size() != 3) {
        ROS_ERROR("[traj_server] hover_p is not 3d!");
      }
      Eigen::Vector3d p, v0;
      p.x() = trajMsg.hover_p[0];
      p.y() = trajMsg.hover_p[1];
      p.z() = trajMsg.hover_p[2];
      v0.setZero();
      publish_cmd(trajMsg.traj_id, p, v0, v0, last_yaw_, 0);  // TODO yaw
      last_yaw_cmd_stamp_ = now;
      return true;
    }
    if (trajMsg.order != 5) {
      ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
      return false;
    }
    if (trajMsg.duration.size() * (trajMsg.order + 1) != trajMsg.coef_x.size()) {
      ROS_ERROR("[traj_server] WRONG trajectory parameters!");
      return false;
    }
    int piece_nums = trajMsg.duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i) {
      int i6 = i * 6;
      cMats[i].row(0) << trajMsg.coef_x[i6 + 0], trajMsg.coef_x[i6 + 1], trajMsg.coef_x[i6 + 2],
          trajMsg.coef_x[i6 + 3], trajMsg.coef_x[i6 + 4], trajMsg.coef_x[i6 + 5];
      cMats[i].row(1) << trajMsg.coef_y[i6 + 0], trajMsg.coef_y[i6 + 1], trajMsg.coef_y[i6 + 2],
          trajMsg.coef_y[i6 + 3], trajMsg.coef_y[i6 + 4], trajMsg.coef_y[i6 + 5];
      cMats[i].row(2) << trajMsg.coef_z[i6 + 0], trajMsg.coef_z[i6 + 1], trajMsg.coef_z[i6 + 2],
          trajMsg.coef_z[i6 + 3], trajMsg.coef_z[i6 + 4], trajMsg.coef_z[i6 + 5];

      dura[i] = trajMsg.duration[i];
    }
    Trajectory traj(dura, cMats);
    if (t > traj.getTotalDuration()) {
      // ROS_ERROR("[traj_server] trajectory too short left!");
      return false;
    }
    Eigen::Vector3d p, v, a;
    p = traj.getPos(t);
    v = traj.getVel(t);
    a = traj.getAcc(t);
    // NOTE yaw
    double yaw = trajMsg.yaw;
    double d_yaw = wrap_pi(yaw - last_yaw_);
    double d_yaw_abs = fabs(d_yaw);
    const double yaw_step_max = yaw_rate_max_ > 0.0 ? yaw_rate_max_ * dt : 0.0;
    if (yaw_step_max > 1e-6 && d_yaw_abs >= yaw_step_max) {
      yaw = last_yaw_ + d_yaw / d_yaw_abs * yaw_step_max;
      d_yaw = yaw - last_yaw_;
    }
    yaw = wrap_pi(yaw);
    const double yaw_dot = d_yaw / dt;
    publish_cmd(trajMsg.traj_id, p, v, a, yaw, yaw_dot);
    last_yaw_ = yaw;
    last_yaw_cmd_stamp_ = now;
    return true;
  }
  return false;
}

void heartbeatCallback(const std_msgs::EmptyConstPtr &msg) {
  heartbeat_time_ = ros::Time::now();
}

void polyTrajCallback(const quadrotor_msgs::PolyTrajConstPtr &msgPtr) {
  if (have_session_ && session_update_time_.toSec() > 1e-5 &&
      msgPtr->start_time + ros::Duration(1e-3) < session_update_time_) {
    ROS_WARN_THROTTLE(1.0, "[traj_server] Ignore stale trajectory from previous session.");
    return;
  }
  trajMsg_ = *msgPtr;
  if (!receive_traj_) {
    trajMsg_last_ = trajMsg_;
    receive_traj_ = true;
  }
}

void trackerSessionCallback(const std_msgs::Int32ConstPtr &msg) {
  const int new_session = msg->data;
  if (have_session_ && new_session == active_session_id_) {
    return;
  }
  active_session_id_ = new_session;
  have_session_ = true;
  session_update_time_ = ros::Time::now();
  reset_runtime_state(false);
  TRACK_WARN("Tracker session switched to %d, runtime cache reset.", active_session_id_);
}

void preemptCallback(const std_msgs::EmptyConstPtr &msg) {
  (void)msg;
  reset_runtime_state(false);
  TRACK_WARN("Preempt received, traj server output suspended.");
}

void cmdOwnerCallback(const std_msgs::Int32ConstPtr &msg) {
  cmd_owner_mode_.store(msg->data, std::memory_order_relaxed);
  have_cmd_owner_.store(true, std::memory_order_relaxed);
}

bool ownerGateOpen() {
  if (!strict_owner_gate_) {
    return true;
  }
  if (!have_cmd_owner_.load(std::memory_order_relaxed)) {
    return true;
  }
  return cmd_owner_mode_.load(std::memory_order_relaxed) == expected_owner_mode_;
}

void cmdCallback(const ros::TimerEvent &e) {
  (void)e;
  if (!ownerGateOpen()) {
    return;
  }
  if (!receive_traj_) {
    return;
  }
  ros::Time time_now = ros::Time::now();
  if ((time_now - heartbeat_time_).toSec() > 0.5) {
    ROS_ERROR_ONCE("[traj_server] Lost heartbeat from the planner, is he dead?");
    if (has_last_cmd_) {
      publish_cmd(trajMsg_.traj_id, last_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), last_yaw_, 0);
    }
    return;
  }
  if (exe_traj(trajMsg_)) {
    trajMsg_last_ = trajMsg_;
    return;
  } else if (exe_traj(trajMsg_last_)) {
    return;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");
  std::string cmd_owner_topic = "/uav_planner/cmd_owner";
  nh.param("cmd_owner_topic", cmd_owner_topic, cmd_owner_topic);
  nh.param("owner_mode", expected_owner_mode_, 2);
  nh.param("strict_owner_gate", strict_owner_gate_, false);
  nh.param("cmd_period_sec", cmd_period_sec_, 0.01);
  nh.param("yaw_rate_max", yaw_rate_max_, 1.0);
  nh.param("odom_topic", odom_topic_, std::string("odom"));

  ros::Subscriber poly_traj_sub = nh.subscribe("trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);
  ros::Subscriber preempt_sub = nh.subscribe("preempt", 10, preemptCallback);
  ros::Subscriber tracker_session_sub = nh.subscribe("tracker_session", 10, trackerSessionCallback);
  ros::Subscriber cmd_owner_sub = nh.subscribe(cmd_owner_topic, 10, cmdOwnerCallback);
  ros::Subscriber odom_sub = nh.subscribe(odom_topic_, 50, odomCallback);

  pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(cmd_period_sec_), cmdCallback);

  ros::Duration(1.0).sleep();

  TRACK_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
