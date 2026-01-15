/**
 * *********************************************************
 *
 * @file: lqr_controller.cpp
 * @brief: Contains the linear quadratic regulator (LQR) local controller class
 * @author: Yang Haodong
 * @date: 2024-01-12
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>

#include "controller/lqr_controller.h"

PLUGINLIB_EXPORT_CLASS(rmp::controller::LQRController, nav_core::BaseLocalPlanner)

namespace rmp
{
namespace controller
{
/**
 * @brief Construct a new LQR controller object
 */
LQRController::LQRController() : initialized_(false), goal_reached_(false), tf_(nullptr)  //, costmap_ros_(nullptr)
{
}

/**
 * @brief Construct a new LQR controller object
 */
LQRController::LQRController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  : LQRController()
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the LQR controller object
 */
LQRController::~LQRController()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void LQRController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    // base
    nh.param("goal_dist_tolerance", goal_dist_tol_, 0.2);
    nh.param("rotate_tolerance", rotate_tol_, 0.5);
    nh.param("base_frame", base_frame_, base_frame_);
    nh.param("map_frame", map_frame_, map_frame_);

    // lookahead
    nh.param("lookahead_time", lookahead_time_, 1.5);
    nh.param("min_lookahead_dist", min_lookahead_dist_, 0.3);
    nh.param("max_lookahead_dist", max_lookahead_dist_, 0.9);

    // linear velocity
    nh.param("max_v", max_v_, 0.5);
    nh.param("min_v", min_v_, 0.0);
    nh.param("max_v_inc", max_v_inc_, 0.5);

    // angular velocity
    nh.param("max_w", max_w_, 1.57);
    nh.param("min_w", min_w_, 0.0);
    nh.param("max_w_inc", max_w_inc_, 1.57);

    // iteration for ricatti solution
    nh.param("max_iter_", max_iter_, 100);
    nh.param("eps_iter_", eps_iter_, 1e-4);

    nh.param("wheel_base", wheel_base_, 0.5);          // 轴距长度
    nh.param("max_steering_angle", max_steering_angle_, 0.7); // 最大转向角(弧度)
    nh.param("max_steering_angle_rate", max_steering_angle_rate_, 1.57); // 转向角速率限制

    ackermann_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 30);
    last_steering_angle_ = 0.0;
    last_steering_time_ = ros::Time::now();


    // weight matrix for penalizing state error while tracking [x,y,theta]
    std::vector<double> diag_vec;
    Q_ = Eigen::Matrix3d::Zero();
    nh.getParam("Q_matrix_diag", diag_vec);
    for (size_t i = 0; i < diag_vec.size(); ++i)
      Q_(i, i) = diag_vec[i];

    // weight matrix for penalizing input error while tracking[v, w]
    nh.getParam("R_matrix_diag", diag_vec);
    R_ = Eigen::Matrix2d::Zero();
    for (size_t i = 0; i < diag_vec.size(); ++i)
      R_(i, i) = diag_vec[i];

    double controller_freqency;
    nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
    d_t_ = 1 / controller_freqency;

    target_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>("/target_point", 10);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
    expect_point_pub_ = nh.advertise<visualization_msgs::Marker>("/lookahead_pose", 100);
    
    // particles_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("particles", 1);

    ROS_INFO("LQR controller initialized!");
  }
  else
  {
    ROS_WARN("LQR controller has already been initialized.");
  }
}

/**
 * @brief Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return true if the plan was updated successfully, else false
 */
bool LQRController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  ROS_INFO("Got new plan");

  // set new plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset plan parameters
  if (goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y)
  {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_theta_ = getYawAngle(global_plan_.back());
    goal_reached_ = false;
  }

  return true;
}

/**
 * @brief Check if the goal pose has been achieved
 * @return true if achieved, false otherwise
 */
bool LQRController::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("LQR controller has not been initialized");
    return false;
  }

  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  return false;
}

void LQRController::publishLookaheadPose(double x, double y, double theta_trj){
  // 1. 创建PoseStamped消息
    geometry_msgs::PoseStamped lookahead_pose;
    
    // 2. 填充头部信息
    lookahead_pose.header.stamp = ros::Time::now();
    lookahead_pose.header.frame_id = "map";  // 通常为"map"或"odom"
    
    // 3. 设置位置坐标
    lookahead_pose.pose.position.x = x;
    lookahead_pose.pose.position.y = y;
    lookahead_pose.pose.position.z = 0.0;  // 2D路径默认z=0
    
    // 4. 设置方向角（通过四元数）
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_trj);  // roll=0, pitch=0, yaw=theta_trj
    lookahead_pose.pose.orientation = tf2::toMsg(q);

    visualization_msgs::Marker marker;
    marker.header = lookahead_pose.header;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose = lookahead_pose.pose;
    marker.scale.x = 0.5;  // 箭头长度
    marker.scale.y = 0.1;  // 箭头宽度
    marker.color.g = 1.0;  // 绿色
    marker.color.a = 1.0;  // 不透明度
    
    // 5. 发布消息
    expect_point_pub_.publish(marker);
}

/**
 * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
 * @param cmd_vel will be filled with the velocity command to be passed to the robot base
 * @return true if a valid trajectory was found, else false
 */
bool LQRController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("LQR controller has not been initialized");
    return false;
  }

  // odometry observation - getting robot velocities in robot frame
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // get robot position in global frame
  geometry_msgs::PoseStamped robot_pose_odom, robot_pose_map;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, map_frame_, robot_pose_odom, robot_pose_map);

  // transform global plan to robot frame
  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_map);

  // calculate look-ahead distance
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double wt = base_odom.twist.twist.angular.z;
  double L = getLookAheadDistance(vt);

  // get the particular point on the path at the lookahead distance
  geometry_msgs::PointStamped lookahead_pt;
  double theta_trj, kappa;
  getLookAheadPoint(L, robot_pose_map, prune_plan, lookahead_pt, theta_trj, kappa);

  // current angle
  double theta = tf2::getYaw(robot_pose_map.pose.orientation);  // [-pi, pi]

  // calculate commands
  // 如果靠近目标点
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back()))
  {
    double e_theta = regularizeAngle(goal_theta_ - theta);

    // orientation reached
    if (!shouldRotateToPath(std::fabs(e_theta)))
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached_ = true;
    }
    // orientation not reached
    else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(base_odom, e_theta / d_t_);
    }
  }
  else
  {
    Eigen::Vector3d s(robot_pose_map.pose.position.x, robot_pose_map.pose.position.y, theta);  // current state
    Eigen::Vector3d s_d(lookahead_pt.point.x, lookahead_pt.point.y, theta_trj);                // desired state
    // vt当前速度，vt*kappa当前角速度
    Eigen::Vector2d u_r(vt, vt * kappa);                                                       // refered input
    Eigen::Vector2d u = _lqrControl(s, s_d, u_r);
    publishLookaheadPose(lookahead_pt.point.x, lookahead_pt.point.y, theta_trj);

    cmd_vel.linear.x = linearRegularization(base_odom, u[0]);
    cmd_vel.angular.z = angularRegularization(base_odom, u[1]);

    
  }

  // publish lookahead pose
  target_pt_pub_.publish(lookahead_pt);

  // publish robot pose
  current_pose_pub_.publish(robot_pose_map);


  // 1. 将角速度转换为转向角
  double steering_angle = 0.0;
  steering_angle = cmd_vel.angular.z ;
  // if (fabs(cmd_vel.linear.x) > 0.01) {  // 避免除零
  //     steering_angle = atan2(wheel_base_ * cmd_vel.angular.z, cmd_vel.linear.x);
  // }

  // 2. 应用转向角物理限制
  // steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

  steering_angle = (steering_angle < -max_steering_angle_) ? -max_steering_angle_ : 
                ((steering_angle > max_steering_angle_) ? max_steering_angle_ : steering_angle);

  std::cout << "u计算得到的值为：" << "vel is: " << cmd_vel.linear.x << " , angle is: " << steering_angle << std::endl;

  // 3. 转向角变化率限制
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_steering_time_).toSec();
  if (dt > 0) {
      double rate = (steering_angle - last_steering_angle_) / dt;
      if (fabs(rate) > max_steering_angle_rate_) {
          steering_angle = last_steering_angle_ + copysign(max_steering_angle_rate_ * dt, rate);
      }
  }

  // 4. 更新转向状态
  last_steering_angle_ = steering_angle;
  last_steering_time_ = current_time;

  // 5. 发布阿克曼指令
  ackermann_msgs::AckermannDriveStamped ackermann_cmd;
  ackermann_cmd.header.stamp = current_time;
  ackermann_cmd.header.frame_id = base_frame_;
  ackermann_cmd.drive.steering_angle = steering_angle;
  ackermann_cmd.drive.speed = cmd_vel.linear.x;
  // std::cout << "发布时的数据为：" << "dirve_speed is: " << ackermann_cmd.drive.speed << " , drive_steering_angle is: " << ackermann_cmd.drive.steering_angle << "================================" << std::endl;
  ackermann_pub_.publish(ackermann_cmd);

  return true;
}

/**
 * @brief Execute LQR control process
 * @param s   current state
 * @param s_d desired state
 * @param u_r refered control
 * @return u  control vector
 */
// 基于LQR（线性二次调节器）算法，计算使机器人从当前状态 s 跟踪目标状态 s_d 所需的最优控制输入 u，结合参考输入 u_r 和状态误差反馈。
Eigen::Vector2d LQRController::_lqrControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r)
{
  Eigen::Vector2d u;
  // 误差向量
  Eigen::Vector3d e(s - s_d);
  // 角度归一化
  e[2] = regularizeAngle(e[2]);

  // state equation on error
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  A(0, 2) = -u_r[0] * sin(s_d[2]) * d_t_;
  A(1, 2) = u_r[0] * cos(s_d[2]) * d_t_;

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 2);
  // B(0, 0) = cos(s_d[2]) * d_t_;
  // B(1, 0) = sin(s_d[2]) * d_t_;
  // B(2, 1) = d_t_;
  B(0,0) = cos(s_d[2]) * d_t_;
  B(1,0) = sin(s_d[2]) * d_t_;
  B(2,0) = tan(u_r[1]) / wheel_base_ * d_t_;  // u_r[1]是参考转向角

  // discrete iteration Ricatti equation
  Eigen::Matrix3d P, P_;
  P = Q_;
  for (int i = 0; i < max_iter_; ++i)
  {
    Eigen::Matrix2d temp = R_ + B.transpose() * P * B;
    P_ = Q_ + A.transpose() * P * A - A.transpose() * P * B * temp.inverse() * B.transpose() * P * A;
    if ((P - P_).array().abs().maxCoeff() < eps_iter_)
      break;
    P = P_;
  }

  // feedback
  Eigen::MatrixXd K = -(R_ + B.transpose() * P_ * B).inverse() * B.transpose() * P_ * A;

  u = u_r + K * e;

  return u;
}

}  // namespace controller
}  // namespace rmp
