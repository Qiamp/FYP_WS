/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

#include "../include/gen_traj.h"

using namespace std;

#define FLIGHT_ALTITUDE 1.0f

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped init_pose;
Eigen::Isometry3d init_odom_pose = Eigen::Isometry3d::Identity();  
//Isometry3d 是一个 4x4 的变换矩阵，左上角的 3x3 子矩阵表示旋转，右上角的 3x1 列向量表示平移
//初始化位置与旋转矩阵为单位矩阵I(Identity)(无旋转&平移)

bool is_odom_init = false;

void state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state = *msg; }

void vision_pose_cbk(const geometry_msgs::PoseStamped::ConstPtr& msg_in) {
  if (!is_odom_init) {
    //rotate 方法允许我们对现有的 Isometry3d 变换应用一个旋转矩阵
    init_odom_pose.rotate(Eigen::Quaterniond(
        msg_in->pose.orientation.w, msg_in->pose.orientation.x,
        msg_in->pose.orientation.y, msg_in->pose.orientation.z));
    //pretranslate 方法允许我们对现有的 Isometry3d 变换应用一个平移向量
    //此处使用pretranslate方法对初始位置进行平移,先平移，再旋转，用于全局坐标系
    init_odom_pose.pretranslate(Eigen::Vector3d(msg_in->pose.position.x,
                                                msg_in->pose.position.y,
                                                msg_in->pose.position.z));
    is_odom_init = true;  //标志位，初始位置已经初始化
  }
}

int _uav_pose_pinrt = 0;
void uav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  current_pose = *msg;
  _uav_pose_pinrt++;
}

int main(int argc, char** argv) {
  YAML::Node traj_config = YAML::LoadFile(
      "home/jay/FYP_WS/src/my_offboard_node/config/traj.yaml");  // traj file path
  std::string file_name = traj_config["file_name"].as<std::string>();
  const float control_freq = traj_config["control_freq"].as<float>();

  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ros::Subscriber sub_pose =
      nh.subscribe("mavros/vision_pose/pose", 1000, vision_pose_cbk);
  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient land_client =
      nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/vision_pose/pose", 1, uav_pose_cb);

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(control_freq);

  // load trajectory
  std::ifstream traj_file;
  traj_file.open(file_name);
  traj_state trj_state;
  std::vector<traj_state> trajs;
  double t, p_x, p_y, p_z, q_w, q_x, q_y, q_z, v_x, v_y, v_z, w_x, w_y, w_z,
      a_x, a_y, a_z, a_rot_x, a_rot_y, a_rot_z, u1, u2, u3, u4;
  while (traj_file >> t >> p_x >> p_y >> p_z >> q_w >> q_x >> q_y >> q_z >>
         v_x >> v_y >> v_z >> w_x >> w_y >> w_z >> a_x >> a_y >> a_z >>
         a_rot_x >> a_rot_y >> a_rot_z >> u1 >> u2 >> u3 >> u4) {
    Eigen::Isometry3d traj_pose = Eigen::Isometry3d::Identity();
    traj_pose.rotate(Eigen::Quaterniond(q_w, q_x, q_y, q_z));
    traj_pose.pretranslate(Eigen::Vector3d(p_x, p_y, p_z));
    traj_pose = init_odom_pose * traj_pose;
    std::cout << "traj_pose: " << traj_pose.translation().transpose()
              << std::endl;
    Eigen::Quaterniond quaternion(traj_pose.rotation().matrix());
    std::cout << "quaternion: " << quaternion.w() << " " << quaternion.x()
              << " " << quaternion.y() << " " << quaternion.z() << std::endl;

    trj_state.t = t;
    trj_state.pos = Eigen::Vector3d(p_x, p_y, p_z);
    trj_state.rotation = Eigen::Quaterniond(q_w, q_x, q_y, q_z);
    trj_state.vel = Eigen::Vector3d(v_x, v_y, v_z);
    trajs.push_back(trj_state);
  }

  std::cout << "Traj size: " << trajs.size() << " , " << file_name << std::endl;

  // wait for FCU connection
  while (ros::ok() && current_state.connected) {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("Connecting to FCT...");
  }

  geometry_msgs::PoseStamped control;
  control.pose.position.x = 0;
  control.pose.position.y = 0;
  control.pose.position.z = FLIGHT_ALTITUDE;

  ROS_INFO("Send a few setpoints before starting");

  for (int i = 100; ros::ok() && i > 0; --i) {
    // execute motion
    local_pos_pub.publish(control);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  mavros_msgs::CommandTOL land_cmd;
  land_cmd.request.yaw = 0;
  land_cmd.request.latitude = 0;
  land_cmd.request.longitude = 0;
  land_cmd.request.altitude = 0;

  ros::Time last_request = ros::Time::now();

  ROS_INFO("Change to offboard mode and arm");
  ROS_INFO("%s\n", current_state.mode.c_str());
  std::cout << "armed: " << current_state.armed << std::endl;

  while (ros::ok() && !current_state.armed) {
    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0))) {
      ROS_INFO("%s\n", current_state.mode.c_str());
      if (set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if (!current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
          ROS_INFO("Vehicle armed");
        } else {
          std::cout << "Try to arm";
        }
        last_request = ros::Time::now();
        std::cout << "current state: " << current_state.armed << std::endl;
      }
    }

    //	std::cout << "Publish control " << std::endl;
    local_pos_pub.publish(control);
    ros::spinOnce();
    rate.sleep();
  }

  control.pose.position.x = current_pose.pose.position.x;
  control.pose.position.y = current_pose.pose.position.y;
  control.pose.position.z = current_pose.pose.position.z;
  init_pose = current_pose;

  // go to the hover waypoint
  for (int index = 300; ros::ok() && index > 0; --index) {
    trj_state = trajs[0];
    control.pose.position.x = trj_state.pos.x();
    control.pose.position.y = trj_state.pos.y();
    control.pose.position.z = trj_state.pos.z();
    control.pose.orientation.w = trj_state.rotation.w();
    control.pose.orientation.x = trj_state.rotation.x();
    control.pose.orientation.y = trj_state.rotation.y();
    control.pose.orientation.z = trj_state.rotation.z();
    local_pos_pub.publish(control);
    ros::spinOnce();
    rate.sleep();
  }

  int index = trajs.size();
  while (index-- && ros::ok()) {
    trj_state = trajs[trajs.size() - index];
    control.pose.position.x = trj_state.pos.x();
    control.pose.position.y = trj_state.pos.y();
    control.pose.position.z = trj_state.pos.z();
    //        control.pose.orientation.w = trj_state.rotation.w();
    //        control.pose.orientation.x = trj_state.rotation.x();
    //        control.pose.orientation.y = trj_state.rotation.y();
    //        control.pose.orientation.z = trj_state.rotation.z();
    std::cout << "Position control: " << index << " " << control.pose.position.x
              << " " << control.pose.position.y << " "
              << control.pose.position.z << std::endl;
    local_pos_pub.publish(control);
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("tring to land");
  while (!(land_client.call(land_cmd) && land_cmd.response.success)) {
    // local_pos_pub.publish(pose);
    ROS_INFO("tring to land");
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
