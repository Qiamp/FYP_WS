#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <deque>
#include <mutex>
#include <Eigen/Core>

enum FlightPhase {
    WAIT_FCU,       // 等待飞控连接
    ARM_DRONE,      // 解锁无人机
    TAKEOFF,        // 起飞悬停
    MOVE_TO_POINT,  // 飞向目标点
    DETECT_TAG,     // 检测AprilTag
    MOVE_TO_TAG,    // 飞向AprilTag位置
    LAND_DRONE      // 执行着陆
};

class Offboard {
public:
    Offboard() : phase_(WAIT_FCU) {
        // 初始化当前定位订阅
        vision_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "mavros/vision_pose/pose", 10, &Offboard::visionPoseCb, this);
        
        tag_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "/tag_detections/tagpose_inertial", 10, &Offboard::tagPoseCb, this);

        
        // 其他ROS通信初始化
        state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, 
            &Offboard::stateCb, this);
        ext_state_sub_ = nh_.subscribe<mavros_msgs::ExtendedState>(
            "mavros/extended_state", 10, &Offboard::extStateCb, this);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "mavros/setpoint_position/local", 10);
        
        arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        // 设置目标点（基于视觉坐标系）
        target_.pose.position.x = 1.5;  // X方向
        target_.pose.position.y = 1.0;  // Y方向
        target_.pose.position.z = 2.0;  // 飞行高度

        // 初始悬停点（视觉坐标系原点）
        hover_target_.pose.position.x = 0.0;
        hover_target_.pose.position.y = 0.0;
        hover_target_.pose.position.z = 1.0;
    }

    void run() {
        ros::Rate loop(20.0);
        
        // 等待连接和视觉定位初始化
        while(ros::ok() && (!current_state_.connected || !uav_pose_received_)) {
            ros::spinOnce();
            loop.sleep();
            ROS_INFO_THROTTLE(1, "Waiting for FCU & vision pose...");
        }

        // 发布初始位置
        for(int i=0; i<100; ++i){
            pose_pub_.publish(target_);
            loop.sleep();
            ros::spinOnce();
        }

        // 主控制循环
        while(ros::ok()) {
            processFlightState();
            ros::spinOnce();
            loop.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    FlightPhase phase_;
    geometry_msgs::PoseStamped current_uav_pose_;
    mavros_msgs::State current_state_;
    mavros_msgs::ExtendedState ext_state_;
    geometry_msgs::PoseStamped target_, hover_target_, tag_target_;
    ros::Time state_start_time_;
    bool uav_pose_received_ = false;

    // AprilTag相关参数
    std::deque<geometry_msgs::Point> tag_position_history_;
    std::mutex tag_pose_mutex_;
    const size_t STABLE_SAMPLES = 30;    // 稳定检测所需样本数
    const double STABLE_THRESHOLD = 0.10; // 位置稳定性阈值（米）
    bool has_tag_position_ = false;

    // ROS通信对象
    ros::Subscriber vision_pose_sub_, tag_pose_sub_, state_sub_, ext_state_sub_;
    ros::Publisher pose_pub_;
    ros::ServiceClient arm_client_, mode_client_;

    void visionPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_uav_pose_ = *msg;
        uav_pose_received_ = true;
    }

    void tagPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(tag_pose_mutex_);
        // 维护固定长度的位置历史记录
        tag_position_history_.push_back(msg->pose.position);
        if(tag_position_history_.size() > STABLE_SAMPLES) {
            tag_position_history_.pop_front();
        }
        
        // 更新最新tag位置（带高度偏移）
        tag_target_ = *msg;
        tag_target_.pose.position.z = current_uav_pose_.pose.position.z; // 高度偏移
        has_tag_position_ = true;
    }

    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    void extStateCb(const mavros_msgs::ExtendedState::ConstPtr& msg) {
        ext_state_ = *msg;
    }

    bool checkPositionReached(const geometry_msgs::PoseStamped& target) {
        // 基于视觉定位的位置检查
        double dx = current_uav_pose_.pose.position.x - target.pose.position.x;
        double dy = current_uav_pose_.pose.position.y - target.pose.position.y;
        double dz = current_uav_pose_.pose.position.z - target.pose.position.z;
        return sqrt(dx*dx + dy*dy + dz*dz) < 0.05;  // 容差
    }

    bool isTagPositionStable() {
        std::lock_guard<std::mutex> lock(tag_pose_mutex_);
        if(tag_position_history_.size() < STABLE_SAMPLES) return false;

        // 计算位置标准差
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for(const auto& p : tag_position_history_) {
            sum.x() += p.x;
            sum.y() += p.y;
            sum.z() += p.z;
        }
        Eigen::Vector3d mean = sum / tag_position_history_.size();

        Eigen::Vector3d variance = Eigen::Vector3d::Zero();
        for(const auto& p : tag_position_history_) {
            variance.x() += pow(p.x - mean.x(), 2);
            variance.y() += pow(p.y - mean.y(), 2);
            variance.z() += pow(p.z - mean.z(), 2);
        }
        variance /= tag_position_history_.size();

        return variance.norm() < STABLE_THRESHOLD;
    }

    void processFlightState() {
        switch(phase_) {
        case WAIT_FCU:
            if(current_state_.connected && uav_pose_received_) {
                phase_ = ARM_DRONE;
                ROS_INFO("[1/6] Systems ready");
            }
            break;


        case ARM_DRONE:
            if(setMode("OFFBOARD") && armDrone(true)) {
                phase_ = TAKEOFF;
                state_start_time_ = ros::Time::now();
                ROS_INFO("[2/6] Armed & Offboard");
            }
            break;

        case TAKEOFF:
            pose_pub_.publish(hover_target_);
            if(checkPositionReached(hover_target_)) {
                phase_ = MOVE_TO_POINT;
                ROS_INFO("[3/6] Reached hover position");
            }
            break;


        case MOVE_TO_POINT:
            pose_pub_.publish(target_);
            if(checkPositionReached(target_)) {
                phase_ = DETECT_TAG;
                state_start_time_ = ros::Time::now();
                ROS_INFO("[4/6] Reached approximate position, detecting tag...");
            }
            break;

        case DETECT_TAG:
            pose_pub_.publish(target_);  // 保持当前位置
            if(isTagPositionStable() && has_tag_position_) {
                phase_ = MOVE_TO_TAG;
                ROS_INFO("[5/6] Tag position stabilized. Moving to tag position...");
            }
            // 超时处理（30秒）
            else if((ros::Time::now() - state_start_time_).toSec() > 30.0) {
                ROS_WARN("Tag detection timeout, force landing");
                setMode("AUTO.LAND");
                phase_ = LAND_DRONE;
            }
            break;

        
        case MOVE_TO_TAG:
            if(has_tag_position_) {
                pose_pub_.publish(tag_target_);
                ROS_INFO("[6/6] Moving to tag position...");
                if(checkPositionReached(tag_target_)) {
                    if(setMode("POSCTL")) {
                        ros::Duration(3.0).sleep();
                        phase_ = LAND_DRONE;
                    }
                }
            } else {
                ROS_WARN_THROTTLE(1, "No valid tag position available");
            }
            break;

        case LAND_DRONE:
            ROS_WARN("Reached tag position. Landing...");
            setMode("AUTO.LAND");
            if(ext_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
                armDrone(false);
                ROS_INFO("Mission Complete");
                ros::shutdown();
            }
            break;
        }
    }

    bool setMode(const std::string& mode) {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = mode;
        return mode_client_.call(srv) && srv.response.mode_sent;
    }

    bool armDrone(bool arm) {
        mavros_msgs::CommandBool srv;
        srv.request.value = arm;
        return arm_client_.call(srv) && srv.response.success;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "offboard_vision_landing");
    Offboard controller;
    controller.run();
    return 0;
}