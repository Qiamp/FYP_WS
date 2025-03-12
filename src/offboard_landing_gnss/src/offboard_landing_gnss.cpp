#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <deque>
#include <mutex>
#include <Eigen/Core>
#include <tf/transform_datatypes.h>

enum FlightPhase {
    WAIT_FCU,       // 等待飞控连接
    ARM_DRONE,      // 解锁无人机
    TAKEOFF,        // 起飞悬停
    MOVE_TO_POINT,  // 飞向目标点
    DETECT_TAG,     // 检测AprilTag
    MOVE_TO_TAG,    // 飞向AprilTag位置
    LAND_DRONE      // 执行着陆
};

class GNSSLanding {
public:
    GNSSLanding() : phase_(WAIT_FCU), init_position_received_(false) {
        // GNSS定位订阅
        gnss_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>(
            "/mavros/global_position/local", 10, &GNSSLanding::gnssPoseCb, this);
        
        // AprilTag GNSS位置订阅
        tag_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "/tag_detections/tagpose_inertial", 10, &GNSSLanding::tagPoseCb, this);

        // MAVROS状态订阅
        state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, 
            &GNSSLanding::stateCb, this);
        ext_state_sub_ = nh_.subscribe<mavros_msgs::ExtendedState>(
            "/mavros/extended_state", 10, &GNSSLanding::extStateCb, this);

        // 控制命令发布
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/mavros/setpoint_position/local", 10);

        // 服务客户端
        arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        // 初始化参数
        target_height_ = 1.5;  // 初始飞行高度
        landing_threshold_ = 0.3;  // 着陆位置容差
    }

    void run() {
        ros::Rate rate(20.0);
        
        // 等待连接和GNSS定位初始化
        while(ros::ok() && (!current_state_.connected || !init_position_received_)) {
            ros::spinOnce();
            rate.sleep();
            ROS_INFO_THROTTLE(1, "Waiting for FCU & GNSS initialization...");
        }

        // 设置初始悬停点（当前GNSS位置上方）
        SetHoverTarget();

        // 发布初始位置
        for(int i=0; i<100; ++i){
            pose_pub_.publish(hover_target_);
            rate.sleep();
            ros::spinOnce();
        }

        // 主控制循环
        while(ros::ok()) {
            ProcessFlightState();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    FlightPhase phase_;
    nav_msgs::Odometry current_gnss_pose_;
    geometry_msgs::PoseStamped hover_target_, target_pose_, tag_target_;
    mavros_msgs::State current_state_;
    mavros_msgs::ExtendedState ext_state_;
    
    // 初始位置相关
    bool init_position_received_;
    double init_lat_, init_lon_, init_alt_;
    float target_height_;
    double landing_threshold_;

    // AprilTag跟踪相关
    std::deque<geometry_msgs::Point> tag_position_history_;
    std::mutex tag_pose_mutex_;
    const size_t STABLE_SAMPLES = 30;
    const double STABLE_THRESHOLD = 0.1;  // GNSS定位稳定性阈值

    // ROS通信对象
    ros::Subscriber gnss_pose_sub_, tag_pose_sub_, state_sub_, ext_state_sub_;
    ros::Publisher pose_pub_;
    ros::ServiceClient arm_client_, mode_client_;

    void gnssPoseCb(const nav_msgs::Odometry::ConstPtr& msg) {
        if(!init_position_received_) {
            // 记录初始位置
            init_lat_ = msg->pose.pose.position.x;
            init_lon_ = msg->pose.pose.position.y;
            init_alt_ = msg->pose.pose.position.z;
            init_position_received_ = true;
        }
        current_gnss_pose_ = *msg;
    }

    void tagPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(tag_pose_mutex_);
        
        // 维护位置历史记录
        tag_position_history_.push_back(msg->pose.position);
        if(tag_position_history_.size() > STABLE_SAMPLES) {
            tag_position_history_.pop_front();
        }
        
        // 更新目标位置（保持当前高度）
        tag_target_ = *msg;
        tag_target_.pose.position.z = current_gnss_pose_.pose.pose.position.z;
    }

    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    void extStateCb(const mavros_msgs::ExtendedState::ConstPtr& msg) {
        ext_state_ = *msg;
    }

    void SetHoverTarget() {
        hover_target_.pose.position.x = init_lat_;
        hover_target_.pose.position.y = init_lon_;
        hover_target_.pose.position.z = init_alt_ + target_height_;
    }

    bool CheckPositionReached(const geometry_msgs::PoseStamped& target, double tolerance) {
        double dx = current_gnss_pose_.pose.pose.position.x - target.pose.position.x;
        double dy = current_gnss_pose_.pose.pose.position.y - target.pose.position.y;
        double dz = current_gnss_pose_.pose.pose.position.z - target.pose.position.z;
        return sqrt(dx*dx + dy*dy + dz*dz) < tolerance;
    }

    bool IsTagPositionStable() {
        std::lock_guard<std::mutex> lock(tag_pose_mutex_);
        if(tag_position_history_.size() < STABLE_SAMPLES) return false;

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

    void ProcessFlightState() {
        switch(phase_) {
            case WAIT_FCU:
                if(current_state_.connected && init_position_received_) {
                    phase_ = ARM_DRONE;
                    ROS_INFO("[1/6] Systems ready");
                }
                break;
                
            case ARM_DRONE:
                if(SetMode("OFFBOARD") && ArmDrone(true)) {
                    phase_ = TAKEOFF;
                    ROS_INFO("[2/6] Armed & Offboard");
                }
                break;
                
            case TAKEOFF:
                pose_pub_.publish(hover_target_);
                if(CheckPositionReached(hover_target_, 0.2)) {
                    phase_ = MOVE_TO_POINT;
                    ROS_INFO("[3/6] Reached hover altitude");
                }
                break;
                
            case MOVE_TO_POINT:
                // 此处可设置航路点或保持悬停
                pose_pub_.publish(hover_target_);
                phase_ = DETECT_TAG;
                ROS_INFO("[4/6] Searching for landing target...");
                break;
                
            case DETECT_TAG:
                if(IsTagPositionStable()) {
                    phase_ = MOVE_TO_TAG;
                    ROS_INFO("[5/6] Target position stabilized");
                }
                break;
                
            case MOVE_TO_TAG:
                if(CheckPositionReached(tag_target_, landing_threshold_)) {
                    phase_ = LAND_DRONE;
                    ROS_INFO("[6/6] Reached landing position");
                } else {
                    pose_pub_.publish(tag_target_);
                }
                break;
                
            case LAND_DRONE:
                if(SetMode("AUTO.LAND")) {
                    if(ext_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
                        ArmDrone(false);
                        ROS_INFO("Landing complete");
                        ros::shutdown();
                    }
                }
                break;
        }
    }

    bool SetMode(const std::string& mode) {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = mode;
        return mode_client_.call(srv) && srv.response.mode_sent;
    }

    bool ArmDrone(bool arm) {
        mavros_msgs::CommandBool srv;
        srv.request.value = arm;
        return arm_client_.call(srv) && srv.response.success;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gnss_landing_node");
    GNSSLanding controller;
    controller.run();
    return 0;
}