#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <deque>
#include <mutex>
#include <vector>
#include <Eigen/Core>

enum FlightPhase {
    WAIT_FCU,       // 等待飞控连接
    ARM_DRONE,      // 解锁无人机
    TAKEOFF,        // 起飞悬停
    MISSION,        // 执行任务
    MOVE_TO_LAND,    // 飞向着陆位置
    LAND_DRONE      // 执行着陆
};

class Offboard {
public:
    Offboard() : phase_(WAIT_FCU) {
        // 初始化当前定位订阅
        vision_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "mavros/vision_pose/pose", 10, &Offboard::visionPoseCb, this);
        

        // 其他ROS通信初始化
        state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, 
            &Offboard::stateCb, this);
        ext_state_sub_ = nh_.subscribe<mavros_msgs::ExtendedState>(
            "mavros/extended_state", 10, &Offboard::extStateCb, this);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "mavros/setpoint_position/local", 10);
        
        arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        // 初始悬停点
        hover_target_.pose.position.x = 0.0;
        hover_target_.pose.position.y = 0.0;
        hover_target_.pose.position.z = 2.0;

        //降落点
        land_target_.pose.position.x = 0.0;
        land_target_.pose.position.y = 0.0;
        land_target_.pose.position.z = 0.1;

        // 初始化任务航点（修改坐标即可自定义路径）
        mission_waypoints_.resize(6);
        // 航点1
        mission_waypoints_[0].pose.position.x = 1.5;
        mission_waypoints_[0].pose.position.y = 0.0;
        mission_waypoints_[0].pose.position.z = 2.0;
        // 航点2
        mission_waypoints_[1].pose.position.x = 1.5;
        mission_waypoints_[1].pose.position.y = 1.5;
        mission_waypoints_[1].pose.position.z = 2.0;
        // 航点3
        mission_waypoints_[2].pose.position.x = -1.5;
        mission_waypoints_[2].pose.position.y =  1.5;
        mission_waypoints_[2].pose.position.z = 2.0;
        // 航点4
        mission_waypoints_[3].pose.position.x = -1.5;
        mission_waypoints_[3].pose.position.y = -1.5;
        mission_waypoints_[3].pose.position.z =  2.0;
        // 航点5
        mission_waypoints_[4].pose.position.x =  1.5;
        mission_waypoints_[4].pose.position.y = -1.5;
        mission_waypoints_[4].pose.position.z =  2.0;
        // 航点6
        mission_waypoints_[5].pose.position.x =  1.5;
        mission_waypoints_[5].pose.position.y =  0.0;
        mission_waypoints_[5].pose.position.z =  2.0;



    }

    void run() {
        ros::Rate loop(20.0);
        
        while(ros::ok() && (!current_state_.connected || !uav_pose_received_)) {
            ros::spinOnce();
            loop.sleep();
            ROS_INFO_THROTTLE(1, "Waiting for FCU & vision pose...");
        }

        for(int i=0; i<100; ++i){
            pose_pub_.publish(hover_target_);
            loop.sleep();
            ros::spinOnce();
        }

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
    geometry_msgs::PoseStamped hover_target_, land_target_;
    std::vector<geometry_msgs::PoseStamped> mission_waypoints_;
    size_t current_waypoint_index_ = 0;
    ros::Time state_start_time_;
    bool uav_pose_received_ = false;

    // ROS通信对象
    ros::Subscriber vision_pose_sub_, tag_pose_sub_, state_sub_, ext_state_sub_;
    ros::Publisher pose_pub_;
    ros::ServiceClient arm_client_, mode_client_;

    void visionPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_uav_pose_ = *msg;
        uav_pose_received_ = true;
    }

    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    void extStateCb(const mavros_msgs::ExtendedState::ConstPtr& msg) {
        ext_state_ = *msg;
    }

    bool checkPositionReached(const geometry_msgs::PoseStamped& target, double tolerance) {
        double dx = current_uav_pose_.pose.position.x - target.pose.position.x;
        double dy = current_uav_pose_.pose.position.y - target.pose.position.y;
        double dz = current_uav_pose_.pose.position.z - target.pose.position.z;
        return sqrt(dx*dx + dy*dy + dz*dz) < tolerance;
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
            if(checkPositionReached(hover_target_, 0.1)) {
                phase_ = MISSION;
                ROS_INFO("[3/6] Reached hover position");
            }
            break;

        case MISSION:
            if(current_waypoint_index_ < mission_waypoints_.size()) {
                pose_pub_.publish(mission_waypoints_[current_waypoint_index_]);
                
                if(checkPositionReached(mission_waypoints_[current_waypoint_index_], 0.08)) {
                    ROS_INFO("[4/6] Reached waypoint %zu/%zu", 
                            current_waypoint_index_+1, mission_waypoints_.size());
                    current_waypoint_index_++;
                    
                    if(current_waypoint_index_ == mission_waypoints_.size()) {
                        phase_ = MOVE_TO_LAND;
                        ROS_WARN("[5/6] All mission waypoints reached");
                    }
                }
            }
            break;
        
        case MOVE_TO_LAND:
        pose_pub_.publish(land_target_);
        if(checkPositionReached(hover_target_, 0.1)) {
            phase_ = LAND_DRONE;
            ROS_INFO("[6/6] Reached land position");
        }
        break;

        case LAND_DRONE:
            armDrone(false);
            ROS_WARN("Disarm");
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
    ros::init(argc, argv, "vicon_mission");
    Offboard controller;
    controller.run();
    return 0;
}