#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>

enum FlightPhase {
    WAIT_FCU,       // 等待飞控连接
    ARM_DRONE,      // 解锁无人机
    TAKEOFF,        // 起飞悬停
    MOVE_TO_POINT,  // 飞向目标点
    LAND_DRONE      // 执行着陆
};

class Offboard {
public:
    Offboard() : phase_(WAIT_FCU) {
        // 初始化ROS通信
        state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &Offboard::stateCb, this);
        ext_state_sub_ = nh_.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, &Offboard::extStateCb, this);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        
        arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        // 设置目标点（X=1,Y=1,Z=1）
        target_.pose.position.x = 1.0;
        target_.pose.position.y = 1.0;
        target_.pose.position.z = 1.0;

        // 设置悬停点（X=0,Y=0,Z=1）
        hover_target_.pose.position.x = 0.0;
        hover_target_.pose.position.y = 0.0;
        hover_target_.pose.position.z = 1.0;
    }

    void run() {
        ros::Rate loop(20.0);
        
        // 等待连接
        while(ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            loop.sleep();
        }

        // 发布初始位置
        for(int i=0; i<100; ++i){
            pose_pub_.publish(target_);
            loop.sleep();
            ros::spinOnce();
        }

        // 主控制循环
        while(ros::ok()) {
            FlightState();
            ros::spinOnce();
            loop.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    FlightPhase phase_;
    mavros_msgs::State current_state_;
    mavros_msgs::ExtendedState ext_state_;
    geometry_msgs::PoseStamped target_;
    geometry_msgs::PoseStamped hover_target_;
    ros::Time state_start_time_;

    // ROS通信对象
    ros::Subscriber state_sub_, ext_state_sub_;
    ros::Publisher pose_pub_;
    ros::ServiceClient arm_client_, mode_client_;

    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    void extStateCb(const mavros_msgs::ExtendedState::ConstPtr& msg) {
        ext_state_ = *msg;
    }

    void FlightState() {
        switch(phase_) {
        case WAIT_FCU:
            if(current_state_.connected) {
                phase_ = ARM_DRONE;
                ROS_INFO("[1/4] FCU Connected");
            }
            break;

        case ARM_DRONE:
            if(setMode("OFFBOARD") && armDrone(true)) {
                phase_ = TAKEOFF;
                state_start_time_ = ros::Time::now();
                ROS_INFO("[2/4] Armed & Offboard");
            }
            break;

        case TAKEOFF:
            pose_pub_.publish(hover_target_);
            if((ros::Time::now() - state_start_time_).toSec() > 3.0) {
                phase_ = MOVE_TO_POINT;
                ROS_INFO("[3/4] Hovered 3s, moving");
            }
            break;

        case MOVE_TO_POINT:
            pose_pub_.publish(target_);
            setMode("POSCTL");
            if((ros::Time::now() - state_start_time_).toSec() > 3.0) {
                setMode("AUTO.LAND");
                phase_ = LAND_DRONE;
                ROS_INFO("[4/4] Landing...");
            }
            break;

        case LAND_DRONE:
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
    ros::init(argc, argv, "offboard_landing_node");
    Offboard controller;
    controller.run();
    return 0;
}