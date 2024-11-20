#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <cmath>

class DroneController
{
public:
    
    bool seenTag = false; // Flag to check if AprilTag is detected
    bool landing = false; // Flag to check if drone is landing

    DroneController()
    {
        // Initialize ROS node
        ros::NodeHandle nh;

        // Initialize target and tag positions
        target_position.pose.position.x = 0;
        target_position.pose.position.y = 0;
        target_position.pose.position.z = 0;

        tag_position.pose.position.x = 0.0;
        tag_position.pose.position.y = 0.0;
        tag_position.pose.position.z = 0.0;

        // Subscribers
        pose_sub = nh.subscribe("/mavros/vision_pose/pose", 10, &DroneController::poseCallback, this);
        apriltag_sub = nh.subscribe("/tag_detections/tagpose_inertial", 10, &DroneController::apriltagCallback, this);
        state_sub = nh.subscribe("/mavros/state", 10, &DroneController::stateCallback, this);

        // Publishers
        vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        // Services
        set_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        arm_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

        // Set mode to OFFBOARD and arm the drone
        setMode("OFFBOARD");
        arm(true);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_position = *msg;
    }

    void stateCallback(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state = *msg;
    }

    void apriltagCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if (msg != nullptr)
        {
            tag_position.pose.position.x = msg->pose.position.x;
            tag_position.pose.position.y = msg->pose.position.y;
            tag_position.pose.position.z = msg->pose.position.z;
            seenTag = true;
        }
        else
        {
            // tag_position.pose.position.x = 0.0;
            // tag_position.pose.position.y = 0.0;
            // tag_position.pose.position.z = 0.0;
            ROS_INFO("No AprilTag detected");
            seenTag = false;
        }
    }

    void setMode(const std::string& mode)
    {
        mavros_msgs::SetMode set_mode_msg;
        set_mode_msg.request.custom_mode = mode;

        if (set_mode_srv.call(set_mode_msg) && set_mode_msg.response.mode_sent)
        {
            ROS_INFO("Set mode to %s", mode.c_str());
        }
        else
        {
            ROS_ERROR("Failed to set mode to %s", mode.c_str());
        }
    }


    void arm(bool arm)
    {
        mavros_msgs::CommandBool arm_msg;
        arm_msg.request.value = arm;

        if (arm_srv.call(arm_msg) && arm_msg.response.success)
        {
            ROS_INFO("Drone armed");
        }
        else
        {
            ROS_ERROR("Failed to arm the drone");
        }
    }

    void land()
    {
        setMode("AUTO.LAND");

        ros::Rate rate(10);  // 10 Hz
        while (ros::ok())
        {
            if (current_state.mode == "AUTO.LAND" && tag_position.pose.position.z < 0.1)  //Need to modify this condition, this condition is not rigorous
            {
                ROS_INFO("Landing completed successfully");
                arm(false);
                break;
            }

            arm(false);

            rate.sleep();
            ros::spinOnce();
        }
    }

    void gotoAirport()
    {
        target_position.pose.position.x = current_position.pose.position.x - tag_position.pose.position.x; 
        target_position.pose.position.y = current_position.pose.position.y - tag_position.pose.position.y; 
        target_position.pose.position.z = current_position.pose.position.z - tag_position.pose.position.z + 1;

        ros::Rate rate(20);  // 20 Hz
        while (ros::ok())
        {
            double distance = calculateDistance_2D(current_position.pose.position, target_position.pose.position);
            if (distance < 0.05)
            {
                ROS_INFO("Reached Airport");
                break;
            }

            publishVelocity(target_position);
            rate.sleep();
            ros::spinOnce();
        }
    }

    void gotoPosition(double x, double y, double z)
    {
        target_position.pose.position.x = x; 
        target_position.pose.position.y = y; 
        target_position.pose.position.z = z;

        ros::Rate rate(20);  // 20 Hz
        while (ros::ok())
        {
            double distance = calculateDistance(current_position.pose.position, target_position.pose.position);
            if (distance < 0.3)
            {
                ROS_INFO("Reached GPS position");
                break;
            }

            publishVelocity(target_position);
            rate.sleep();
            ros::spinOnce();
        }
    }


    double calculateDistance(const geometry_msgs::Point& current, const geometry_msgs::Point& target)
    {
        return std::sqrt(std::pow(current.x - target.x, 2) + std::pow(current.y - target.y, 2) + std::pow(current.z - target.z, 2));
    }

    double calculateDistance_2D(const geometry_msgs::Point& current, const geometry_msgs::Point& target)
    {
        return std::sqrt(std::pow(current.x - target.x, 2) + std::pow(current.y - target.y, 2) );
    }

    void publishVelocity(const geometry_msgs::PoseStamped& target)
    {
        geometry_msgs::Twist velocity_msg;

        // Simple proportional controller
        double kp = 0.5;
        velocity_msg.linear.x = kp * (target.pose.position.x - current_position.pose.position.x);
        velocity_msg.linear.y = kp * (target.pose.position.y - current_position.pose.position.y);
        velocity_msg.linear.z = kp * (target.pose.position.z - current_position.pose.position.z);

        vel_pub.publish(velocity_msg);
    }

private:
    ros::Subscriber state_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber apriltag_sub;
    ros::Publisher vel_pub;
    ros::ServiceClient set_mode_srv;
    ros::ServiceClient arm_srv;

    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped target_position;
    geometry_msgs::PoseStamped tag_position;
    geometry_msgs::PoseStamped current_position;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offboard_landing");

    DroneController controller;
    controller.gotoPosition(0.0, 0.0, 1.0);
    if (controller.seenTag)
    {
        controller.gotoAirport();
    }
    else
    {
        ROS_INFO("No AprilTag detected. Landing at GPS position");
    }
    controller.land();
    return 0;
}
