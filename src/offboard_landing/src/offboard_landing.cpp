#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <cmath>

class DroneController
{
public:
    DroneController()
    {
        // Initialize ROS node
        ros::NodeHandle nh;

        // Initialize target and current positions
        target_position.pose.position.x = 0;
        target_position.pose.position.y = 0;
        target_position.pose.position.z = 0;

        // Subscribers
        pose_sub = nh.subscribe("/mavros/vision_pose/pose", 10, &DroneController::poseCallback, this);

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

    void gotoPosition(double x, double y, double z)
    {
        target_position.pose.position.x = x;
        target_position.pose.position.y = y;
        target_position.pose.position.z = z;

        ros::Rate rate(20);  // 20 Hz
        while (ros::ok())
        {
            double distance = calculateDistance(current_position.pose.position, target_position.pose.position);
            if (distance < 0.1)
            {
                ROS_INFO("Reached target position");
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
    ros::Subscriber pose_sub;
    ros::Publisher vel_pub;
    ros::ServiceClient set_mode_srv;
    ros::ServiceClient arm_srv;

    geometry_msgs::PoseStamped target_position;
    geometry_msgs::PoseStamped current_position;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offboard_landing");

    DroneController controller;
    controller.gotoPosition(0.0, 0.0, 1.0);  
    controller.gotoPosition(2.0, 0.0, 1.0);   
    controller.gotoPosition(2.0, 2.0, 1.0);   
    controller.gotoPosition(0.0, 2.0, 1.0);  
    controller.gotoPosition(0.0, 0.0, 1.0); 

    return 0;
}
