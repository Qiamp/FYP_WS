#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <Eigen/Geometry>
#include <deque>
#include <memory>

struct AprilTagTransformer {
    AprilTagTransformer() : nh_("~"), is_filter_initialized_(false), last_detection_time_(0) {
        // 订阅GNSS和姿态数据
        attitude_sub_ = nh_.subscribe("/mavros/local_position/pose", 10,
            &AprilTagTransformer::PoseCallback, this);
        april_tag_sub_ = nh_.subscribe("/tag_detections", 10,
            &AprilTagTransformer::aprilTagCallback, this);

        // 初始化发布者
        body_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/tag_detections/tagpose_body", 10);
        inertial_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/tag_detections/tagpose_inertial", 10);

        // 加载参数
        nh_.param("filter_alpha", filter_alpha_, 0.2);
        nh_.param("max_time_gap", max_time_gap_, 1.5);
        nh_.param("buffer_size", buffer_size_, 15);
        nh_.param("position_threshold", position_threshold_, 0.05);

        // 初始化相机到机体系变换矩阵
        initTransforms();
    }

    ros::NodeHandle nh_;
    ros::Subscriber gnss_sub_, attitude_sub_, april_tag_sub_;
    ros::Publisher body_pose_pub_, inertial_pose_pub_;
    
    geometry_msgs::Point current_gnss_position_;
    geometry_msgs::Quaternion current_attitude_;
    bool has_gnss_position_ = false;
    bool has_attitude_ = false;
    
    Eigen::Isometry3d T_body_camera_;
    Eigen::Vector3d filtered_position_;
    bool is_filter_initialized_;
    double filter_alpha_;
    double max_time_gap_;
    ros::Time last_detection_time_;

    std::deque<Eigen::Vector3d> position_buffer_;
    int buffer_size_;
    double position_threshold_;

    void initTransforms() {
        // 保持原始相机到机体系的变换参数
        T_body_camera_ = Eigen::Isometry3d::Identity();
        
        Eigen::Matrix3d rotation;
        rotation << -0.02215281, -0.99669549, -0.07814961,
                    -0.99769073,  0.0270596,  -0.06229757,
                     0.0642064,   0.07658907, -0.99499329;
        T_body_camera_.linear() = rotation;
        T_body_camera_.translation() << 0.00, 0.00, 0.00;
    }

    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_gnss_position_ = msg->pose.position;
        current_attitude_ = msg->pose.orientation;
        has_gnss_position_ = true;
        has_attitude_ = true;
        ROS_DEBUG_THROTTLE(5, "GNSS position, Attitude updated");
    }

    void aprilTagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
        if (msg->detections.empty()) {
            ROS_WARN_THROTTLE(1, "[AprilTag] No tags detected!");
            return;
        }

        ROS_INFO_THROTTLE(1, "[AprilTag] Detected %zu tags", msg->detections.size());

        const auto& detection = msg->detections[0];
        const auto& tag_pose_camera = detection.pose.pose.pose;

        try {
            // 完整的相机坐标系到机体系变换
            Eigen::Isometry3d pose_camera = Eigen::Isometry3d::Identity();
            pose_camera.translation() << tag_pose_camera.position.x,
                                        tag_pose_camera.position.y,
                                        tag_pose_camera.position.z;
            pose_camera.linear() = Eigen::Quaterniond(
                tag_pose_camera.orientation.w,
                tag_pose_camera.orientation.x,
                tag_pose_camera.orientation.y,
                tag_pose_camera.orientation.z
            ).toRotationMatrix();

            // 应用相机到机体系的变换
            Eigen::Isometry3d pose_body = T_body_camera_ * pose_camera;
            
            // 保留手动坐标修正
            pose_body.translation() <<  
                -tag_pose_camera.position.y + 0.23,
                -tag_pose_camera.position.x,
                -tag_pose_camera.position.z - 0.08;

            publishBodyPose(pose_body, msg->header.stamp);

            if (!has_gnss_position_ || !has_attitude_) {
                ROS_WARN_THROTTLE(1, "Waiting for GNSS/attitude data...");
                return;
            }

            // 构建惯性系变换矩阵
            Eigen::Isometry3d T_inertial_body = Eigen::Isometry3d::Identity();
            T_inertial_body.linear() = Eigen::Quaterniond(
                current_attitude_.w,
                current_attitude_.x,
                current_attitude_.y,
                current_attitude_.z
            ).toRotationMatrix();
            T_inertial_body.translation() << 
                current_gnss_position_.x,
                current_gnss_position_.y,
                current_gnss_position_.z;

            Eigen::Isometry3d pose_inertial = T_inertial_body * pose_body;
            pose_inertial.translation().y() -= 0.23;

            // 应用滤波器
            applyPositionFilter(pose_inertial, msg->header.stamp);
            updatePositionBuffer(pose_inertial.translation());

            if (checkPublishConditions()) {
                publishInertialPose(pose_inertial, msg->header.stamp);
            }

        } catch (const std::exception& e) {
            ROS_ERROR("Transform error: %s", e.what());
        }
    }

    void applyPositionFilter(Eigen::Isometry3d& pose, const ros::Time& stamp) {
        Eigen::Vector3d current_pos = pose.translation();
        
        if (is_filter_initialized_) {
            double dt = (stamp - last_detection_time_).toSec();
            if (dt > max_time_gap_) {
                ROS_WARN("[AprilTag] %.1fs gap detected, resetting filter", dt);
                position_buffer_.clear();
                is_filter_initialized_ = false;
            }
        }

        if (!is_filter_initialized_) {
            filtered_position_ = current_pos;
            is_filter_initialized_ = true;
            ROS_INFO("[AprilTag] Filter initialized");
        } else {
            filtered_position_ = filter_alpha_ * current_pos 
                               + (1.0 - filter_alpha_) * filtered_position_;
        }

        pose.translation() = filtered_position_;
        last_detection_time_ = stamp;
    }

    void updatePositionBuffer(const Eigen::Vector3d& position) {
        position_buffer_.push_back(position);
        if (position_buffer_.size() > buffer_size_) {
            position_buffer_.pop_front();
        }
    }

    bool checkPublishConditions() {
        if (position_buffer_.size() < buffer_size_) {
            ROS_INFO_THROTTLE(5, "Buffering data... (%zu/%d)", 
                            position_buffer_.size(), buffer_size_);
            return false;
        }

        Eigen::Vector3d min_val = position_buffer_.front();
        Eigen::Vector3d max_val = position_buffer_.front();
        
        for (const auto& pos : position_buffer_) {
            min_val = min_val.cwiseMin(pos);
            max_val = max_val.cwiseMax(pos);
        }

        Eigen::Vector3d ranges = max_val - min_val;
        if (ranges.x() < position_threshold_ &&
            ranges.y() < position_threshold_ &&
            ranges.z() < position_threshold_) 
        {
            ROS_INFO_THROTTLE(2, "Position stable | X±%.3fm Y±%.3fm Z±%.3fm", 
                             ranges.x()/2, ranges.y()/2, ranges.z()/2);
            return true;
        }

        ROS_WARN_THROTTLE(1, "Position noisy | X:%.3fm Y:%.3fm Z:%.3fm", 
                         ranges.x(), ranges.y(), ranges.z());
        return false;
    }

    void publishBodyPose(const Eigen::Isometry3d& pose, const ros::Time& stamp) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "body_link";
        
        msg.pose.position.x = pose.translation().x();
        msg.pose.position.y = pose.translation().y();
        msg.pose.position.z = pose.translation().z();
        
        Eigen::Quaterniond q(pose.linear());
        q.normalize();
        msg.pose.orientation.w = q.w();
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        
        body_pose_pub_.publish(msg);
    }

    void publishInertialPose(const Eigen::Isometry3d& pose, const ros::Time& stamp) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        
        msg.pose.position.x = pose.translation().x();
        msg.pose.position.y = pose.translation().y();
        msg.pose.position.z = pose.translation().z();
        
        Eigen::Quaterniond q(pose.linear());
        q.normalize();
        msg.pose.orientation.w = q.w();
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        
        inertial_pose_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "apriltag_pose_pub_gnss");
    AprilTagTransformer transformer;
    ros::spin();
    return 0;
}