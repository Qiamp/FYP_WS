#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <Eigen/Geometry>
#include <memory>

struct AprilTagTransformer {
    AprilTagTransformer() : nh_("~"), is_filter_initialized_(false), last_detection_time_(0) {
        drone_pose_sub_ = nh_.subscribe("/mavros/vision_pose/pose", 10, 
            &AprilTagTransformer::dronePoseCallback, this);
        april_tag_sub_ = nh_.subscribe("/tag_detections", 10,
            &AprilTagTransformer::aprilTagCallback, this);
        
        body_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/tag_detections/tagpose_body", 10);
        inertial_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/tag_detections/tagpose_inertial", 10);

        // 加载滤波参数
        nh_.param("filter_alpha", filter_alpha_, 0.2);  // 平滑系数
        nh_.param("max_time_gap", max_time_gap_, 1.5);   // 最大允许时间间隔重置阈值

        initTransforms();
    }

    ros::NodeHandle nh_;
    ros::Subscriber drone_pose_sub_, april_tag_sub_;
    ros::Publisher body_pose_pub_, inertial_pose_pub_;
    
    geometry_msgs::PoseStamped current_drone_pose_;
    bool has_drone_pose_ = false;
    
    Eigen::Isometry3d T_body_camera_;
    Eigen::Vector3d filtered_position_;  // 滤波后的位置
    bool is_filter_initialized_;         // 滤波器初始化标志
    double filter_alpha_;                // EMA滤波系数
    double max_time_gap_;                // 最大允许时间间隔
    ros::Time last_detection_time_;      // 上次检测时间戳

    void initTransforms() {
        T_body_camera_ = Eigen::Isometry3d::Identity();
        
        Eigen::Matrix3d rotation;
        rotation << -0.02215281, -0.99669549, -0.07814961,
                    -0.99769073,  0.0270596,  -0.06229757,
                     0.0642064,   0.07658907, -0.99499329;
        T_body_camera_.linear() = rotation;
        T_body_camera_.translation() << 0.00, 0.00, 0.00;
    }

    void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_drone_pose_ = *msg;
        has_drone_pose_ = true;
    }

    void aprilTagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
        if (msg->detections.empty()) {
            ROS_WARN_THROTTLE(1, "[AprilTag] No tags detected!");  // 未检测到标签时的警告
            return;
        }

        ROS_INFO_THROTTLE(1, "[AprilTag] Detected %zu tags", msg->detections.size());  // 检测到的标签

        const auto& detection = msg->detections[0];
        const auto& tag_pose_camera = detection.pose.pose.pose;

        try {
            // 转换姿态到机体坐标系
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

            // 相机到机体的完整变换
            Eigen::Isometry3d pose_body = T_body_camera_ * pose_camera;
            // 手动填充pose_body的平移为直接获取的三维坐标
            pose_body.translation() <<  -tag_pose_camera.position.y + 0.23, //Body X轴
                                        -tag_pose_camera.position.x, //Body Y轴
                                        -tag_pose_camera.position.z - 0.08; //Body Z轴

            // 发布机体坐标系下的完整姿态
            publishBodyPose(pose_body, msg->header.stamp);

            if (!has_drone_pose_) return;

            // 转换到惯性坐标系
            Eigen::Isometry3d T_inertial_body = Eigen::Isometry3d::Identity();
            T_inertial_body.linear() = Eigen::Quaterniond(
                current_drone_pose_.pose.orientation.w,
                current_drone_pose_.pose.orientation.x,
                current_drone_pose_.pose.orientation.y,
                current_drone_pose_.pose.orientation.z
            ).toRotationMatrix();
            T_inertial_body.translation() << current_drone_pose_.pose.position.x,
                                           current_drone_pose_.pose.position.y,
                                           current_drone_pose_.pose.position.z;

            Eigen::Isometry3d pose_inertial = T_inertial_body * pose_body;

            // 应用位置滤波器
            applyPositionFilter(pose_inertial, msg->header.stamp);
            publishInertialPose(pose_inertial, msg->header.stamp);
        } catch (const std::exception& e) {
            ROS_ERROR("Transform error: %s", e.what());
        }
    }

    void applyPositionFilter(Eigen::Isometry3d& pose, const ros::Time& stamp) {
        Eigen::Vector3d current_pos = pose.translation();
        
        // 检查时间间隔决定是否重置滤波器
        if (is_filter_initialized_) {
            double dt = (stamp - last_detection_time_).toSec();
            if (dt > max_time_gap_) {
                ROS_WARN("[AprilTag] Detected %.1f seconds interval, resetting filter", dt); // 警告，有时间间隔，重置滤波器
                is_filter_initialized_ = false;
            }
        }

        // 更新滤波器
        if (!is_filter_initialized_) {
            filtered_position_ = current_pos;
            is_filter_initialized_ = true;
            ROS_INFO("[AprilTag] Position filter initialized"); // 信息，滤波器已初始化
        } else {
            filtered_position_ = filter_alpha_ * current_pos 
                               + (1.0 - filter_alpha_) * filtered_position_;
        }

        pose.translation() = filtered_position_;
        last_detection_time_ = stamp;
    }

    void publishBodyPose(const Eigen::Isometry3d& pose, const ros::Time& stamp) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "body_link";
        
        // 位置
        msg.pose.position.x = pose.translation().x();
        msg.pose.position.y = pose.translation().y();
        msg.pose.position.z = pose.translation().z();
        
        // 方向
        Eigen::Quaterniond q(pose.linear());
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
        msg.pose.orientation.w = q.w();
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        
        inertial_pose_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "apriltag_pose_pub");
    AprilTagTransformer transformer;
    ros::spin();
    return 0;
}