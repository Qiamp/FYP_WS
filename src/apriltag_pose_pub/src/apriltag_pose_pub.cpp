#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <Eigen/Geometry>
#include <memory>

struct AprilTagTransformer {
    AprilTagTransformer() : nh_("~") {
        // 初始化订阅器和发布器
        drone_pose_sub_ = nh_.subscribe("/mavros/vision_pose/pose", 10, 
            &AprilTagTransformer::dronePoseCallback, this);
        april_tag_sub_ = nh_.subscribe("/tag_detections", 10,
            &AprilTagTransformer::aprilTagCallback, this);
        
        body_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/tag_detections/tagpose_body", 10);
        inertial_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/tag_detections/tagpose_inertial", 10);

        // 初始化固定变换矩阵（示例值，需要根据实际标定结果修改）
        initTransforms();
    }

    ros::NodeHandle nh_;
    ros::Subscriber drone_pose_sub_, april_tag_sub_;
    ros::Publisher body_pose_pub_, inertial_pose_pub_;
    
    geometry_msgs::PoseStamped current_drone_pose_;
    bool has_drone_pose_ = false;
    
    // 使用Eigen的等距变换类型存储坐标变换
    Eigen::Isometry3d T_body_camera_;  // 相机到机体的变换

    void initTransforms() {
        /******************************************************************
        * 初始化相机到机体的固定变换矩阵 (T_body_camera)
        * 需要根据实际标定结果设置旋转矩阵和平移向量
        * 示例值：
        *   R = [-0.02215281  -0.99669549  -0.07814961
        *        -0.99769073   0.0270596   -0.06229757
        *         0.0642064    0.07658907  -0.99499329]
        *   t = [0.00427512, -0.00026231, -0.00273313]
        ******************************************************************/
        T_body_camera_ = Eigen::Isometry3d::Identity();
        
        // 设置旋转矩阵
        Eigen::Matrix3d rotation;
        rotation << -0.02215281, -0.99669549, -0.07814961,
                    -0.99769073,  0.0270596,  -0.06229757,
                     0.0642064,   0.07658907, -0.99499329;
        T_body_camera_.linear() = rotation;

        // 设置平移向量 (单位：米)
        T_body_camera_.translation() << 0.00, -0.000, -0.00;
    }

    void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_drone_pose_ = *msg;
        has_drone_pose_ = true;
    }

    void aprilTagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
        if (msg->detections.empty()) {
            ROS_WARN_THROTTLE(1, "No AprilTag detected");
            return;
        }

        // 使用第一个检测到的标签
        const auto& detection = msg->detections[0];
        const auto& tag_in_camera = detection.pose.pose.pose.position;

        try {

            // 将Apriltag的位置从相机坐标系转换到机体坐标系
            Eigen::Vector3d tag_in_body = cameraToBody(tag_in_camera);


            // 发布apriltag相对于body_link的位置
            publishBodyPose(tag_in_body, msg->header.stamp);

            if (!has_drone_pose_) {
                ROS_WARN_THROTTLE(1, "Waiting for drone pose...");
                return;
            }

            // 将Apriltag的位置从机体坐标系转换到惯性坐标系
            Eigen::Vector3d body_in_inertial = bodyToInertial(tag_in_body);

            // 发布apriltag相对于map的位置
            publishInertialPose(body_in_inertial, msg->header.stamp);
        } catch (const std::exception& e) {
            ROS_ERROR("Transform error: %s", e.what());
        }
    }

    Eigen::Vector3d cameraToBody(const geometry_msgs::Point& point) {
        // 将点从相机坐标系转换到机体坐标系
        Eigen::Vector3d p_camera(point.x, point.y, point.z);
        Eigen::Vector3d p_body = T_body_camera_ * p_camera;
        return p_body;
    }

    Eigen::Vector3d bodyToInertial(const Eigen::Vector3d& point) {
        // 构造无人机当前位姿的变换矩阵
        Eigen::Isometry3d T_inertial_body = Eigen::Isometry3d::Identity();
        
        // 设置旋转
        Eigen::Quaterniond q(
            current_drone_pose_.pose.orientation.w,
            current_drone_pose_.pose.orientation.x,
            current_drone_pose_.pose.orientation.y,
            current_drone_pose_.pose.orientation.z
        );
        T_inertial_body.linear() = q.toRotationMatrix();

        // 设置平移
        T_inertial_body.translation() << 
            current_drone_pose_.pose.position.x,
            current_drone_pose_.pose.position.y,
            current_drone_pose_.pose.position.z;

        // 执行坐标变换
        return T_inertial_body * point;
    }

    void publishBodyPose(const Eigen::Vector3d& position, const ros::Time& stamp) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "body_link";  // 机体坐标系
        msg.pose.position.x = position.x();
        msg.pose.position.y = position.y();
        msg.pose.position.z = position.z();
        body_pose_pub_.publish(msg);
    }

    void publishInertialPose(const Eigen::Vector3d& position, const ros::Time& stamp) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";  // 惯性坐标系
        msg.pose.position.x = position.x();
        msg.pose.position.y = position.y();
        msg.pose.position.z = position.z();
        inertial_pose_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "apriltag_coordinate_transform");
    AprilTagTransformer transformer;
    ros::spin();
    return 0;
}