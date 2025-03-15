#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <deque>
#include <mutex>
#include <Eigen/Core>
#include <tf/transform_datatypes.h>
#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>

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
    GNSSLanding() : phase_(WAIT_FCU), home_set_(false), tag_position_set_(false), current_local_pose_received_(false) {
        // GNSS定位订阅
        gnss_raw_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>(
            "/mavros/global_position/raw/fix", 10, &GNSSLanding::gnssRawCb, this);
        
        // 本地位置订阅（使用PoseStamped）
        gnss_local_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "/mavros/local_position/pose", 10, &GNSSLanding::gnssLocalCb, this);
        
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

        // AprilTag大致坐标参数
        tag_lat_ = 31.2304;    // 纬度
        tag_lon_ = 121.4737;   // 经度
        tag_alt_ = 15.0;       // 海拔高度（单位：米）
        
        // 初始化飞行参数
        target_height_ = 1.5;      // 初始飞行高度
        landing_threshold_ = 0.1;  // 着陆位置容差
    }

    void run() {
        ros::Rate rate(20.0);
        
        // 等待连接和GNSS初始化
        while(ros::ok() && (!current_state_.connected || !home_set_ || !current_local_pose_received_)) {
            ros::spinOnce();
            rate.sleep();
            ROS_INFO_THROTTLE(1, "Waiting for FCU & GNSS & Local Pose initialization...");
        }

        // 设置初始悬停点（当前本地坐标系正上方）
        SetHoverTarget();

        // 计算AprilTag目标坐标（基于当前本地坐标系）
        ComputeApproxTagPosition();

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
    
    // 定位相关
    sensor_msgs::NavSatFix home_position_;
    geometry_msgs::PoseStamped current_local_pose_;
    geometry_msgs::PoseStamped hover_target_, approx_tag_target_, precise_tag_target_;
    bool home_set_, tag_position_set_, current_local_pose_received_;
    
    // AprilTag参数
    double tag_lat_, tag_lon_, tag_alt_;
    float target_height_;
    double landing_threshold_;

    // AprilTag跟踪
    std::deque<geometry_msgs::Point> tag_position_history_;
    std::mutex tag_pose_mutex_;
    const size_t STABLE_SAMPLES = 30;
    const double STABLE_THRESHOLD = 0.1;  // GNSS定位稳定性阈值

    // ROS通信
    ros::Subscriber gnss_raw_sub_, gnss_local_sub_, tag_pose_sub_, state_sub_, ext_state_sub_;
    ros::Publisher pose_pub_;
    ros::ServiceClient arm_client_, mode_client_;
    mavros_msgs::State current_state_;
    mavros_msgs::ExtendedState ext_state_;

    // GNSS原始定位回调
    void gnssRawCb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if(!home_set_) {
            home_position_ = *msg;
            home_set_ = true;
            ROS_INFO("Home position set: [%.7f, %.7f, %.2f]", 
                    home_position_.latitude,
                    home_position_.longitude,
                    home_position_.altitude);
        }
    }

    // 本地定位回调
    void gnssLocalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_local_pose_ = *msg;
        current_local_pose_received_ = true;
    }

    // AprilTag定位回调
    void tagPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(tag_pose_mutex_);
        
        // 维护位置历史记录
        tag_position_history_.push_back(msg->pose.position);
        if(tag_position_history_.size() > STABLE_SAMPLES) {
            tag_position_history_.pop_front();
        }
        
        // 更新精确目标位置
        precise_tag_target_ = *msg;
        precise_tag_target_.pose.position.z = current_local_pose_.pose.position.z;
        tag_position_set_ = true;
    }

    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    void extStateCb(const mavros_msgs::ExtendedState::ConstPtr& msg) {
        ext_state_ = *msg;
    }

    // 计算AprilTag大致本地坐标（基于当前本地坐标系）
    void ComputeApproxTagPosition() {
        // 获取当前本地坐标系原点对应的UTM坐标
        geographic_msgs::GeoPoint current_geo;
        current_geo.latitude = home_position_.latitude;
        current_geo.longitude = home_position_.longitude;
        current_geo.altitude = home_position_.altitude;

        geodesy::UTMPoint current_utm;
        geodesy::fromMsg(current_geo, current_utm);

        // 计算目标点UTM坐标
        geographic_msgs::GeoPoint tag_geo;
        tag_geo.latitude = tag_lat_;
        tag_geo.longitude = tag_lon_;
        tag_geo.altitude = tag_alt_;

        geodesy::UTMPoint tag_utm;
        geodesy::fromMsg(tag_geo, tag_utm);

        // 转换为本地ENU坐标系偏移（相对于当前本地坐标系原点）
        approx_tag_target_.pose.position.x = tag_utm.easting - current_utm.easting;
        approx_tag_target_.pose.position.y = tag_utm.northing - current_utm.northing;
        approx_tag_target_.pose.position.z = target_height_;
        
        ROS_INFO("Computed approximate tag position: [%.2f, %.2f, %.2f]", 
                approx_tag_target_.pose.position.x,
                approx_tag_target_.pose.position.y,
                approx_tag_target_.pose.position.z);
    }

    // 设置悬停目标（当前本地坐标系正上方）
    void SetHoverTarget() {
        hover_target_.pose.position.x = current_local_pose_.pose.position.x;
        hover_target_.pose.position.y = current_local_pose_.pose.position.y;
        hover_target_.pose.position.z = target_height_;
        ROS_INFO("Set hover target at current position: [%.2f, %.2f, %.2f]", 
                hover_target_.pose.position.x,
                hover_target_.pose.position.y,
                hover_target_.pose.position.z);
    }

    bool CheckPositionReached(const geometry_msgs::PoseStamped& target, double tolerance) {
        double dx = current_local_pose_.pose.position.x - target.pose.position.x;
        double dy = current_local_pose_.pose.position.y - target.pose.position.y;
        double dz = current_local_pose_.pose.position.z - target.pose.position.z;
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
                if(current_state_.connected && home_set_) {
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
                pose_pub_.publish(approx_tag_target_);
                if(CheckPositionReached(approx_tag_target_, 0.2)) {
                    phase_ = DETECT_TAG;
                    ROS_INFO("[4/6] Reached approximate tag position");
                }
                break;
                
            case DETECT_TAG:
                if(IsTagPositionStable() && tag_position_set_) {
                    phase_ = MOVE_TO_TAG;
                    ROS_INFO("[5/6] Target position stabilized");
                }
                break;
                
            case MOVE_TO_TAG:
                if(CheckPositionReached(precise_tag_target_, landing_threshold_)) {
                    phase_ = LAND_DRONE;
                    ROS_INFO("[6/6] Reached landing position");
                } else {
                    pose_pub_.publish(precise_tag_target_);
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
    ros::init(argc, argv, "offboard_landing_gnss_node");
    GNSSLanding controller;
    controller.run();
    return 0;
}