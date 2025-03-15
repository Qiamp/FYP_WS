#include <ros/ros.h>
#include <mavros_msgs/GPSRAW.h>
#include <nmea_msgs/Sentence.h>
#include <iomanip>
#include <sstream>
#include <cmath>

class GpsConverter {
public:
    GpsConverter() : nh_("~") {
        rtcm_pub_ = nh_.advertise<nmea_msgs::Sentence>("/nmea", 10);
        gps_sub_ = nh_.subscribe("/mavros/gpsstatus/gps1/raw", 10, &GpsConverter::gpsCallback, this);
    }

    void gpsCallback(const mavros_msgs::GPSRAW::ConstPtr& msg) {
        nmea_msgs::Sentence rtcm_msg;
        rtcm_msg.header.stamp = ros::Time::now();
        rtcm_msg.sentence = constructNmeaSentence(msg);
        rtcm_pub_.publish(rtcm_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher rtcm_pub_;
    ros::Subscriber gps_sub_;

    std::string constructNmeaSentence(const mavros_msgs::GPSRAW::ConstPtr& msg) {
        // Convert latitude/longitude from 1e7 degrees to degrees
        double lat_deg = msg->lat / 1e7;
        double lon_deg = msg->lon / 1e7;

        // Convert to degrees and minutes format
        int lat_deg_int = static_cast<int>(lat_deg);
        double lat_min = (std::fabs(lat_deg - lat_deg_int)) * 60;
        char lat_dir = lat_deg_int >= 0 ? 'N' : 'S';
        lat_deg_int = std::abs(lat_deg_int);

        int lon_deg_int = static_cast<int>(lon_deg);
        double lon_min = (std::fabs(lon_deg - lon_deg_int)) * 60;
        char lon_dir = lon_deg_int >= 0 ? 'E' : 'W';
        lon_deg_int = std::abs(lon_deg_int);

        // 使用header中的时间戳（假设为UTC时间）
        ros::Time time = msg->header.stamp;
        uint64_t total_seconds = time.sec;
        int hours = total_seconds / 3600 % 24;
        int minutes = (total_seconds % 3600) / 60;
        int seconds = total_seconds % 60;
        int milliseconds = time.nsec / 1000000;  // 纳秒转毫秒

        // HDOP处理（根据MAVROS定义eph单位为cm）
        float hdop = msg->eph / 100.0;  // cm转米

        std::ostringstream oss;
        oss << "$GPGGA," 
            << std::setfill('0') 
            << std::setw(2) << hours
            << std::setw(2) << minutes
            << std::setw(2) << seconds
            << "." << std::setw(3) << milliseconds << ","
            << std::setw(2) << lat_deg_int 
            << std::fixed << std::setprecision(4) << std::setw(7) << lat_min << "," << lat_dir << ","
            << std::setw(3) << lon_deg_int
            << std::fixed << std::setprecision(4) << std::setw(7) << lon_min << "," << lon_dir << ","
            << static_cast<int>(msg->fix_type) << ","
            << static_cast<int>(msg->satellites_visible) << ","
            << std::fixed << std::setprecision(1) << hdop << ","
            << std::fixed << std::setprecision(1) << msg->alt / 1000.0 << ",M,,";

        // Calculate checksum
        std::string sentence = oss.str();
        char checksum = 0;
        for(size_t i = 1; i < sentence.size(); ++i) {
            checksum ^= sentence[i];
        }
        
        oss << "*" << std::hex << std::uppercase 
            << std::setw(2) << static_cast<int>(checksum);
        return oss.str();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_rtcm_converter");
    GpsConverter converter;
    ros::spin();
    return 0;
}