#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "std_msgs/msg/header.hpp"
#include "ros2_api.h"
#include "ldlidar_driver.h"
#include "ldlidar_datatype.h"
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ldlidar::LDType GetLidarTypeFromString(const std::string& product_name) {
    if (product_name == "LDLiDAR_LD14") {
        return ldlidar::LDType::LD_14;
    } else if (product_name == "LDLiDAR_LD14P") {
        return ldlidar::LDType::LD_14P_4000HZ; 
    }
    RCLCPP_ERROR(rclcpp::get_logger("GetLidarTypeFromString"),
                 "Unknown product_name: %s. Defaulting to LD_14.", product_name.c_str());
    return ldlidar::LDType::LD_14;
}

uint64_t GetSystemTimeStamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
      std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return (uint64_t)tmp.count();
}


class LDLiDARSLROS2Node : public rclcpp::Node {
public:
    LDLiDARSLROS2Node() : Node("ldlidar_publisher_node") {
        this->declare_parameter<std::string>("product_name", "LDLiDAR_LD14");
        this->declare_parameter<std::string>("laser_scan_topic_name", "scan");
        this->declare_parameter<std::string>("point_cloud_2d_topic_name", "pointcloud2d");
        this->declare_parameter<std::string>("frame_id", "base_laser");
        this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
        this->declare_parameter<int>("serial_baudrate", 115200);
        this->declare_parameter<bool>("laser_scan_dir", true); // true = CCW, false = CW
        this->declare_parameter<bool>("enable_angle_crop_func", false);
        this->declare_parameter<double>("angle_crop_min", 0.0); // degrees
        this->declare_parameter<double>("angle_crop_max", 359.0); // degrees

        std::string product_name_str = this->get_parameter("product_name").as_string();
        std::string laser_scan_topic_name = this->get_parameter("laser_scan_topic_name").as_string();
        std::string point_cloud_topic_name = this->get_parameter("point_cloud_2d_topic_name").as_string();
        
        scan_setting_.frame_id = this->get_parameter("frame_id").as_string();
        std::string port_name_str = this->get_parameter("port_name").as_string();
        int serial_baudrate_val = this->get_parameter("serial_baudrate").as_int();
        scan_setting_.laser_scan_dir = this->get_parameter("laser_scan_dir").as_bool();
        scan_setting_.enable_angle_crop_func = this->get_parameter("enable_angle_crop_func").as_bool();
        scan_setting_.angle_crop_min = this->get_parameter("angle_crop_min").as_double();
        scan_setting_.angle_crop_max = this->get_parameter("angle_crop_max").as_double();

        RCLCPP_INFO(this->get_logger(), "SDK Version: %s", lidar_driver_.GetLidarSdkVersionNumber().c_str());
        RCLCPP_INFO(this->get_logger(), "Product Name: %s", product_name_str.c_str());
        RCLCPP_INFO(this->get_logger(), "Laser Scan Topic: %s", laser_scan_topic_name.c_str());
        RCLCPP_INFO(this->get_logger(), "Point Cloud Topic: %s", point_cloud_topic_name.c_str());
        RCLCPP_INFO(this->get_logger(), "Frame ID: %s", scan_setting_.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "Port Name: %s", port_name_str.c_str());
        RCLCPP_INFO(this->get_logger(), "Serial Baudrate: %d", serial_baudrate_val);
        RCLCPP_INFO(this->get_logger(), "Laser Scan Dir (CCW): %s", scan_setting_.laser_scan_dir ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Angle Crop: %s", scan_setting_.enable_angle_crop_func ? "true" : "false");
        if (scan_setting_.enable_angle_crop_func) {
            RCLCPP_INFO(this->get_logger(), "Angle Crop Min: %.1f deg", scan_setting_.angle_crop_min);
            RCLCPP_INFO(this->get_logger(), "Angle Crop Max: %.1f deg", scan_setting_.angle_crop_max);
        }
        
        // pubs
        laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(laser_scan_topic_name, 10);
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(point_cloud_topic_name, 10);

        // init lidar driver
        lidar_driver_.RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp));
        lidar_driver_.EnableFilterAlgorithnmProcess(true); 

        ldlidar::LDType lidar_type = GetLidarTypeFromString(product_name_str);

        if (lidar_driver_.Start(lidar_type, port_name_str, serial_baudrate_val)) {
            RCLCPP_INFO(this->get_logger(), "LiDAR driver started successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to start LiDAR driver. Please check connections and permissions.");
            rclcpp::shutdown();
            return;
        }

        if (!lidar_driver_.WaitLidarCommConnect(1000)) {
             RCLCPP_ERROR(this->get_logger(), "LiDAR communication error or timeout.");
             lidar_driver_.Stop();
             rclcpp::shutdown();
             return;
        }
        RCLCPP_INFO(this->get_logger(), "LiDAR communication established.");

        double lidar_spin_hz = 10.0;
        if (lidar_driver_.GetLidarScanFreq(lidar_spin_hz)) {
             RCLCPP_INFO(this->get_logger(), "LiDAR spin frequency: %.2f Hz", lidar_spin_hz);
        } else {
             RCLCPP_WARN(this->get_logger(), "Could not get LiDAR spin frequency, using default %.2f Hz for polling.", lidar_spin_hz);
        }
        if (lidar_spin_hz < 1.0) lidar_spin_hz = 10.0;
        
        auto CaculateScanTime = [](const ldlidar::Points2D& points) -> double {
          if (points.empty()) return 0.0;
          return static_cast<double>(points.back().stamp - points.front().stamp) * 1e-9;
        };
        scan_time_ = CaculateScanTime(scan_points_);
        if (scan_time_ < 1e-3 && lidar_spin_hz > 1.0) {
             scan_time_ = 1.0 / lidar_spin_hz;
        }


        poll_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>((1.0 / lidar_spin_hz) * 1000.0 * 0.9)),
            std::bind(&LDLiDARSLROS2Node::scan_publish_timer_callback, this));
    }

    ~LDLiDARSLROS2Node() {
        RCLCPP_INFO(this->get_logger(), "Stopping LiDAR driver.");
        lidar_driver_.Stop();
    }

private:
    void scan_publish_timer_callback() {
        ldlidar::Points2D laser_scan_points;
        ldlidar::LidarStatus status = lidar_driver_.GetLaserScanData(laser_scan_points, 1000);

        if (status == ldlidar::LidarStatus::NORMAL) {
            if (laser_scan_points.empty()) {
                // RCLCPP_WARN(this->get_logger(), "Received empty scan data.");
                return;
            }
            
            if (laser_scan_points.size() > 1) {
                scan_time_ = static_cast<double>(laser_scan_points.back().stamp - laser_scan_points.front().stamp) * 1e-9;
                if (scan_time_ < 1e-3) { 
                    double freq = 10.0;
                    lidar_driver_.GetLidarScanFreq(freq);
                    if (freq > 1.0) scan_time_ = 1.0/freq; else scan_time_ = 0.1; //default 10Hz
                }
            }


            std::sort(laser_scan_points.begin(), laser_scan_points.end(), 
                      [](const ldlidar::PointData& a, const ldlidar::PointData& b) {
                          return a.angle < b.angle;
                      });
            
            if (!scan_setting_.laser_scan_dir) {
                std::reverse(laser_scan_points.begin(), laser_scan_points.end());
                for (auto& pt : laser_scan_points) {
                    pt.angle = (360.0f - pt.angle);
                    if (pt.angle < 0.0f) pt.angle += 360.0f;
                    if (pt.angle >= 360.0f) pt.angle -= 360.0f;
                }
                // Re-sort if reversing changed order significantly (e.g. around 0/360 cusp)
                 std::sort(laser_scan_points.begin(), laser_scan_points.end(), 
                      [](const ldlidar::PointData& a, const ldlidar::PointData& b) {
                          return a.angle < b.angle;
                      });
            }


            publish_laser_scan(laser_scan_points);
            publish_point_cloud(laser_scan_points);

        } else if (status == ldlidar::LidarStatus::DATA_TIME_OUT) {
            RCLCPP_WARN(this->get_logger(), "LiDAR data timeout.");
        } else if (status == ldlidar::LidarStatus::DATA_WAIT) {
        } else if (status == ldlidar::LidarStatus::ERROR) {
            RCLCPP_ERROR(this->get_logger(), "LiDAR error occurred. Error code: %d", lidar_driver_.GetLidarErrorCode());
        } else if (status == ldlidar::LidarStatus::STOP) {
            RCLCPP_INFO(this->get_logger(), "LiDAR is stopped.");
        }
    }

    void publish_laser_scan(const ldlidar::Points2D& points) {
        if (points.empty()) return;

        auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        
        scan_msg->header.stamp = rclcpp::Time(points.front().stamp);
        scan_msg->header.frame_id = scan_setting_.frame_id;

        scan_msg->angle_min = points.front().angle * M_PI / 180.0;
        scan_msg->angle_max = points.back().angle * M_PI / 180.0;
        
        if (scan_msg->angle_max < scan_msg->angle_min) {
             if (points.back().angle < points.front().angle + 1.0 && points.size() > 100) {
                 scan_msg->angle_max += 2.0 * M_PI;
             }
        }


        size_t num_points = points.size();
        if (num_points > 1) {
            scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / static_cast<float>(num_points - 1);
        } else {
            scan_msg->angle_increment = 0.0f;
        }
        
        if (num_points > 1 && scan_time_ > 1e-6) {
             scan_msg->time_increment = scan_time_ / static_cast<float>(num_points - 1);
        } else {
             scan_msg->time_increment = 0.0f;
        }
        scan_msg->scan_time = scan_time_;

        scan_msg->range_min = 0.12;
        scan_msg->range_max = 8.0;

        scan_msg->ranges.resize(num_points);
        scan_msg->intensities.resize(num_points);

        for (size_t i = 0; i < num_points; ++i) {
            float current_angle_deg = points[i].angle;
            float distance_m = static_cast<float>(points[i].distance) / 1000.0f;
            float intensity_val = static_cast<float>(points[i].intensity);

            if (scan_setting_.enable_angle_crop_func &&
                current_angle_deg >= scan_setting_.angle_crop_min &&
                current_angle_deg <= scan_setting_.angle_crop_max) {
                scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
                scan_msg->intensities[i] = 0.0f;
            } else {
                if (distance_m < scan_msg->range_min || distance_m > scan_msg->range_max) {
                    scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
                } else {
                    scan_msg->ranges[i] = distance_m;
                }
                scan_msg->intensities[i] = intensity_val;
            }
        }
        laser_scan_pub_->publish(std::move(scan_msg));
    }

    void publish_point_cloud(const ldlidar::Points2D& points) {
        if (points.empty()) return;

        auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud>();
        cloud_msg->header.stamp = rclcpp::Time(points.front().stamp);
        cloud_msg->header.frame_id = scan_setting_.frame_id;

        cloud_msg->points.resize(points.size());
        
        sensor_msgs::msg::ChannelFloat32 intensity_channel;
        intensity_channel.name = "intensity";
        intensity_channel.values.resize(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            float current_angle_deg = points[i].angle;
            
            if (scan_setting_.enable_angle_crop_func &&
                current_angle_deg >= scan_setting_.angle_crop_min &&
                current_angle_deg <= scan_setting_.angle_crop_max) {

                cloud_msg->points[i].x = 0.0f;
                cloud_msg->points[i].y = 0.0f;
                cloud_msg->points[i].z = 0.0f;
                intensity_channel.values[i] = 0.0f;
            } else {
                float distance_m = static_cast<float>(points[i].distance) / 1000.0f;
                float angle_rad = current_angle_deg * M_PI / 180.0f;
                
                cloud_msg->points[i].x = distance_m * std::cos(angle_rad);
                cloud_msg->points[i].y = distance_m * std::sin(angle_rad);
                cloud_msg->points[i].z = 0.0f;
                intensity_channel.values[i] = static_cast<float>(points[i].intensity);
            }
        }
        cloud_msg->channels.push_back(intensity_channel);
        point_cloud_pub_->publish(std::move(cloud_msg));
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr point_cloud_pub_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
    
    ldlidar::LDLidarDriver lidar_driver_;
    LaserScanSetting scan_setting_;
    ldlidar::Points2D scan_points_;
    double scan_time_ = 0.1;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LDLiDARSLROS2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}