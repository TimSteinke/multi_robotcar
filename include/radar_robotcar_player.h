#ifndef SRC_RADAR_ROBOTCAR_PLAYER_INCLUDE_RADAR_ROBOTCAR_PLAYER_H_
#define SRC_RADAR_ROBOTCAR_PLAYER_INCLUDE_RADAR_ROBOTCAR_PLAYER_H_

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>

namespace sensor {
// namespace globals (will be defined in radar_robotcar_player.cpp)
extern const std::string mono_left_frame, mono_right_frame, mono_rear_frame,
    stereo_left_frame, stereo_centre_frame, stereo_right_frame, radar_frame,
    lidar_left_frame, lidar_right_frame, gps_frame, base_frame;
extern ros::Time t0_oxford, t0_wall;
extern std::string tf_prefix;
extern ros::Duration clip_duration;
extern std::string play_until_timestamp;

struct CameraCalibration {
  // Image heigth, width
  Eigen::Vector2d image_size;
  // Rotation Matrix, R
  Eigen::Matrix<double, 3, 3> R;
  // Projection Matrix, P
  Eigen::Matrix<double, 3, 4> P = Eigen::Matrix<double, 3, 4>::Identity();
  // Camera intrinsics, K
  Eigen::Matrix<double, 3, 3> K = Eigen::Matrix3d::Identity();
  // Distortion parameters, default empty.
  Eigen::Matrix<double, 1, 5> D = Eigen::Matrix<double, 1, 5>::Zero();
  // convert them to ROS
  void to_caminfo_msg(sensor_msgs::CameraInfo &info);
  void read_calibration(sensor_msgs::CameraInfo &info, std::string filename);
};

// TODO: refactor to have a common abstract superclass for all these sensors?
// too much of the code is duplicated.
class mono {
private:
  std::string models_path;
  std::string dataset;
  ros::NodeHandle nh;
  std::vector<std::string> stamps_left_str, stamps_right_str, stamps_rear_str;
  double avg_frametime_nsecs;

  sensor_msgs::CameraInfo info_mono_left, info_mono_right, info_mono_rear;

  // const std::vector<float> trans_left = {-0.0905, 1.6375,  0.2803,
  //                                        0.2079,  -0.2339, 1.2321};
  // const std::vector<float> trans_right = {-0.2587, -1.6810, 0.3226,
  //                                         -0.1961, -0.2469, -1.2675};
  // const std::vector<float> trans_rear = {-2.0582, 0.0894,  0.3675,
  //                                        -0.0119, -0.2498, 3.1283};

public:
  std::vector<ros::Time> stamps_ros_left, stamps_ros_right, stamps_ros_rear;

  mono(const std::string &sdk_path, const std::string &dataset_path,
       ros::NodeHandle node);
  void publish(std::string cam);

  std::thread thread_left() {
    return std::thread(&mono::publish, this, "left");
  }
  std::thread thread_right() {
    return std::thread(&mono::publish, this, "right");
  }
  std::thread thread_rear() {
    return std::thread(&mono::publish, this, "rear");
  }
}; // end class mono

class stereo {
private:
  std::string models_path;
  std::string dataset_path_;
  std::string source_path;
  ros::NodeHandle nh;
  std::vector<std::string> stamps_str;
  double avg_frametime_nsecs;

  sensor_msgs::CameraInfo info_wide_left, info_wide_right;
  sensor_msgs::CameraInfo info_narrow_left, info_narrow_right;

  // from SDK extrinsics
  // std::vector<float> trans_left = {0, 0, 0, 0, 0, 0};
  // std::vector<float> trans_centre = {0, 0.119997, 0, 0, 0, 0};
  // std::vector<float> trans_right = {0, 0.239983, 0, 0, 0, 0};

public:
  std::vector<ros::Time> stamps_ros;

  stereo(const std::string &sdk_path, const std::string &dataset_path,
         ros::NodeHandle nh);
  void publish();
}; // class stereo

class lidar {
private:
  ros::NodeHandle nh;
  std::string left_path, right_path;
  std::vector<std::string> stamps_left_str, stamps_right_str;
  double avg_frametime_nsecs;

  //  From SDK extinsics
  // std::vector<float> trans_left = {-0.60072,   -0.34077,  -0.26837,
  //                                  -0.0053948, -0.041998, -3.1337};
  // std::vector<float> trans_right = {-0.61153,  0.55676,   -0.27023,
  //                                   0.0027052, -0.041999, -3.1357};

public:
  std::vector<ros::Time> stamps_left_ros, stamps_right_ros;

  lidar(const std::string &dataset_path, ros::NodeHandle node);
  void publish(std::string left_or_right);
  std::thread thread_left() {
    return std::thread(&lidar::publish, this, "left");
  }
  std::thread thread_right() {
    return std::thread(&lidar::publish, this, "right");
  }
}; // class lidar

class gps {
private:
  struct GpsStructure {
    std::string timestamp;
    uint num_satellites;
    double latitude, longitude, altitude;
    double latitude_sigma, longitude_sigma, altitude_sigma;
    double northing, easting, down;
    std::string utm_zone;
  };
  struct InsStructure {
    std::string timestamp, ins_status;
    double latitude, longitude, altitude;
    double northing, easting, down;
    std::string utm_zone;
    double velocity_north, velocity_east, velocity_down;
    float roll, pitch, yaw;
  };
  ros::NodeHandle nh;
  std::string gps_path, ins_path;
  std::vector<GpsStructure> gps_readings;
  std::vector<InsStructure> ins_readings;

  //  From SDK extinsics
  // std::vector<float> gps_trans = {-1.7132, 0.1181, 1.1948,
  //                                 -0.0125, 0.0400, 0.0050};
  // std::vector<float> ins_trans = {-1.7132, 0.1181, 1.1948,
  //                                 -0.0125, 0.0400, 0.0050};

public:
  std::vector<ros::Time> stamps_gps, stamps_ins;
  gps(const std::string &dataset_path, ros::NodeHandle nh);
  void publishgps();
  void publishins();
  void publish_ins_pose_solution();
}; // class gps/ins

// namespace functions
ros::Time timestamp_to_rostime(const std::string &timestamp);

int find_alignment(std::vector<ros::Time> stamps_1,
                   std::vector<ros::Time> stamps_2);

size_t get_idx_after_seconds(const std::vector<ros::Time> &timestamps,
                             ros::Duration clip_time);

double get_average_frametime_nsecs(const std::vector<ros::Time> &stamps);

void read_timestamps(const std::string &filename,
                     std::vector<std::string> &timestamps_str,
                     std::vector<ros::Time> &timestamps_rostime);

size_t find_target_idx(const std::vector<ros::Time> &timestamps,
                       const ros::Time &desired_time);

void read_img_to_msg(std::string path, std::string frame, ros::Time stamp,
                     int cv_encoding, sensor_msgs::ImagePtr &img_msg_result);

ros::Time oxford_time_to_now(const ros::Time &oxford_time);

} // end namespace sensor

#endif /* SRC_RADAR_ROBOTCAR_PLAYER_INCLUDE_RADAR_ROBOTCAR_PLAYER_H_ */
