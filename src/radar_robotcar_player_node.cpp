#include <csignal>
#include <errno.h>
#include <radar_robotcar_player.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "radar_robotcar_player",
            ros::init_options::AnonymousName);

  ros::NodeHandle nh("~");

  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

  std::string dataset_path, sdk_path;
  nh.getParam("dataset", dataset_path);
  nh.getParam("sdk", sdk_path);
  ROS_INFO("Using Dataset: %s\n SDK: %s", dataset_path.c_str(),
           sdk_path.c_str());

  std::string tf_prefix;
  nh.getParam("tf_prefix", tf_prefix);
  sensor::tf_prefix = tf_prefix;

  double clip_first_secs, startup_delay_secs;
  nh.param("clip_first_secs", clip_first_secs, 0.);
  if (clip_first_secs > 0.0) {
    ROS_INFO("Node %s will clip first %f seconds from the beginning of each "
             "sequence.",
             tf_prefix.c_str(), clip_first_secs);
  }
  sensor::clip_duration = ros::Duration(clip_first_secs);
  nh.param("startup_delay_secs", startup_delay_secs, 0.);

  // instantiate sensor classes
  sensor::mono mono(sdk_path, dataset_path, nh);
  sensor::stereo stereo(sdk_path, dataset_path, nh);
  sensor::lidar lidar(dataset_path, nh);
  sensor::gps gps(dataset_path, nh);

  //  now we find the earliest timestamp overall as the common starting point:
  sensor::t0_oxford = std::min(
      {mono.stamps_ros_left[0], mono.stamps_ros_right[0],
       mono.stamps_ros_rear[0], stereo.stamps_ros[0], lidar.stamps_left_ros[0],
       lidar.stamps_right_ros[0], gps.stamps_gps[0], gps.stamps_ins[0]});

  // wait for startup_delay
  if (startup_delay_secs > 0.) {
    ROS_INFO("Player node %s will start in %fs", sensor::tf_prefix.c_str(),
             startup_delay_secs);
    ros::Duration(startup_delay_secs).sleep();
    ROS_INFO("Player node %s will playing", sensor::tf_prefix.c_str());
  }

  // set common starting wall time
  sensor::t0_wall = ros::Time::now();

  // make threads
  std::thread thread_mono_left = mono.thread_left();
  std::thread thread_mono_right = mono.thread_right();
  std::thread thread_mono_rear = mono.thread_rear();

  std::thread thread_stereo(&sensor::stereo::publish, &stereo);

  std::thread thread_lidar_left = lidar.thread_left();
  std::thread thread_lidar_right = lidar.thread_right();

  std::thread thread_gps(&sensor::gps::publishgps, &gps);
  std::thread thread_ins(&sensor::gps::publishins, &gps);

  // wait for threads to finish
  thread_mono_left.join();
  thread_mono_right.join();
  thread_mono_rear.join();
  thread_stereo.join();
  thread_lidar_left.join();
  thread_lidar_right.join();
  thread_gps.join();
  thread_ins.join();
}
