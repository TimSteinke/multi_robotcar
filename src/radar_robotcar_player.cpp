#include <radar_robotcar_player.h>

namespace sensor {

// global frame id definitions. TODO: make configurable
const std::string mono_left_frame = "mono_left", mono_right_frame = "mono_right", mono_rear_frame = "mono_rear",
                  stereo_left_frame = "stereo_left", stereo_centre_frame = "stereo_centre",
                  stereo_right_frame = "stereo_right", lidar_left_frame = "velodyne_left",
                  lidar_right_frame = "velodyne_right", gps_frame = "gps_ins";

// note that stereo_left_frame is the reference, all other sensors have
// transforms from this frame to theirs.
const std::string base_frame = stereo_left_frame;
std::string tf_prefix = "";
ros::Duration clip_duration;

// have a common starting time in dataset timeframe
ros::Time t0_oxford;
// and current wall time
ros::Time t0_wall;

//////////////////////////////////////
// Camera Calibration ///////////////
////////////////////////////////////

void CameraCalibration::to_caminfo_msg(sensor_msgs::CameraInfo &info) {
  for (size_t i = 0; i < 5; ++i) info.D.push_back(D(0, i));  // unknown, push 0

  for (size_t i = 0; i < 3; ++i)
    for (size_t j = 0; j < 4; ++j) {
      if (j < 3) {
        info.K[i * 3 + j] = K(i, j);
        info.R[i * 3 + j] = R(i, j);
      }
      info.P[i * 4 + j] = P(i, j);
    }
  info.distortion_model = "plumb_bob";
}

void CameraCalibration::read_calibration(sensor_msgs::CameraInfo &info, std::string filename) {
  std::ifstream infile(filename, std::ios::in);
  std::string strline;
  getline(infile, strline);

  size_t index;

  // looks funny, better not touch anything
  index = strline.find_first_of(' ');
  K(0, 0) = stod(strline.substr(0, index));
  strline = strline.substr(++index);

  index = strline.find_first_of(' ');
  K(1, 1) = stod(strline.substr(0, index));
  strline = strline.substr(++index);

  index = strline.find_first_of(' ');
  K(0, 2) = stod(strline.substr(0, index));
  strline = strline.substr(++index);

  K(1, 2) = stod(strline);

  R << 0.0, -0.0, 1.0, 1.0, 0.0, -0.0, 0.0, 1.0, 0.0;

  P.block(0, 0, 3, 3) = R;
  getline(infile, strline);
  index = strline.find_last_of(' ');
  P(0, 3) = stod(strline.substr(index, strline.size() - index));
  getline(infile, strline);
  index = strline.find_last_of(' ');
  P(1, 3) = stod(strline.substr(index, strline.size() - index));

  infile.close();

  to_caminfo_msg(info);
}

//////////////////////////////////////////////
// Mono cameras /////////////////////////////
////////////////////////////////////////////

mono::mono(const std::string &sdk_path, const std::string &dataset_path, ros::NodeHandle node)
    : nh(node), dataset(dataset_path) {
  models_path = sdk_path + "/models/";

  std::string fname_left = dataset_path + "/mono_left.timestamps";
  std::string fname_right = dataset_path + "/mono_right.timestamps";
  std::string fname_rear = dataset_path + "/mono_rear.timestamps";

  read_timestamps(fname_left, stamps_left_str, stamps_ros_left);
  read_timestamps(fname_right, stamps_right_str, stamps_ros_right);
  read_timestamps(fname_rear, stamps_rear_str, stamps_ros_rear);

  avg_frametime_nsecs = get_average_frametime_nsecs(stamps_ros_left);
  avg_frametime_nsecs += get_average_frametime_nsecs(stamps_ros_right);
  avg_frametime_nsecs += get_average_frametime_nsecs(stamps_ros_rear);
  avg_frametime_nsecs /= 3;

  ROS_INFO("Average frametime for mono cams: %fms (frequency: %fHz)", avg_frametime_nsecs / 1e6,
           1 / (avg_frametime_nsecs / 1e9));

  CameraCalibration mono_left_cali, mono_right_cali, mono_rear_cali;
  mono_left_cali.read_calibration(info_mono_left, models_path + "mono_left.txt");
  mono_right_cali.read_calibration(info_mono_right, models_path + "mono_right.txt");
  mono_rear_cali.read_calibration(info_mono_rear, models_path + "mono_rear.txt");

  info_mono_left.height = info_mono_left.width = 1024;
  info_mono_right.height = info_mono_right.width = 1024;
  info_mono_rear.height = info_mono_rear.width = 1024;
}

void mono::publish(std::string cam) {
  std::string img_topic = "mono/" + cam;
  std::string info_topic = "mono/info/" + cam;
  std::string source_path = dataset + "/mono_" + cam + "/";

  std::vector<std::string> stamps_str;
  std::vector<ros::Time> stamps_ros;
  std::string frame;
  sensor_msgs::CameraInfo info_msg;

  // this function is called three times: once for each mono cam.
  if (cam == "left") {
    stamps_str = stamps_left_str;
    stamps_ros = stamps_ros_left;
    frame = mono_left_frame;
    info_msg = info_mono_left;

  } else if (cam == "right") {
    stamps_str = stamps_right_str;
    stamps_ros = stamps_ros_right;
    frame = mono_right_frame;
    info_msg = info_mono_right;

  } else if (cam == "rear") {
    stamps_str = stamps_rear_str;
    stamps_ros = stamps_ros_rear;
    frame = mono_rear_frame;
    info_msg = info_mono_rear;

  } else {
    throw std::invalid_argument("Unknown mono camera selected");
  }

  // initialize publishers
  ros::Publisher pub_image = nh.advertise<sensor_msgs::Image>(img_topic, 10);
  ros::Publisher pub_caminfo = nh.advertise<sensor_msgs::CameraInfo>(info_topic, 10);

  ROS_INFO("Will now publish mono images on %s", pub_image.getTopic().c_str());

  sensor_msgs::ImagePtr image_msg;

  // set the rate to 10kHz = 100 usecs
  ros::Rate loop_rate(10000);

  size_t i = 0;
  while (i < stamps_str.size() && ros::ok()) {
    std::string filename = stamps_str[i] + ".png";
    std::string filepath = source_path + filename;

    read_img_to_msg(filepath, frame, oxford_time_to_now(stamps_ros[i]), cv::COLOR_BayerBG2BGR, image_msg);

    info_msg.header.frame_id = tf_prefix + "/" + frame;

    info_msg.header.stamp = oxford_time_to_now(stamps_ros[i]);

    // wait until it's time to publish this message
    ros::Duration t_wall_since_start = ros::Time::now() - t0_wall;
    ros::Duration next_publishing_time = stamps_ros[i] - t0_oxford;
    while (t_wall_since_start < next_publishing_time && ros::ok()) {
      // wait until it's time to publish the next message
      loop_rate.sleep();
      t_wall_since_start = ros::Time::now() - t0_wall;
    }

    // publish the prepared msgs
    pub_image.publish(image_msg);
    pub_caminfo.publish(info_msg);

    ros::spinOnce();

    // check if we're falling behind and if so: skip frames
    ros::Duration t_oxford_since_start = stamps_ros[i] - t0_oxford;
    double abs_time_error = std::abs((t_wall_since_start - t_oxford_since_start).toNSec());

    if (abs_time_error > avg_frametime_nsecs * 2 && i < stamps_str.size() - 1) {
      ros::Time target_time = t0_oxford + t_wall_since_start;
      i = find_target_idx(stamps_ros, target_time);
    }

    i++;
  }
  ROS_INFO("Mono %s publisher thread done", cam.c_str());
}

///////////////////////////////////////////////
// Stereo Camera /////////////////////////////
/////////////////////////////////////////////

stereo::stereo(const std::string &sdk_path, const std::string &dataset_path, ros::NodeHandle nh)
    : nh(nh), dataset_path_(dataset_path) {
  models_path = sdk_path + "/models/";
  source_path = dataset_path_ + "/stereo";

  read_timestamps(dataset_path_ + "/stereo.timestamps", stamps_str, stamps_ros);
  avg_frametime_nsecs = get_average_frametime_nsecs(stamps_ros);

  ROS_INFO("Average frametime for Bumblebee stereo cam: %fms (frequency: %fHz)", avg_frametime_nsecs / 1e6,
           1 / (avg_frametime_nsecs / 1e9));

  // prepare camera info
  CameraCalibration wide_left_cali, wide_right_cali;
  CameraCalibration narrow_left_cali, narrow_right_cail;
  // wide model
  wide_left_cali.read_calibration(info_wide_left, models_path + "stereo_wide_left.txt");
  wide_right_cali.read_calibration(info_wide_right, models_path + "stereo_wide_right.txt");

  // narrow model
  narrow_left_cali.read_calibration(info_narrow_left, models_path + "stereo_narrow_left.txt");
  narrow_right_cail.read_calibration(info_narrow_right, models_path + "stereo_narrow_right.txt");

  info_wide_left.header.frame_id = tf_prefix + "/" + stereo_left_frame;
  info_wide_right.header.frame_id = tf_prefix + "/" + stereo_right_frame;

  info_narrow_left.header.frame_id = tf_prefix + "/" + stereo_left_frame;
  info_narrow_right.header.frame_id = tf_prefix + "/" + stereo_centre_frame;

  info_narrow_left.height = info_narrow_right.height = info_wide_left.height = info_wide_right.height = 960;
  info_narrow_left.width = info_narrow_right.width = info_wide_left.width = info_wide_right.width = 1280;
}

void stereo::publish() {
  std::string left_path, centre_path, right_path;

  ros::Publisher left_img_pub = nh.advertise<sensor_msgs::Image>("stereo/left", 10);
  ros::Publisher centre_img_pub = nh.advertise<sensor_msgs::Image>("stereo/centre", 10);
  ros::Publisher right_img_pub = nh.advertise<sensor_msgs::Image>("stereo/right", 10);

  ros::Publisher left_wide_info_pub = nh.advertise<sensor_msgs::CameraInfo>("stereo/info/wide/left", 10);
  ros::Publisher right_wide_info_pub = nh.advertise<sensor_msgs::CameraInfo>("stereo/info/wide/right", 10);

  ros::Publisher left_narrow_info_pub = nh.advertise<sensor_msgs::CameraInfo>("stereo/info/narrow/left", 10);
  ros::Publisher right_narrow_info_pub = nh.advertise<sensor_msgs::CameraInfo>("stereo/info/narrow/right", 10);

  ROS_INFO("Will now begin publishing stereo images on: %s, %s, %s", left_img_pub.getTopic().c_str(),
           centre_img_pub.getTopic().c_str(), right_img_pub.getTopic().c_str());

  // loop rate 10kHz = 100usec (just for inner loop)
  ros::Rate loop_rate(10000);
  size_t i = 0;

  while (i < stamps_str.size() && ros::ok()) {
    std::string filename = stamps_str[i] + ".png";
    left_path = source_path + "/left/" + filename;
    centre_path = source_path + "/centre/" + filename;
    right_path = source_path + "/right/" + filename;

    sensor_msgs::ImagePtr img_left, img_right, img_centre;
    ros::Time stamp_now = oxford_time_to_now(stamps_ros[i]);

    read_img_to_msg(left_path, stereo_left_frame, stamp_now, cv::COLOR_BayerGR2BGR, img_left);
    read_img_to_msg(centre_path, stereo_centre_frame, stamp_now, cv::COLOR_BayerGR2BGR, img_right);
    read_img_to_msg(right_path, stereo_right_frame, stamp_now, cv::COLOR_BayerGR2BGR, img_centre);

    info_wide_left.header.stamp = stamp_now;
    info_wide_right.header.stamp = stamp_now;

    info_narrow_left.header.stamp = stamp_now;
    info_narrow_right.header.stamp = stamp_now;

    ros::Duration t_wall_since_start = ros::Time::now() - t0_wall;
    ros::Duration next_publishing_time = stamps_ros[i] - t0_oxford;

    while (t_wall_since_start < next_publishing_time && ros::ok()) {
      // wait until it's time to publish these messages
      loop_rate.sleep();
      t_wall_since_start = ros::Time::now() - t0_wall;
    }

    left_img_pub.publish(img_left);
    centre_img_pub.publish(img_centre);
    right_img_pub.publish(img_right);

    left_wide_info_pub.publish(info_wide_left);
    right_wide_info_pub.publish(info_wide_right);

    left_narrow_info_pub.publish(info_narrow_left);
    right_narrow_info_pub.publish(info_narrow_right);

    ros::spinOnce();

    // TODO: all this is boilerplate to synchronize the loop to real time.
    // extract this somehow to avoid duplication in the other thread functions
    ros::Duration t_oxford_since_start = stamps_ros[i] - t0_oxford;

    double abs_time_error = std::abs((t_wall_since_start - t_oxford_since_start).toNSec());

    if (abs_time_error > avg_frametime_nsecs * 2 && i < stamps_str.size() - 1) {
      ros::Time target_time = t0_oxford + t_wall_since_start;
      i = find_target_idx(stamps_ros, target_time);
    }

    i++;
  }

  ROS_INFO("Stereo publisher thread done");
}

////////////////////////////////////////////////////
// Velodyne Lidar (left and right) ////////////////
//////////////////////////////////////////////////

lidar::lidar(const std::string &dataset_path, ros::NodeHandle nh) : nh(nh) {
  // TODO: edit from here
  left_path = dataset_path + "/velodyne_left/";
  right_path = dataset_path + "/velodyne_right/";

  std::ifstream infile_left, infile_right;
  std::string strline, subline;

  infile_left.open(dataset_path + "/velodyne_left.timestamps", std::ios::in);
  if (infile_left.is_open()) std::cout << "Reading velodyne_left timestamps from this txt." << std::endl;
  while (!infile_left.eof()) {
    getline(infile_left, strline);
    subline = strline.substr(0, strline.find_last_of(' '));
    stamps_left_str.push_back(subline);
  }
  infile_left.close();
  stamps_left_str.pop_back();

  read_timestamps(dataset_path + "/velodyne_left.timestamps", stamps_left_str, stamps_left_ros);
  read_timestamps(dataset_path + "/velodyne_right.timestamps", stamps_right_str, stamps_right_ros);

  avg_frametime_nsecs =
      (get_average_frametime_nsecs(stamps_left_ros) + get_average_frametime_nsecs(stamps_right_ros)) / 2;

  ROS_INFO("Average frametime for Velodyne 3D LiDARs: %fms (frequency: %fHz)", avg_frametime_nsecs / 1e6,
           1 / (avg_frametime_nsecs / 1e9));
}

void lidar::publish(std::string left_or_right) {
  ros::Publisher lidar_pub;
  std::string topic;
  std::string path;
  std::string frame;
  std::vector<std::string> stamps_str;
  std::vector<ros::Time> stamps_ros;

  if (left_or_right == "left") {
    topic = "lidar/left";
    path = left_path;
    stamps_ros = stamps_left_ros;
    stamps_str = stamps_left_str;
    frame = lidar_left_frame;

  } else if (left_or_right == "right") {
    topic = "lidar/right";
    path = right_path;
    stamps_ros = stamps_right_ros;
    stamps_str = stamps_right_str;
    frame = lidar_right_frame;

  } else {
    throw std::invalid_argument("Lidar thread started with invalid sensor selector");
  }

  lidar_pub = nh.advertise<sensor_msgs::PointCloud2>(topic, 1);

  ROS_INFO("Will now publish LiDAR data on: %s", lidar_pub.getTopic().c_str());

  pcl::PointCloud<pcl::PointXYZINormal> points;
  points.clear();
  points.reserve(1e6);

  // loop rate 10kHz = 100usec (just for inner loop)
  ros::Rate loop_rate(10000);

  size_t i = 0;

  while (i < stamps_str.size() && ros::ok()) {
    std::string filename = stamps_str[i] + ".bin";
    std::string filepath = path + filename;

    // get the total number of entries in file
    size_t num_entries = 0;
    long start, end;
    std::ifstream file(filepath, std::ios::in | std::ios::binary);

    if (!file) {
      ROS_ERROR("Could not open pointcloud file: %s", filepath.c_str());
      ros::shutdown();
    }

    start = file.tellg();
    file.seekg(0, std::ios::end);
    end = file.tellg();
    file.close();
    num_entries = (end - start) / (4 * sizeof(float));

    // re-open file to read points
    file = std::ifstream(filepath, std::ios::in | std::ios::binary);

    for (size_t i = 0; i < num_entries; ++i) {  // N*4, not 4*N!!
      if (file.good() && !file.eof()) {
        pcl::PointXYZINormal point;
        file.read((char *)&point.x, sizeof(float));
        points.push_back(point);
      }
    }
    for (size_t i = 0; i < num_entries; ++i)
      if (file.good() && !file.eof()) file.read((char *)&points[i].y, sizeof(float));
    for (size_t i = 0; i < num_entries; ++i)
      if (file.good() && !file.eof()) file.read((char *)&points[i].z, sizeof(float));
    for (size_t i = 0; i < num_entries; ++i)
      if (file.good() && !file.eof()) file.read((char *)&points[i].intensity, sizeof(float));

    file.close();

    sensor_msgs::PointCloud2 points_msg;
    pcl::toROSMsg(points, points_msg);
    points_msg.header.stamp = oxford_time_to_now(stamps_ros[i]);
    points_msg.header.frame_id = tf_prefix + "/" + frame;

    ros::Duration t_wall_since_start = ros::Time::now() - t0_wall;

    ros::Duration next_publishing_time = stamps_ros[i] - t0_oxford;
    while (t_wall_since_start < next_publishing_time && ros::ok()) {
      // wait until it's time to publish this message
      loop_rate.sleep();
      t_wall_since_start = ros::Time::now() - t0_wall;
      ROS_DEBUG_THROTTLE(0.1, "%s wait", topic.c_str());
    }

    // publish the messages
    lidar_pub.publish(points_msg);

    // uncomment to debug lidar publish timing (lots of spam!)
    // ROS_DEBUG("%s publish %s (wall: %f)", topic.c_str(),
    // stamps_str[i].c_str(), ros::Time::now().toSec());

    ros::spinOnce();

    points.clear();

    // check if we're falling behind and maybe skip some frames
    ros::Duration t_oxford_since_start = stamps_ros[i] - t0_oxford;
    double abs_time_error = std::abs((t_wall_since_start - t_oxford_since_start).toNSec());

    if (abs_time_error > avg_frametime_nsecs * 2 && i < stamps_str.size() - 1) {
      ros::Time target_time = t0_oxford + t_wall_since_start;
      int old_i = i;
      i = find_target_idx(stamps_ros, target_time);
      ROS_DEBUG_THROTTLE(0.1, "%s skip %d", topic.c_str(), static_cast<int>(i) - old_i);
    }

    i++;
  }
  ROS_INFO("Lidar %s publisher thread done", left_or_right.c_str());
}

gps::gps(const std::string &dataset_path, ros::NodeHandle nh) : nh(nh) {
  ins_path = dataset_path + "/gps/ins.csv";
  gps_path = dataset_path + "/gps/gps.csv";
  std::ifstream infile;
  infile.open(gps_path, std::ios::in);
  if (infile.is_open()) {
    ROS_DEBUG("Reading GPS data from: %s", gps_path.c_str());
  } else {
    throw std::invalid_argument("GPS file not found!");
  }

  std::string strline;
  size_t index;
  getline(infile, strline);

  while (getline(infile, strline)) {
    GpsStructure gps_reading;

    // TODO: C++ must have csv parsing libraries right?
    index = strline.find_first_of(',');
    gps_reading.timestamp = (strline.substr(0, index)).substr(3);
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    gps_reading.num_satellites = stoi(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    gps_reading.latitude = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    gps_reading.longitude = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    gps_reading.altitude = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    gps_reading.latitude_sigma = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    gps_reading.longitude_sigma = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    gps_reading.altitude_sigma = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    gps_reading.northing = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    gps_reading.easting = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    gps_reading.down = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    gps_reading.utm_zone = strline;

    gps_readings.push_back(gps_reading);
    stamps_gps.push_back(timestamp_to_rostime(gps_reading.timestamp));
  }
  infile.close();

  infile.open(ins_path, std::ios::in);
  if (infile.is_open()) {
    ROS_DEBUG("Reading INS data from: %s", ins_path.c_str());
  } else {
    throw std::invalid_argument("INS file not found!");
  }

  getline(infile, strline);

  while (getline(infile, strline)) {
    InsStructure ins_reading;

    index = strline.find_first_of(',');
    ins_reading.timestamp = (strline.substr(0, index)).substr(3);
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    ins_reading.ins_status = strline.substr(0, index);
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    ins_reading.latitude = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    ins_reading.longitude = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    ins_reading.altitude = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    ins_reading.northing = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    ins_reading.easting = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    ins_reading.down = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    ins_reading.utm_zone = strline.substr(0, index);
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    ins_reading.velocity_north = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    ins_reading.velocity_east = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    ins_reading.velocity_down = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    ins_reading.roll = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(',');
    ins_reading.pitch = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    ins_reading.yaw = stod(strline);

    ins_readings.push_back(ins_reading);
    stamps_ins.push_back(timestamp_to_rostime(ins_reading.timestamp));
  }
  infile.close();

  // apply clipping here
  size_t clip_idx_gps = get_idx_after_seconds(stamps_gps, clip_duration);
  size_t clip_idx_ins = get_idx_after_seconds(stamps_ins, clip_duration);

  stamps_gps.erase(stamps_gps.begin(), stamps_gps.begin() + clip_idx_gps);
  gps_readings.erase(gps_readings.begin(), gps_readings.begin() + clip_idx_gps);

  stamps_ins.erase(stamps_ins.begin(), stamps_ins.begin() + clip_idx_ins);
  ins_readings.erase(ins_readings.begin(), ins_readings.begin() + clip_idx_ins);
}

void gps::publishgps() {
  ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps/gps", 10);

  ROS_INFO("Will now publish GPS data to %s", gps_pub.getTopic().c_str());

  // set the rate to 10kHz = 100 usecs
  ros::Rate loop_rate(10000);

  size_t i = 0;
  while (i < gps_readings.size() && ros::ok()) {
    sensor_msgs::NavSatFix gps_position;
    gps_position.header.frame_id = tf_prefix + "/" + gps_frame;
    gps_position.header.stamp = oxford_time_to_now(stamps_gps[i]);
    // TODO: why are these hardcoded?
    gps_position.status.status = 0;
    gps_position.status.service = 1;
    gps_position.latitude = gps_readings[i].latitude;
    gps_position.longitude = gps_readings[i].longitude;
    gps_position.altitude = gps_readings[i].altitude;

    gps_pub.publish(gps_position);

    // wait until it's time to publish this message
    ros::Duration t_wall_since_start = ros::Time::now() - t0_wall;
    ros::Duration next_publishing_time = stamps_gps[i] - t0_oxford;
    while (t_wall_since_start < next_publishing_time && ros::ok()) {
      // wait until it's time to publish the next message
      loop_rate.sleep();
      t_wall_since_start = ros::Time::now() - t0_wall;
    }

    // actually publish the message
    ros::spinOnce();

    // No frame-skip here, since GPS data is very small, i don't think it's
    // necessary
    i++;
  }
  ROS_INFO("GPS publisher thread done");
}

void gps::publish_ins_pose_solution() {
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ins/pose", 10);
  tf::TransformBroadcaster tf_broadcaster;
  tf::TransformListener tf_listener;

  if (!tf_listener.waitForTransform(tf_prefix + "/" + "base_link", tf_prefix + "/" + gps_frame, ros::Time::now(),
                                    ros::Duration(2.0))) {
    ROS_ERROR("TF between gps_ins frame and base link not found!");
    ros::shutdown();
  }

  tf::StampedTransform tf_gps_to_base_link;
  tf_listener.lookupTransform(tf_prefix + "/" + "base_link", tf_prefix + "/" + gps_frame, ros::Time::now(),
                              tf_gps_to_base_link);

  ROS_INFO("Will now publish INS pose solution %s and on tf as transform between world and %s",
           pose_pub.getTopic().c_str(), (tf_prefix + "/base_link").c_str());

  float world_zero_northing, world_zero_easting, world_zero_down;
  nh.getParam("world_zero_northing", world_zero_northing);
  nh.getParam("world_zero_easting", world_zero_easting);
  nh.getParam("world_zero_down", world_zero_down);

  // set the rate to 10kHz = 100 usecs
  ros::Rate loop_rate(10000);

  size_t i = 0;
  while (i < ins_readings.size() && ros::ok()) {
    float x = ins_readings[i].easting - world_zero_easting;
    float y = ins_readings[i].northing - world_zero_northing;
    float z = ins_readings[i].down - world_zero_down;

    float roll = ins_readings[i].roll;
    float pitch = ins_readings[i].pitch;
    float yaw = ins_readings[i].yaw;

    tf::Transform pose_in_gps_frame;
    pose_in_gps_frame.setOrigin(tf::Vector3(x, y, z));
    pose_in_gps_frame.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));

    tf::Transform pose_in_base_link = tf_gps_to_base_link * pose_in_gps_frame;

    geometry_msgs::TransformStamped transform_msg;
    tf::transformTFToMsg(pose_in_base_link, transform_msg.transform);
    transform_msg.header.frame_id = "world";
    transform_msg.header.stamp = oxford_time_to_now(stamps_ins[i]);
    transform_msg.child_frame_id = tf_prefix + "/" + "base_link";

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = transform_msg.header;
    pose_msg.pose.position.x = transform_msg.transform.translation.x;
    pose_msg.pose.position.y = transform_msg.transform.translation.y;
    pose_msg.pose.position.z = transform_msg.transform.translation.z;

    pose_msg.pose.orientation = transform_msg.transform.rotation;

    // wait until it's time to publish this message
    ros::Duration t_wall_since_start = ros::Time::now() - t0_wall;
    ros::Duration next_publishing_time = stamps_ins[i] - t0_oxford;
    while (t_wall_since_start < next_publishing_time && ros::ok()) {
      // wait until it's time to publish the next message
      loop_rate.sleep();
      t_wall_since_start = ros::Time::now() - t0_wall;
    }

    // publish the message
    pose_pub.publish(pose_msg);
    tf_broadcaster.sendTransform(transform_msg);

    ros::spinOnce();

    i++;
  }
  ROS_INFO("INS pose publisher thread done");
}

void gps::publishins() {
  ros::Publisher ins_pub = nh.advertise<sensor_msgs::NavSatFix>("ins/gps", 10);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("ins/imu", 10);
  ROS_INFO("Will now publish INS data on %s", ins_pub.getTopic().c_str());
  ROS_INFO("Will now publish IMU data on %s", imu_pub.getTopic().c_str());

  // set the rate to 10kHz = 100 usecs
  ros::Rate loop_rate(10000);

  size_t i = 0;
  while (i < ins_readings.size() && ros::ok()) {
    sensor_msgs::NavSatFix gps;
    gps.header.frame_id = tf_prefix + "/" + gps_frame;
    gps.header.stamp = oxford_time_to_now(stamps_ins[i]);
    gps.status.status = 0;
    gps.status.service = 1;
    gps.latitude = ins_readings[i].latitude;
    gps.longitude = ins_readings[i].longitude;
    gps.altitude = ins_readings[i].altitude;

    sensor_msgs::Imu imu;
    tf::Quaternion q = tf::createQuaternionFromRPY(ins_readings[i].roll, ins_readings[i].pitch, ins_readings[i].yaw);
    imu.header.frame_id = tf_prefix + "/" + gps_frame;
    imu.header.stamp = gps.header.stamp;
    imu.orientation.x = q.x();
    imu.orientation.y = q.y();
    imu.orientation.z = q.z();
    imu.orientation.w = q.w();
    // TODO: we have a velocity solution from IMU, but how to publish?

    // wait until it's time to publish this message
    ros::Duration t_wall_since_start = ros::Time::now() - t0_wall;
    ros::Duration next_publishing_time = stamps_ins[i] - t0_oxford;
    while (t_wall_since_start < next_publishing_time && ros::ok()) {
      // wait until it's time to publish the next message
      loop_rate.sleep();
      t_wall_since_start = ros::Time::now() - t0_wall;
    }

    // publish the message
    ins_pub.publish(gps);
    imu_pub.publish(imu);

    ros::spinOnce();

    i++;
  }
  ROS_INFO("INS publisher thread done");
}

//////////////////////////////////////////////
// Namespaces functions /////////////////////
////////////////////////////////////////////

ros::Time timestamp_to_rostime(const std::string &timestamp) {
  // given a timestamp in microseconds since epoch, convert to rostime
  // convert usec to nsec
  int64_t nsec = stol(timestamp) * 1000;
  return ros::Time().fromNSec(nsec);
}

void read_timestamps(const std::string &filename, std::vector<std::string> &timestamps_str,
                     std::vector<ros::Time> &timestamps_rostime) {
  std::ifstream fs;
  fs.open(filename, std::ios::in);
  if (fs.is_open()) {
    ROS_DEBUG("Reading timestamps from %s", filename.c_str());
    std::string strline, subline;
    while (getline(fs, strline)) {
      subline = strline.substr(0, strline.find_last_of(' '));
      timestamps_str.push_back(subline);
      timestamps_rostime.push_back(timestamp_to_rostime(subline));
    }
    fs.close();
  } else {
    ROS_ERROR("Timestamps file not found: %s", filename.c_str());
  }

  // apply clipping here
  size_t clip_idx = get_idx_after_seconds(timestamps_rostime, clip_duration);
  timestamps_rostime.erase(timestamps_rostime.begin(), timestamps_rostime.begin() + clip_idx);
  timestamps_str.erase(timestamps_str.begin(), timestamps_str.begin() + clip_idx);
}

int find_alignment(std::vector<ros::Time> stamps_1, std::vector<ros::Time> stamps_2) {
  // figure out how 'misaligned' the left and right timelists are
  double min_tdist = std::numeric_limits<double>::max();
  int min_shift = 0;
  for (int shift = -25; shift < 25; shift++) {
    double avg_tdist = 0;
    int n = 1;
    for (int j = 0; j < 100; j++) {
      ros::Duration t_dist;
      if (shift >= 0) {
        t_dist = stamps_1[j] - stamps_2[j + shift];
      } else {
        t_dist = stamps_1[j - shift] - stamps_2[j];
      }
      double tdist_sec = abs(t_dist.toSec());
      avg_tdist += (tdist_sec - avg_tdist) / n;
      n += 1;
    }
    if (avg_tdist < min_tdist) {
      min_tdist = avg_tdist;
      min_shift = shift;
    }
  }
  return min_shift;
}

size_t get_idx_after_seconds(const std::vector<ros::Time> &timestamps, ros::Duration clip_time) {
  /* Assumes the list of stamps to be ordered and longer than clip_time. Returns
   * the index where the timestamp is about clip_time after the first timestamp.
   */
  size_t i = 0;
  ros::Duration time_passed(0);
  while (i < timestamps.size() - 1 && time_passed < clip_time) {
    time_passed += timestamps[i + 1] - timestamps[i];
    i++;
  }
  if (time_passed < clip_time) {
    // we failed, throw an error
    throw std::invalid_argument("get_idx_after_seconds: clip time was longer than given timelist");
  }
  return i;
}

double get_average_frametime_nsecs(const std::vector<ros::Time> &stamps) {
  // given a list of timestamps, compute the average time a single frame takes
  // (inverse frequency). For efficiency, only the first 1000 entries are
  // averaged.

  std::vector<u_int64_t> frametimes;
  double avg_frametime = 0;
  int n = 1;
  for (size_t i = 1; i < 1000; ++i) {
    u_int64_t frametime = (stamps[i] - stamps[i - 1]).toNSec();
    // online average equiv. to sum/n
    avg_frametime += (frametime - avg_frametime) / n;
    n++;
  }
  return avg_frametime;
}

void read_img_to_msg(std::string path, std::string frame, ros::Time stamp, int cv_encoding,
                     sensor_msgs::ImagePtr &img_msg_result) {
  cv::Mat image = cv::imread(path, -1);
  cv::cvtColor(image, image, cv_encoding);
  img_msg_result = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  img_msg_result->header.frame_id = tf_prefix + "/" + stereo_left_frame;
  img_msg_result->header.stamp = stamp;
}

size_t find_target_idx(const std::vector<ros::Time> &timestamps, const ros::Time &target_time) {
  // assumes timestamps to be ordered sequentially. Returns the index i in
  // timestamps that is just before the target_time, i.e.:
  //               timestamps[i] <= target_time <= timestamps[i+1]
  // if the target_time is before the first timestamp, an invalidargument exception is thrown.
  // if the target_time is after the last timestamp, will return the last idx
  if (target_time < timestamps[0]) {
    throw std::invalid_argument("target_time must be after earliest timestamp!");
  }
  if (target_time > timestamps[timestamps.size() - 1]) {
    // return last idx
    return timestamps.size() - 1;
  }

  for (size_t i = 0; i < timestamps.size() - 1; ++i) {
    if (timestamps[i] <= target_time && target_time <= timestamps[i + 1]) return i;
  }

  // if this error is thrown, the assumptions are wrong.
  throw std::range_error("can't find matching target index");
}

ros::Time oxford_time_to_now(const ros::Time &oxford_time) {
  ros::Time result(t0_wall + (oxford_time - t0_oxford));
  return result;
}

}  // end namespace sensor
