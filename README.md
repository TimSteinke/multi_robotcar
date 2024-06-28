# ROS Radar Robotcar Player
This is a ROS node (targeting ROS noetic) for playing the Lidar, Camera, and GPS/INS modalities of the [Oxford Radar RobotCar Dataset](https://oxford-robotics-institute.github.io/radar-robotcar-dataset/) in real-time.

**Note:** This is a rewrite of [radar-robotcar-dataset-ros](https://github.com/Rongxi-Zhang/radar-robotcar-dataset-ros/tree/ros1) by Rongxi Zhang. The original license ([GPL v3](./LICENSE)) has been kept.

**Changes**:
- No saving to rosbag
- No radar support

- Sensor streams play **synchronously** by coupling playback to real-time
- Static TF publishing via **URDF**
- Delayed playback and 'clipping' feature to start in the middle of a sequence (see parameters in launch file)
- Can play **multiple sequences** in parallel to simulate a multi-agent setup
- Some refactoring and renaming
- Uses ROS logging interface

## Dataset
Obtain the data and other resources from here (requires permission):
- [Oxford Radar RobotCar Dataset](https://oxford-robotics-institute.github.io/radar-robotcar-dataset/): Download for any dataset you want: files for mono cameras, stereo camera, lidar left and right (processed point clouds in .bin format), and GPS/INS. Extract to the same folder, then set the path in launch file.
- [Base Robotcar SDK](https://github.com/ori-mrg/robotcar-dataset-sdk): Important for getting camera calibrations, no code is used. Clone and set path in launch file.
- [Radar Robotcar SDK Extension](https://github.com/oxford-robotics-institute/radar-robotcar-dataset-sdk): Not required, but includes a downloader.

### Sensor Suite
<div align=center>
<img src = pictures/radar-robotcar.png width="450" height="450" />
</div>

Only the following sensors are played (no 2D lidars, no radar):

- **Cameras:**
1 x Point Grey Bumblebee XB3
3 x Point Grey Grasshopper2

- **GPS/INS:**
1 x NovAtel SPAN-CPT ALIGN inertial and GPS navigation system

- **3D LiDAR:**
2 x Velodyne HDL-32E - Mounted to the left and right of the Navtech CTS350-X radar.

## Building and running
0. Download the [dataset and SDK](#dataset)
1. Place this ROS package in the `catkin_ws/src` directory of your catkin workspace.
2. Install dependencies manually (See CMakeLists.txt and package.xml) or via rosdep:
    `rosdep install --from-paths -i -y catkin_ws/src`
3. Build the workspace:
    `catkin build -DCMAKE_BUILD_TYPE=Release` or `catkin build -DCMAKE_BUILD_TYPE=Debug`
4. Make sure to source your `catkin_ws/devel/setup.bash` again.
5. Adjust the settings in `launch/radar_robotcar_player.launch` and run:
    `roslaunch radar_robotcar_player radar_robotcar_player.launch`

## Rviz
The included Rviz config displays all six cameras and the Lidar:
`rviz -d ./launch/rviz/radar_robotcar_player.rviz`

## Published Topics
**Frames:**
The frames are configured in `src/radar_robotcar_player.cpp` and `launch/urdf/robotcar.xml`:
``` c++
const std::string mono_left_frame = "mono_left",
                  mono_right_frame = "mono_right",
                  mono_rear_frame = "mono_rear",
                  stereo_left_frame = "stereo_left",
                  stereo_centre_frame = "stereo_centre",
                  stereo_right_frame = "stereo_right",
                  lidar_left_frame = "velodyne_left",
                  lidar_right_frame = "velodyne_right",
                  gps_frame = "gps_ins";
```
They will be prefixed by the `agent_name`, e.g. `robotcar_0/mono_left`. The launch file will start an URDF publisher for each robotcar instance.

**Topics:**
``` bash
# GPS/INS
gps/gps
ins/gps
ins/imu

# Lidar
lidar/left
lidar/right

# Mono info
mono/info/left
mono/info/rear
mono/info/right

# Mono images
mono/left
mono/rear
mono/right

# Stereo info
stereo/info/narrow/left
stereo/info/narrow/right
stereo/info/wide/left
stereo/info/wide/right

# Stereo images
stereo/centre
stereo/left
stereo/right
```
