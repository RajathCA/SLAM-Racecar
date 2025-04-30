`sudo DISPLAY=$DISPLAY MY_UID=$(id -u) MY_GID=$(id -g) docker compose run orb_slam3_22_humble`


`sudo apt install ros-humble-rosbag2-storage-mcap`

```
ros2 bag info --storage mcap rosbag2_2024_10_11-15_46_38_filtered_1.mcap \
  | awk '/\/.*\/?.*/' | sort 
```
ros2 bag info --storage mcap ./slow_data/rosbag2_2024_01_06-12_52_49_0.mcap \
  | awk '/\/.*\/?.*/' | sort 


ros2 launch orb_slam3_ros2_wrapper mono_inertial.launch.py



for f in $(ls rosbag2_2024_10_11-15_46_38_filtered_*.mcap | sort -V | grep -v 'filtered_0.mcap'); do echo "==== Playing: $f ===="; ros2 bag play "$f"; done


ros2 bag play /root/rosbag2_2024_10_11-15_46_38_filtered/rosbag2_2024_10_11-15_46_38_filtered_1.mcap   
ros2 bag play /root/rosbag2_2024_10_11-15_46_38_filtered/rosbag2_2024_10_11-15_46_38_filtered_2.mcap   
ros2 bag play /root/rosbag2_2024_10_11-15_46_38_filtered/rosbag2_2024_10_11-15_46_38_filtered_3.mcap   
ros2 bag play /root/rosbag2_2024_10_11-15_46_38_filtered/rosbag2_2024_10_11-15_46_38_filtered_4.mcap   
 ros2 bag play /root/rosbag2_2024_10_11-15_46_38_filtered/rosbag2_2024_10_11-15_46_38_filtered_5.mcap   --remap /camera/image_raw:=/vimba_front/image   --remap /imu:=/vectornav/imu


ros2 bag play --start-paused --storage mcap ./slow_data/rosbag2_2024_01_06-12_52_49_0.mcap --read-ahead-queue-size 1000 \
--topics /vimba_left/image \
/vimba_left/camera_info \
/vectornav/raw/imu \
/vectornav/imu \
/vectornav/imu/uncompensated \
/vectornav/imu_compensated \
/vectornav/imu_uncompensated \
/tf \
/tf_static 


ros2 bag play --start-paused --storage mcap ./slow_data/rosbag2_2024_01_06-12_52_49_0_reordered/rosbag2_2024_01_06-12_52_49_0_reordered_0.mcap --read-ahead-queue-size 1000 \
--topics /vimba_left/image \
/vimba_left/camera_info \
/vectornav/raw/imu \
/vectornav/imu \
/vectornav/imu/uncompensated \
/vectornav/imu_compensated \
/vectornav/imu_uncompensated \
/tf \
/tf_static 


ros2 bag record /robot/map_data /robot/map_points /robot/robot_pose_slam /robot/slam_info /robot/visible_landmarks /robot/visible_landmarks_pos -o orbslam_output/slow_data/monocular --storage mcap

ros2 topic list
 ros2 topic echo /robot/robot_pose_slam


docker exec -it orb-slam3-ros2-docker-orb_slam3_22_humble-run-2ebe7fc4cb82 /bin/bash


cd /home/orb/ORB_SLAM3/ && sudo chmod +x build.sh && ./build.sh
cd /root/colcon_ws/ && colcon build --symlink-install && source install/setup.bash



cd /home/orb/ORB_SLAM3/ && sudo chmod +x build.sh && ./build.sh

cd /root/colcon_ws/ && colcon build --symlink-install && source install/setup.bash


@SORT
ros2 run my_slam_tools replay_sorted_bag.py ./slow_data/rosbag2_2024_01_06-12_52_49_0.mcap

docker exec -it orb-slam3-ros2-docker-orb_slam3_22_humble-run-799c9cdb05f1 bin/bash