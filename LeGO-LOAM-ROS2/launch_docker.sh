xhost + 

docker run --init -it --rm \
  --name lego-loam-ros2-container \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ./codebase:/root/dev_ws/src/LeGO-LOAM-ROS2 \
  -v /mnt/rysong/slam_racecar/LeGO-LOAM-ROS2/ros2bags:/root/dev_ws/ros2bags \
  -v /mnt/rysong/slam_racecar/rosbag2_2024_01_06-12_52_49:/root/dev_ws/racecar_ros2bags_slow \
  -v /mnt/rysong/slam_racecar/rosbag2_2024_10_11-15_46_38_filtered:/root/dev_ws/racecar_ros2bags_fast \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=/root/.Xauthority \
  -e QT_X11_NO_MITSHM=1 \
  --net host \
  --privileged \
  lego-loam-ros2:galactic bash
