xhost + 

docker run --init -it --rm \
  --name lego-loam-ros2-container \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /mnt/rysong/slam_racecar/LeGO-LOAM-ROS2/ros2bags:/root/dev_ws/ros2bags \
  -v /mnt/rysong/slam_racecar/rosbag2_2024_10_11-15_46_38_filtered:/root/dev_ws/racecar_ros2bags \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=/root/.Xauthority \
  --net host \
  --privileged \
  -e QT_X11_NO_MITSHM=1 \
  lego-loam-ros2:galactic bash
