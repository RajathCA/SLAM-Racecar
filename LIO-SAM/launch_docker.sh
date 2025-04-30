  xhost +
  docker run --init -it --rm \
  --name liosam-humble-jammy-container \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /mnt/rysong/slam_racecar/LIO-SAM:/root/ros2_ws/src/LIO-SAM \
  -v /mnt/rysong/slam_racecar/PrepData:/root/ros2_ws/src/PrepData \
  -v /mnt/rysong/slam_racecar/get_lio_sam_traj:/root/ros2_ws/src/get_lio_sam_traj \
  -v /mnt/rysong/slam_racecar/rosbag2_2024_01_06-12_52_49:/root/ros2_ws/src/LIO-SAM/racecar_rosbags \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=/root/.Xauthority \
  --net host \
  --privileged \
  -e QT_X11_NO_MITSHM=1 \
  liosam-humble-jammy \
  bash

# --runtime=nvidia --gpus all \
