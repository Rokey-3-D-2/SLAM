# 단축 실행 명령어
# 순서대로 실행

localize() {
  if [ -z "$1" ]; then
    echo "Usage: ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot1 map:=$HOME/rokey_ws/maps/first_map.yaml"
    return 1
  fi
  ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot$1 map:=$HOME/rokey_ws/maps/first_map.yaml
}

rv() {
  if [ -z "$1" ]; then
    echo "Usage: ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot1"
    return 1
  fi
  ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot$1
}

nav() {
  if [ -z "$1" ]; then
    echo "Usage: ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1"
    return 1
  fi
  ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot$1
}

rqt_tf() {
  ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
}

tb_launch() {
  ros2 launch rokey_pjt tb4_combined.launch.py
}
