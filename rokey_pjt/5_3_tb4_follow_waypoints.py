#!/usr/bin/env python3

import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

# ======================
# 초기 설정 (파일 안에서 직접 정의)
# ======================
INITIAL_POSE_POSITION = [0.001, 0.001]
INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH

GOAL_POSES_IMG = [
    ([-0.850, 0.890], TurtleBot4Directions.WEST),       # map_img1
    ([-0.948, -1.495], TurtleBot4Directions.SOUTH),     # map_img2
    ([0.468, -2.098], TurtleBot4Directions.EAST),       # map_img3
    ([1.872, -1.534], TurtleBot4Directions.EAST),       # map_img4
    ([1.956, 0.727], TurtleBot4Directions.EAST),        # map_img5
]

GOAL_POSES_BOT = [
    ([-0.27, 0.63], TurtleBot4Directions.NORTH),        # map_bot1
    ([-0.40, -1.5], TurtleBot4Directions.NORTH),        # map_bot2
    ([0.36, -1.63], TurtleBot4Directions.WEST),         # map_bot3
    ([1.24, -1.74], TurtleBot4Directions.SOUTH),        # map_bot4
    ([1.52, 0.61], TurtleBot4Directions.SOUTH),         # map_bot5
]
# ======================

def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()

    if not navigator.getDockedStatus():
        navigator.info('Docking before initializing pose')
        navigator.dock()

    initial_pose = navigator.getPoseStamped(INITIAL_POSE_POSITION, INITIAL_POSE_DIRECTION)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    navigator.undock()

    goal_pose_msgs = [navigator.getPoseStamped(position, direction) for position, direction in GOAL_POSES_BOT]
    navigator.startFollowWaypoints(goal_pose_msgs)

    navigator.dock()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
