#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
import time

GOAL_POSES_BOT = [
    [0.0350, 0.6523, 0.0],  # map_bot1
    [-0.0619, -1.6371, 0.0],  # map_bot2
    [0.4564, -0.8648, 0.0],  # map_bot3
    [0.8353, -1.6557, 0.0],  # map_bot4
    [1.0313, 0.6398, 0.0],   # map_bot5
    # return to docking station
    [0.4564, -0.8648, 0.0],  # map_bot3
    [0.0350, 0.6523, 0.0],  # map_bot1 
]

def create_pose(pose:list, navigator):
    x, y, yaw_deg = pose
    """x, y, yaw(도 단위) → PoseStamped 생성"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y

    yaw_rad = yaw_deg * 3.141592 / 180.0
    q = quaternion_from_euler(0, 0, yaw_rad)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


def main():
    rclpy.init()

    # 도킹 및 경로 이동용 Navigator
    dock_navigator = TurtleBot4Navigator()
    nav_navigator = BasicNavigator()

    # 1. 초기 위치 설정
    initial_pose = create_pose([0.2509, 0.7195, 270.0], nav_navigator)
    nav_navigator.setInitialPose(initial_pose)
    nav_navigator.get_logger().info(f'초기 위치 설정 중...')
    time.sleep(1.0) #AMCL이 초기 pose 처리 시 필요한 시간과 TF를 얻을 수 있게 됨
    
    nav_navigator.waitUntilNav2Active()

    # 2. 도킹 상태면 언도킹
    if dock_navigator.getDockedStatus():
        dock_navigator.get_logger().info('도킹 상태 → 언도킹')
        dock_navigator.undock()

    # 3. 개별 Pose 생성 (경유지 명시)
    # waypoints = [
    #     create_pose(-0.02, -1.39, 0.0, nav_navigator),
    #     create_pose(-2.77, -1.29, 180.0, nav_navigator),
    #     create_pose(-0.30, -0.04, -90.0, nav_navigator)
    # ]
    waypoints = [create_pose(GOAL_POSES_BOT[i], nav_navigator) for i in range(len(GOAL_POSES_BOT))]

    # 4. Waypoint 경로 이동 시작
    nav_start = nav_navigator.get_clock().now()
    nav_navigator.followWaypoints(waypoints)

    # 5. 이동 중 피드백 확인
    while not nav_navigator.isTaskComplete():
        feedback = nav_navigator.getFeedback()
        if feedback:
            elapsed = nav_navigator.get_clock().now() - nav_start
            nav_navigator.get_logger().info(
                f'현재 waypoint: {feedback.current_waypoint + 1}/{len(waypoints)}, '
                f'경과 시간: {elapsed.nanoseconds / 1e9:.1f}초'
            )

    # 6. 도달한 waypoint 인덱스 확인
    result_index = nav_navigator.getResult()
    nav_navigator.get_logger().info(f'Waypoint {result_index} 까지 도달 완료')

    # 7. 자동 도킹 요청
    dock_navigator.dock()
    dock_navigator.get_logger().info('도킹 요청 완료')

    # 8. 종료 처리
    dock_navigator.destroy_node()
    nav_navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
