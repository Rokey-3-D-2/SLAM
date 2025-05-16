import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
import time

INIT_POSE = [0.2509, 0.7195, 270.0]

GOAL_POSES_BOT = [
    # start to patrol
    [0.0350, 0.6523, 180.0],    # map_bot1
    [-0.0619, -1.6371, 180.0],  # map_bot2
    [0.4564, -0.8648, 270.0],   # map_bot3
    [0.8353, -1.6557, 0.0],     # map_bot4
    [1.0313, 0.6398, 0.0],      # map_bot5

    # return to docking station
    [0.4564, -0.8648, 180.0],   # map_bot3
    [0.0350, 0.6523, 0.0],      # map_bot1 
]

INIT_LOADING_TIME = 2.0

def create_pose(x, y, yaw_deg, navigator):
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

    dock_navigator = TurtleBot4Navigator()
    nav_navigator = BasicNavigator()

    # 초기 pose 설정
    initial_pose = create_pose(INIT_POSE, nav_navigator)
    nav_navigator.setInitialPose(initial_pose)
    nav_navigator.get_logger().info(f'초기 위치 설정 중... {int(INIT_LOADING_TIME)}s')
    time.sleep(INIT_LOADING_TIME) #AMCL이 초기 pose 처리 시 필요한 시간과 TF를 얻을 수 있게 됨

    nav_navigator.waitUntilNav2Active()

    if dock_navigator.getDockedStatus():
        dock_navigator.get_logger().info('도킹 상태 → 언도킹')
        dock_navigator.undock()

    # 목적지 경유지 3개 (한 번에 전달)
    # goal_poses = [
    #     create_pose(-0.02, -1.39, 0.0, nav_navigator),
    #     create_pose(-1.65, -1.10, 90.0, nav_navigator),
    #     create_pose(-2.77, -1.29, 180.0, nav_navigator),
    # ]
    goal_poses = [create_pose(GOAL_POSES_BOT[i], nav_navigator) for i in range(len(GOAL_POSES_BOT))]

    # 한 번에 경로 전송
    nav_navigator.goThroughPoses(goal_poses)

    while not nav_navigator.isTaskComplete():
        feedback = nav_navigator.getFeedback()
        if feedback:
            nav_navigator.get_logger().info(f'경유지 이동 중, 남은 거리: {feedback.distance_remaining:.2f} m')

    result = nav_navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        nav_navigator.get_logger().info('모든 경유지 도달 완료')
        dock_navigator.dock()
    else:
        nav_navigator.get_logger().warn(f'실패 코드: {result}')

    # 각 포즈로 순차 이동하며 1초 정지
    for i, goal in enumerate(goal_poses):
        nav_navigator.goToPose(goal)

        while not nav_navigator.isTaskComplete():
            feedback = nav_navigator.getFeedback()
            if feedback:
                nav_navigator.get_logger().info(
                    f'{i+1}번째 경유지 이동 중, 남은 거리: {feedback.distance_remaining:.2f} m'
                )

        result = nav_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            nav_navigator.get_logger().info(f'{i+1}번째 경유지 도달. 1초 정지...')
            time.sleep(1.0)
        else:
            nav_navigator.get_logger().warn(f'{i+1}번째 경유지 실패. 코드: {result}')
            break  # 실패 시 중단

    # 도킹 시도
    dock_navigator.get_logger().info('경로 완료. 도킹 시도...')
    dock_navigator.dock()

    dock_navigator.destroy_node()
    nav_navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
