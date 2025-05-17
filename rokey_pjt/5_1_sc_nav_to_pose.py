import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
import time

# pose
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

# constant
INIT_LOADING_TIME = 5.0
WAITING_FOR_DETECTION = 2.0

def create_pose(pose, navigator: BasicNavigator) -> PoseStamped:
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

def get_init_pose(navigator: BasicNavigator) -> None:
    initial_pose = create_pose(INIT_POSE, navigator)
    navigator.setInitialPose(initial_pose)
    navigator.get_logger().info(f'초기 위치 설정 중... {int(INIT_LOADING_TIME)}s')
    time.sleep(INIT_LOADING_TIME) #AMCL이 초기 pose 처리 시 필요한 시간과 TF를 얻을 수 있게 됨
    navigator.waitUntilNav2Active()

def undock(navigator: TurtleBot4Navigator) -> None:
    if navigator.getDockedStatus():
        navigator.get_logger().info('현재 도킹 상태 → 언도킹 시도')
        navigator.undock()
    else:
        navigator.get_logger().info('언도킹 상태에서 시작')

def get_goal_poses(navigator: BasicNavigator) -> list:
    return [create_pose(GOAL_POSES_BOT[i], navigator) for i in range(len(GOAL_POSES_BOT))]

def get_feedback(i, navigator: BasicNavigator):
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            if i > 0:
                navigator.get_logger().info(f'{i+1}번째 경유지 이동 중, 남은 거리: {feedback.distance_remaining:.2f} m')
            else:
                navigator.get_logger().info(f'[복귀] 남은 거리: {feedback.distance_remaining:.2f} m')

def move(goal_poses, navigator: BasicNavigator) -> None:
    for i, goal in enumerate(goal_poses):
        navigator.goToPose(goal)

        get_feedback(i, navigator)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            if i < 5:
                navigator.get_logger().info(f'{i+1}번째 경유지 도달. {WAITING_FOR_DETECTION}초 정지...')
                time.sleep(WAITING_FOR_DETECTION)
            else:
                navigator.get_logger().info(f'{i+1}번째 경유지 도달. 복귀 중')
        else:
            raise RuntimeError(f'{i+1}번째 경유지 이동 실패. 상태: {result}')

def recovery(e: RuntimeError, navigator: BasicNavigator) -> None:
    navigator.get_logger().error(f'오류 발생: {e} → 1번 위치로 재이동 시도')

    # 1번 위치 재이동 시도
    recovery_pose = create_pose(*GOAL_POSES_BOT[0], navigator)
    navigator.goToPose(recovery_pose)

    get_feedback(0, navigator)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('복구 위치 도달 성공')
    else:
        navigator.get_logger().error(f'복구 위치 도달 실패. 코드: {result}')

def dock(navigator: TurtleBot4Navigator) -> None:
    navigator.get_logger().info('경로 완료. 도킹 시도...')
    navigator.dock()

def terminate(dock_navigator: TurtleBot4Navigator, nav_navigator: BasicNavigator) -> None:
    dock_navigator.destroy_node()
    nav_navigator.destroy_node()
    rclpy.shutdown()

def main():
    rclpy.init()

    # 두 navigator 인스턴스 생성
    dock_navigator = TurtleBot4Navigator()
    nav_navigator = BasicNavigator(node_name='navigator_robot4')

    # 1. 초기 위치 설정
    # initial_pose = create_pose(INIT_POSE, nav_navigator)
    # nav_navigator.setInitialPose(initial_pose)
    # nav_navigator.get_logger().info(f'초기 위치 설정 중... {int(INIT_LOADING_TIME)}s')
    # time.sleep(INIT_LOADING_TIME) #AMCL이 초기 pose 처리 시 필요한 시간과 TF를 얻을 수 있게 됨
    # nav_navigator.waitUntilNav2Active()
    get_init_pose(nav_navigator)

    # 2. 언도킹
    # if dock_navigator.getDockedStatus():
    #     dock_navigator.get_logger().info('현재 도킹 상태 → 언도킹 시도')
    #     dock_navigator.undock()
    # else:
    #     dock_navigator.get_logger().info('언도킹 상태에서 시작')
    undock(dock_navigator)

    # 3. 목표 위치 및 이동 명령
    # goal_poses = [create_pose(GOAL_POSES_BOT[i], nav_navigator) for i in range(len(GOAL_POSES_BOT))]
    # for i, goal in enumerate(goal_poses):
    #     nav_navigator.goToPose(goal)

    #     while not nav_navigator.isTaskComplete():
    #         feedback = nav_navigator.getFeedback()
    #         if feedback:
    #             nav_navigator.get_logger().info(f'{i+1}번째 경유지 이동 중, 남은 거리: {feedback.distance_remaining:.2f} m')

    #     result = nav_navigator.getResult()
    #     if result == TaskResult.SUCCEEDED:
    #         if i < 5:
    #             nav_navigator.get_logger().info(f'{i+1}번째 경유지 도달. {WAITING_FOR_DETECTION}초 정지...')
    #             time.sleep(WAITING_FOR_DETECTION)
    #         else:
    #             nav_navigator.get_logger().info(f'{i+1}번째 경유지 도달. 복귀 중')
    #     else:
    #         nav_navigator.get_logger().warn(f'{i+1}번째 경유지 실패. 코드: {result}')
    #         break  # 실패 시 중단
    goal_poses = get_goal_poses(nav_navigator)

    try:
        move(goal_poses, nav_navigator)
    except RuntimeError as e:
        recovery(e, nav_navigator)
    
    dock(dock_navigator)
    terminate(dock_navigator, nav_navigator)

    # goal_pose = create_pose(-0.87, -1.21, 90.0, nav_navigator)  # EAST
    # nav_navigator.goToPose(goal_pose)

    # 4. 이동 중 피드백 표시
    # while not nav_navigator.isTaskComplete():
    #     feedback = nav_navigator.getFeedback()
    #     if feedback:
    #         remaining = feedback.distance_remaining
    #         nav_navigator.get_logger().info(f'남은 거리: {remaining:.2f} m')

    # 5. 결과 확인
    # result = nav_navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     nav_navigator.get_logger().info('목표 위치 도달 성공')
    #     dock_navigator.dock()
    #     dock_navigator.get_logger().info('도킹 요청 완료')
    # elif result == TaskResult.CANCELED:
    #     nav_navigator.get_logger().warn('이동이 취소되었습니다.')
    # elif result == TaskResult.FAILED:
    #     error_code, error_msg = nav_navigator.getTaskError()
    #     nav_navigator.get_logger().error(f'이동 실패: {error_code} - {error_msg}')
    # else:
    #     nav_navigator.get_logger().warn('알 수 없는 결과 코드 수신')

    # 도킹 시도
    # dock_navigator.get_logger().info('경로 완료. 도킹 시도...')
    # dock_navigator.dock()

    # 6. 종료
    # dock_navigator.destroy_node()
    # nav_navigator.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
