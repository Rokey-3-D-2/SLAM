cmake_minimum_required(VERSION 3.8)
project(rokey_pjt)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rokey_pjt_interfaces REQUIRED)

# 파이썬 모듈 설치
ament_python_install_package(${PROJECT_NAME})

# # 인터페이스 메시지를 이 패키지 타겟에 연결
# rosidl_target_interfaces(${PROJECT_NAME}
#   rokey_pjt_interfaces__rosidl_typesupport_cpp
# )

ament_package()
