from setuptools import find_packages, setup

package_name = 'rokey_pjt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lhj',
    maintainer_email='hojun7889@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'beep_node = rokey_pjt.0_beep_node:main',
            'det_obj = rokey_pjt.1_1_tb4_yolov8_obj_det:main',
            'det_obj_thread = rokey_pjt.1_2_tb4_yolov8_obj_det_thread:main',
            'det_obj_track = rokey_pjt.1_3_tb4_yolov8_obj_det_track:main',
            'depth_checker = rokey_pjt.2_1_tb4_depth_checker:main',
            'depth_checker_click = rokey_pjt.2_2_tb4_depth_checker_mouse_click:main',
            'yolo_depth_checker = rokey_pjt.3_tb4_yolov8_bbox_depth_checker:main',
            'tf_trans = rokey_pjt.4_1_tb4_tf_point_transform:main',
            'object_xyz_marker = rokey_pjt.4_2_tb4_yolo_depth_tf_rviz:main',
            'nav_to_pose = rokey_pjt.5_1_tb4_nav_to_pose:main',
            'nav_through_poses = rokey_pjt.5_2_tb4_nav_through_poses:main',
            'follow_waypoints = rokey_pjt.5_3_tb4_follow_waypoints:main',
            'sc_nav_to_pose = rokey_pjt.5_1_sc_nav_to_pose:main',
            'sc_nav_through_poses = rokey_pjt.5_2_sc_nav_through_poses:main',
            'sc_follow_waypoints = rokey_pjt.5_3_sc_follow_waypoints:main',
            'tf_service_server = rokey_pjt.final_tf_transform_service:main'
        ],
    },
)
