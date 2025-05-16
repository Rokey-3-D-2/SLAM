from setuptools import find_packages, setup

package_name = 'rokey_pjt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='choin',
    maintainer_email='choin22222@naver.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_depth_checker = rokey_pjt.3_tb4_yolo_bbox_depth_checker:main',
            'tf_trans = rokey_pjt.tf_point_transform:main',
            'object_xyz_marker = rokey_pjt.yolo_depth_tf_rviz:main',
            'nav_to_pose = rokey_pjt.5nav_to_pose:main',
            'nav_through_poses = rokey_pjt.nav_through_poses:main',
            'follow_waypoints = rokey_pjt.follow_waypoints:main'
        ],
    },
)
