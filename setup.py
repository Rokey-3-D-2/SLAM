from setuptools import find_packages, setup

package_name = "rokey_pjt"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lhj",
    maintainer_email="hojun7889@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "capture_image = rokey_pjt.1_tb4_capture_image:main",
            "cont_cap_image = rokey_pjt.2_tb4_cont_capture_image:main",
            "det_obj = rokey_pjt.3_tb4_yolov8_obj_det:main",
            "det_obj_thread = rokey_pjt.4_tb4_yolov8_obj_det_thread:main",
            "det_obj_track = rokey_pjt.5_tb4_yolov8_obj_det_track:main",
            "depth_checker = rokey_pjt.depth_checker:main",
            "depth_checker_click = rokey_pjt.depth_checker_mouse_click:main",
            "yolo_depth_checker = rokey_pjt.3_tb4_yolo_bbox_depth_checker:main",
            "tf_trans = rokey_pjt.tf_point_transform:main",
            "object_xyz_marker = rokey_pjt.yolo_depth_tf_rviz:main",
            "image_viewer = rokey_pjt.image_viewer:main",
            "subscribe_err = rokey_pjt.subscribe_err:main",
            "publish_err = rokey_pjt.publish_err:main",
        ],
    },
)
