# chuyun_opencv

C++ OpenCV algorithms for ROS 2 Humble.

The demo node provides:

- HSV color segmentation for red, green, and blue regions.
- Contour and polygon based shape recognition for triangles, rectangles, and circles.
- OpenCV template matching with a built-in textured template or a user supplied image.
- A deterministic synthetic image publisher so the algorithms can be tested without a camera.

## Build

    source /opt/ros/humble/setup.bash
    cd /root/chuyun_ws
    colcon build --packages-select chuyun_opencv --symlink-install
    source install/setup.bash

## Run the synthetic test

    ros2 launch chuyun_opencv opencv_demo.launch.py

Topics:

- `/opencv/image_test` - synthetic input image (`bgr8`)
- `/opencv/image_annotated` - annotated result (`bgr8`)
- `/opencv/color_mask` - combined color mask (`mono8`)
- `/opencv/detections` - JSON detection summary (`std_msgs/String`)

Verify from another shell:

    source /opt/ros/humble/setup.bash
    source /root/chuyun_ws/install/setup.bash
    ros2 topic hz /opencv/image_annotated
    ros2 topic echo /opencv/detections --once

## Process a real ROS image topic

Stop synthetic input and pass an image topic:

    ros2 run chuyun_opencv opencv_demo_node --ros-args \
      -p use_test_image:=false -p input_topic:=/camera/image_raw
