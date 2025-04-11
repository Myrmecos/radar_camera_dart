
# general setup
1. before running a program, do: `source install/setup.bash`
2. to compile the camera file, do: `colcon build --event-handlers console_cohesion+`
3. to run the camera_launch.cpp, do: `ros2 launch hikrobot_camera camera_launch.py` or `ros2 run hikrobot_camera hikrobot_camera`

# Camera calibration
1. run: `ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.029 image:=/hikrobot_image`

# listen to images
1. first publish image. use `ros2 run hikrobot_camera image_pub` to run image publisher
2. python build `colcon build --packages-select img_sub`
3. python run  `ros2 run img_sub listener`