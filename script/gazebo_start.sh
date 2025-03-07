ros2 daemon start

colcon build --symlink-install --packages-up-to  unitree_go2_description adog_leg_hardware
colcon build --packages-up-to ocs2_quadruped_controller leg_pd_controller
source install/setup.bash 
ros2 launch unitree_go2_description gazebo.launch.py