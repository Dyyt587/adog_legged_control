ros2 daemon start

colcon build --packages-up-to ocs2_quadruped_controller

source install/setup.bash 
ros2 launch ocs2_quadruped_controller mujoco.launch.py pkg_description:=go2_description
