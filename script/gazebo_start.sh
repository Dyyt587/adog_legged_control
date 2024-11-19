ros2 daemon start

colcon build --packages-up-to  unitree_go2_description 
colcon build --packages-select ocs2_quadruped_controller adog_legged_hardware_interface adog_legged_interfaces 
source install/setup.bash 
ros2 launch unitree_go2_description gazebo.launch.py
