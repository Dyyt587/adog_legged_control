source install/setup.bash 
colcon build --packages-ignore hardware_unitree_mujoco rl_quadruped_controller  ocs2_raisim_core ocs2_mpcnet_core ocs2_ballbot_mpcnet ocs2_legged_robot_raisim ocs2_legged_robot_mpcnet pinocchio --continue-on-error
