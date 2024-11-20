ros2 daemon start
###
 # @Author: Dyyt587 805207319@qq.com
 # @Date: 2024-11-19 23:07:50
 # @LastEditors: Dyyt587 805207319@qq.com
 # @LastEditTime: 2024-11-20 10:34:18
 # @FilePath: /adog_ws/adog_legged_control/script/rtab_lidar_start.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 

source install/setup.bash 
 colcon build --packages-select rtamp_test
ros2 launch  rtamp_test rtamap_lidar_gazebo.launch.py
