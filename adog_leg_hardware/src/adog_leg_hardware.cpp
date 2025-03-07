// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "adog_leg_hardware/adog_leg_hardware.hpp"
#include "adog_leg_hardware/usb_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <iostream>
#include <filesystem>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>


#define FL_USB
#define FR_USB
#define LL_USB
#define LR_USB

#ifdef FL_USB
  USB FL("/dev/FL_serial");
#endif
#ifdef FR_USB
  USB FR("/dev/FR_serial");
#endif
#ifdef LL_USB
  USB LL("/dev/LL_serial");
#endif
#ifdef LR_USB
  USB LR("/dev/LR_serial");
#endif

namespace adog_leg_hardware
{

  void *FL_Thread_Callback(void *arg)
  {
    // int fd=open("/home/dyyt/adog_ws/adog_leg_hardware/src/test.txt",O_RDWR);
    // write(fd,"laiyang\n",8);
    while(1){
      // static int i=0;
      // i++;
      // if(i%100==0)
      // {
      //   i=0;
      //   RCLCPP_INFO(rclcpp::get_logger("Pthread_FL"), "FL_RUN");
      // }
      #ifdef FL_USB
        FL.USB_Receive();
      #endif
      
    }

  }
  void *FR_Thread_Callback(void *arg)
  {
    while(1){
      // static int i=0;
      // i++;
      // if(i%100==0)
      // {
      //   i=0;
      //   RCLCPP_INFO(rclcpp::get_logger("Pthread_FR"), "FR_RUN");
      //   //std::cout<<"FL_RUN"<<std::endl;
      // }
      #ifdef FR_USB
        FR.USB_Receive();
      #endif
    }
  }
  void *LL_Thread_Callback(void *arg)
  {
    while(1){
    // static int i=0;
    // i++;
    // if(i%100==0)
    // {
    //   i=0;
    //   RCLCPP_INFO(rclcpp::get_logger("Pthread_LL"), "LL_RUN");
    //   //std::cout<<"FL_RUN"<<std::endl;
    // }
    //sleep(1);
      #ifdef LL_USB
        LL.USB_Receive();
      #endif
    }  
  }
  void *LR_Thread_Callback(void *arg)
  {
    int ret=0;
    while(1){
    // static int i=0;
    // i++;
    // if(i%100==0)
    // {
    //   i=0;
    //   RCLCPP_INFO(rclcpp::get_logger("Pthread_LR"), "LR_RUN");
    //   //std::cout<<"FL_RUN"<<std::endl;
    // }
      #ifdef LR_USB
        //ret=0;
        ret=LR.USB_Receive();
        // if(ret==1)
        // {
        //   RCLCPP_INFO(rclcpp::get_logger("Pthread_LR"), "LR_RUN");
        // }

      #endif
    }
  }
hardware_interface::CallbackReturn LegSystemHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    joint_torque_command_.assign(12, 0);
    joint_position_command_.assign(12, 0);
    joint_velocities_command_.assign(12, 0);
    joint_kp_command_.assign(12, 0);
    joint_kd_command_.assign(12, 0);

    joint_position_.assign(12, 0);
    joint_velocities_.assign(12, 0);
    joint_effort_.assign(12, 0);

    // for(std::size_t i=0;i<4;i++)
    // {
    //   joint_position_command_[i*3+1]=0.72;
    //   joint_position_command_[i*3+2]=-1.44;
    //   joint_position_[i*3+1]=0.72;
    //   joint_position_[i*3+2]=-1.44;
    // }
    // joint_position_command_[0]=-0.2;
    // joint_position_command_[3]=0.2;
    // joint_position_command_[6]=-0.2;
    // joint_position_command_[9]=0.2;
    // joint_position_[0]=-0.2;
    // joint_position_[3]=0.2;
    // joint_position_[6]=-0.2;
    // joint_position_[9]=0.2;


    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // RRBotSystemPositionOnly has exactly one state and command interface on each joint
      if (joint.command_interfaces.size() < 5)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("LegSystemHardware"),
            "Joint '%s' has %zu command interfaces found. 5 expected at lest.", joint.name.c_str(),
            joint.command_interfaces.size());
        // return hardware_interface::CallbackReturn::ERROR;
      }
      std::string cmd_name[] = {
          hardware_interface::HW_IF_EFFORT,
          hardware_interface::HW_IF_VELOCITY,
          hardware_interface::HW_IF_POSITION,
          "kp",
          "kd"};
      for (unsigned int i = 0; i < joint.command_interfaces.size(); ++i)
      {
        if (joint.command_interfaces[i].name != cmd_name[i])
        {
          RCLCPP_WARN(
              rclcpp::get_logger("LegSystemHardware"),
              "Joint '%s' have %s command interfaces found(the %ds command). '%s' expected.", joint.name.c_str(),
              joint.command_interfaces[0].name.c_str(), i + 1, cmd_name[i].c_str());
          // return hardware_interface::CallbackReturn::ERROR;
        }
      }

      if (joint.state_interfaces.size() != 3)
      {
        RCLCPP_WARN(
            rclcpp::get_logger("LegSystemHardware"),
            "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        // return hardware_interface::CallbackReturn::ERROR;
      }
      std::string state_name[] = {

          hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY,
          hardware_interface::HW_IF_EFFORT

      };
      for (unsigned int i = 0; i < joint.state_interfaces.size(); ++i)
      {
        if (joint.state_interfaces[i].name != state_name[i])
        {
          RCLCPP_WARN(
              rclcpp::get_logger("LegSystemHardware"),
              "Joint '%s' have %s state interfaces found(the %ds state). '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[0].name.c_str(), i + 1, state_name[i].c_str());
          // return hardware_interface::CallbackReturn::ERROR;
        }
      }
      for (const auto &interface : joint.state_interfaces)
      {
        joint_state_interfaces[interface.name].push_back(joint.name);
      }
      for (const auto &interface : joint.command_interfaces)
      {
        joint_command_interfaces[interface.name].push_back(joint.name);
      }
    }
    //存储ros_control的imu硬件接口名字部分
    imu_name_=info_.sensors[0].name;
    for(const auto &imu_state_interfaces : info_.sensors[0].state_interfaces)
    {
       imu_state_names_.emplace_back(imu_state_interfaces.name);
    }
    //存储ros_control的foot_force硬件接口名字部分
    foot_force_name_=info_.sensors[1].name;
    for(const auto &foot_force:info_.sensors[1].state_interfaces)
    {
      foot_force_interfaces_.emplace_back(foot_force.name);
    }
    // std::cout<<foot_force_interfaces_[0]<<std::endl;
    // std::cout<<foot_force_interfaces_[3]<<std::endl;
    // std::cout<<info_.sensors[1].name<<std::endl;
    joint_torque_command_.assign(12, 0);
    joint_position_command_.assign(12, 0);
    joint_velocities_command_.assign(12, 0);
    joint_kp_command_.assign(12, 0);
    joint_kd_command_.assign(12, 0);

    joint_position_.assign(12, 0.0);
    joint_velocities_.assign(12, 0.0);
    joint_effort_.assign(12, 0.0);


    imu_states_.assign(10, 0);
    foot_force_values_.assign(4,0);

    return hardware_interface::CallbackReturn::SUCCESS;
  }


  hardware_interface::CallbackReturn LegSystemHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Configuring ...please wait...");
    static pthread_t FL_Thread;
    static pthread_t FR_Thread;
    static pthread_t LL_Thread;
    static pthread_t LR_Thread;

    int ret=0;
    ret=pthread_create(&FL_Thread,NULL,FL_Thread_Callback,NULL);
    if(ret!=0)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("LegSystemHardware"),"pthread_create fail");
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("LegSystemHardware"),"pthread_create success %ld %ld",FL_Thread,getpid());
    }
    ret=pthread_create(&FR_Thread,NULL,FR_Thread_Callback,NULL);
    if(ret!=0)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("LegSystemHardware"),"pthread_create fail");
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("LegSystemHardware"),"pthread_create success");
    }
    ret=pthread_create(&LL_Thread,NULL,LL_Thread_Callback,NULL);
    if(ret!=0)
    {
          RCLCPP_INFO(
        rclcpp::get_logger("LegSystemHardware"),"pthread_create fail");
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("LegSystemHardware"),"pthread_create success");
    }
    ret=pthread_create(&LR_Thread,NULL,LR_Thread_Callback,NULL);
    if(ret!=0)
    {
        RCLCPP_INFO(
        rclcpp::get_logger("LegSystemHardware"),"pthread_create fail");
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("LegSystemHardware"),"pthread_create success");
    }
    RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> LegSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "12");
    int ind = 0;
    for (const auto &joint_name : joint_state_interfaces["position"])
    {
      state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name : joint_state_interfaces["velocity"])
    {
      state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name : joint_state_interfaces["effort"])
    {
      state_interfaces.emplace_back(joint_name, "effort", &joint_effort_[ind++]);
    }
    ind=0;
    // std::cout<<"imu_number:"<<imu_state_names_.size()<<std::endl;
    for(const auto &imu:imu_state_names_)
    {
      state_interfaces.emplace_back(imu_name_,imu,&imu_states_[ind++]);
    }
    // std::cout<<ind<<std::endl;
    ind=0;
    for(const auto &foot:foot_force_interfaces_)
    {
      state_interfaces.emplace_back(foot_force_name_,foot,&foot_force_values_[ind++]);
    }
    ind=0;
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> LegSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "13");

    int ind = 0;
    for (const auto &joint_name : joint_command_interfaces["position"])
    {
      // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "17");

      command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name : joint_command_interfaces["velocity"])
    {
      // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "18");

      command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name : joint_command_interfaces["effort"])
    {

      // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "19");

      command_interfaces.emplace_back(joint_name, "effort", &joint_torque_command_[ind]);
      ind++;
    }
    ind = 0;
    for (const auto &joint_name : joint_command_interfaces["kp"])
    {
      // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "14");

      command_interfaces.emplace_back(joint_name, "kp", &joint_kp_command_[ind]);
      ind++;
    }
    ind = 0;
    for (const auto &joint_name : joint_command_interfaces["kd"])
    {
      // RCLCPP_FATAL(rclcpp::get_logger("LegSystemHardware"), "15");

      command_interfaces.emplace_back(joint_name, "kd", &joint_kd_command_[ind]);
      ind++;
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn LegSystemHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("LegSystemHardware"), "Activating ...please wait...");

    // for (int i = 0; i < hw_start_sec_; i++)
    // {
    //   rclcpp::sleep_for(std::chrono::seconds(1));
    //   RCLCPP_INFO(
    //       rclcpp::get_logger("LegSystemHardware"), "%.1f seconds left...",
    //       hw_start_sec_ - i);
    // }
    // // END: This part here is for exemplary purposes - Please do not copy to your production code

    // // command and state should be equal when starting
    // for (uint i = 0; i < hw_states_.size(); i++)
    // {
    //   hw_commands_[i] = hw_states_[i];
    // }

    //RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LegSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    static int i = 0;
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    if (i++ % 10000 == 0)
      RCLCPP_INFO(
          rclcpp::get_logger("LegSystemHardware"), "Deactivating ...please wait...");

    // for (int i = 0; i < hw_stop_sec_; i++)
    // {
    //   rclcpp::sleep_for(std::chrono::seconds(1));
    //   RCLCPP_INFO(
    //       rclcpp::get_logger("LegSystemHardware"), "%.1f seconds left...",
    //       hw_stop_sec_ - i);
    // }

    if (i++ % 10000 == 0)
      RCLCPP_INFO(rclcpp::get_logger("LegSystemHardware"), "Successfully deactivated!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type LegSystemHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    static int j = 0;
    j++;
    // joint_effort_=joint_torque_command_;
    // joint_position_=joint_position_command_;
    // joint_velocities_=joint_velocities_command_;
    // imu_states_={0.001,0.001,-0.091,0.996,-0,0.001,0.001,-0.008,0.036,9.776};
    // foot_force_values_={33.75,33.75,33.75,33.75};
    #ifdef FL_USB

        USB_ReceivePackageTypedef FL_receive_package=FL.USB_Get_Date();

        joint_effort_[0]= FL_receive_package.motor_ut0.tor;
        joint_position_[0] = FL_receive_package.motor_ut0.pos;
        joint_velocities_[0] =FL_receive_package.motor_ut0.speed;

        joint_effort_[1]= FL_receive_package.motor_dm.tor;
        joint_position_[1] = FL_receive_package.motor_dm.pos;
        joint_velocities_[1] =FL_receive_package.motor_dm.speed;

        joint_effort_[2]= FL_receive_package.motor_ut1.tor;
        joint_position_[2] = FL_receive_package.motor_ut1.pos;
        joint_velocities_[2] =FL_receive_package.motor_ut1.speed;


        // if(j%100==0)
        // {
        //     // RCLCPP_INFO(rclcpp::get_logger("FLDATA"), "%f,%f,%f",joint_position_[0],joint_position_[1],joint_position_[2]); 
        //     RCLCPP_INFO(rclcpp::get_logger("FLDATA"), "%f,%f,%f",joint_velocities_[0],joint_velocities_[1],joint_velocities_[2]); 
        // }

    #endif

    #ifdef FR_USB

      USB_ReceivePackageTypedef FR_receive_package=FR.USB_Get_Date();
      joint_effort_[3]= FR_receive_package.motor_ut0.tor;
      joint_position_[3] = FR_receive_package.motor_ut0.pos;
      joint_velocities_[3] =FR_receive_package.motor_ut0.speed;

      joint_effort_[4]= FR_receive_package.motor_dm.tor;
      joint_position_[4] = FR_receive_package.motor_dm.pos;
      joint_velocities_[4] =FR_receive_package.motor_dm.speed;

      joint_effort_[5]= FR_receive_package.motor_ut1.tor;
      joint_position_[5] = FR_receive_package.motor_ut1.pos;
      joint_velocities_[5] =FR_receive_package.motor_ut1.speed;


      // if(j%100==0)
      // {
      //     //RCLCPP_INFO(rclcpp::get_logger("FRDATA"), "%f,%f,%f",joint_position_[3],joint_position_[4],joint_position_[5]); 
      //     RCLCPP_INFO(rclcpp::get_logger("FRDATA"), "%f,%f,%f",joint_velocities_[3],joint_velocities_[4],joint_velocities_[5]); 
      // }

    #endif

    #ifdef LL_USB
        USB_ReceivePackageTypedef LL_receive_package=LL.USB_Get_Date();
        joint_effort_[6]= LL_receive_package.motor_ut0.tor;
        joint_position_[6] = LL_receive_package.motor_ut0.pos;
        joint_velocities_[6] =LL_receive_package.motor_ut0.speed;

        joint_effort_[7]= LL_receive_package.motor_dm.tor;
        joint_position_[7] = LL_receive_package.motor_dm.pos;
        joint_velocities_[7] =LL_receive_package.motor_dm.speed;

        joint_effort_[8]= LL_receive_package.motor_ut1.tor;
        joint_position_[8] = LL_receive_package.motor_ut1.pos;
        joint_velocities_[8] =LL_receive_package.motor_ut1.speed;

      // if(j%100==0)
      // {
      //     RCLCPP_INFO(rclcpp::get_logger("LLDATA"), "%f,%f,%f",joint_position_[6],joint_position_[7],joint_position_[8]); 
      //     RCLCPP_INFO(rclcpp::get_logger("LLDATA"), "%f,%f,%f",joint_velocities_[6],joint_velocities_[7],joint_velocities_[8]); 
      // }
    #endif

    #ifdef LR_USB

        USB_ReceivePackageTypedef LR_receive_package=LR.USB_Get_Date();
        joint_effort_[9]= LR_receive_package.motor_ut0.tor;
        joint_position_[9] = LR_receive_package.motor_ut0.pos;
        joint_velocities_[9] =LR_receive_package.motor_ut0.speed;

        joint_effort_[10]= LR_receive_package.motor_dm.tor;
        joint_position_[10] = LR_receive_package.motor_dm.pos;
        joint_velocities_[10] =LR_receive_package.motor_dm.speed;

        joint_effort_[11]= LR_receive_package.motor_ut1.tor;
        joint_position_[11] = LR_receive_package.motor_ut1.pos;
        joint_velocities_[11] =LR_receive_package.motor_ut1.speed;
      if(j%100==0)
      {
          // RCLCPP_INFO(rclcpp::get_logger("LRDATA"), "%f,%f,%f",joint_position_[9],joint_position_[10],joint_position_[11]); 
          //RCLCPP_INFO(rclcpp::get_logger("LRDATA"), "%f,%f,%f",joint_velocities_[9],joint_velocities_[10],joint_velocities_[11]); 
      }
    #endif

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type LegSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    static int j = 0;
    j++;

    #ifdef FL_USB
        USB_SendPackageTypedef FL_send_package;
        FL_send_package.motor_ut0.tor=joint_torque_command_[0];
        FL_send_package.motor_ut0.pos=joint_position_command_[0];
        FL_send_package.motor_ut0.speed=joint_velocities_command_[0];
        FL_send_package.motor_ut0.kp=joint_kp_command_[0];
        FL_send_package.motor_ut0.kd=joint_kd_command_[0];

        FL_send_package.motor_dm.tor=joint_torque_command_[1];
        FL_send_package.motor_dm.pos=joint_position_command_[1];
        FL_send_package.motor_dm.speed=joint_velocities_command_[1];
        FL_send_package.motor_dm.kp=joint_kp_command_[1];
        FL_send_package.motor_dm.kd=joint_kd_command_[1];

        FL_send_package.motor_ut1.tor=joint_torque_command_[2];
        FL_send_package.motor_ut1.pos=joint_position_command_[2];
        FL_send_package.motor_ut1.speed=joint_velocities_command_[2];
        FL_send_package.motor_ut1.kp=joint_kp_command_[2];
        FL_send_package.motor_ut1.kd=joint_kd_command_[2];

        // FL_send_package.motor_ut0.tor=-0.52;
        // FL_send_package.motor_ut0.pos=0;
        // FL_send_package.motor_ut0.speed=0;
        // FL_send_package.motor_ut0.kp=0.1;
        // FL_send_package.motor_ut0.kd=0;

        // FL_send_package.motor_dm.tor=0;
        // FL_send_package.motor_dm.pos=1;
        // FL_send_package.motor_dm.speed=0;
        // FL_send_package.motor_dm.kp=8;
        // FL_send_package.motor_dm.kd=0;

        // FL_send_package.motor_ut1.tor=0;
        // FL_send_package.motor_ut1.pos=-1.4;
        // FL_send_package.motor_ut1.speed=0;
        // FL_send_package.motor_ut1.kp=0.08;
        // FL_send_package.motor_ut1.kd=0;


        // FL_send_package.motor_ut0.tor=0;
        // FL_send_package.motor_ut0.pos=0;
        // FL_send_package.motor_ut0.speed=0;
        // FL_send_package.motor_ut0.kp=0;
        // FL_send_package.motor_ut0.kd=0;

        // FL_send_package.motor_dm.tor=0;
        // FL_send_package.motor_dm.pos=0;
        // FL_send_package.motor_dm.speed=0;
        // FL_send_package.motor_dm.kp=0;
        // FL_send_package.motor_dm.kd=0;

        // FL_send_package.motor_ut1.tor=0;
        // FL_send_package.motor_ut1.pos=0;
        // FL_send_package.motor_ut1.speed=0;
        // FL_send_package.motor_ut1.kp=0;
        // FL_send_package.motor_ut1.kd=0;

        FL.USB_Set_Date(FL_send_package);
        FL.USB_Send();
    #endif

    #ifdef FR_USB
        USB_SendPackageTypedef FR_send_package;
        FR_send_package.motor_ut0.tor=joint_torque_command_[3];
        FR_send_package.motor_ut0.pos=joint_position_command_[3];
        FR_send_package.motor_ut0.speed=joint_velocities_command_[3];
        FR_send_package.motor_ut0.kp=joint_kp_command_[3];
        FR_send_package.motor_ut0.kd=joint_kd_command_[3];

        FR_send_package.motor_dm.tor=joint_torque_command_[4];
        FR_send_package.motor_dm.pos=joint_position_command_[4];
        FR_send_package.motor_dm.speed=joint_velocities_command_[4];
        FR_send_package.motor_dm.kp=joint_kp_command_[4];
        FR_send_package.motor_dm.kd=joint_kd_command_[4];

        FR_send_package.motor_ut1.tor=joint_torque_command_[5];
        FR_send_package.motor_ut1.pos=joint_position_command_[5];
        FR_send_package.motor_ut1.speed=joint_velocities_command_[5];
        FR_send_package.motor_ut1.kp=joint_kp_command_[5];
        FR_send_package.motor_ut1.kd=joint_kd_command_[5];

        // FR_send_package.motor_ut0.tor=0.48;
        // FR_send_package.motor_ut0.pos=0;
        // FR_send_package.motor_ut0.speed=0;
        // FR_send_package.motor_ut0.kp=0.1;
        // FR_send_package.motor_ut0.kd=0;

        // FR_send_package.motor_dm.tor=0;
        // FR_send_package.motor_dm.pos=1;
        // FR_send_package.motor_dm.speed=0;
        // FR_send_package.motor_dm.kp=8;
        // FR_send_package.motor_dm.kd=0;

        // FR_send_package.motor_ut1.tor=0;
        // FR_send_package.motor_ut1.pos=-1.4;
        // FR_send_package.motor_ut1.speed=0;
        // FR_send_package.motor_ut1.kp=0.08;
        // FR_send_package.motor_ut1.kd=0;

        // FR_send_package.motor_ut0.tor=0;
        // FR_send_package.motor_ut0.pos=0;
        // FR_send_package.motor_ut0.speed=0;
        // FR_send_package.motor_ut0.kp=0;
        // FR_send_package.motor_ut0.kd=0;

        // FR_send_package.motor_dm.tor=0;
        // FR_send_package.motor_dm.pos=0;
        // FR_send_package.motor_dm.speed=0;
        // FR_send_package.motor_dm.kp=0;
        // FR_send_package.motor_dm.kd=0;

        // FR_send_package.motor_ut1.tor=0;
        // FR_send_package.motor_ut1.pos=0;
        // FR_send_package.motor_ut1.speed=0;
        // FR_send_package.motor_ut1.kp=0;
        // FR_send_package.motor_ut1.kd=0;

        FR.USB_Set_Date(FR_send_package);
        FR.USB_Send();
    #endif

    #ifdef LL_USB
      USB_SendPackageTypedef LL_send_package;
      LL_send_package.motor_ut0.tor=joint_torque_command_[6];
      LL_send_package.motor_ut0.pos=joint_position_command_[6];
      LL_send_package.motor_ut0.speed=joint_velocities_command_[6];
      LL_send_package.motor_ut0.kp=joint_kp_command_[6];
      LL_send_package.motor_ut0.kd=joint_kd_command_[6];

      LL_send_package.motor_dm.tor=joint_torque_command_[7];
      LL_send_package.motor_dm.pos=joint_position_command_[7];
      LL_send_package.motor_dm.speed=joint_velocities_command_[7];
      LL_send_package.motor_dm.kp=joint_kp_command_[7];
      LL_send_package.motor_dm.kd=joint_kd_command_[7];

      LL_send_package.motor_ut1.tor=joint_torque_command_[8];
      LL_send_package.motor_ut1.pos=joint_position_command_[8];
      LL_send_package.motor_ut1.speed=joint_velocities_command_[8];
      LL_send_package.motor_ut1.kp=joint_kp_command_[8];
      LL_send_package.motor_ut1.kd=joint_kd_command_[8];

      // LL_send_package.motor_ut0.tor=-0.48;
      // LL_send_package.motor_ut0.pos=0;
      // LL_send_package.motor_ut0.speed=0;
      // LL_send_package.motor_ut0.kp=0.1;
      // LL_send_package.motor_ut0.kd=0;

      // LL_send_package.motor_dm.tor=0;
      // LL_send_package.motor_dm.pos=1;
      // LL_send_package.motor_dm.speed=0;
      // LL_send_package.motor_dm.kp=8;
      // LL_send_package.motor_dm.kd=0;

      // LL_send_package.motor_ut1.tor=0;
      // LL_send_package.motor_ut1.pos=-1.4;
      // LL_send_package.motor_ut1.speed=0;
      // LL_send_package.motor_ut1.kp=0.08;
      // LL_send_package.motor_ut1.kd=0;

      // LL_send_package.motor_ut0.tor=0;
      // LL_send_package.motor_ut0.pos=0;
      // LL_send_package.motor_ut0.speed=0;
      // LL_send_package.motor_ut0.kp=0;
      // LL_send_package.motor_ut0.kd=0;

      // LL_send_package.motor_dm.tor=0;
      // LL_send_package.motor_dm.pos=0;
      // LL_send_package.motor_dm.speed=0;
      // LL_send_package.motor_dm.kp=0;
      // LL_send_package.motor_dm.kd=0;

      // LL_send_package.motor_ut1.tor=0;
      // LL_send_package.motor_ut1.pos=0;
      // LL_send_package.motor_ut1.speed=0;
      // LL_send_package.motor_ut1.kp=0;
      // LL_send_package.motor_ut1.kd=0;

      LL.USB_Set_Date(LL_send_package);
      LL.USB_Send();
    #endif
    #ifdef LR_USB
        USB_SendPackageTypedef LR_send_package;
        LR_send_package.motor_ut0.tor=joint_torque_command_[9];
        LR_send_package.motor_ut0.pos=joint_position_command_[9];
        LR_send_package.motor_ut0.speed=joint_velocities_command_[9];
        LR_send_package.motor_ut0.kp=joint_kp_command_[9];
        LR_send_package.motor_ut0.kd=joint_kd_command_[9];

        LR_send_package.motor_dm.tor=joint_torque_command_[10];
        LR_send_package.motor_dm.pos=joint_position_command_[10];
        LR_send_package.motor_dm.speed=joint_velocities_command_[10];
        LR_send_package.motor_dm.kp=joint_kp_command_[10];
        LR_send_package.motor_dm.kd=joint_kd_command_[10];

        LR_send_package.motor_ut1.tor=joint_torque_command_[11];
        LR_send_package.motor_ut1.pos=joint_position_command_[11];
        LR_send_package.motor_ut1.speed=joint_velocities_command_[11];
        LR_send_package.motor_ut1.kp=joint_kp_command_[11];
        LR_send_package.motor_ut1.kd=joint_kd_command_[11];

        // LR_send_package.motor_ut0.tor=0.52;
        // LR_send_package.motor_ut0.pos=0;
        // LR_send_package.motor_ut0.speed=0;
        // LR_send_package.motor_ut0.kp=0.1;
        // LR_send_package.motor_ut0.kd=0;

        // LR_send_package.motor_dm.tor=0;
        // LR_send_package.motor_dm.pos=1;
        // LR_send_package.motor_dm.speed=0;
        // LR_send_package.motor_dm.kp=8;
        // LR_send_package.motor_dm.kd=0;

        // LR_send_package.motor_ut1.tor=0;
        // LR_send_package.motor_ut1.pos=-1.4;
        // LR_send_package.motor_ut1.speed=0;
        // LR_send_package.motor_ut1.kp=0.08;
        // LR_send_package.motor_ut1.kd=0;

        // LR_send_package.motor_ut0.tor=0;
        // LR_send_package.motor_ut0.pos=0;
        // LR_send_package.motor_ut0.speed=0;
        // LR_send_package.motor_ut0.kp=0;
        // LR_send_package.motor_ut0.kd=0;

        // LR_send_package.motor_dm.tor=0;
        // LR_send_package.motor_dm.pos=0;
        // LR_send_package.motor_dm.speed=0;
        // LR_send_package.motor_dm.kp=0;
        // LR_send_package.motor_dm.kd=0;

        // LR_send_package.motor_ut1.tor=0;
        // LR_send_package.motor_ut1.pos=0;
        // LR_send_package.motor_ut1.speed=0;
        // LR_send_package.motor_ut1.kp=0;
        // LR_send_package.motor_ut1.kd=0;

        LR.USB_Set_Date(LR_send_package);
        LR.USB_Send();
    #endif
    return hardware_interface::return_type::OK;
  }

} // namespace ros2_control_demo_example_12

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    adog_leg_hardware::LegSystemHardware,
    hardware_interface::SystemInterface)