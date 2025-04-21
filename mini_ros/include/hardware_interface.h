#ifndef ROBOTIS_MINI_HARDWARE_INTERFACE_H
#define ROBOTIS_MINI_HARDWARE_INTERFACE_H

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

// Control table address
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 37

#define ADDR_P_GAIN 29
#define ADDR_I_GAIN 28
#define ADDR_D_GAIN 27
#define ADDR_MOVING_SPEED 32

#define P_GAIN_VAL 15
#define I_GAIN_VAL 15
#define D_GAIN_VAL 0
#define PRESENT_VOLT_VAL 50 
#define RETURN_DELAY_VAL 50

#define ADDR_320_VOLT 45
#define ADDR_320_ReturnDelay 5
#define LEN_GOAL_POSITION 4
#define LEN_PRESENT_POSITION 4

// Protocol version
#define PROTOCOL_VERSION 2.0 // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define JOINT_SIZE 16
#define BAUDRATE 1000000           // Default Baudrate of DYNAMIXEL XL 320
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

#define PI 3.1412
#define MILLION 1000000L

typedef struct _Joint
{
  double position;
  double velocity;
  double effort;
  double position_command;
} Joint;

namespace robotis_mini_hw
{
  class HardwareInterface : public hardware_interface::RobotHW
  {
  public:
    HardwareInterface(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~HardwareInterface() {}

    void read();
    void write();

  private:
    void registerControlInterfaces();
    bool initDynamixels(void);
    bool initJoints(void);

    int32_t convertRadian2Value(double);
    double convertValue2Radian(int32_t);

    // ROS NodeHandle
    ros::NodeHandle node_handle_;
    ros::NodeHandle priv_node_handle_;

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    dynamixel::GroupSyncRead *groupSyncRead;
    dynamixel::GroupSyncWrite *groupSyncWrite;

    // ROS Parameters
    std::string port_name_;
    int32_t baud_rate_;

    std::vector<Joint> joints_;
    std::string joint_names[16] = {"r_shoulder_joint", "l_shoulder_joint", "r_biceps_joint","l_biceps_joint",
                                    "r_elbow_joint", "l_elbow_joint", "r_hip_joint", "l_hip_joint",
                                    "r_thigh_joint", "l_thigh_joint", "r_knee_joint", "l_knee_joint",
                                    "r_ankle_joint", "l_ankle_joint", "r_foot_joint", "l_foot_joint"};

    int DXL_ID[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
  };

} // namespace robotis_mini_hw
#endif // ROBOTIS_MINI_HARDWARE_INTERFACE_H
