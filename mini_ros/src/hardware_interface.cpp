#include "../include/hardware_interface.h"

namespace robotis_mini_hw
{
    HardwareInterface::HardwareInterface(ros::NodeHandle nh, ros::NodeHandle private_nh)
        : node_handle_(nh),
          priv_node_handle_(private_nh)
    {
        /************************************************************
         ** Initialize ROS parameters
         ************************************************************/
        port_name_ = priv_node_handle_.param<std::string>("usb_port", "/dev/ttyUSB0");
        baud_rate_ = priv_node_handle_.param<int32_t>("baud_rate", 1000000);

        initDynamixels();
        registerControlInterfaces();
        initJoints();
    }

    bool HardwareInterface::initDynamixels(void)
    {
        portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        groupSyncRead = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

        if (!portHandler->openPort())
        {
            ROS_ERROR("Failed to open the port!");
        }

        if (!portHandler->setBaudRate(baud_rate_))
        {
            ROS_ERROR("Failed to set the baudrate!");
        }

        uint8_t dxl_error = 0;  
        for (int i = 1; i < 17; i++)
        {
            packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, 0, &dxl_error);
            packetHandler->write1ByteTxRx(portHandler, i, ADDR_320_ReturnDelay, RETURN_DELAY_VAL, &dxl_error);
            // packetHandler->write1ByteTxRx(portHandler, i, ADDR_320_VOLT, PRESENT_VOLT_VAL, &dxl_error);
            // packetHandler->write1ByteTxRx(portHandler, i, ADDR_P_GAIN, P_GAIN_VAL, &dxl_error);
            // packetHandler->write1ByteTxRx(portHandler, i, ADDR_I_GAIN, I_GAIN_VAL, &dxl_error);
            // packetHandler->write1ByteTxRx(portHandler, i, ADDR_D_GAIN, D_GAIN_VAL, &dxl_error);
            packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        }
        return true;
    }

    void HardwareInterface::registerControlInterfaces()
    {

        joints_.resize(JOINT_SIZE);

        for (int i = 0; i < JOINT_SIZE; i++)
        {
            // initialize joint vector
            Joint joint;
            joints_[i] = joint;

            // connect and register the joint state interface
            hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                                    &joints_[i].position,
                                                                    &joints_[i].velocity,
                                                                    &joints_[i].effort);
            joint_state_interface_.registerHandle(joint_state_handle);

            // connect and register the joint position interface
            hardware_interface::JointHandle position_joint_handle(joint_state_handle, &joints_[i].position_command);
            position_joint_interface_.registerHandle(position_joint_handle);
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
    }

    bool HardwareInterface::initJoints()
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        double initial_pose[16] = {0,0,0,0,0,0,0,0, -0.884667221680268,0.884667221680268,1.8619286016716308,-1.8619286016716308,
                                    0.9772613799913628, -0.9772613799913628, 0.0, 0.0};                                

        for (uint8_t i = 0; i < JOINT_SIZE; i++)
        {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, 1, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i);
                return false;
            }
            joints_[(uint16_t) i].position = initial_pose[i];
            joints_[(uint16_t) i].position_command = initial_pose[i];
        }
        return true;
    }

    int32_t HardwareInterface::convertRadian2Value(double radian)
    {
        // Range of XL320 motor -150 to 150 deg
        int32_t max_position = 1023;
        int32_t min_position = 0;
        double max_radian = 2.61799;
        double min_radian = -2.61799;

        int32_t value = 0;
        int32_t zero_position = (max_position + min_position)/2;

        if (radian > 0)
        {
            value = (radian * (max_position - zero_position) / max_radian) + zero_position;
        }
        else if (radian < 0)
        {
            value = (radian * (min_position - zero_position) / min_radian) + zero_position;
        }
        else
        {
            value = zero_position;
        }

        return value;
    }

    
    double HardwareInterface::convertValue2Radian(int32_t value)
    {
        // Range of XL320 motor -150 to 150 deg
        int32_t max_position = 1023;
        int32_t min_position = 0;
        double max_radian = 2.61799;
        double min_radian = -2.61799;

        double radian = 0.0;
        int32_t zero_position = (max_position + min_position)/2;

        if (value > zero_position)
        {
            radian = (double)(value - zero_position) * max_radian / (double)(max_position - zero_position);
        }
        else if (value < zero_position)
        {
            radian = (double)(value - zero_position) * min_radian / (double)(min_position - zero_position);
        }

        return radian;
        // ((double)value - 512) / 195.3786
    }

    void HardwareInterface::read()
    {
        for (int i = 0; i < JOINT_SIZE; i++)
        {
            joints_[(uint16_t) i].position = joints_[(uint16_t) i].position_command;
        }
    }

    void HardwareInterface::write()
    {
        for (int i = 0; i < JOINT_SIZE; i++)
        {
            uint8_t param_goal_position[2];
            uint32_t position_command = (unsigned int) convertRadian2Value(joints_[(uint16_t) i].position_command);
            param_goal_position[0] = DXL_LOBYTE(position_command);
            param_goal_position[1] = DXL_HIBYTE(position_command);
            groupSyncWrite->addParam((uint8_t)(i + 1), param_goal_position);
        }
        groupSyncWrite->txPacket();
        groupSyncWrite->clearParam();
    }
} // namespace robotis_mini_hw