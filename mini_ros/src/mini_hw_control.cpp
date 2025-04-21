#include "../include/hardware_interface.h"
#include "std_msgs/Float32.h"

void timerCallback(robotis_mini_hw::HardwareInterface &hardware_interface,
                   controller_manager::ControllerManager &cm,
                   ros::Time &last_time)
{
  ros::Time curr_time = ros::Time::now();
  ros::Duration elapsed_time = curr_time - last_time;
  last_time = curr_time;

  hardware_interface.read();
  cm.update(ros::Time::now(), elapsed_time);
  hardware_interface.write();

}

int main(int argc, char **argv)
{
  // init
  ros::init(argc, argv, "mini_hw_control");
  ros::NodeHandle node_handle("");
  ros::NodeHandle priv_node_handle("~");
  
  robotis_mini_hw::HardwareInterface hardware_interface(node_handle, priv_node_handle);
  controller_manager::ControllerManager cm(&hardware_interface, node_handle);

  // update
  ros::CallbackQueue queue;
  ros::AsyncSpinner spinner(1, &queue);
  spinner.start();
  ros::Time last_time = ros::Time::now();
  ros::TimerOptions timer_options(
    ros::Duration(0.010), // 10ms
    boost::bind(timerCallback, boost::ref(hardware_interface), 
                               boost::ref(cm), 
                               boost::ref(last_time)),
    &queue);
  ros::Timer timer = node_handle.createTimer(timer_options);
  ros::spin(); // loop rate is ___hz
  return 0;
}
