#include "ros/ros.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <cmath>
#include "gait_planning.h"
#include "foot_planning.h"
#include <fcntl.h>
#include <termios.h>
#include <sensor_msgs/Imu.h>
#define STDIN_FILENO 0

#define PI 3.1415
#define ESC_ASCII_VALUE 0x1b

double t = 0.0, t_elapsed = 0.0, t_loop = 0.0;
char input = 0;
int t_init = 5.0;
long t_sleep = 0;
uint64_t diff1, diff2;
long tick = 0;
struct timespec t0, t_start, t_end1, t_end2;
double x_RH0 = 0.0, y_RH0 = -174.00, z_RH0 = -12.00, x_LH0 = 0.0, y_LH0 = 174.0, z_LH0 = -12.0;
double x_RF0 = -15.0, y_RF0 = -(33.0 + 0.0), z_RF0 = -196.0, Roll_RF0 = 0.0, Pitch_RF0 = 0.0;
double x_LF0 = -15.0, y_LF0 = 33.0 + 0.0, z_LF0 = -196.0, Roll_LF0 = 0.0, Pitch_LF0 = 0.0;
double x_RH, y_RH = 0.0, z_RH = 0.0, x_LH = 0.0, y_LH = 0.0, z_LH = 0.0;
double x_RF = 0.0, y_RF = 0.0, z_RF = 0.0, Roll_RF = 0.0, Pitch_RF = -0.02, x_LF = 0.0, y_LF = 0.0, z_LF = 0.0, Roll_LF = 0.0, Pitch_LF = -0.02;
double z0 = -170.0, z_h = 0.0;
double t_step = 1.0, x_size = 0.0, y_size = 0.0;
int n_step = 0;
double ds_time = 0.5;
double imu_roll_raw = 0.0, imu_roll_ofst = 0.0, imu_pitch_raw = 0.0, ax_raw = 0.0, ay_raw = 0.0, az_raw = 0.0, ax = 0.0, ay = 0.0, az = 0.0;
double imu_pitch_old1 = 0.0, imu_pitch_old2 = 0.0, imu_roll_old1 = 0.0, imu_roll_old2 = 0.0, imu_roll_init = 0.0, imu_pitch_init = 0.0, imu_roll = 0.0, imu_pitch = 0.0;
double ax_old1 = 0.0, ay_old1 = 0.0, az_old1, ax_old2 = 0.0, ay_old2 = 0.0, az_old2 = 0.0, ay_old3 = 0.0, ay_old4 = 0.0;
double pitch_ankle_gain, roll_ankle_gain, pitch_disp_gain, roll_disp_gain, ay_gain, ax_gain, steps_per_meter;
bool use_perception, is_sim;


std::vector<double> joint_angle(16);
std::vector<double> P_CoM(3);
coder::array<double, 1U> com_x;
coder::array<double, 1U> com_y;
coder::array<double, 1U> ZMP_x_amp;
coder::array<double, 1U> ZMP_y_amp;

void IK_RH(float x, float y, float z, std::vector<float> &positions);
void IK_LH(float x, float y, float z, std::vector<float> &positions);
void IK_RF(float x, float y, float z, float th_r, float th_p, std::vector<float> &positions);
void IK_LF(float x, float y, float z, float th_r, float th_p, std::vector<float> &positions);
void COM_calc(std::vector<double> joint_angle, std::vector<double> &P_CoM);

int getch();
int kbhit(void);
char getKey();
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
void initPose(std::vector<float> &positions);
void scanPose(std::vector<float> &positions);

int main(int argc, char **argv)
{
     ros::init(argc, argv, "zmp_walking");
     ros::NodeHandle nh;
  
     ros::WallTime start_t, end_t, t0;
     ros::Rate loop_rate(200);
 
     // Step Parameters
     nh.getParam("/x_size", x_size);
     nh.getParam("/y_size", y_size);
     nh.getParam("/n_step", n_step);
     nh.getParam("/t_step", t_step);
     nh.getParam("/ds_time", ds_time);
     nh.getParam("/z_h", z_h);
     nh.getParam("/pitch_ankle_gain", pitch_ankle_gain);
     nh.getParam("/roll_ankle_gain", roll_ankle_gain);
     nh.getParam("/pitch_disp_gain", pitch_disp_gain);
     nh.getParam("/roll_disp_gain", roll_disp_gain);
     nh.getParam("/ay_gain", ay_gain);
     nh.getParam("/ax_gain", ax_gain);
     nh.getParam("/use_perception", use_perception);
     nh.getParam("/steps_per_meter", steps_per_meter);
     nh.getParam("/is_sim", is_sim);

     if(is_sim){
          x_size *= -1;
          z_h *= -1;
     }

     ros::Subscriber sub = nh.subscribe("imu/data", 1000, imu_callback);
     ros::Publisher com_pub_ = nh.advertise<visualization_msgs::Marker>("/com", 5);

     ros::Publisher joint_angle_cmd_pub_;
     if (is_sim)
          joint_angle_cmd_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/robotis_mini/sim_position_controller/command", 5);
     else
          joint_angle_cmd_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/hw_position_controller/command", 5);
     

     ros::Duration(0.5).sleep();
     std::vector<float> posis(16);
     std_msgs::Float64MultiArray joint_angle_cmd_msg;

     while (ros::ok())
     {
          if (use_perception)
          {
               scanPose(posis);          
               joint_angle_cmd_msg.data.clear();
               joint_angle_cmd_msg.data.resize(16);
               for (int i = 0; i < 16; i++)
                    joint_angle_cmd_msg.data[i] = (double)posis[i];
               joint_angle_cmd_pub_.publish(joint_angle_cmd_msg);
               ros::Duration(0.5).sleep();

               printf("Calculating number walking steps from detected ball position...\n");
               geometry_msgs::PointConstPtr detected_ball_pos =
                    ros::topic::waitForMessage<geometry_msgs::Point>("/detected_ball_pos", nh, ros::Duration(300.0));
               if (detected_ball_pos == NULL)
               { 
                    ROS_ERROR("Timed out while waiting for a message on topic detected_ball_pos!");
                    return 1;
               }

               double ball_pos_z = detected_ball_pos->z;
               double walk_dist = ball_pos_z - 0.02;
               n_step = (int) (walk_dist * steps_per_meter);
               if (n_step%2 != 0) n_step+=1;
               ROS_INFO_STREAM("Calculated num_step: " << n_step);
          }

          printf("Calculating ZMP and CoM trajectories...\n");
          gaitplanning(x_size, y_size, t_step, n_step, ds_time, com_x, com_y, ZMP_x_amp, ZMP_y_amp);

          initPose(posis);          
          joint_angle_cmd_msg.data.clear();
          joint_angle_cmd_msg.data.resize(16);
          for (int i = 0; i < 16; i++)
               joint_angle_cmd_msg.data[i] = (double)posis[i];
          joint_angle_cmd_pub_.publish(joint_angle_cmd_msg);
          ros::Duration(0.5).sleep();

          std::cout << "Press any key to start\n"
                    << std::endl;
          printf("Press any key to continue! (or press ESC to quit!)\n");
          if (getch() == ESC_ASCII_VALUE)
               break;

          t_elapsed = 0;
          t0 = ros::WallTime::now();

          while (t_elapsed < (n_step + 3) * t_step)
          {
               start_t = ros::WallTime::now();
               input = getKey();
               if ((input == ESC_ASCII_VALUE))
               {
                    break;
               }
               ROS_INFO("Loop: %2.2f ms  Elapsed Time: %4.2f s | ax: %2.2f  ay: %2.2f  az: %2.2f   | pitch: %2.2f    roll: %2.2f   ", t_loop, t_elapsed, ax, ay, az, imu_pitch, imu_roll);
               

               // Motion Generation
               x_RH = x_RH0;
               x_LH = x_LH0;
               y_RH = y_RH0 + 100.0;
               y_LH = y_LH0 - 100.0;
               z_RH = z_RH0 - 90.0;
               z_LH = z_LH0 - 90.0;

               foot_planning(t_elapsed, t_step, ZMP_x_amp, ZMP_y_amp, com_x, com_y, ds_time, n_step, z_h, z0, x_RF0, y_LF0, &x_RF, &y_RF, &z_RF, &x_LF, &y_LF, &z_LF);

               Pitch_RF = -0.00 + pitch_ankle_gain * imu_pitch;
               Pitch_LF = -0.00 + pitch_ankle_gain * imu_pitch;
               Roll_RF = roll_ankle_gain * imu_roll;
               Roll_LF = roll_ankle_gain * imu_roll;
               x_LF = x_LF + ax_gain * ax;
               x_RF = x_RF + ax_gain * ax;

               y_LF = y_LF + roll_disp_gain * (imu_roll+(0.4)) + ay_gain * ay ;
               y_RF = y_RF + roll_disp_gain * (imu_roll+(0.4)) + ay_gain * ay ;
               IK_RH(x_RH, y_RH, z_RH, posis);
               IK_LH(x_LH, y_LH, z_LH, posis);
               IK_RF(x_RF, y_RF, z_RF, Roll_RF, Pitch_RF, posis);
               IK_LF(x_LF, y_LF, z_LF, Roll_LF, Pitch_LF, posis);

               joint_angle_cmd_msg.data.clear();
               joint_angle_cmd_msg.data.resize(16);
               for (int i = 0; i < 16; i++)
                    joint_angle_cmd_msg.data[i] = (double)posis[i];
               joint_angle_cmd_pub_.publish(joint_angle_cmd_msg);

               tick++; // For general tick
                       //simple IMU filter
               imu_roll = (imu_roll_raw - imu_roll_init + imu_roll_old1 + imu_roll_old2) / 3.0;
               imu_roll_old2 = imu_roll_old1;
               imu_roll_old1 = imu_roll;

               imu_pitch = (imu_pitch_raw - imu_pitch_init + imu_pitch_old1 + imu_pitch_old2) / 3.0;
               imu_pitch_old2 = imu_pitch_old1;
               imu_pitch_old1 = imu_pitch;

               ax = (ax_raw + ax_old1 + ax_old2) / 3.0;
               ax_old2 = ax_old1;
               ax_old1 = ax;

               ay = (ay_raw + ay_old1 + ay_old2 + ay_old3 + ay_old4) / 5.0;
               ay_old4 = ay_old3;
               ay_old3 = ay_old2;
               ay_old2 = ay_old1;
               ay_old1 = ay;

               az = (az_raw + az_old1 + az_old2) / 3.0;
               az_old2 = az_old1;
               az_old1 = az;

               loop_rate.sleep();
               ros::spinOnce();

               // Measure Loop time
               end_t = ros::WallTime::now();
               t_elapsed = (end_t - t0).toNSec() * 1e-9;
               t_loop = (end_t - start_t).toNSec() * 1e-6;
          }
     }

     initPose(posis);
     joint_angle_cmd_msg.data.clear();
     joint_angle_cmd_msg.data.resize(16);
     for (int i = 0; i < 16; i++)
          joint_angle_cmd_msg.data[i] = (double)posis[i];
     joint_angle_cmd_pub_.publish(joint_angle_cmd_msg);
     ros::Duration(0.5).sleep();

     ros::shutdown();
     return 0;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
     double qx = msg->orientation.x;
     double qy = msg->orientation.y;
     double qz = msg->orientation.z;
     double qw = msg->orientation.w;
     double ax_imu = msg->linear_acceleration.x;
     double ay_imu = msg->linear_acceleration.y;
     double az_imu = msg->linear_acceleration.z;

     double sinr_cosp = 2 * (qw * qx + qy * qz);
     double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);

     double sinp = 2 * (qw * qy - qz * qx);
     ax_raw = ax_imu;
     ay_raw = ay_imu;
     az_raw = az_imu;
     imu_roll_raw = -std::atan(sinr_cosp / cosr_cosp) - 0.1;
     imu_pitch_raw = std::asin(sinp) + 0.1;
}

void scanPose(std::vector<float> &positions)
{
     //scan pose
     x_RH = x_RH0;
     x_LH = x_LH0;
     y_RH = y_RH0 + 100.0;
     y_LH = y_LH0 - 100.0;
     z_RH = z_RH0 - 90.0;
     z_LH = z_LH0 - 90.0;

     x_RF = x_RF0 + 30;
     y_RF = y_RF0;
     z_RF = z_RF0 + 20;

     x_LF = x_LF0 + 30;
     y_LF = y_LF0;
     z_LF = z_LF0 + 20; 

     IK_RH(x_RH, y_RH, z_RH, positions);
     IK_LH(x_LH, y_LH, z_LH, positions);
     IK_RF(x_RF, y_RF, z_RF, Roll_RF, Pitch_RF, positions);
     IK_LF(x_LF, y_LF, z_LF, Roll_LF, Pitch_LF, positions);

     positions[8] -= 0.3;
     positions[9] += 0.3;
}

void initPose(std::vector<float> &positions)
{
     //initial pose
     x_RH = x_RH0;
     x_LH = x_LH0;
     y_RH = y_RH0 + 100.0;
     y_LH = y_LH0 - 100.0;
     z_RH = z_RH0 - 90.0;
     z_LH = z_LH0 - 90.0;

     x_RF = x_RF0;
     y_RF = y_RF0;
     z_RF = z_RF0 + 20;

     x_LF = x_LF0;
     y_LF = y_LF0;
     z_LF = z_LF0 + 20; 

     IK_RH(x_RH, y_RH, z_RH, positions);
     IK_LH(x_LH, y_LH, z_LH, positions);
     IK_RF(x_RF, y_RF, z_RF, Roll_RF, Pitch_RF, positions);
     IK_LF(x_LF, y_LF, z_LF, Roll_LF, Pitch_LF, positions);
}

void IK_RH(float x, float y, float z, std::vector<float> &positions)
{

     float L_sh = 39.0; // Origin to arm roll joint
     float L_a1 = 18.0; // Shoulder bracket horizontal distance
     float L_a2 = 12.0; // Shoulder bracket vertical distance
     float L_a3 = 45.0; // Upper arm length
     float L_a4 = 72.0; // Lower arm length

     float x_0 = 0.0;
     float y_0 = -174.0;
     float z_0 = -12.0;

     float th1 = 0.0, th3 = 0.0, th5 = 0.0;
     int DXL_POS_1 = 512, DXL_POS_3 = 512, DXL_POS_5 = 512;

     // Right Arm
     float x_RH = x;
     float y_RH = y;
     float z_RH = z;
     float x_RH0 = x_RH;
     float y_RH0 = y_RH + (L_sh + L_a1);
     float z_RH0 = z_RH;

     th1 = -atan(x_RH0 / (z_RH0 + 0.00001));

     float R1_RH = sqrt((x_RH0 - L_a2 * sin(th1)) * (x_RH0 - L_a2 * sin(th1)) + y_RH0 * y_RH0 + (z_RH0 + L_a2 * cos(th1)) * (z_RH0 + L_a2 * cos(th1)));

     if (R1_RH > 117)
     {
          R1_RH = 117;
     }
     else
     {
          R1_RH = R1_RH;
     }

     float alpha_RH = acos((L_a3 * L_a3 + L_a4 * L_a4 - R1_RH * R1_RH) / (2 * L_a3 * L_a4));

     // Elbow Joint angle
     th5 = -PI + alpha_RH;
     float R2_RH = sqrt((x_RH0 - L_a2 * sin(th1)) * (x_RH0 - L_a2 * sin(th1)) + (z_RH0 + L_a2 * cos(th1)) * (z_RH0 + L_a2 * cos(th1)));

     // Shoulder Joint angle
     if (z_RH > 0)
     {
          th3 = PI / 2 + (atan(y_RH0 / R2_RH) + acos((L_a3 * L_a3 + R1_RH * R1_RH - L_a4 * L_a4) / (2 * L_a3 * R1_RH)));
     }
     else
     {
          th3 = -PI / 2 + (-atan(y_RH0 / R2_RH) + acos((L_a3 * L_a3 + R1_RH * R1_RH - L_a4 * L_a4) / (2 * L_a3 * R1_RH)));
     }

     DXL_POS_1 = (512 + (int)(th1 * 195.3786));
     DXL_POS_3 = (512 + (int)(th3 * 195.3786));
     DXL_POS_5 = (512 + (int)(th5 * 195.3786));

     positions[0] = th1;
     positions[2] = th3;
     positions[4] = th5;
}

void IK_LH(float x, float y, float z, std::vector<float> &positions)
{
     float L_sh = 39.0; // Origin to arm roll joint
     float L_a1 = 18.0; // Shoulder bracket horizontal distance
     float L_a2 = 12.0; // Shoulder bracket vertical distance
     float L_a3 = 45.0; // Upper arm length
     float L_a4 = 72.0; // Lower arm length

     // Position of Left hand in Initial Pose (all the left arm motor angle == 0)

     float x_0 = 0.0;
     float y_0 = 174.0;
     float z_0 = -12.0;

     float th2 = 0.0, th4 = 0.0, th6 = 0.0;
     int DXL_POS_2 = 0, DXL_POS_4 = 0, DXL_POS_6 = 0;
     // Left Arm
     float x_LH = x;
     float y_LH = y;
     float z_LH = z;

     float x_LH0 = x_LH;
     float y_LH0 = y_LH - (L_sh + L_a1);
     float z_LH0 = z_LH;

     th2 = atan(x_LH0 / (z_LH0 + 0.00001));

     // Range of XL320 motor -150 to 150 deg

     float R1_LH = sqrt((x_LH0 - L_a2 * sin(th2)) * (x_LH0 - L_a2 * sin(th2)) + y_LH0 * y_LH0 + (z_LH0 + L_a2 * cos(th2)) * (z_LH0 + L_a2 * cos(th2)));

     if (R1_LH > 117)
     {
          R1_LH = 117;
     }
     else
     {
          R1_LH = R1_LH;
     }

     float alpha_LH = acos((L_a3 * L_a3 + L_a4 * L_a4 - R1_LH * R1_LH) / (2 * L_a3 * L_a4));

     // Elbow Joint angle
     th6 = PI - alpha_LH;

     float R2_LH = sqrt((x_LH0 - L_a2 * sin(th2)) * (x_LH0 - L_a2 * sin(th2)) + (z_LH0 + L_a2 * cos(th2)) * (z_LH0 + L_a2 * cos(th2)));

     // Shoulder Joint angle
     if (z_LH0 > 0)
     {
          th4 = -((atan(R2_LH / y_LH0) + acos((L_a3 * L_a3 + R1_LH * R1_LH - L_a4 * L_a4) / (2 * L_a3 * R1_LH))));
     }
     else
     {
          th4 = -(-PI / 2 + (atan(y_LH0 / R2_LH) + acos((L_a3 * L_a3 + R1_LH * R1_LH - L_a4 * L_a4) / (2 * L_a3 * R1_LH))));
     }

     DXL_POS_2 = (512 + (int)(th2 * 195.3786));
     DXL_POS_4 = (512 + (int)(th4 * 195.3786));
     DXL_POS_6 = (512 + (int)(th6 * 195.3786));

     positions[1] = th2;
     positions[3] = th4;
     positions[5] = th6;
}

void IK_RF(float x, float y, float z, float th_r, float th_p, std::vector<float> &positions)
{
     float L_by = 24.0; // Origin to pelvis vertical length
     float L_bz = 72.0; // Pelvis horizontal length
     float L_bx = 15.0; // Shoulder joint axis to Leg center (On Sagittal Plane)
     float L_l1 = 6.0;  // Pelvis Roll axis to pitch axis
     float L_l2 = 45.0; // Thigh Length
     float L_l3 = 42.0; // Shank Length
     float L_l4 = 31.0; // Ankle Length
     float L_f = 9.0;   // Foot horizontal length

     float x_0 = 15.0;
     float y_0 = -24.0;
     float z_0 = -196.0;
     float x_RF = 0;
     float y_RF = 0;
     float z_RF = 0;

     float th7 = 0.0, th9 = 0.0, th11 = 0.0, th13 = 0.0, th15 = 0.0;
     int DXL_POS_7 = 512, DXL_POS_9 = 512, DXL_POS_11 = 512, DXL_POS_13 = 512, DXL_POS_15 = 512;

     x_RF = x;
     y_RF = y;
     z_RF = z;

     float pos_RF[3] = {
         -L_bz - z_RF - L_f * sin(th_r) - L_l4 * cos(th_p) * cos(th_r),
         L_by + y_RF + L_f * cos(th_r) - L_l4 * cos(th_p) * sin(th_r),
         x_RF - L_bx + L_l4 * sin(th_p)};

     th7 = atan(pos_RF[1] / pos_RF[0]);
     float R1_RF = sqrt((pos_RF[0] - L_l1 * cos(th7)) * (pos_RF[0] - L_l1 * cos(th7)) + (pos_RF[1] - L_l1 * sin(th7)) * (pos_RF[1] - L_l1 * sin(th7)) + pos_RF[2] * pos_RF[2]);
     float alpha_RF = acos((L_l2 * L_l2 + L_l3 * L_l3 - R1_RF * R1_RF) / (2 * L_l2 * L_l3));
     th11 = PI - alpha_RF;
     float R2_RF = sqrt((pos_RF[0] - L_l1 * cos(th7)) * (pos_RF[0] - L_l1 * cos(th7)) + (pos_RF[1] - L_l1 * sin(th7)) * (pos_RF[1] - L_l1 * sin(th7)));

     th9 = -((atan(pos_RF[2] / R2_RF) + acos((L_l2 * L_l2 + R1_RF * R1_RF - L_l3 * L_l3) / (2 * L_l2 * R1_RF))));

     th13 = -asin(cos(th9 + th11) * cos(th_r) * cos(th7) * sin(th_p) - sin(th9 + th11) * cos(th_p) + cos(th9 + th11) * sin(th_p) * sin(th_r) * sin(th7));
     th15 = -asin(sin(th_r - th7) * cos(th_p));

     positions[6] = th7;
     positions[8] = th9;
     positions[10] = th11;
     positions[12] = th13;
     positions[14] = th15;
}

void IK_LF(float x, float y, float z, float th_r, float th_p, std::vector<float> &positions)
{

     float L_by = 24.0; // Origin to pelvis vertical length
     float L_bz = 72.0; // Pelvis horizontal length
     float L_bx = 15.0; // Shoulder joint axis to Leg center (On Sagittal Plane)
     float L_l1 = 6.0;  // Pelvis Roll axis to pitch axis
     float L_l2 = 45.0; // Thigh Length
     float L_l3 = 42.0; // Shank Length
     float L_l4 = 31.0; // Ankle Length
     float L_f = 9.0;   // Foot horizontal length

     float x_0 = 15.0;
     float y_0 = 24.0;
     float z_0 = -196.0;
     float x_LF = 0;
     float y_LF = 0;
     float z_LF = 0;

     float th8 = 0.0, th10 = 0.0, th12 = 0.0, th14 = 0.0, th16 = 0.0;
     int DXL_POS_8 = 512, DXL_POS_10 = 512, DXL_POS_12 = 512, DXL_POS_14 = 512, DXL_POS_16 = 512;

     x_LF = x;
     y_LF = y;
     z_LF = z;

     float pos_LF[3] = {
         L_f * sin(th_r) - z_LF - L_bz - L_l4 * cos(th_p) * cos(th_r),
         y_LF - L_by - L_f * cos(th_r) - L_l4 * cos(th_p) * sin(th_r),
         x_LF - L_bx + L_l4 * sin(th_p)};

     th8 = atan(pos_LF[1] / pos_LF[0]);
     float R1_LF = sqrt((pos_LF[0] - L_l1 * cos(th8)) * (pos_LF[0] - L_l1 * cos(th8)) + (pos_LF[1] - L_l1 * sin(th8)) * (pos_LF[1] - L_l1 * sin(th8)) + pos_LF[2] * pos_LF[2]);
     float alpha_LF = acos((L_l2 * L_l2 + L_l3 * L_l3 - R1_LF * R1_LF) / (2 * L_l2 * L_l3));

     th12 = -PI + alpha_LF;
     float R2_LF = sqrt((pos_LF[0] - L_l1 * cos(th8)) * (pos_LF[0] - L_l1 * cos(th8)) + (pos_LF[1] - L_l1 * sin(th8)) * (pos_LF[1] - L_l1 * sin(th8)));

     th10 = (atan(pos_LF[2] / R2_LF) + acos((L_l2 * L_l2 + R1_LF * R1_LF - L_l3 * L_l3) / (2 * L_l2 * R1_LF)));

     th14 = -acos(cos(th10 + th12) * cos(th_p) - sin(th10 + th12) * cos(th_r) * cos(th8) * sin(th_p) - sin(th10 + th12) * sin(th_p) * sin(th_r) * sin(th8));
     th16 = -asin(sin(th_r - th8) * cos(th_p));

     positions[7] = th8;
     positions[9] = th10;
     positions[11] = th12;
     positions[13] = th14;
     positions[15] = th16;
}

int getch()
{
     struct termios oldt, newt;
     int ch;
     tcgetattr(STDIN_FILENO, &oldt);
     newt = oldt;
     newt.c_lflag &= ~(ICANON | ECHO);
     tcsetattr(STDIN_FILENO, TCSANOW, &newt);
     ch = getchar();
     tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
     return ch;
}

int kbhit(void)
{
     struct termios oldt, newt;
     int ch;
     int oldf;

     tcgetattr(STDIN_FILENO, &oldt);
     newt = oldt;
     newt.c_lflag &= ~(ICANON | ECHO);
     tcsetattr(STDIN_FILENO, TCSANOW, &newt);
     oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
     fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

     ch = getchar();

     tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
     fcntl(STDIN_FILENO, F_SETFL, oldf);

     if (ch != EOF)
     {
          ungetc(ch, stdin);
          return 1;
     }
     return 0;
}

char getKey()
{
     if (kbhit())
     {
          return getch();
     }
     return '\0';
}
