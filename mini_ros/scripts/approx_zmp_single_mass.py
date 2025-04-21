#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs as tf_geo
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker
import numpy as np
from std_msgs.msg import Float32MultiArray
import math

class MINI_Approx_ZMP_Single_Mass: 
    
    def __init__(self):
        rospy.init_node('mini_approx_zmp_single_mass', anonymous=True)
        self.com_x_pos_queue = []
        self.com_y_pos_queue = []
        self.com_z_pos_queue = []
        self.com_stamp_queue = []

        rospy.Subscriber("/com", Marker, self.com_callback)
        self.approx_zmp = rospy.Publisher("/approx_zmp_single_mass", Float32MultiArray, queue_size=10)
        self.approx_zmp_marker = rospy.Publisher("/approx_zmp_single_mass_marker", Marker, queue_size=10)

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        self.marker = marker

    def calc_single_mass_zmp(self, x_pos, y_pos, z_pos, x_accl, y_accl, z_accl, g = 9.81, pz = 0.30):
        px = 0
        py = 0

        # TODO: Implement multi mass ZMP equation to solve for px, py 
        # using provided variables: x_pos, y_pos, z_pos, x_accl, y_accl, z_accl, g, pz
        px = x_pos - (z_pos - pz)*x_accl/(z_accl + g)
        py = y_pos - (z_pos - pz)*y_accl/(z_accl + g)
        return px, py

    def com_callback(self, msg):
        com_pos = msg.pose.position
        queue_len = 100

        self.com_x_pos_queue.append(com_pos.x)
        self.com_y_pos_queue.append(com_pos.y)
        self.com_z_pos_queue.append(com_pos.z)
        self.com_stamp_queue.append(msg.header.stamp)

        if len(self.com_x_pos_queue) > queue_len:
            self.com_x_pos_queue.pop(0)
            self.com_y_pos_queue.pop(0)
            self.com_z_pos_queue.pop(0)  
            self.com_stamp_queue.pop(0)

            idx1 = int(queue_len/3)
            idx2 = int(queue_len*2/3)
            x_pos = [np.average(self.com_x_pos_queue[:idx1]),
                         np.average(self.com_x_pos_queue[idx1:idx2]),
                         np.average(self.com_x_pos_queue[idx2:])]
            y_pos = [np.average(self.com_y_pos_queue[:idx1]),
                         np.average(self.com_y_pos_queue[idx1:idx2]),
                         np.average(self.com_y_pos_queue[idx2:])]
            z_pos = [np.average(self.com_z_pos_queue[:idx1]),
                         np.average(self.com_z_pos_queue[idx1:idx2]),
                         np.average(self.com_z_pos_queue[idx2:])]
            dt1 = (self.com_stamp_queue[idx1] - self.com_stamp_queue[0]).to_sec()
            dt2 = (self.com_stamp_queue[idx2] - self.com_stamp_queue[idx1]).to_sec()


            x_vel = [(x_pos[1]-x_pos[0])/dt1,
                    (x_pos[2]-x_pos[1])/dt2]
            x_accl = x_vel[1]-x_vel[0] / dt2

            y_vel = [(y_pos[1]-y_pos[0])/dt1,
                    (y_pos[2]-y_pos[1])/dt2]
            y_accl = y_vel[1]-y_vel[0] / dt2

            z_vel = [(z_pos[1]-z_pos[0])/dt1,
                    (z_pos[2]-z_pos[1])/dt2]
            z_accl = z_vel[1]-z_vel[0] / dt2

            if math.isnan(z_accl):
                return

            zmp_x, zmp_y = self.calc_single_mass_zmp(x_pos[0], y_pos[0], z_pos[0], x_accl, y_accl, z_accl)

            zmp = Float32MultiArray()
            zmp.data = [zmp_x,zmp_y]
            self.approx_zmp.publish(zmp)

            self.marker.pose.position.x = zmp_x
            self.marker.pose.position.y = zmp_y 
            self.marker.pose.position.z = -0.165
            self.marker.header.stamp = rospy.Time().now()
            self.approx_zmp_marker.publish(self.marker)
             
if __name__ == '__main__':
    mini_approx_zmp_single_mass = MINI_Approx_ZMP_Single_Mass()
    rate = rospy.Rate(100)
    rospy.sleep(2.0)
    
    while not rospy.is_shutdown():
        rate.sleep()