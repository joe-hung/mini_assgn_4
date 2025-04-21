#!/usr/bin/env python3
import rospy
import geometry_msgs.msg 
import tf2_ros
import tf2_geometry_msgs as tf_geo
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import WrenchStamped, PointStamped
import numpy as np


class MINI_Measured_ZMP: 
    
    def __init__(self):
        rospy.init_node('mini_measured_zmp', anonymous=True)

        self.left_zmp_point = PointStamped()
        self.right_zmp_point = PointStamped()

        self.zmp_x_queue = []
        self.zmp_y_queue = []
        self.zmp_z_queue = []

        RF_FT_sensor = rospy.Subscriber("/RF_FT_sensor", WrenchStamped, self.RF_FT_sensor_callback)
        LF_FT_sensor = rospy.Subscriber("/LF_FT_sensor", WrenchStamped, self.LF_FT_sensor_callback)

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        self.init_zmp_marker()

        rospy.sleep(2.0)

    def init_zmp_marker(self):
        self.zmp_marker_pub = rospy.Publisher('measured_zmp', Marker, queue_size=1)
        marker =  Marker()
        marker.header.frame_id = "l_foot_link"
        marker.header.stamp = rospy.Time()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.b = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        self.zmp_marker = marker
        
    def RF_FT_sensor_callback(self, msg):
        force = msg.wrench.force
        torque = msg.wrench.torque
        # fixing sensor axis coord frame
        tx = torque.z
        ty = torque.y
        fx = force.z
        fy = force.y
        fz = -1*force.x
        d = 0.03

        # TODO: Implement measured ZMP equation to solve for right foot px, py 
        # using provided variables: tx, ty, fx, fy, fz, d
        pRx = (-ty-fx*d)/fz
        pRy = (-tx-fy*d)/fz

        self.right_force_z = fz
        self.right_zmp_point.header.stamp = rospy.Time.now()
        self.right_zmp_point.header.frame_id = "right_foot_link"
        self.right_zmp_point.point.x = d
        self.right_zmp_point.point.y = pRy
        self.right_zmp_point.point.z = pRx
        

    def LF_FT_sensor_callback(self, msg):
        force = msg.wrench.force
        torque = msg.wrench.torque
        # fixing sensor axis coord frame
        tx = torque.z
        ty = -torque.y
        fx = force.z
        fy = -force.y
        fz = force.x
        d = 0.03

        # TODO: Implement measured ZMP equation to solve for left foot px, py 
        # using provided variables: tx, ty, fx, fy, fz, d
        pLx = (-ty-fx*d)/fz
        pLy = (-tx-fy*d)/fz
        
        self.left_force_z = fz
        self.left_zmp_point.header.stamp = rospy.Time.now()
        self.left_zmp_point.header.frame_id = "left_foot_link"
        self.left_zmp_point.point.x = -d
        self.left_zmp_point.point.y = -pLy
        self.left_zmp_point.point.z = pLx
 

    def start(self):
        trans_l = self.tfBuffer.lookup_transform("base_link", "l_foot_link", rospy.Time())
        trans_r = self.tfBuffer.lookup_transform("base_link", "r_foot_link", rospy.Time())
        left_zmp = tf_geo.do_transform_point(self.left_zmp_point, trans_l)
        right_zmp = tf_geo.do_transform_point(self.right_zmp_point, trans_r)


        # TODO: Implement measured ZMP equation to solve for both feet px, py 
        # using provided variables: right_zmp.point, left_zmp.point, self.right_force_z, self.left_force_z
        px = (right_zmp.point.x*self.right_force_z + left_zmp.point.x*self.left_force_z) / ( self.right_force_z + self.left_force_z )
        py = (right_zmp.point.y*self.right_force_z + left_zmp.point.y*self.left_force_z) / ( self.right_force_z + self.left_force_z )
        pz = -0.165

        queue_len = 200
        self.zmp_x_queue.append(px)
        self.zmp_y_queue.append(py)
        self.zmp_z_queue.append(pz)

        if len(self.zmp_z_queue) > queue_len:
            self.zmp_x_queue.pop(0)
            self.zmp_y_queue.pop(0)
            self.zmp_z_queue.pop(0)  

            self.zmp_marker.header.frame_id = "base_link"
            self.zmp_marker.pose.position.x = np.average(self.zmp_x_queue) 
            self.zmp_marker.pose.position.y = np.average(self.zmp_y_queue)
            self.zmp_marker.pose.position.z = -0.165
            self.zmp_marker_pub.publish(self.zmp_marker)
    
if __name__ == '__main__':
    mini_measured_zmp = MINI_Measured_ZMP()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        mini_measured_zmp.start()

        rate.sleep()
