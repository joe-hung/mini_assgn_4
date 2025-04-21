#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs as tf_geo
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker
import numpy as np

class MINI_Approx_ZMP_Multi_Mass: 
    
    def __init__(self):
        rospy.init_node('mini_approx_zmp_multi_mass', anonymous=True)

        self.base_link_frame = rospy.get_param("~base_link_frame", "base_link")
        self.total_mass = 0
        #get robot description from URDF
        robot = URDF.from_parameter_server()
        self.links = robot.link_map
        self.inertial_dict = {}

        #Delete links, which contain no mass description
        unnecessary_links = []
        for link in self.links:
            if self.links[link].inertial == None or self.links[link].inertial.origin == None:
                unnecessary_links.append(link)

        for link in unnecessary_links:
            del self.links[link]
        
        #Calculate the total mass of the robot
        for link in self.links:
            self.links[link].com_x_pos_queue = []
            self.links[link].com_y_pos_queue = []
            self.links[link].com_z_pos_queue = []
            self.links[link].com_stamp_queue = []
            self.total_mass += self.links[link].inertial.mass
            self.inertial_dict[link] = self.links[link].inertial.to_yaml()

        rospy.loginfo("Mass of robot is %f", self.total_mass)

        self.init_marker()

    def init_marker(self):
        marker = Marker()
        marker.header.frame_id = self.base_link_frame
        marker.header.stamp = rospy.Time()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        self.marker = marker
    
    def start(self):
        """
        TODO: Calculate whole body CoM 
        """
        #initializations for tf and marker
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        P_CoM_link = geometry_msgs.msg.PointStamped()
        
        pub = rospy.Publisher('approx_zmp_multi_mass_marker', Marker, queue_size=1)
        
        rate = rospy.Rate(20)
        rospy.sleep(2.0)
        queue_len = 200

        while not rospy.is_shutdown():

            px_sum = 0
            py_sum = 0
            denom_sum = 0
            g = 9.81
            pz = 0.30

            for link in self.links:
                try:
                    link_info = self.links[link]
                    #get transformation matrix of link (target, source)
                    trans = tfBuffer.lookup_transform(self.base_link_frame, link, rospy.Time())
                    P_CoM_link.point.x = link_info.inertial.origin.xyz[0]
                    P_CoM_link.point.y = link_info.inertial.origin.xyz[1]
                    P_CoM_link.point.z = link_info.inertial.origin.xyz[2]
                    P_CoM_link.header.frame_id = link
                    P_CoM_link.header.stamp = rospy.get_rostime()

                    P_CoM_n = tf_geo.do_transform_point(P_CoM_link, trans)
                    m_i = link_info.inertial.mass

                    link_info.com_x_pos_queue.append(P_CoM_n.point.x)
                    link_info.com_y_pos_queue.append(P_CoM_n.point.y)
                    link_info.com_z_pos_queue.append(P_CoM_n.point.z)
                    link_info.com_stamp_queue.append(rospy.Time.now())

                    if len(link_info.com_x_pos_queue) > queue_len:
                        link_info.com_x_pos_queue.pop(0)
                        link_info.com_y_pos_queue.pop(0)
                        link_info.com_z_pos_queue.pop(0)
                        link_info.com_stamp_queue.pop(0)

                        idx1 = int(queue_len/3)
                        idx2 = int(queue_len*2/3)
                        x_pos = [np.average(link_info.com_x_pos_queue[:idx1]),
                                    np.average(link_info.com_x_pos_queue[idx1:idx2]),
                                    np.average(link_info.com_x_pos_queue[idx2:])]
                        y_pos = [np.average(link_info.com_y_pos_queue[:idx1]),
                                    np.average(link_info.com_y_pos_queue[idx1:idx2]),
                                    np.average(link_info.com_y_pos_queue[idx2:])]
                        z_pos = [np.average(link_info.com_z_pos_queue[:idx1]),
                                    np.average(link_info.com_z_pos_queue[idx1:idx2]),
                                    np.average(link_info.com_z_pos_queue[idx2:])]
                        dt1 = (link_info.com_stamp_queue[idx1] - link_info.com_stamp_queue[0]).to_sec()
                        dt2 = (link_info.com_stamp_queue[idx2] - link_info.com_stamp_queue[idx1]).to_sec()


                        x_vel = [(x_pos[1]-x_pos[0])/dt1,
                                (x_pos[2]-x_pos[1])/dt2]
                        x_accl = x_vel[1]-x_vel[0] / dt2

                        y_vel = [(y_pos[1]-y_pos[0])/dt1,
                                (y_pos[2]-y_pos[1])/dt2]
                        y_accl = y_vel[1]-y_vel[0] / dt2

                        z_vel = [(z_pos[1]-z_pos[0])/dt1,
                                (z_pos[2]-z_pos[1])/dt2]
                        z_accl = z_vel[1]-z_vel[0] / dt2

                        import math
                        if math.isnan(z_accl):
                            return

                        m = self.links[link].inertial.mass
                        x = x_pos[0]
                        y = y_pos[0]
                        z = z_pos[0]
                        x_ddot = x_accl
                        y_ddot = y_accl
                        z_ddot = z_accl

                        # TODO: Implement multi mass ZMP equation to solve for px, py 
                        # using provided variables: m, x, y, z, x_ddot, y_ddot, z_ddot
                        px_sum += m*((z_ddot + g)*x - (z - pz)*x_ddot)/ (m*(z_ddot + g))
                        py_sum += m*((z_ddot + g)*y - (z - pz)*y_ddot)/ (m*(z_ddot + g))
                        denom_sum += 1

                except tf2_ros.TransformException as err:
                    rospy.logerr("TF error in COM computation %s", err)
    
                if denom_sum != 0:
                    # TODO: Implement multi mass ZMP equation to solve for px, py 
                    # using provided variables: m, x, y, z, x_ddot, y_ddot, z_ddot
                    px = m*((z_ddot + g)*x - (z - pz)*x_ddot)/ (m*(z_ddot + g))
                    py = m*((z_ddot + g)*y - (z - pz)*y_ddot)/ (m*(z_ddot + g))

                    self.marker.pose.position.x = px
                    self.marker.pose.position.y = py
                    self.marker.pose.position.z = -0.165
                    self.marker.header.stamp = rospy.Time.now()
                    pub.publish(self.marker)

            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn("Moved backwards in time.")
             
if __name__ == '__main__':
    mini_approx_zmp_multi_mass = MINI_Approx_ZMP_Multi_Mass()
    mini_approx_zmp_multi_mass.start()