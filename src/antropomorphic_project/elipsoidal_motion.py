#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from planar_3dof_control.msg import EndEffector
from math import sin, cos, pi

class EE_Client(object):
    def __init__(self):
        self.pub_end_effector_commads= rospy.Publisher("/ee_pose_commands", EndEffector, queue_size=1)
        self._rate = rospy.Rate(20.0)
        self.unitary_angle = 0.0
        self.height = 0.0
        self.height_delta = 0.1
        self.up_flag = True
        self.height_MAX = 0.5
        self.height_MIN = -0.5
        self.a = 1.5
        self.b = 1.0
        self.elipse_delta = 0.1
        self.elipse_MIN = 0.8
        self.elipse_MAX = 1.7
        
    def generate_elipse_points(self, delta=0.03):
        x = self.a * cos(self.unitary_angle)
        y = self.b * sin(self.unitary_angle)

        self.unitary_angle += delta
        if self.unitary_angle >= 2*pi:
            self.unitary_angle = 0.0
            
            if self.up_flag:
                self.height += self.height_delta
                self.a -= self.elipse_delta
                self.b -= self.elipse_delta
                
                if self.a < self.elipse_MIN:
                    self.a= self.elipse_MIN
                if self.b < self.elipse_MIN:
                    self.b= self.elipse_MIN
            else:
                self.height -= self.height_delta

                self.a += self.elipse_delta
                self.b += self.elipse_delta

                if self.a > self.elipse_MAX:
                    self.a= self.elipse_MAX
                if self.b > self.elipse_MAX:
                    self.b= self.elipse_MAX

            if self.height >= self.height_MAX:
                self.up_flag = False

            if self.height <= self.height_MIN:
                self.up_flag = True

        return x, y, self.height


    def start_loop(self):

        while not rospy.is_shutdown():
            
            x_elipse, y_elipse, z_elipse = self.generate_elipse_points()
            ee_msg = EndEffector()
            ee_pose = Vector3()
            ee_pose.x = x_elipse
            ee_pose.y = y_elipse
            ee_pose.z = z_elipse
            # We set this by default, if not possible soltion
            # The movier will try the other options
            elbow_policy = "plus-plus"
            ee_msg.ee_xy_theta = ee_pose
            ee_msg.elbow_policy.data = elbow_policy
            print("Elipse Point="+str(ee_pose)+", elbow="+str(elbow_policy))
            self.pub_end_effector_commads.publish(ee_msg)
            self._rate.sleep()

def main():
    rospy.init_node('circular_motion_ee')
    ee_client_obj = EE_Client()
    try:
        ee_client_obj.start_loop()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()