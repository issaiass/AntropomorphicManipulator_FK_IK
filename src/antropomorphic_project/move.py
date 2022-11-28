#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time
​
"""
Topics To Write on:
type: std_msgs/Float64
/antropomorphic_arm/joint1_position_controller/command
/antropomorphic_arm/joint2_position_controller/command
/antropomorphic_arm/joint3_position_controller/command
​
"""
​
class JointMover(object):
​
    def __init__(self):
​
        rospy.loginfo("JointMover Initialising...")
        self.pub_joint1 = rospy.Publisher('/antropomorphic_arm/joint1_position_controller/command',
                                                            Float64,
                                                            queue_size=1)
        self.pub_joint2 = rospy.Publisher('/antropomorphic_arm/joint2_position_controller/command',
                                                             Float64,
                                                             queue_size=1)
        self.pub_joint3 = rospy.Publisher('/antropomorphic_arm/joint3_position_controller/command',
                                                           Float64,
                                                           queue_size=1)
​
        self.check_connection()
​
    def move_all_joints(self, theta_1, theta_2, theta_3):
        theta_1_msg = Float64()
        theta_1_msg.data = theta_1
        theta_2_msg = Float64()
        theta_2_msg.data = theta_2
        theta_3_msg = Float64()
        theta_3_msg.data = theta_3
        self.pub_joint1.publish(theta_1_msg)
        self.pub_joint2.publish(theta_2_msg)
        self.pub_joint3.publish(theta_3_msg)
​
        rospy.logwarn("Moving to Angles= theta_1="+str(theta_1)+", theta_2="+str(theta_2)+", theta_3="+str(theta_3))
​
​
    def check_topic_connection(self, publisher_object, topic_name):
​
        self.rate = rospy.Rate(10)  # 10hz
        conections_joint_1 = publisher_object.get_num_connections()
        rospy.logdebug(conections_joint_1)
        while conections_joint_1 == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to "+str(topic_name)+" yet so we wait and try again")
​
            conections_joint_1 = publisher_object.get_num_connections()
​
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                rospy.logdebug("ROSInterruptException Triggered")
            
​
        rospy.logdebug(str(topic_name)+" Publisher Connected")
​
    def check_connection(self):
        """
        Checks publisher is working
        :return:
        """
        self.check_topic_connection(self.pub_joint1, "pub_joint1")
        self.check_topic_connection(self.pub_joint2, "pub_joint2")
        self.check_topic_connection(self.pub_joint3, "pub_joint3")
​
        rospy.logdebug("All Publishers READY")
​
​
​
if __name__ == "__main__":
    rospy.init_node('move',log_level=rospy.DEBUG)
    obj = JointMover()
    # Plus-plus
    theta_pos_1 = [0.7853981633974483, 0.09188093307208842, 1.0471975511965976]
    # Plus-minus
    theta_pos_2 = [0.7853981633974483, 1.139078484268686, -1.0471975511965976]
    # minus-Plus
    theta_pos_3 = [-2.356194490192345, 2.002514169321107, 1.0471975511965976]
    # minus-minus
    theta_pos_4 = [-2.356194490192345, 3.049711720517705, -1.0471975511965976]
​
    pos_array = [theta_pos_1,theta_pos_2,theta_pos_3,theta_pos_4]
​
    i = 0
    while not rospy.is_shutdown():
        new_pos = pos_array[i]
        obj.move_all_joints(new_pos[0],new_pos[1],new_pos[2])
        i += 1
        if i==4:
            i=0
        rospy.loginfo("Sleep...")
        time.sleep(3)
        rospy.loginfo("Sleep...END")
​
    
​