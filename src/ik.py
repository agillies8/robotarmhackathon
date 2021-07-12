#!/usr/bin/env python3
#publishes a message with the joint positions given x,y,z positions.
#follows ROS right hand rule conventions
import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16, String

x_input = None
y_input = None
z_input = None

j1_output = None
j2_output = None
j3_output = None

class Circle:
    def __init__(self, center, radius):
        self.c = np.array(center)
        self.r = radius

    def intersect(self, other):
        dist = np.linalg.norm(self.c - other.c)
        if dist > self.r + other.r:
            # No contact
            return None
        elif dist == 0 and self.r == other.r:
            # Coincident
            return np.inf
        elif dist + min(self.r, other.r) < max(self.r, other.r):
            # Contained
            return None
        else:
            # Two intersections
            a = (self.r**2 - other.r**2 + dist**2) / (2 * dist)
            h = np.sqrt(self.r**2 - a**2)

            p2 = self.c + (a * (other.c - self.c)) / dist
            i1 = np.array(p2)
            i1[0] += h * (other.c[1] - self.c[1]) / dist
            i1[1] -= h * (other.c[0] - self.c[0]) / dist
            i2 = np.array(p2)
            i2[0] -= h * (other.c[1] - self.c[1]) / dist
            i2[1] += h * (other.c[0] - self.c[0]) / dist
            return i1, i2

class IK_Solver():
    def __init__(self):
        self.j2_center = np.array([0, rospy.get_param("J2_BASE_Z_OFFSET")])
        self.link_1 = rospy.get_param("LINK_1_LENGTH")
        self.link_2 = rospy.get_param("LINK_2_LENGTH")
        self.link_3 = rospy.get_param("LINK_3_LENGTH")
        self.link_3_z_offset = rospy.get_param("LINK_3_Z_OFFSET")
        self.joint_command_publisher=rospy.Publisher('command_joints', PoseStamped, queue_size=10)
        self.valid = None
        self.elbow = None

    def pose_callback(self, msg):
        x_input = msg.pose.position.x
        y_input = msg.pose.position.y
        z_input = msg.pose.position.z

        j1_output = math.atan(y_input/x_input)

        radial_distance = math.sqrt(x_input**2 + y_input**2)
        j5_center = np.array([radial_distance - self.link_3, z_input+self.link_3_z_offset]) #the goal pose in the arm plane

        c1 = Circle(self.j2_center, self.link_1)
        c2 = Circle(j5_center, self.link_2)
        points = c1.intersect(c2)
        if points is not None and points != np.inf:
            # valid, pick higher point
            self.valid = True
            if points[0][1] > points[1][1]:
                self.elbow = points[0]
            else:
                self.elbow = points[1]
        else:
            self.valid = False
        #rospy.loginfo(f'elbow_x: {self.elbow[0]-self.j2_center[0]} elbow_y: {self.elbow[1]-self.j2_center[1]}')

        if self.elbow[1]-self.j2_center[1] > 0:
            j2_output = -1*math.asin(self.elbow[0]/self.link_1)
        else:
            j2_output = 3.1415 + math.asin(self.elbow[0]/self.link_1)
        j3_output = math.acos((radial_distance-self.link_3-self.elbow[0])/self.link_2) #- j2_output
        joints_pose = PoseStamped()
        joints_pose.header = msg.header
        joints_pose.pose.position.x = j1_output
        joints_pose.pose.position.y = j2_output
        joints_pose.pose.position.z = j3_output
        self.joint_command_publisher.publish(joints_pose)
        
        
def main():
    rospy.init_node('ik_solver')
   
    ik_solver = IK_Solver()

    sub1=rospy.Subscriber('command_pose', PoseStamped, ik_solver.pose_callback)
    
    
    rate=rospy.Rate(10)
 
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
