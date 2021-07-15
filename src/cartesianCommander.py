#!/usr/bin/env python3
#publishes absolute motor commands message from a keyboard input

import sys
from select import select
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool


def commander():
  timeout = 60
  rospy.init_node('motor_commander')
  cmdpublisher = rospy.Publisher('command_pose', PoseStamped, queue_size = 1)
  homepub=rospy.Publisher('home_axis', Bool, queue_size=1)
  gripperpub = rospy.Publisher('gripper', Bool, queue_size = 1)
  rate=rospy.Rate(1)
  gripper_msg = Bool()
  gripper_msg.data = None

  while not rospy.is_shutdown():
    print("\n\nEnter an absolute motion command as 'x,y,z'.\nUse '1,1,1' to stop all axes.\nUse '999' on x '999,0,0' to begin homing \n Use 555 on x to toggle gripper:")
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        x = [sys.stdin.readline()]
        command_data = [float(i) for i in x[0].split(',')]
        command = PoseStamped()
        command.header.stamp=rospy.Time.now()
        command.header.frame_id = 'world'
        command.pose.position.x = command_data[0]
        command.pose.position.y = command_data[1]
        command.pose.position.z = command_data[2]

        if command_data[0] == 999:
            msg = Bool()
            msg.data = True
            print("\nHoming Axes")
            homepub.publish(msg)
        elif command_data[0] == 555:
            if gripper_msg.data == True:
                gripper_msg.data = False
            else:
                gripper_msg.data = True
            gripperpub.publish(gripper_msg)
        
        else:
            print("\nRelative motion command recieved: ")
            print(command_data)        
            cmdpublisher.publish(command)
    else:
      print("No input. Moving on...")
    rate.sleep()

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass