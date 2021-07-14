#!/usr/bin/env python3

import threading

#import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Robot!
---------------------------
Moving axis:
   u    <-z axis->    o
   j    <-y axis->    l
   m    <-x axis->    .

a : home

CTRL-C to quit
"""

moveBindings = {
        'u':(0,0,0.005,False),
        'o':(0,0,-0.005,False),
        'j':(0,0.005,0,False),
        'l':(0,-0.005,0,False),
        'm':(0.005,0,0,False),
        '.':(-0.005,0,0,False),
        'a':(0,0,0,True)
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('command_pose', PoseStamped, queue_size = 1)
        self.homepub = rospy.Publisher('home_axis', Bool, queue_size = 1)
        self.x_axis = 0.25
        self.y_axis = 0
        self.z_axis = 0.12
        self.home = False
        self.condition = threading.Condition()
        self.done = False
        self.command = PoseStamped()
        self.command.pose.position.x = 0.25
        self.command.pose.position.y = 0.0
        self.command.pose.position.z = 0.12

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x_axis, y_axis, z_axis, home):
        self.condition.acquire()
        self.x_axis = x_axis
        self.y_axis = y_axis
        self.z_axis = z_axis
        self.home = home
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        #self.update(0, 0, 0, False)
        self.join()

    def run(self):
       
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            self.command.header.stamp=rospy.Time.now()
            self.command.header.frame_id = 'world'

            # Copy state into twist message.
            if self.x_axis == 0.005:
                self.command.pose.position.x += 0.005
            elif self.x_axis == -0.005:
                self.command.pose.position.x -= 0.005

            if self.y_axis == 0.005:
                self.command.pose.position.y += 0.005
            elif self.y_axis == -0.005:
                self.command.pose.position.y -= 0.005

            if self.z_axis == 0.005:
                self.command.pose.position.z += 0.005
            elif self.z_axis == -0.005:
                self.command.pose.position.z -= 0.005
            print(f'x_axis: {self.command.pose.position.x}, y_axis: {self.command.pose.position.y}, z_axis: {self.command.pose.position.z}')


            self.condition.release()

            # Publish.
            if self.home == True:
                msg = Bool()
                msg.data = True
                self.homepub.publish(msg)
            else:
                self.publisher.publish(self.command)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_bump_keyboard')

    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x_axis = 0.25
    y_axis = 0
    z_axis = 0.12
    home = 0
    status = 0

    try:
        #pub_thread.wait_for_subscribers()
        pub_thread.update(x_axis, y_axis, z_axis, home)

        print(msg)
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x_axis = moveBindings[key][0]
                y_axis = moveBindings[key][1]
                z_axis = moveBindings[key][2]
                home = moveBindings[key][3]

                print(x_axis, y_axis, z_axis, home)
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x_axis == 0 and y_axis == 0 and z_axis == 0 and home == False:
                    continue
                # x_axis = 0
                # y_axis = 0
                # z_axis = 0
                home = False
                print (x_axis, y_axis, z_axis, home)
                if (key == '\x03'):
                    break
 
            pub_thread.update(x_axis, y_axis, z_axis, home)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)