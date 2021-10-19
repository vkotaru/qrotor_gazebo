#!/usr/bin/env python
import rospy
from qrotor_gazebo.msg import Command
from geometry_msgs.msg import Vector3

def talker():
    pub = rospy.Publisher('command', Command, queue_size=10)
    rospy.init_node('command', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Command()
        position = Vector3()
        position.x, position.y, position.z = 1, 1, 2
        msg.command.append(position)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass