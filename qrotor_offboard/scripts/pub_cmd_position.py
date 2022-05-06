#!/usr/bin/env python
import rospy
from qrotor_gazebo_plugin.msg import Command
from geometry_msgs.msg import Vector3

def publish_cmds():
    pub = []
    pub.append(rospy.Publisher('/falcon/command', Command, queue_size=10))
    rospy.init_node('command', anonymous=True)
    rospy.loginfo("Initialzing publish command rosnode")
    rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        msg = Command()
        msg.mode = Command.MODE_POSITION
        position = Vector3()
        position.x, position.y, position.z = 1, 0, 2
        msg.command.append(position)
        pub[0].publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_cmds()
    except rospy.ROSInterruptException:
        pass
