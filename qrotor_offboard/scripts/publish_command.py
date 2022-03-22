#!/usr/bin/env python
import rospy
from qrotor_gazebo_plugin.msg import Command
from geometry_msgs.msg import Vector3


def talker():
    pub = []
    for i in range(4):
        pub.append(rospy.Publisher('/falcon'+str(i) +
                   '/command', Command, queue_size=10))
    rospy.init_node('command', anonymous=True)
    rospy.loginfo("Initialzing publish command rosnode")
    rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        for i in range(4):
            msg = Command()
            msg.mode = Command.MODE_POSITION
            position = Vector3()
            if i == 0:
                position.x, position.y, position.z = 1, 0, 4
            if i == 1:
                position.x, position.y, position.z = 0, 1, 4
            if i == 2:
                position.x, position.y, position.z = -1, 0, 4
            if i == 3:
                position.x, position.y, position.z = 0, -1, 4
            msg.command.append(position)
            pub[i].publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
