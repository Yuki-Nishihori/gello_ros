#!/usr/bin/env python3
from gello_ros.agents.gello_publisher import GelloPublisher
import rospy

if __name__ == '__main__':
    rospy.init_node('gello_publish_node')
    agent = GelloPublisher(port=rospy.get_param("~gello_port", None))
    try:
        agent.publish_joint_states()
    except rospy.ROSInterruptException:
        pass