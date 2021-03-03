#!/usr/bin/env python3
#
# tb3_sensor_to_sonar.py
# ################################################################################
# HHT from W-HS, 21.01.2021
#
# brings sensor readings from turtlebot sensorstate to sonar topic
# ------------------------------------------------------------------

import rospy
import std_msgs.msg
from turtlebot3_msgs.msg import SensorState


class SensorState_test():

    value = 0

    def __init__(self):
        self.rate = rospy.Rate(10)

        self.sensorstate_pub = rospy.Publisher('sensor_state',
                                               SensorState,
                                               queue_size=10)

        rospy.loginfo("Sending SensorState")

        self.create_msg()
        self.pub_msg()
        pass

    def pub_msg(self):
        self.create_msg()
        self.sensorstate_pub.publish(self.sensorstate_msg)
        pass

    def create_msg(self):
        self.sensorstate_msg = SensorState()
        sonar = std_msgs.msg.Float32()
        sonar = self.value % 10
        print(sonar)
        self. value = self.value + 1
        self.sensorstate_msg.sonar = sonar
        pass


if __name__ == '__main__':
    rospy.init_node('SensorState_test', anonymous=True)
    try:
        sensorstate = SensorState_test()
        while not rospy.is_shutdown():
            sensorstate.pub_msg()
            sensorstate.rate.sleep()
        pass
    except rospy.ROSInterruptException:
        pass
