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
from sensor_msgs.msg import Range
from turtlebot3_msgs.msg import SensorState
from math import inf


class TB3_sensor_to_sonar():

    radiation_type = 0
    field_of_view = 0.5
    min_range = 0.05
    max_range = 0.99

    def __init__(self):
        self.rate = rospy.Rate(10)

        self.sonar_left_pub = rospy.Publisher('sonar_left',
                                              Range,
                                              queue_size=10)
        self.sonar_right_pub = rospy.Publisher('sonar_right',
                                               Range,
                                               queue_size=10)

        self.sensor_sub = rospy.Subscriber('sensor_state',
                                           SensorState,
                                           self.get_sonar,
                                           queue_size=10)

        rospy.loginfo("Converting Sensor")

        self.create_sonar_msgs()
        self.pub_sonar()
        pass

    def pub_sonar(self):
        self.sonar_left_pub.publish(self.sonar_left)
        self.sonar_right_pub.publish(self.sonar_right)
        pass

    def validate_sonar(self, sonar):
        if sonar == 0:
            sonar = inf
        elif sonar < (self.min_range*100):
            sonar = self.min_range
        elif sonar > (self.max_range*100):
            sonar = inf
        else:
            sonar = sonar / 100
        return sonar

    def get_sonar(self, sensor_state):
        self.sonar_left.range = self.validate_sonar(sensor_state.cliff)
        self.sonar_right.range = self.validate_sonar(sensor_state.sonar)
        self.sonar_left.header.stamp = rospy.Time.now()
        self.sonar_right.header.stamp = rospy.Time.now()
        self.pub_sonar()
        pass

    def create_sonar_msgs(self):
        header_l = std_msgs.msg.Header()
        header_l.stamp = rospy.Time.now()
        header_l.frame_id = 'sonar_left_link'
        self.sonar_left = Range()
        self.sonar_left.range = 0
        self.sonar_left.radiation_type = self.radiation_type
        self.sonar_left.field_of_view = self.field_of_view
        self.sonar_left.min_range = self.min_range
        self.sonar_left.max_range = self.max_range
        self.sonar_left.header = header_l

        header_r = std_msgs.msg.Header()
        header_r.stamp = rospy.Time.now()
        header_r.frame_id = 'sonar_right_link'
        self.sonar_right = Range()
        self.sonar_right.range = 0
        self.sonar_right.radiation_type = self.radiation_type
        self.sonar_right.field_of_view = self.field_of_view
        self.sonar_right.min_range = self.min_range
        self.sonar_right.max_range = self.max_range
        self.sonar_right.header = header_r
        pass


if __name__ == '__main__':
    rospy.init_node('tb3_sensor_to_sonar', anonymous=True)
    try:
        sonar = TB3_sensor_to_sonar()
        while not rospy.is_shutdown():
            sonar.rate.sleep()
        pass
    except rospy.ROSInterruptException:
        pass
