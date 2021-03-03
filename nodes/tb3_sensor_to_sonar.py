#!/usr/bin/env python3
#
# tb3_sensor_to_sonar.py
# ################################################################################
# HHT from W-HS, 21.01.2021
#
# Konvertieren von Sonar-Daten aus Turtlebot3 sensor_state Topic
# in Topics sonar_left und sonar_right als Range Message mit Median-Filter
# ------------------------------------------------------------------

import rospy
import std_msgs.msg
from sensor_msgs.msg import Range
from turtlebot3_msgs.msg import SensorState
from math import inf


class TB3_sensor_to_sonar():

    # Parameter
    radiation_type = 0
    field_of_view = 0.33
    min_range = 0.05
    max_range = 0.6
    sonar_l_array = []
    sonar_l_index = 0
    sonar_r_array = []
    sonar_r_index = 0
    sonar_max_index = 2
    sonar_median_index = 1

    # Init Funktion. Erstellt Publisher und Subscriber
    # Erstellt die Listen fuer den Filter und die ROS Nachrichten
    def __init__(self):
        self.rate = rospy.Rate(50)

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

        for x in range(self.sonar_max_index + 1):
            self.sonar_l_array.append(inf)
            self.sonar_r_array.append(inf)
            pass

        self.create_sonar_msgs()
        self.pub_sonar()
        pass

    # Publisher Funktion. Sendet sonar_left und sonar_right
    def pub_sonar(self):
        self.sonar_left_pub.publish(self.sonar_left)
        self.sonar_right_pub.publish(self.sonar_right)
        pass

    # Median Funktion. Fuegt einen Wert der Liste hinzu.
    # Sortiert eine Kopie der Liste und gibt den mittleren Wert
    # sowie den inkremetierten Zeiger zurueck.
    def median(self, sonar_array, sonar_index, sonar_value):
        sonar_array[sonar_index] = sonar_value
        print(f"Rolling Window\n{sonar_arra# inkremetiert den Zeiger auf die Liste. _index], sonar_index
        pass

    # Funktion prueft den Sensorwert und gibt entsprechend der
    # Grenzen einen angepassten Wert zurueck
    def validate_sonar(self, sonar):
        if sonar == 0:
            sonar = inf
        elif sonar < (self.min_range*100):
            sonar = self.min_range
        elif sonar > (self.max_range*100):
            sonar = self.max_range
        else:
            sonar = sonar / 100
        return sonar

    # Callback Funktion fuer das sensor_state Topic
    # Schreibt die gefilterten Werte in die ROS Nachrichten,
    # aktuallisiert den Zeitstempel, und ruft die Publisher Funktion
    def get_sonar(self, sensor_state):
        retval = self.validate_sonar(sensor_state.cliff)
        retval = self.median(self.sonar_l_array, self.sonar_l_index, retval)
        self.sonar_left.range, self.sonar_l_index = retval
        print(retval)
        retval = self.validate_sonar(sensor_state.sonar)
        retval = self.median(self.sonar_r_array, self.sonar_r_index, retval)
        self.sonar_right.range, self.sonar_r_index = retval
        print(retval)

        self.sonar_left.header.stamp = rospy.Time.now()
        self.sonar_right.header.stamp = rospy.Time.now()
        self.pub_sonar()
        pass

    # Funktion zum Erstellen der ROS Nachrichten
    def create_sonar_msgs(self):
        header_l = std_msgs.msg.Header()
        header_l.stamp = rospy.Time.now()
        header_l.frame_id = 'base_sonar_front_left'
        self.sonar_left = Range()
        self.sonar_left.range = 0
        self.sonar_left.radiation_type = self.radiation_type
        self.sonar_left.field_of_view = self.field_of_view
        self.sonar_left.min_range = self.min_range
        self.sonar_left.max_range = self.max_range
        self.sonar_left.header = header_l

        header_r = std_msgs.msg.Header()
        header_r.stamp = rospy.Time.now()
        header_r.frame_id = 'base_sonar_front_right'
        self.sonar_right = Range()
        self.sonar_right.range = 0
        self.sonar_right.radiation_type = self.radiation_type
        self.sonar_right.field_of_view = self.field_of_view
        self.sonar_right.min_range = self.min_range
        self.sonar_right.max_range = self.max_range
        self.sonar_right.header = header_r
        pass


# Main Funktion
if __name__ == '__main__':
    rospy.init_node('tb3_sensor_to_sonar', anonymous=True)
    try:
        sonar = TB3_sensor_to_sonar()
        while not rospy.is_shutdown():
            sonar.rate.sleep()
        pass
    except rospy.ROSInterruptException:
        pass
