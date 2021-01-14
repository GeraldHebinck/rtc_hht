#!/usr/bin/env python3
# edited by GH, 14.01.2021
#
# sonar_to_costmap.py
# ################################################################################
# edited WHS, OJ , 23.12.2020 #
#
# brings Sonar detected Obstacles into move_base local costmap
# using point_cloud - message
#
# edit
# copy content of turtlebot3.burger.gazebo_sonar.xacro
#              to turtlebot3.burger.gazebo_sonar.xacro
# copy content of turtlebot3.burger.urdf_sonar.xacro
#              to turtlebot3.burger.urdf.xacro
# edit costmap_common_params_burger.yaml
#    observation_sources: scan sonar
#    scan: ...
#    sonar: {sensor_frame: base_link, data_type: PointCloud,
#             topic: /sonar/point_cloud, marking: true, clearing: true}
#
# edit move_base.launch  => /cmd_vel to /move_base/cmd_vel
#     <arg name="cmd_vel_topic" default="/move_base/cmd_vel" />
#
# usage
#   $1 roslaunch turtlebot3_gazebo turtlebot3_house.launch
#   $2 roslaunch turtlebot3_navigation turtlebot3_navigation.launch
#                map_file:=$HOME/catkin_ws/src/rtc/rtc_maps/gazebo_house_map_2020_12_07.yaml
#   $3 roslaunch rts sonar_twist_mux.launch
#   $4 rosrun rtc sonar_obstacle_avoidance.py
#   $5 rosrun rtc sonar_to_costmap.py
# ------------------------------------------------------------------

import rospy
import std_msgs.msg
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointCloud


class Sonar_to_Point_Cloud():
    def __init__(self):
        rospy.loginfo("Publishing PointCloud")

        self.cloud_pub = rospy.Publisher('sonar/point_cloud',
                                         PointCloud,
                                         queue_size=10)

        # receiving sonar
        # check for "real Topic!"
        self.sonar_sub = rospy.Subscriber('sonar',
                                          Range,
                                          self.get_sonar,
                                          queue_size=10)

        self.dist = 0.0
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.rate.sleep()

    def get_sonar(self, sensor_data):
        # rospy.loginfo(" Sonar Data Left received ")
        self.dist = sensor_data.range
        self.cloud_build()

    def cloud_build(self):
        # add sonar readings (robot-local coordinate frame) to cloud
        pl = Point32()
        pm = Point32()
        pr = Point32()
        # Instanziiere leere PointCloud
        cloud = PointCloud()
        # filling pointcloud header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'
        cloud.header = header

        # Linke Seite
        if(self.dist < 0.30 and self.dist > 0.05):
            pl.x = self.dist + 0.05
            pl.y = 0.05
            pl.z = 0.0
            cloud.points.append(pl)

            pm.x = self.dist + 0.05
            pm.y = 0.0
            pm.z = 0.0
            cloud.points.append(pm)

            pr.x = self.dist + 0.05
            pr.y = -0.05
            pr.z = 0.0
            cloud.points.append(pr)

        # Senden
        self.cloud_pub.publish(cloud)


if __name__ == '__main__':
    rospy.init_node('sonar_to_pointcloud', anonymous=True)
    try:
        sonar = Sonar_to_Point_Cloud()
    except rospy.ROSInterruptException:
        pass
