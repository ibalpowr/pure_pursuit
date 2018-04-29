#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from tf.transformations import euler_from_quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight
from std_msgs.msg import Int32
import math, sys

# Number of waypoints published by /waypoint_updater
LOOKAHEAD_WPS = 100
DEBUG = True

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb,
                         queue_size = 1)
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb,
                         queue_size = 1)
        if DEBUG:
            rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.lights_cb)
        else:
            rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_publisher = rospy.Publisher('/final_waypoints',
                                                         Lane, queue_size=1)

        self.decel_limit = abs(rospy.get_param('~decel_limit', -5))

        self.current_pose = None
        self.base_waypoints = None
        self.final_waypoints = None
        self.current_closest_seq = None
        self.previous_pose_timestamp = 0
        self.stopline_wp_idx = -1

        # Debugging parameters
        self.lights = None
        self.light_wp_idx = None
        self.num_lights = None
        self.closest_light_idx = 0

        self.loop()

    # Debugging functions
    def initialize_lights_wp_ind(self):
        if self.base_waypoints and self.lights:
            self.num_lights = len(self.lights)
            self.light_wp_idx = [None] * self.num_lights
            for i, light in enumerate(self.lights):
                self.light_wp_idx[i] = self.get_closest_idx(light.pose.pose, self.base_waypoints)

    def update_closest_light(self, current_seq):
        if self.light_wp_idx and current_seq > self.light_wp_idx[self.closest_light_idx]:
            self.closest_light_idx += 1
            self.closest_light_idx %= self.num_lights

    def lights_cb(self, msg):
        if not self.light_wp_idx:
            self.lights = msg.lights
            self.initialize_lights_wp_ind()

        self.lights = msg.lights

    # Call back functions
    def traffic_cb(self, msg):
        self.stop_line_wp_idx = msg.data

    def current_pose_cb(self, msg):
        self.current_pose = msg.pose

    def base_waypoints_cb(self, msg):
        self.base_waypoints = msg.waypoints

    # Utility functions
    def get_closest_idx(self, current, points):
        shortest_dist = float(sys.maxint)
        closest_idx = 0
        if points:
            for index, point in enumerate(points):
                dist = self.distance(current.position,
                                     point.pose.pose.position)
                if dist < shortest_dist:
                    shortest_dist = dist
                    closest_idx = index
        return closest_idx

    def get_closest_waypoint(self):
        self.current_closest_seq = self.get_closest_idx(self.current_pose, self.base_waypoints)

        closest_waypoint_x = self.base_waypoints[self.current_closest_seq].pose.pose.position.x
        closest_waypoint_y = self.base_waypoints[self.current_closest_seq].pose.pose.position.y

        pose_x = self.current_pose.position.x
        pose_y = self.current_pose.position.x

        heading = math.atan2(closest_waypoint_y - pose_y,
                             closest_waypoint_x - pose_x)
        _, _, yaw = euler_from_quaternion(np.array([self.current_pose.orientation.x,
                                                    self.current_pose.orientation.y,
                                                    self.current_pose.orientation.z,
                                                    self.current_pose.orientation.w]))
        if (abs(yaw-heading) > (math.pi / 4)):
            self.current_closest_seq += 2

        return self.current_closest_seq

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def wp_distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance(self, p1, p2):
        x, y = p1.x - p2.x, p1.y - p2.y
        return math.sqrt(x*x + y*y)

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            dist = self.wp_distance (waypoints, i, stop_idx)
            vel = math.sqrt(2 * self.decel_limit * dist)
            if vel < 1.0:
                vel = 0.

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp

    def get_final_waypoints(self):
        self.current_closest_seq = self.get_closest_waypoint()
        #print "Closest WP IDX: {}".format(self.current_closest_seq)

        if DEBUG:
            self.stopline_wp_idx = -1
            self.update_closest_light(self.current_closest_seq)
            if self.lights and self.light_wp_idx and self.lights[self.closest_light_idx].state == 0:
                self.stopline_wp_idx = self.light_wp_idx[self.closest_light_idx] - 30
            #print "Next Red Light WP IDX: {}".format(self.stopline_wp_idx)

        farthest_index = self.current_closest_seq + LOOKAHEAD_WPS
        b_waypoints = self.base_waypoints[self.current_closest_seq:farthest_index]

        if self.stopline_wp_idx == -1 or self.stopline_wp_idx >= farthest_index:
            return b_waypoints
        else:
            #print "Decelerating..."
            return self.decelerate_waypoints(b_waypoints,
                                             self.current_closest_seq)

    def publish_waypoints(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_publisher.publish(lane)

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_pose and self.base_waypoints:
                waypoints = self.get_final_waypoints()
                self.publish_waypoints(waypoints)
            rate.sleep()

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
