#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped

from styx_msgs.msg import Lane, Waypoint

import math, sys

# number of waypoints published by /waypoint_updater
LOOKAHEAD_WPS = 200

class WaypointUpdater(object):
    def __init__(self):
        # initializes ros node /waypoint_updater
        rospy.init_node('waypoint_updater')
        
        # topic /current_pose comes from bridge.py
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb,
                         queue_size = 1)
        
        # topic /base_waypoints comes from waypoint_loader.py
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb,
                         queue_size = 1)

        # create an instance of rospy.Publisher
        self.final_waypoints_publisher = rospy.Publisher('/final_waypoints', Lane, 
                                                    queue_size=1)

        self.current_pose = None
        self.base_waypoints = None
        self.final_waypoints = None
        
        self.current_closest_seq = None
        self.previous_pose_timestamp = 0

        rospy.spin()

    def current_pose_cb(self, msg):
        # self.current_pose = /current_pose - header
        self.current_pose = msg.pose
        current_pose_timestamp = msg.header.stamp.secs*1000000 + msg.header.stamp.nsecs
        time_difference = current_pose_timestamp - self.previous_pose_timestamp
        # will update only if the time difference is becoming greater than 1 second
        if time_difference > 1000000:
            self.previous_pose_timestamp = current_pose_timestamp
            self.togo()
            
    def base_waypoints_cb(self, msg):
        # self.base_waypoints = /base_waypoints - header
        self.base_waypoints = msg.waypoints
              
    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)
        
    def get_closest_waypoint(self):
        shortest_dist = float(sys.maxint)
        closest_seq = 0
        if self.base_waypoints:
            for waypoint in self.base_waypoints[::]:
                dist = self.distance(self.current_pose.position, waypoint.pose.pose.position)
                if dist < shortest_dist:
                    # how often this is updated?
                    # whenever there is new /current_pose
                    shortest_dist = dist;
                    closest_seq = waypoint.pose.header.seq
        return closest_seq
            
    def get_final_waypoints(self):
        # 1, 2, 3, 4, 5
        # len = 5 ... seq = 2 ... lookahead = 2
        # 2
        self.current_closest_seq = self.get_closest_waypoint()
        if self.base_waypoints:
            # because python index is left open and right close ... ( ]
            idx = self.current_closest_seq - 1
            if (idx + LOOKAHEAD_WPS) < len(self.base_waypoints):
                return self.base_waypoints[idx:idx+LOOKAHEAD_WPS]
            
    def togo(self):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = self.get_final_waypoints()
        self.final_waypoints_publisher.publish(lane)
        
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
