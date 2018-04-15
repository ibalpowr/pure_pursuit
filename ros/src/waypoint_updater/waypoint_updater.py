#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped

from styx_msgs.msg import Lane, Waypoint

from tf.transformations import euler_from_quaternion

import math, sys

# number of waypoints published by /waypoint_updater
LOOKAHEAD_WPS = 50
MAX_DECEL = 0.5

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

	#rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # create an instance of rospy.Publisher
        self.final_waypoints_publisher = rospy.Publisher('/final_waypoints', Lane, 
                                                    queue_size=1)




        self.current_pose = None
        self.base_waypoints = None
        self.final_waypoints = None
        
        self.current_closest_seq = None
        self.previous_pose_timestamp = 0
	self.stopline_wp_idx = -1

        rospy.spin()

    def traffic_cb(self, msg):
	self.stopline_wp_idx = msg.data
	

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
	waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
	return dist

    def current_pose_cb(self, msg):
        # self.current_pose = /current_pose - header
        self.current_pose = msg.pose
        current_pose_timestamp = msg.header.stamp.secs*1000000 + msg.header.stamp.nsecs
        time_difference = current_pose_timestamp - self.previous_pose_timestamp
        # will update only if the time difference is becoming greater than 1 second
#        if time_difference > 1000000:
#            self.previous_pose_timestamp = current_pose_timestamp
        self.togo()
            
    def base_waypoints_cb(self, msg):
        # self.base_waypoints = /base_waypoints - header
        self.base_waypoints = msg.waypoints
              
    def distance(self, p1, p2):
#        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
	x, y = p1.x - p2.x, p1.y - p2.y
        return math.sqrt(x*x + y*y )
        
    def get_closest_waypoint(self):
        shortest_dist = float(sys.maxint)
        closest_seq = 0
        if self.base_waypoints:
#       	for waypoint in self.base_waypoints[::]:
		for index, waypoint in enumerate(self.base_waypoints):
	                dist = self.distance(self.current_pose.position, waypoint.pose.pose.position)
		        if dist < shortest_dist:
			    # how often this is updated?
			    # whenever there is new /current_pose
			    shortest_dist = dist;
#		            closest_seq = waypoint.pose.header.seq
			    closest_seq = index

	rospy.loginfo('CLOSEST_IDX : %d', closest_seq)
        return closest_seq

    def decelerate_waypoints(self, waypoints, closest_idx):
	temp = []

	for i, wp in enumerate(waypoints):
		p = Waypoint()
		p.pose = wp.pose

		stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
		dist = self.distance (waypoints, i, stop_idx)
		vel = math.sqrt(2 * MAX_DECEL * dist)
		if vel < 1.0:
			vel = 0.

		p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
		temp.append(p)
	return temp
            
    def get_final_waypoints(self):
        # 1, 2, 3, 4, 5
        # len = 5 ... seq = 2 ... lookahead = 2
        # 2

	lane = Lane()

        self.current_closest_seq = self.get_closest_waypoint()
	
	closest_waypoint_x = self.base_waypoints[self.current_closest_seq].pose.pose.position.x
	closest_waypoint_y = self.base_waypoints[self.current_closest_seq].pose.pose.position.y

	pose_x = self.current_pose.position.x
	pose_y = self.current_pose.position.x

	heading = math.atan2(closest_waypoint_y - pose_y, closest_waypoint_x - pose_x)
	(_, _, yaw) = euler_from_quaternion(np.array([self.current_pose.orientation.x,
						      self.current_pose.orientation.y,
						      self.current_pose.orientation.z,
						      self.current_pose.orientation.w]))

	if (abs(yaw-heading) > (math.pi / 4)):
		self.current_closest_seq += 1

	farthest_index = self.current_closest_seq + LOOKAHEAD_WPS

	b_waypoints = self.base_waypoints[self.current_closest_seq:farthest_index]

	if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >=farthest_index):
		return b_waypoints
	else:
		return self.decelerate_waypoints(b_waypoints, self.current_closest_seq)


#        if self.base_waypoints:
            # because python index is left open and right close ... ( ]
#            idx = self.current_closest_seq - 1
#            if (idx + LOOKAHEAD_WPS) < len(self.base_waypoints):
#                return self.base_waypoints[idx:idx+LOOKAHEAD_WPS]
#		return self.base_waypoints[self.current_closest_seq:self.current_closest_seq+LOOKAHEAD_WPS]
            
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
