#!/usr/bin/env python

import rospy
# apr14sat2018gary----
import numpy as np

from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

# apr14sat2018gary----
from tf.transformations import euler_from_quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight

import math, sys

# number of waypoints published by /waypoint_updater
# apr14sat2018gary----
# apr30mon2018jz1107
# need longer lookahead for sharp curve and stopping early
# may01tue20180703
# set it back to 100 for monitor_joe.py
#LOOKAHEAD_WPS = 200
LOOKAHEAD_WPS = 100

# apr28sat2018karim----
#DEBUG = True
DEBUG = False

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

        # apr30mon2018jz0145
        # subscribing both is o.k.
        # apr28sat2018karim----
        #if DEBUG:
        #    rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.lights_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.lights_cb)
        #else:
        #    rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # apr26thur2018jz0007
        # published from tl_detector.py
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size = 1)

        # create an instance of rospy.Publisher
        self.final_waypoints_publisher = rospy.Publisher('/final_waypoints', Lane, 
                                                    queue_size=1)
        # apr29sun2018jz0808
        # the value is set in dbw_sim.launch
        # for red traffic light
        self.decel_limit = abs(rospy.get_param('~decel_limit', -5))
        # may04fri2018jz0404
        # a bit more decel
        #self.decel_limit = abs(rospy.get_param('~decel_limit', -3))

        self.current_pose = None
        self.base_waypoints = None
        self.final_waypoints = None
        # apr30monp2018jz0810
        #self.stop_line_wp = None
        self.stopline_wp_idx = -1
        
        self.current_closest_seq = None
        # apr28sat2018jz2035
        #self.previous_pose_timestamp = 0

        # apr30mon2018jz0310
        # comment them out by now
        # apr28sat2018karim----
        # Debugging parameters
        #self.lights = None
        #self.light_wp_idx = None
        #self.num_lights = None
        #self.closest_light_idx = 0

        # apr14sat2018gary----
        #rospy.spin()
        
        # may04fri2018jz0237
        # better control the final waypoint publishing rate
        #self.loop()        
        #rate = rospy.Rate(1)   # 1Hz
        # may05sat2018jz0318
        # for better stopping
        rate = rospy.Rate(2)   # 2Hz
        while not rospy.is_shutdown():
            if self.current_pose and self.base_waypoints:
                waypoints = self.get_final_waypoints()
                self.publish_waypoints(waypoints)
                rospy.loginfo("final waypoints published")
            rate.sleep()

    # apr30mon2018jz0309
    # comment them out by now
    # apr28sat2018karim----
    ### Debugging functions
    #def initialize_lights_wp_ind(self):
    #    if self.base_waypoints and self.lights:
    #        self.num_lights = len(self.lights)
    #        self.light_wp_idx = [None] * self.num_lights
    #        for i, light in enumerate(self.lights):
    #            self.light_wp_idx[i] = self.get_closest_idx(light.pose.pose, 
    #                                                       self.base_waypoints)

    #def update_closest_light(self, current_seq):
    #    if self.light_wp_idx and 
    #                        current_seq > self.light_wp_idx[self.closest_light_idx]:
    #        self.closest_light_idx += 1
    #        self.closest_light_idx %= self.num_lights

    # Utility functions
    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

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

    def current_pose_cb(self, msg):
        # self.current_pose = /current_pose - header
        self.current_pose = msg.pose
        # apr14sat2018gary----
        #current_pose_timestamp = msg.header.stamp.secs*1000000 + msg.header.stamp.nsecs
        #time_difference = current_pose_timestamp - self.previous_pose_timestamp
        ## will update only if the time difference is becoming greater than 1 second
        #if time_difference > 1000000:
        #    self.previous_pose_timestamp = current_pose_timestamp
        #    self.togo()
            
    def base_waypoints_cb(self, msg):
        # self.base_waypoints = /base_waypoints - header
        self.base_waypoints = msg.waypoints

        # apr27fri2018jz2212
        # verify the loading of base waypoints
        print("the length of base waypoints: ", len(self.base_waypoints))
        print(" ")
        rospy.logwarn("first velocity: %f", self.base_waypoints[0].twist.twist.linear.x)
        rospy.logwarn("last velocity: %f", self.base_waypoints[-1].twist.twist.linear.x)
        rospy.logwarn("[10901]: %f", self.base_waypoints[10901].twist.twist.linear.x)
        rospy.loginfo("last sequence id: %s", self.base_waypoints[-1].pose.header.seq)
        
    # apr25wed2018jz0208
    def traffic_cb (self, msg):
        # apr30mon2018jz0809
        #self.stop_line_wp = msg.data
        self.stopline_wp_idx = msg.data
        
        # verify the message
        #print("waypoint updater receives stop line wp as ", self.stop_line_wp)
        print("waypoint updater receives stop line wp as ", self.stopline_wp_idx)
        print(" ")

    # apr30mon2018jz0305
    # comment it out by now
    # apr28sat2018karim----
    def lights_cb(self, msg):
    #    if not self.light_wp_idx:
    #        self.lights = msg.lights
    #        self.initialize_lights_wp_ind()

    #    self.lights = msg.lights
        pass 
 
    # apr14sat2018gary----   
    #def get_closest_waypoint(self):
    #    shortest_dist = float(sys.maxint)
    #    closest_seq = 0
    #    if self.base_waypoints:
    #        for waypoint in self.base_waypoints[::]:
    #            dist = self.distance(self.current_pose.position, waypoint.pose.pose.position)
    #            if dist < shortest_dist:
    #                # how often this is updated?
    #                # whenever there is new /current_pose
    #                shortest_dist = dist;
    #                closest_seq = waypoint.pose.header.seq
    #    return closest_seq
 
    # apr28sat2018karim----
    def get_closest_waypoint(self):
        # apr30mon2018jz0737
        # name conflict in get_final_waypoints()
        #self.current_closest_seq = self.get_closest_idx(self.current_pose, 
        #                                                self.base_waypoints)
        closest_seq = 0
        closest_seq = self.get_closest_idx(self.current_pose, 
                                           self.base_waypoints)
                                           
        # apr30mon2018jz0741
        # apr14sat2018gary----
        #closest_waypoint_x = self.base_waypoints[self.current_closest_seq].pose.pose.position.x
        #closest_waypoint_y = self.base_waypoints[self.current_closest_seq].pose.pose.position.y
        closest_waypoint_x = self.base_waypoints[closest_seq].pose.pose.position.x
        closest_waypoint_y = self.base_waypoints[closest_seq].pose.pose.position.y

        pose_x = self.current_pose.position.x
        pose_y = self.current_pose.position.x

        heading = math.atan2(closest_waypoint_y - pose_y,
                             closest_waypoint_x - pose_x)
        _, _, yaw = euler_from_quaternion(np.array([self.current_pose.orientation.x,
                                                    self.current_pose.orientation.y,
                                                    self.current_pose.orientation.z,
                                                    self.current_pose.orientation.w]))
        if (abs(yaw-heading) > (math.pi / 4)):
            # apr30mon2018jz0718
            # ? why change 1 to 2
            # apr28sat2018karim----
            #self.current_closest_seq += 1
            #self.current_closest_seq += 2
            closest_seq += 2
            # may04fri2018jz0020
            rospy.logwarn("closest wp was behind")

        # apr30mon2018jz0743
        #return self.current_closest_seq
        return closest_seq

    # apr28sat2018karim----
    def wp_distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    # apr14sat2018gary----
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            # may03thur2018jz0531
            # the car head cross the stop line a bit ... even the tire is good
            #stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            #stop_idx = max(self.stopline_wp_idx - closest_idx - 5, 0)
            # may04fri2018jz0416
            # a bit more early
            stop_idx = max(self.stopline_wp_idx - closest_idx - 8, 0)
            dist = self.wp_distance (waypoints, i, stop_idx)
            # apr28sat2018karim----
            #vel = math.sqrt(2 * MAX_DECEL * dist)
            vel = math.sqrt(2 * self.decel_limit * dist)
            if vel < 1.0:
                vel = 0.

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp
           
    def get_final_waypoints(self):

        self.current_closest_seq = self.get_closest_waypoint()
        # apr30mon2018jz0241
        # check it
        #rospy.loginfo("current closest sequence #: %s", self.current_closest_seq)
        #print(" ")

        # apr14sat2018gary----
        #if self.base_waypoints:
        #    # because python index is left open and right close ... ( ]
        #    idx = self.current_closest_seq - 1
        #    if (idx + LOOKAHEAD_WPS) < len(self.base_waypoints):
        #        return self.base_waypoints[idx:idx+LOOKAHEAD_WPS]

        # apr30mon2018jz0322
        farthest_index = None

        farthest_index = self.current_closest_seq + LOOKAHEAD_WPS
        # apr30mon2018jz0320
        b_waypoints = self.base_waypoints[self.current_closest_seq:farthest_index]
        #return self.base_waypoints[self.current_closest_seq:farthest_index]

        # apr14sat2018gary----
        if self.stopline_wp_idx == -1 or self.stopline_wp_idx >= farthest_index:
            return b_waypoints
        else:
            print "Decelerating..."
            return self.decelerate_waypoints(b_waypoints,
                                             self.current_closest_seq)

    # apr14sat2018gary----            
    #def togo(self):
    #    lane = Lane()
    #    lane.header.frame_id = '/world'
    #    lane.header.stamp = rospy.Time(0)
    #    lane.waypoints = self.get_final_waypoints()
    #    self.final_waypoints_publisher.publish(lane)
    def publish_waypoints(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_publisher.publish(lane)

    # may04fri2018jz0239
    # to better control the final waypoint publishing rate ...
    # move the loop() inside the __init__
    #def loop(self):
    #    # apr28sat2018jz2045
    #    #rate = rospy.Rate(30) # 30Hz
    #    rate = rospy.Rate(10) # 10Hz
    #    
    #    # may03thur2018jz2326
    #    # to check how often /final_waypoints are published
    #    previous_timestamp = rospy.get_time()
    #    while not rospy.is_shutdown():
    #        
    #        if self.current_pose and self.base_waypoints:
    #            waypoints = self.get_final_waypoints()
    #            # may03thur2018jz2330
    #            current_timestamp = rospy.get_time()
    #            self.publish_waypoints(waypoints)
    #
    #            # may03thur2018jz2326
    #            # to check how often /final_waypoints are published
    #            rospy.logwarn("final waypoints publishing rate is: %f",
    #                           1.0/(current_timestamp - previous_timestamp))
    #           previous_timestamp = current_timestamp
    #    rate.sleep()

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
