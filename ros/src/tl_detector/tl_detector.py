#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np   # apr24tue2018jz0020
import math   # apr24tue2018jz0020

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        # apr24tue2018jz0028
        self.bridge = CvBridge()
        self.light_classifier = None   
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()
        
        # apr24tue2018jz0028
        # a dry run test
        img_full_np = self.light_classifier.load_image(np.zeros((800,600,3)))
        self.light_classifier.detect_frame(img_full_np)
        
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        # apr24tue2018jz0024
        #sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=28800000) # for 10 raw 800x600x3 uint8 frames 

        # apr24tue2018jz0043
        # load eight stop lines positions [x,y]
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        # apr25wed2018jz0232
        # not msg.pose ... 
        # because the msg's time stamp can be useful
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # apr24tue2018jz1837
        # self.waypoints.pose or self.waypoints.twist has time stamp inside
        #self.waypoints = waypoints
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # apr24tue2018jz2124
        # for true state of traffic lines
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        # apr24tue2018jz0614
        # a new image coming in
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    # apr23mon2018manali----
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_distance = 1000000   # apr24tue2018jz0419 ... just a large number

        # apr24tue2018jz0421
        #closest_idx = -1
        closest_idx = None

        pose_x = pose.position.x
        pose_y = pose.position.y

        # apr24tue2018jz0416
        # unused stuff
        #orientation = pose.orientation
        #euler = tf.transformations.euler_from_quaternion(
        #                        [orientation.x,
        #                        orientation.y,
        #                        orientation.z,
        #                        orientation.w])
        #yaw = euler[2]

        if self.waypoints is not None:
            # apr24tue2018jz0425
            #for i in range(len(self.waypoints.waypoints)):
            #    wp_x = self.waypoints.waypoints[i].pose.pose.position.x
            #    wp_y = self.waypoints.waypoints[i].pose.pose.position.y

            for i in range(len(self.waypoints)):
                wp_x = self.waypoints[i].pose.pose.position.x
                wp_y = self.waypoints[i].pose.pose.position.y
			
                distance = math.sqrt((pose_x - wp_x)**2 + (pose_y - wp_y)**2)

                # apr24tue2018jz0416
                # the following codes are not used
                ## Since we want the closest waypoint ahead, we need to calculate 
                ## the angle between the car and the waypoint
                #psi = np.arctan2(pose_y - wp_y, pose_x - wp_x)
                #dtheta = np.abs(psi - yaw)

                if distance < closest_distance:
                    closest_distance = distance
                    closest_idx = i
        return closest_idx
        #closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        #return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        
        # apr24tue2018jz0046
        #Convert image to RGB format
        processed_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        #convert image to np array
        img_full_np = self.light_classifier.load_image(processed_img)

        # apr24tue2018jz0048
        unknown = False
        light_state = None

        # apr24tue2018jz0048
        print("detection begins")
        b, conf, cls_idx = self.light_classifier.detect_frame(img_full_np)
        print("detection ends")
        if np.array_equal(b, np.zeros(4)):
            print ('unknown')
            print(" ")
            unknown = True
        else:
            if cls_idx == 1.0:
                light_state = TrafficLight.GREEN
                rospy.logwarn("upcoming light is green with state %s", light_state)
                print(" ")
            elif cls_idx == 2.0:
                light_state = TrafficLight.RED
                rospy.logwarn("upcoming light is red with state %s", light_state)
                print(" ")
            elif cls_idx == 3.0:
                light_state = TrafficLight.YELLOW
                rospy.logwarn("upcoming light is yellow with state %s", light_state)
                print(" ")
            elif cls_idx == 4.0:
                light_state = TrafficLight.UNKNOWN
                rospy.logwarn("unknown")
                print(" ")
            else:
                light_state = TrafficLight.UNKNOWN
                rospy.logwarn("unknown")
                print(" ")

        # apr24tue2018jz0048
        return light_state

        # apr24tue2018jz0048
        #Get classification
        #return self.light_classifier.get_classification(cv_image)


    # apr23mon2018gaurav----
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        #line_wp_idx = None   # idx for stop line ... apr24tue2018jz0618 

        # apr24tue2018jz0618
        # list of positions that correspond to the line to stop 
        # in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        # apr24tue2018jz0619
        # check if the pose of the car is available
        if (self.pose):
            # apr24tue2018jz0407
            # incompatible parameters
            #car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            car_wp_idx = self.get_closest_waypoint(self.pose.pose)

            # apr24tue2018jz0112
            # check car's current position
            rospy.loginfo("current car position: %d", car_wp_idx)
            print(" ")

            # apr24tue2018jz1840
            #diff = len(self.waypoints.waypoints)
            #diff = len(self.waypoints)

            # apr24tue2018jz2003
            # iterate over 8 traffic lights
            #for i, light in enumerate(self.lights):

            # apr25wed2018jz2224
            for stop_line_position in stop_line_positions:
                #line = stop_line_positions[i]
                #stop_line = stop_line_positions[i]

                # apr24tue2018jz0408
                # convert to Pose ros msg
                stop_line_pose = Pose()

                # apr24tue2018jz2015
                #stop_line_pose.position.x = stop_line[0]
                #stop_line_pose.position.y = stop_line[1]
                stop_line_pose.position.x = stop_line_position[0]
                stop_line_pose.position.y = stop_line_position[1]
                #temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                temp_wp_idx = self.get_closest_waypoint(stop_line_pose)

                # apr24tue2018jz0623
                #d = temp_wp_idx - car_wp_idx

                #if d >= 0 and d < diff:
                # check if the stop line is ahead of the car 
                if temp_wp_idx >= car_wp_idx:
                    if closest_light is None:   # assign initial values
                        closest_light = temp_wp_idx
                    elif temp_wp_idx < closest_light:
                        closest_light = temp_wp_idx
                        
            # apr25wed2018jz2244
            #rospy.loginfo("closest stop line: %d", closest_light)
            rospy.loginfo("closest stop line: %s", closest_light)
            print(" ")


        if closest_light:
            state = self.get_light_state(closest_light)
            return closest_light, state

        return -1, TrafficLight.UNKNOWN

        #light = None

        ## List of positions that correspond to the line to stop in front of for a given intersection
        #stop_line_positions = self.config['stop_line_positions']
        #if(self.pose):
        #    car_position = self.get_closest_waypoint(self.pose.pose)

        ##TODO find the closest visible traffic light (if one exists)

        #if light:
        #    state = self.get_light_state(light)
        #    return light_wp, state
        #self.waypoints = None
        #return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
