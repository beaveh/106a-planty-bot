################################################################################
#
# OccupancyGrid2d class listens for LaserScans and builds an occupancy grid.
#
################################################################################

import rospy
import tf2_ros
import tf
import math

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist
import numpy as np

class OccupancyGrid2d(object):
    def __init__(self):
        self._intialized = False

        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self.counter = 0
        self.x = 0.5

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/grid_map_2d"

        # Load parameters.
        if not self.LoadParameters():
            # rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        if not self.RegisterCallbacks():
            # rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        # Set up the map.
        self._map = np.zeros((self._x_num, self._y_num))

        self._initialized = True
        return True

    def LoadParameters(self):
        # Random downsampling fraction, i.e. only keep this fraction of rays.
        if not rospy.has_param("~random_downsample"):
            return False
        self._random_downsample = rospy.get_param("~random_downsample")

        if not rospy.has_param("~x/num"):
            return False
        self._x_num = rospy.get_param("~x/num")

        if not rospy.has_param("~x/min"):
            return False
        self._x_min= rospy.get_param("~x/min")

        if not rospy.has_param("~x/max"):
            return False
        self._x_max = rospy.get_param("~x/max")


        if not rospy.has_param("~y/num"):
            return False
        self._y_num = rospy.get_param("~y/num")

        if not rospy.has_param("~y/min"):
            return False
        self._y_min = rospy.get_param("~y/min")


        if not rospy.has_param("~y/max"):
            return False
        self._y_max = rospy.get_param("~y/max")

        self._x_res = (self._x_max - self._x_min) / self._x_num
        self._y_res = (self._y_max - self._y_min) / self._y_num

       
        if not rospy.has_param("~topics/vis"):
            return False
        self._vis_topic = rospy.get_param("~topics/vis")


        if not rospy.has_param("~topics/sensor"):
            return False
        self._sensor_topic = rospy.get_param("~topics/sensor")


        if not rospy.has_param("~frames/fixed"):
            return False
        self._fixed_frame = rospy.get_param("~frames/fixed")


        if not rospy.has_param("~frames/sensor"):
            return False
        self._sensor_frame = rospy.get_param("~frames/sensor")

        return True

    def RegisterCallbacks(self):
        # Subscriber.
        self._sensor_sub = rospy.Subscriber(self._sensor_topic,
                                            LaserScan,
                                            self.SensorCallback,
                                            queue_size=1)

        # Publisher.
        self._vis_pub = rospy.Publisher(self._vis_topic,
                                        Marker,
                                        queue_size=10)

        self.scan_pub = rospy.Publisher('/Scan1', LaserScan, queue_size=10)

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


        return True
    def fnGoStraight(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

    def fnTurn(self, theta):
        Kp = 0.6

        angular_z = Kp * theta

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

    # Callback to process sensor measurements.
    def SensorCallback(self, msg):

      

        
        tempr = []

        maxi = len(msg.ranges) - 10
        
        for idx, r in enumerate(msg.ranges):
            if idx < 10: 
                tempr.append(r)
            elif idx > 10 and idx < maxi:
                tempr.append(0.0)
            else:
                tempr.append(r)



        
        check = filter(lambda x: x < 0.5 and x != 0, tempr)
      
        if len(list(check)) == 0:
            self.fnGoStraight()
        else:
            self.counter += 1
            self.fnTurn(self.x)
            if self.counter > 50:
                self.x = self.x * -1
                self.counter = -50
           


        msg.ranges = tempr
        msg.range_max = 0.4
       
        self.scan_pub.publish(msg)

        








        if not self._initialized:
            rospy.logerr("%s: Was not initialized.", self._name)
            return



        # Get our current pose from TF.
        try:
            pose = self._tf_buffer.lookup_transform(
                self._fixed_frame, self._sensor_frame, rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # Writes an error message to the ROS log but does not raise an exception
            # rospy.logerr("%s: Could not extract pose from TF.", self._name)
            return

        # Extract x, y coordinates and heading (yaw) angle of the turtlebot, 
        # assuming that the turtlebot is on the ground plane.
        sensor_x = pose.transform.translation.x
        sensor_y = pose.transform.translation.y
        if abs(pose.transform.translation.z) > 0.05:
            rospy.logwarn("%s: Turtlebot is not on ground plane.", self._name)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [pose.transform.rotation.x, pose.transform.rotation.y,
             pose.transform.rotation.z, pose.transform.rotation.w])
        if abs(roll) > 0.1 or abs(pitch) > 0.1:
            rospy.logwarn("%s: Turtlebot roll/pitch is too large.", self._name)

        threshold_dist = 3

                # Loop over all ranges in the LaserScan.
        for idx, r in enumerate(msg.ranges):

            # Randomly throw out some rays to speed this up.
            if np.random.rand() > self._random_downsample:
                continue
            elif np.isnan(r):
                continue


            # Get angle of this ray in fixed frame.
            # TODO!
            angle_robot = msg.angle_min + idx * msg.angle_increment
            angle_frame = angle_robot + yaw

            if r > threshold_dist:
                rospy.logwarn("range is hit")



            # Throw out this point if it is too close or too far away.
            if r > msg.range_max:
                rospy.logwarn("%s: Range %f > %f was too large.",
                              self._name, r, msg.range_max)
                continue
            if r < msg.range_min:
                rospy.logwarn("%s: Range %f < %f was too small.",
                              self._name, r, msg.range_min)
                continue