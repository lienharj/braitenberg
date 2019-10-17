#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml

from duckietown import DTROS
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped

velocity = WheelsCmdStamped()

def brightness(self, image):

    image = cv2.imdecode(np.fromstring(image, np.uint8), cv2.IMREAD_COLOR)
    image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    val_left = image[:,0:159,2]
    val_right = image[:,160:319,2]
    diff = np.mean(val_left) - np.mean(val_right)
    limit = 15
    print diff
    if diff>limit:
        speed_l = 0.3
        speed_r = 1
        print "left"
    elif diff < -limit:
        speed_l = 1
        speed_r = 0.3
        print "right"
    else:
        speed_l = 0.8
        speed_r = 0.8
        print "straight"

    velocity.vel_left, velocity.vel_right = self.speedToCmd(speed_l, speed_r)
    print velocity.vel_left, velocity.vel_right
    velocity.header.stamp = rospy.get_rostime()
    self.pub.publish(velocity)


class BraitenbergNode(DTROS):
    """Braitenberg Behaviour

    This node implements Braitenberg vehicle behavior on a Duckiebot.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node
            that ROS will use

    Configuration:
        ~gain (:obj:`float`): scaling factor applied to the desired
            velocity, taken from the robot-specific kinematics
            calibration
        ~trim (:obj:`float`): trimming factor that is typically used
            to offset differences in the behaviour of the left and
            right motors, it is recommended to use a value that results
            in the robot moving in a straight line when forward command
            is given, taken from the robot-specific kinematics calibration
        ~baseline (:obj:`float`): the distance between the two wheels
            of the robot, taken from the robot-specific kinematics
            calibration
        ~radius (:obj:`float`): radius of the wheel, taken from the
            robot-specific kinematics calibration
        ~k (:obj:`float`): motor constant, assumed equal for both
            motors, taken from the robot-specific kinematics calibration
        ~limit (:obj:`float`): limits the final commands sent to the
            motors, taken from the robot-specific kinematics calibration

    Subscriber:
        ~image/compressed (:obj:`CompressedImage`): The acquired camera
            images

    Publisher:
        ~wheels_cmd (:obj:`duckietown_msgs.msg.WheelsCmdStamped`): The
            wheel commands that the motors will execute

    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(BraitenbergNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        # Use the kinematics calibration for the gain and trim
        self.parameters['~gain'] = None
        self.parameters['~trim'] = None
        self.parameters['~baseline'] = None
        self.parameters['~radius'] = None
        self.parameters['~k'] = None
        self.parameters['~limit'] = None

        # Set parameters using a robot-specific yaml file if such exists
        self.readParamFromFile()
        self.updateParameters()

        # Wait for the automatic gain control
        # of the camera to settle, before we stop it
        rospy.sleep(2.0)
        rospy.set_param('/%s/camera_node/exposure_mode' % self.veh_name, 'off')
        rospy.set_param('/%s/camera_node/res_w' %self.veh_name, 320)
        rospy.set_param('/%s/camera_node/res_h' %self.veh_name, 240)

        self.log("Initialized")

        self.sub = rospy.Subscriber('~/django/camera_node/image/compressed', CompressedImage , self.callback)
        self.pub = rospy.Publisher('~/django/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

    def callback(self, data):

        #rospy.loginfo("message received!")
        brightness(self, data.data)

    def speedToCmd(self, speed_l, speed_r):
        """Applies the robot-specific gain and trim to the
        output velocities

        Applies the motor constant k to convert the deisred wheel speeds
        to wheel commands. Additionally, applies the gain and trim from
        the robot-specific kinematics configuration.

        Args:
            speed_l (:obj:`float`): Desired speed for the left
                wheel (e.g between 0 and 1)
            speed_r (:obj:`float`): Desired speed for the right
                wheel (e.g between 0 and 1)

        Returns:
            The respective left and right wheel commands that need to be
                packed in a `WheelsCmdStamped` message

        """

        # assuming same motor constants k for both motors
        k_r = self.parameters['~k']
        k_l = self.parameters['~k']

        # adjusting k by gain and trim
        k_r_inv = (self.parameters['~gain'] + self.parameters['~trim'])\
                  / k_r
        k_l_inv = (self.parameters['~gain'] - self.parameters['~trim'])\
                  / k_l

        # conversion from motor rotation rate to duty cycle
        u_r = speed_r * k_r_inv * 6
        u_l = speed_l * k_l_inv * 6

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = self.trim(u_r,
                                -self.parameters['~limit'],
                                self.parameters['~limit'])
        u_l_limited = self.trim(u_l,
                                -self.parameters['~limit'],
                                self.parameters['~limit'])

        return u_l_limited, u_r_limited

    def readParamFromFile(self):
        """
        Reads the saved parameters from
        `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml` or
        uses the default values if the file doesn't exist. Adjsuts
        the ROS paramaters for the node with the new values.

        """
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use the default values from the config folder if a
        # robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.log("Kinematics calibration file %s does not "
                     "exist! Using the default file." % fname, type='warn')
            fname = self.getFilePath('default')

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["gain", "trim", "baseline", "k", "radius", "limit"]:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass

    def getFilePath(self, name):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.

        Args:
            name (:obj:`str`): the Duckiebot name

        Returns:
            :obj:`str`: the full path to the robot-specific
                calibration file

        """
        cali_file_folder = '/data/config/calibrations/kinematics/'
        cali_file = cali_file_folder + name + ".yaml"
        return cali_file

    def trim(self, value, low, high):
        """
        Trims a value to be between some bounds.

        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound

        Returns:
            the trimmed value
        """

        return max(min(value, high), low)

    def onShutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown."""

        # MAKE SURE THAT THE LAST WHEEL COMMAND YOU PUBLISH IS ZERO,
        # OTHERWISE YOUR DUCKIEBOT WILL CONTINUE MOVING AFTER
        # THE NODE IS STOPPED

        # PUT YOUR CODE HERE

        stop = WheelsCmdStamped()
        stop.vel_left, stop.vel_right = self.speedToCmd(0.0, 0.0)
        #stop.header.stamp = rospy.get_rostime()
        self.pub.publish(stop)
        self.pub.publish(stop)
        self.pub.publish(stop)
        for i in range(20):
            self.pub.publish(stop)
            print i

        super(BraitenbergNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    camera_node = BraitenbergNode(node_name='braitenberg')
    # Keep it spinning to keep the node alive
    rospy.spin()
