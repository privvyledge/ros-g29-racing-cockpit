#!/usr/bin/env python3
"""
Subscribes to commands matching a custom message and sends commands to the steering wheel.
"""

# import legacy modules first (compulsorily at the top)
from __future__ import division, print_function, absolute_import

# standard library imports
import math
from math import sin, cos, tan, pi, fabs, radians
import sys

# third-party library imports
import rospy

# import ros messages
from std_msgs.msg import Empty, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, Twist

# import local modules
from g29_force_feedback.msg import ForceFeedback


__author__ = 'Boluwatife Olabiran'
__license__ = 'GPLv3'
__maintainer__ = 'Boluwatife Olabiran'
__email__ = 'bso19a@fsu.edu'


class DriverNode(object):
    """docstring for ClassName"""

    def __init__(self):
        """Constructor for OdometryNode"""
        super(DriverNode, self).__init__()
        self.MAX_STEERING_ANGLE = float(rospy.get_param('~max_steering_angle', 0.49))
        self.MAX_STEERING_WHEEL_ANGLE = int(rospy.get_param('~max_steering_wheel_angle', 470))
        self.MAX_TORQUE = rospy.get_param('~max_torque', 470)

        # safety commands
        self.ignore = rospy.get_param('~ignore', False)  # Ignore driver overrides
        self.enable = rospy.get_param('~enable', True)  # Use enable and disable buttons
        # ToDo: implement rolling counter with message publisher callbacks
        self.count = rospy.get_param('~count',
                                     False)  # Increment counter to enable watchdog.
        self.svel = rospy.get_param('~svel', 0.0)  # Steering command speed
        self.global_enable_status = rospy.get_param('~global_enable_status', False)
        self.global_disable_status = rospy.get_param('~global_disable_status', False)

        # joystick constants
        self.AXIS_STEER_1 = 0
        self.BTN_STEER_MULT_1 = 7

        # car parameters
        self.WHEELBASE = 1.0
        self.normalized_steering_angle = 1 / self.MAX_STEERING_ANGLE  # normalizes the steering angle to [-1, 1]

        # actuator parameters
        self.actuation_data = {"stamp": rospy.Time.now(),
                               "valid_command": False,
                               "steering_wheel_angle": 0.0,
                               "accelerator_pedal_command": 0.0,
                               "brake_pedal_command": 0.0,
                               "torque": 0.0}

        # subscribe to actuator commands. todo
        subscriber_queue_size = 1
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=subscriber_queue_size)

        # setup loop frequency. Will be enforced if the loop completes execution within the limit, otherwise ignored.
        self.loop_rate = 100.0  # Hz
        self.loop_sample_time = 1.0 / self.loop_rate  # s
        self.rate = rospy.Rate(self.loop_rate)

        # instantiate publishers
        period = 0.02  # a.k.a. duration in s (or 1/Hz)
        publisher_queue_size = 1
        self.ff_pub = rospy.Publisher('ff_target', ForceFeedback, queue_size=publisher_queue_size)
        # rospy.Timer(rospy.Duration(period), self.ff_publisher,
        #             oneshot=False, reset=True)

    def run(self):
        while not rospy.is_shutdown():
            # process loop here. Put in whatever functions and calculations need to be run here.
            if rospy.Time.now() - self.actuation_data.get('stamp') > rospy.Duration(0.1):
                self.actuation_data["valid_command"] = False

            ff_msg = ForceFeedback()
            ff_msg.position = self.actuation_data["steering_wheel_angle"]
            ff_msg.torque = self.actuation_data["torque"]

            # publish data after calculation if necessary
            if self.actuation_data["valid_command"]:
                self.ff_pub.publish(ff_msg)

            # to enforce rate/sample time
            self.rate.sleep()

        # whatever is written here will also be processed once a shutdown is initiated but is not guaranteed.
        # Use the shutdown method instead.
        pass

    # def cmd_callback(self, msg):
    #     self.actuation_data["valid_command"] = True
    #     self.actuation_data["stamp"] = rospy.Time.now()
    #     self.actuation_data["steering_wheel_angle"] = msg.steering_wheel_angle
    #     self.actuation_data["torque"] = 0.0
    #     self.actuation_data["accelerator_pedal_command"] = msg.accelerator_pedal_command
    #     self.actuation_data["brake_pedal_command"] = msg.brake_pedal_command

    def joystick_callback(self, msg):
        steering_joy = msg.axes[self.AXIS_STEER_1] * self.MAX_STEERING_ANGLE
        normalized_steering = self.normalized_steering_angle * steering_joy
        self.actuation_data["steering_wheel_angle"] = steering_joy
        # self.MAX_STEERING_WHEEL_ANGLE * normalized_steering assumes a linear (not affine) relationship
        # between the steering angle and steering wheel angle which might not be the case
        steering_mult = msg.buttons[self.BTN_STEER_MULT_1]
        # for safety limit maximum steering wheel angle if steering_mult is not pressed
        if not steering_mult:
            self.actuation_data["steering_wheel_angle"] *= 0.5

    def ff_publisher(self, event):
        if event.current_real - self.actuation_data.get('stamp') > rospy.Duration(0.1):
            self.actuation_data["valid_command"] = False

        ff_msg = ForceFeedback()
        ff_msg.position = self.actuation_data["steering_wheel_angle"]
        ff_msg.torque = self.actuation_data["torque"]

        if self.actuation_data["valid_command"]:
            self.ff_pub.publish(ff_msg)

    def shutdown(self):
        self.actuation_data["steering_wheel_angle"] = 0.0
        self.actuation_data["torque"] = 0.0
        self.actuation_data["accelerator_pedal_command"] = 0.0
        self.actuation_data["brake_pedal_command"] = 0.0

        ff_msg = ForceFeedback()
        ff_msg.position = self.actuation_data["steering_wheel_angle"]
        ff_msg.torque = self.actuation_data["torque"]
        self.ff_pub.publish(ff_msg)


def main(args):
    # args will be a list of commands passed
    optional_nodename = 'driver_node'  # this will be overwritten if a nodename is specified in a launch file
    rospy.init_node('{}'.format(optional_nodename))
    nodename = rospy.get_name()  # this gets the actual nodename whether the node is launched or run separately
    rospy.loginfo("{} node started.".format(nodename))  # just log that the node has started.
    driver_instance = DriverNode()

    # use rospy.on_shutdown() to perform clean shutdown tasks, e.g. saving a file, shutting down motors, etc.
    rospy.on_shutdown(driver_instance.shutdown)
    # could also use atexit.register(driver_instance.shutdown) to avoid trusting rospy

    try:
        # run the main functions here
        driver_instance.run()
        # rospy.spin()  # only necessary if not publishing (i.e. subscribing only)
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        # this exception block most likely won't be called since there is a shutdown method in the class that will
        # override this and shutdown however is needed but is here just in case.
        rospy.loginfo('Encountered {}. Shutting down.'.format(e))

        # try:
        #     sys.exit(0)
        # except SystemExit:
        #     os._exit(0)


if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    main(myargv)  # ROS compatible way to handle command line arguments, i.e main(sys.argv)
