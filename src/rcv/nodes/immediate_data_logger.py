#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Joy

class ImmediateDataLogger:
    def __init__(self):
        # Open file to write CSV data
        self.file = open("/tmp/vehicle_immediate_data.csv", "w")
        # CSV header: stamp, pwm, velocity, steering_angle, joy_axes, joy_buttons
        self.file.write("stamp,pwm,velocity,steering_angle,joy_axes,joy_buttons\n")

        # Variables to store the latest data from each topic
        self.pwm = None
        self.velocity = None
        self.steering_angle = None
        self.joy_axes = []
        self.joy_buttons = []

        # Set up subscribers
        rospy.Subscriber("/vehicle_0/pwm", Int32, self.callback_pwm)
        rospy.Subscriber("/vehicle_0/velocity", Float32, self.callback_velocity)
        rospy.Subscriber("/vehicle_0/steering_angle", Int32, self.callback_steering)
        rospy.Subscriber("/joy", Joy, self.callback_joy)

    def callback_pwm(self, msg):
        self.pwm = msg.data
        self.log_data()

    def callback_velocity(self, msg):
        self.velocity = msg.data
        self.log_data()

    def callback_steering(self, msg):
        self.steering_angle = msg.data
        self.log_data()

    def callback_joy(self, msg):
        """
        For logging, we can convert the arrays to strings.
        Example: "[0.0, 1.0]" for axes, "[0,0,1,0]" for buttons
        """
        self.joy_axes = list(msg.axes)
        self.joy_buttons = list(msg.buttons)
        self.log_data()

    def log_data(self):
        """Immediately write the latest data to the CSV file."""
        now = rospy.Time.now().to_sec()

        # Use 'nan' or another placeholder if no data has been received yet
        pwm_val       = self.pwm if self.pwm is not None else 'nan'
        vel_val       = self.velocity if self.velocity is not None else 'nan'
        steer_val     = self.steering_angle if self.steering_angle is not None else 'nan'

        # Convert axes and buttons to string (or keep them as is if you prefer)
        joy_axes_str    = str(self.joy_axes) if self.joy_axes else '[]'
        joy_buttons_str = str(self.joy_buttons) if self.joy_buttons else '[]'

        # Write a single line for the current update
        line = f"{now},{pwm_val},{vel_val},{steer_val},{joy_axes_str},{joy_buttons_str}\n"
        self.file.write(line)
        # Flush to ensure data is actually written
        self.file.flush()

    def on_shutdown(self):
        """Clean up before exiting."""
        self.file.close()

if __name__ == "__main__":
    rospy.init_node("immediate_data_logger", anonymous=True)
    logger = ImmediateDataLogger()
    rospy.on_shutdown(logger.on_shutdown)
    rospy.spin()
