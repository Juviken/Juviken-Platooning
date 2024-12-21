#!/usr/bin/env python3

import rospy
import message_filters
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Joy

class ApproxDataLogger:
    def __init__(self):
        # Open the file for writing (CSV format)
        self.file = open("/tmp/vehicle_synchronized_data.csv", "w")
        # CSV header
        self.file.write("stamp,pwm,velocity,steering_angle,joy_axes,joy_buttons\n")

        # Create message_filters subscribers
        sub_pwm      = message_filters.Subscriber("/vehicle_0/pwm", Int32)
        sub_velocity = message_filters.Subscriber("/vehicle_0/velocity", Float32)
        sub_steering = message_filters.Subscriber("/vehicle_0/steering_angle", Int32)
        sub_joy      = message_filters.Subscriber("/joy", Joy)

        # ApproximateTimeSynchronizer:
        #  - queue_size: number of messages buffered
        #  - slop: allowed time difference (seconds)
        #  - allow_headerless=True: because Int32/Float32 have no header
        #    and Joy does have a header, we must allow mixing.
        ats = message_filters.ApproximateTimeSynchronizer(
            [sub_pwm, sub_velocity, sub_steering, sub_joy],
            queue_size=10,
            slop=0.1,
            allow_headerless=True
        )

        # Register the common callback
        ats.registerCallback(self.sync_callback)

    def sync_callback(self, pwm_msg, velocity_msg, steering_msg, joy_msg):
        """
        This callback is triggered once the four messages align in time
        (within the 'slop' range).
        """
        now = rospy.Time.now().to_sec()

        # Extract data
        pwm_val = pwm_msg.data
        vel_val = velocity_msg.data
        steer_val = steering_msg.data

        # Convert Joy arrays to string
        joy_axes_str    = str(list(joy_msg.axes))
        joy_buttons_str = str(list(joy_msg.buttons))

        # Write a single line to the CSV
        line = f"{now},{pwm_val},{vel_val},{steer_val},{joy_axes_str},{joy_buttons_str}\n"
        self.file.write(line)
        self.file.flush()

    def on_shutdown(self):
        self.file.close()

if __name__ == "__main__":
    rospy.init_node("approx_data_logger", anonymous=True)
    logger = ApproxDataLogger()
    rospy.on_shutdown(logger.on_shutdown)
    rospy.spin()
