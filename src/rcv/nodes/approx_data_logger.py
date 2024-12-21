#!/usr/bin/env python3

import rospy
import message_filters
from std_msgs.msg import Int32, Float32

class SynchronizedDataLogger:
    def __init__(self):
        # Open the file for writing (CSV format)
        self.file = open("/tmp/vehicle_synchronized_data.csv", "w")
        self.file.write("stamp,pwm,velocity,steering_angle\n")

        # Create message filters subscribers
        sub_pwm      = message_filters.Subscriber("/vehicle_0/pwm", Int32)
        sub_velocity = message_filters.Subscriber("/vehicle_0/velocity", Float32)
        sub_steering = message_filters.Subscriber("/vehicle_0/steering_angle", Int32)

        # ApproximateTimeSynchronizer:
        #  - queue_size controls how many messages are buffered.
        #  - slop is the allowed delay (in seconds) between messages to still be synchronized.
        #  - allow_headerless=True means messages without a std_msgs/Header are allowed.
        ts = message_filters.ApproximateTimeSynchronizer(
            [sub_pwm, sub_velocity, sub_steering],
            queue_size=10,
            slop=0.1,
            allow_headerless=True
        )

        # Register the common callback for synchronized messages
        ts.registerCallback(self.sync_callback)

    def sync_callback(self, pwm_msg, velocity_msg, steering_msg):
        """
        This callback is triggered once the three messages align in time 
        within the specified 'slop' range.
        """
        now = rospy.Time.now().to_sec()
        pwm_val = pwm_msg.data
        velocity_val = velocity_msg.data
        steering_val = steering_msg.data

        # Write a single line to the CSV
        line = f"{now},{pwm_val},{velocity_val},{steering_val}\n"
        self.file.write(line)
        # Flush to ensure data is actually written out
        self.file.flush()

    def on_shutdown(self):
        self.file.close()

if __name__ == "__main__":
    rospy.init_node("synchronized_data_logger", anonymous=True)
    logger = SynchronizedDataLogger()
    rospy.on_shutdown(logger.on_shutdown)
    rospy.spin()
