#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, Float32

class DataLogger:
    def __init__(self):
        # You can open a file here for writing:
        self.file = open("/tmp/vehicle_data.csv", "w")
        self.file.write("stamp,pwm,velocity,steering_angle\n")

        self.pwm = None
        self.velocity = None
        self.steering = None

        rospy.Subscriber("/vehicle_0/pwm", Int32, self.callback_pwm)
        rospy.Subscriber("/vehicle_0/velocity", Float32, self.callback_velocity)
        rospy.Subscriber("/vehicle_0/steering_angle", Int32, self.callback_steering_angle)
        rospy.Subscriber("/vehicle_0/control", Int32, self.callback_pwm)

        # Use a timer to periodically save data (e.g., 50 Hz).
        rospy.Timer(rospy.Duration(0.02), self.periodic_write)

    def callback_pwm(self, msg):
        self.pwm = msg.data

    def callback_velocity(self, msg):
        self.velocity = msg.data

    def callback_steering_angle(self, msg):
        self.steering = msg.data

    def periodic_write(self, event):
        # Current ROS time
        now = rospy.Time.now().to_sec()  
        # Only write if we have received data from all topics
        if self.pwm is not None and self.velocity is not None and self.steering is not None:
            line = f"{now},{self.pwm},{self.velocity},{self.steering}\n"
            self.file.write(line)

    def on_shutdown(self):
        # Close your file or do final cleanup
        self.file.close()

if __name__ == "__main__":
    rospy.init_node("data_logger")
    logger = DataLogger()
    rospy.on_shutdown(logger.on_shutdown)
    rospy.spin()
