#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Float32, Bool
from sensor_msgs.msg import Joy, Range

class ImmediateDataLogger:
    def __init__(self):
        # Fetch parameters
        self.vehicle_id    = rospy.get_param("VEHICLE_ID", "vehicle_0")
        self.follower_car  = rospy.get_param("FOLLOWER_CAR", False)

        filename = f"/tmp/{self.vehicle_id}_immediate_data.csv"
        rospy.loginfo(f"[ImmediateDataLogger] Logging to {filename}")
        self.file = open(filename, "w")

        # If not a follower, we log Joy. If follower, we log distance/has_target/offset/control.
        if not self.follower_car:
            self.file.write("stamp,pwm,velocity,steering_angle,joy_axes,joy_buttons\n")
        else:
            self.file.write(
                "stamp,pwm,velocity,steering_angle,distance,has_target,"
                "target_center_offset,control\n"
            )

        # Common data
        self.pwm = None
        self.velocity = None
        self.steering_angle = None

        # Joy data
        self.joy_axes = []
        self.joy_buttons = []

        # Follower data
        self.distance = None
        self.has_target = None
        self.target_center_offset = None
        self.control = None

        # Common vehicle topics
        rospy.Subscriber(f"/{self.vehicle_id}/pwm",            Int32,   self.callback_pwm)
        rospy.Subscriber(f"/{self.vehicle_id}/velocity",       Float32, self.callback_velocity)
        rospy.Subscriber(f"/{self.vehicle_id}/steering_angle", Int32,   self.callback_steering)

        if not self.follower_car:
            # Subscribe to /joy
            rospy.Subscriber("/joy", Joy, self.callback_joy)
        else:
            # Follower: distance=Range, offset=Int32, control=Float32
            rospy.Subscriber(f"/{self.vehicle_id}/distance",             Range, self.callback_distance)
            rospy.Subscriber(f"/{self.vehicle_id}/has_target",           Bool,  self.callback_has_target)
            rospy.Subscriber(f"/{self.vehicle_id}/target_center_offset", Int32, self.callback_target_center_offset)
            rospy.Subscriber(f"/{self.vehicle_id}/control",              Float32, self.callback_control)

    # ------------------- Common Callbacks ------------------- #
    def callback_pwm(self, msg):
        self.pwm = msg.data
        self.log_data()

    def callback_velocity(self, msg):
        self.velocity = msg.data
        self.log_data()

    def callback_steering(self, msg):
        self.steering_angle = msg.data
        self.log_data()

    # ------------------- Joy Callback (non-follower) ------------------- #
    def callback_joy(self, msg):
        self.joy_axes = list(msg.axes)
        self.joy_buttons = list(msg.buttons)
        self.log_data()

    # ------------------- Follower Callbacks ------------------- #
    def callback_distance(self, msg):
        # Range message => .range
        self.distance = msg.range
        self.log_data()

    def callback_has_target(self, msg):
        self.has_target = msg.data
        self.log_data()

    def callback_target_center_offset(self, msg):
        # Int32 => .data
        self.target_center_offset = msg.data
        self.log_data()

    def callback_control(self, msg):
        # Float32 => .data
        self.control = msg.data
        self.log_data()

    # ------------------- Logging ------------------- #
    def log_data(self):
        now = rospy.Time.now().to_sec()

        # Common fields
        pwm_val   = self.pwm if self.pwm is not None else 'nan'
        vel_val   = self.velocity if self.velocity is not None else 'nan'
        steer_val = self.steering_angle if self.steering_angle is not None else 'nan'

        if not self.follower_car:
            # Joy scenario
            joy_axes_str    = str(self.joy_axes) if self.joy_axes else '[]'
            joy_buttons_str = str(self.joy_buttons) if self.joy_buttons else '[]'
            line = f"{now},{pwm_val},{vel_val},{steer_val},{joy_axes_str},{joy_buttons_str}\n"
        else:
            # Follower scenario
            distance_val   = self.distance if self.distance is not None else 'nan'
            has_target_val = self.has_target if self.has_target is not None else 'nan'
            offset_val     = self.target_center_offset if self.target_center_offset is not None else 'nan'
            control_val    = self.control if self.control is not None else 'nan'

            line = (
                f"{now},{pwm_val},{vel_val},{steer_val},"
                f"{distance_val},{has_target_val},{offset_val},{control_val}\n"
            )

        self.file.write(line)
        self.file.flush()

    def on_shutdown(self):
        self.file.close()

if __name__ == "__main__":
    rospy.init_node("immediate_data_logger", anonymous=True)
    logger = ImmediateDataLogger()
    rospy.on_shutdown(logger.on_shutdown)
    rospy.spin()
