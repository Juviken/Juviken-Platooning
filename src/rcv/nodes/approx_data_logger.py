#!/usr/bin/env python3

import rospy
import message_filters
from std_msgs.msg import Int32, Float32, Bool
from sensor_msgs.msg import Joy

class ApproxDataLogger:
    def __init__(self):
        # Fetch parameters
        self.vehicle_id   = rospy.get_param("VEHICLE_ID", "vehicle_0")
        self.follower_car = rospy.get_param("FOLLOWER_CAR", False)

        # Build filename
        filename = f"/tmp/{self.vehicle_id}_synchronized_data.csv"
        rospy.loginfo(f"[ApproxDataLogger] Logging to {filename}")
        self.file = open(filename, "w")

        # Prepare the subscriber list for message_filters
        # Common subscribers: pwm, velocity, steering
        sub_pwm      = message_filters.Subscriber(f"/{self.vehicle_id}/pwm",            Int32)
        sub_velocity = message_filters.Subscriber(f"/{self.vehicle_id}/velocity",       Float32)
        sub_steering = message_filters.Subscriber(f"/{self.vehicle_id}/steering_angle", Int32)

        # We'll store them in a list that we'll pass to the synchronizer
        subscribers_list = [sub_pwm, sub_velocity, sub_steering]

        if not self.follower_car:
            # Not follower -> also track /joy
            # Write CSV header accordingly
            self.file.write("stamp,pwm,velocity,steering_angle,joy_axes,joy_buttons\n")

            sub_joy = message_filters.Subscriber("/joy", Joy)
            subscribers_list.append(sub_joy)

            # Create ATS with 4 topics
            self.sync = message_filters.ApproximateTimeSynchronizer(
                subscribers_list,
                queue_size=10,
                slop=0.1,
                allow_headerless=True  # needed since pwm/velocity/steering are headerless
            )
            self.sync.registerCallback(self.callback_not_follower)

        else:
            # FOLLOWER_CAR == True -> track distance, has_target, target_center_offset, control
            # Write CSV header
            self.file.write(
                "stamp,pwm,velocity,steering_angle,distance,has_target,"
                "target_center_offset,control\n"
            )

            sub_distance  = message_filters.Subscriber(f"/{self.vehicle_id}/distance",             Float32)
            sub_has_target= message_filters.Subscriber(f"/{self.vehicle_id}/has_target",           Bool)
            sub_offset    = message_filters.Subscriber(f"/{self.vehicle_id}/target_center_offset", Float32)
            sub_control   = message_filters.Subscriber(f"/{self.vehicle_id}/control",              Int32)

            # Add them to the list, so we have 7 topics total
            subscribers_list.extend([sub_distance, sub_has_target, sub_offset, sub_control])

            # Create ATS with 7 topics
            self.sync = message_filters.ApproximateTimeSynchronizer(
                subscribers_list,
                queue_size=10,
                slop=0.1,
                allow_headerless=True
            )
            self.sync.registerCallback(self.callback_follower)

    # ------------------------------------------------------
    #  Callbacks for Not-Follower and Follower
    # ------------------------------------------------------

    def callback_not_follower(self, pwm_msg, velocity_msg, steering_msg, joy_msg):
        """Approx. sync callback when not a follower."""
        now = rospy.Time.now().to_sec()

        pwm_val    = pwm_msg.data
        vel_val    = velocity_msg.data
        steer_val  = steering_msg.data

        joy_axes_str    = str(list(joy_msg.axes))
        joy_buttons_str = str(list(joy_msg.buttons))

        line = f"{now},{pwm_val},{vel_val},{steer_val},{joy_axes_str},{joy_buttons_str}\n"
        self.file.write(line)
        self.file.flush()

    def callback_follower(self, pwm_msg, velocity_msg, steering_msg,
                          distance_msg, has_target_msg, offset_msg, control_msg):
        """Approx. sync callback when FOLLOWER_CAR == True."""
        now = rospy.Time.now().to_sec()

        pwm_val      = pwm_msg.data
        vel_val      = velocity_msg.data
        steer_val    = steering_msg.data
        distance_val = distance_msg.data
        has_target_val = has_target_msg.data
        offset_val     = offset_msg.data
        control_val    = control_msg.data

        line = (
            f"{now},{pwm_val},{vel_val},{steer_val},"
            f"{distance_val},{has_target_val},{offset_val},{control_val}\n"
        )
        self.file.write(line)
        self.file.flush()

    def on_shutdown(self):
        self.file.close()


if __name__ == "__main__":
    rospy.init_node("approx_data_logger", anonymous=True)
    logger = ApproxDataLogger()
    rospy.on_shutdown(logger.on_shutdown)
    rospy.spin()
