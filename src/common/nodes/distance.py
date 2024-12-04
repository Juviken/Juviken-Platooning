#!/usr/bin/env python3
import rospy
import time

from sensor_msgs.msg import Range

from common.ultrasonic_driver import UltrasonicDriver

class DistanceController:
    def __init__(self):
        self.__id = rospy.get_param("VEHICLE_ID")
        self.__current_reading = 0
        self.__average_distance = 0.0
        self.__readings_per_publish = rospy.get_param("ULTRASONIC_SAMPLES_PER_PUBLISH")

        self.driver_1 = UltrasonicDriver(
            rospy.get_param("ULTRASONIC_TRIGGER_1"),
            rospy.get_param("ULTRASONIC_ECHO_1"))

        self.driver_2 = UltrasonicDriver(
            rospy.get_param("ULTRASONIC_TRIGGER_2"),
            rospy.get_param("ULTRASONIC_ECHO_2"))
        
        self.driver_3 = UltrasonicDriver(
            rospy.get_param("ULTRASONIC_TRIGGER_3"),
            rospy.get_param("ULTRASONIC_ECHO_3"))

        self.distance_publisher = rospy.Publisher(
            f"{self.__id}/distance",
            Range,
            queue_size=rospy.get_param("MESSAGE_QUEUE_SIZE"))

        rospy.Timer(rospy.Duration(rospy.get_param("DISTANCE_PUBLISH_PERIOD")),
                    self.publish_current_distance)

    def create_range_message(self, distance):
        """
        Creates a Range message with the given distance value.
        """
        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.header.frame_id = "/base_link"
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = self.driver_2.fov
        range_msg.min_range = self.driver_2.min_range
        range_msg.max_range = self.driver_2.max_range
        range_msg.range = distance
        return range_msg

    def publish_current_distance(self, event):
        """
        Publishes the current distance to the vehicle/distance topic.
        """
        """
        distance_m = [0, 0, 0]
        driver_distance = [self.driver_1.get_distance(), self.driver_2.get_distance(), self.driver_3.get_distance()]

        for i in range(3):
            if self.__current_reading < self.__readings_per_publish:
                self.__average_distance += driver_distance[i]
                self.__current_reading += 1
                continue  # Exit the function; we need more readings

            distance_m[i] = (self.__average_distance / self.__readings_per_publish) / 100

            self.__current_reading = 0
            self.__average_distance = 0

        self.distance_publisher.publish(self.create_range_message(min(distance_m[2])))"""
        
        distance_m = [0, 0, 0]
        driver_distance = [self.driver_1.get_distance(), self.driver_2.get_distance(), self.driver_3.get_distance()]

        for i in range(3):
            for j in range(self.__readings_per_publish):
                self.__average_distance += driver_distance[i]
                self.__current_reading += 1

            distance = self.__average_distance / self.__readings_per_publish
            distance_m[i] = distance / 100

        self.distance_publisher.publish(self.create_range_message(distance_m[0]))

        self.__current_reading = 0
        self.__average_distance = 0

    def stop(self):
        """
        Stops the distance node.
        """
        self.distance_publisher.unregister()
        self.driver_1.cleanup()
        self.driver_2.cleanup()
        self.driver_3.cleanup()


if __name__ == "__main__":
    rospy.init_node("distance_node", anonymous=True)
    controller = DistanceController()
    rospy.on_shutdown(controller.stop)

    rospy.loginfo("Distance node started.")
    rospy.spin()
