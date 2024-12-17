#!/usr/bin/env python3
import rospy
import time

from sensor_msgs.msg import Range

from common.ultrasonic_driver import UltrasonicDriver

class DistanceController:
    def __init__(self):
        self.__id = rospy.get_param("VEHICLE_ID")
        self.__current_reading = [0, 0, 0]
        self.__average_distance = [0, 0, 0]
        self.__distance_m = [0, 0, 0]
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

    def publish_current_distance(self, event): #gets called every "DISTANCE_PUBLISH_PERIOD" seconds
        """
        Publishes the current distance to the vehicle/distance topic.
        """
        #collects "readings_per_publish" data samples from first ultrasound sensor
        if self.__current_reading[0] < self.__readings_per_publish: 
            self.__average_distance[0] += self.driver_1.get_distance()
            self.__current_reading[0] += 1
            return
        
        #computes the average over collected data samples and converts to m from cm
        elif self.__current_reading[0] == self.__readings_per_publish: 
            distance = self.__average_distance[0] / self.__readings_per_publish
            self.__distance_m[0] = distance / 100
        
        #-----||----- seconds ultrasounds sensor
        if self.__current_reading[1] < self.__readings_per_publish:
            self.__average_distance[1] += self.driver_2.get_distance()
            self.__current_reading[1] += 1
            return
        elif self.__current_reading[1] == self.__readings_per_publish:
            distance = self.__average_distance[1] / self.__readings_per_publish
            self.__distance_m[1] = distance / 100
        
        #-----||----- third ultrasounds sensor
        if self.__current_reading[2] < self.__readings_per_publish:
            self.__average_distance[2] += self.driver_3.get_distance()
            self.__current_reading[2] += 1
            return
        elif self.__current_reading[2] == self.__readings_per_publish:
            distance = self.__average_distance[2] / self.__readings_per_publish
            self.__distance_m[2] = distance / 100
        
        #publish the smallest distance of the three sensors, change how this value is chosen
        #to make cars work in environment with obstacles
        self.distance_publisher.publish(self.create_range_message(min(self.__distance_m)))

        #resets average readings for next publish period
        self.__average_distance = [0, 0, 0]
        #resets "current reading", used to keep track of which reading we are on
        self.__current_reading = [0, 0, 0]

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
