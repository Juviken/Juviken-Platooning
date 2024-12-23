#!/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import Range
from std_msgs.msg import Int32, Float32, Bool
from rcv.velocity_mapper import VelocityMapper
from scipy.optimize import minimize

class MPCController:
    def __init__(self):
        # Vehicle ID
        self.__id = rospy.get_param("VEHICLE_ID")

        # Determine if leader
        self.__is_leader = (self.__id == "vehicle_0")

        # Control period for the MPC solver
        self.__period = rospy.get_param("MPC_CONTROL_PERIOD", 0.1)  # Example default: 0.1 seconds

        # MPC parameters
        self.horizon = rospy.get_param('horizon', 10)      
        self.dt = rospy.get_param('dt', 0.1)
        self.v_des_min = rospy.get_param('v_des_min', 0.0)
        self.v_des_max = rospy.get_param('v_des_max', 2.0)

        # System parameters
        self.mass = rospy.get_param('mass', 1.8)
        self.drag = rospy.get_param('drag', 0.1)
        self.nonlinear_drag = rospy.get_param('nonlinear_drag', False)

        self.v_history = [0.0, 0.0]  # Initialize velocity history (v(t-1), v(t-2))
        self.u_history = [0.0, 0.0]  # Initialize PWM history (u(t-1), u(t-2))
        # ARX coefficients
        self.a1 = 0.972869359263652
        self.a2 = -0.023137644767055
        self.b1 = -0.000000159414697
        self.b2 = 0.000000239463951

        # Cost weights
        self.weight_distance = rospy.get_param('weight_distance', 1.0)
        self.weight_velocity_error = rospy.get_param('weight_velocity_error', 0.5)
        self.weight_v_des = rospy.get_param('weight_v_des', 0.01)
        self.weight_v_des_change = rospy.get_param('weight_v_des_change', 0.01)

        # Target distance
        self.target_distance = rospy.get_param('target_distance', 1.0)

        # Velocity Mapper parameters (global)
        map_path = rospy.get_param('VELOCITY_PWM_MAP_PATH')
        poly_degree = rospy.get_param('VELOCITY_PWM_MAP_POLYFIT_DEGREE')
        self.__idle = rospy.get_param("IDLE_MOTOR")
        self.__min_forward = rospy.get_param("MIN_FORWARD_MOTOR")
        self.__max_forward = rospy.get_param("MAX_FORWARD_MOTOR")
        self.__max_reverse = rospy.get_param("MAX_REVERSE_MOTOR")

        # Initialize the velocity mapper
        self.__velocity_mapper = VelocityMapper(
            map_path,
            poly_degree,
            self.__idle,
            self.__min_forward,
            self.__max_forward
        )

        self.__message_queue_size = rospy.get_param("MESSAGE_QUEUE_SIZE", 10)

        # State variables
        self.__current_distance = 0.0
        self.__current_velocity = 0.0
        self.__current_leader_velocity = 0.0
        self.__esc_calibrated = False
        self.__has_target = False
        self.__previous_v_des = 0.0
        self.__desired_pwm = self.__idle  # Not strictly needed, but for consistency
        self.__can_brake = False

        # Publishers
        self.pwm_publisher = rospy.Publisher(
            f"{self.__id}/pwm",
            Int32,
            queue_size=self.__message_queue_size)

        self.control_publisher = rospy.Publisher(
            f"{self.__id}/control",
            Float32,
            queue_size=self.__message_queue_size)

        # Subscribers
        self.distance_subscriber = rospy.Subscriber(
            f"{self.__id}/distance",
            Range,
            self.__callback_distance,
            queue_size=self.__message_queue_size)

        self.velocity_subscriber = rospy.Subscriber(
            f"{self.__id}/velocity",
            Float32,
            self.__callback_velocity,
            queue_size=self.__message_queue_size)

        if not self.__is_leader:
            self.leader_velocity_subscriber = rospy.Subscriber(
                "/vehicle_0/velocity",
                Float32,
                self.__callback_leader_velocity,
                queue_size=self.__message_queue_size)

        self.esc_calibrated_subscriber = rospy.Subscriber(
            f"{self.__id}/esc_calibrated",
            Bool,
            self.__callback_esc_calibrated,
            queue_size=self.__message_queue_size)

        self.has_target_subscriber = rospy.Subscriber(
            f"{self.__id}/has_target",
            Bool,
            self.__callback_has_target,
            queue_size=self.__message_queue_size)

        # Timer for MPC step
        rospy.Timer(rospy.Duration(self.__period), self.__perform_step)

        # Logging (optional, similar to PID)
        if not self.__is_leader:
            self.log_file = open(f"/home/user/logs/{self.__id}_mpc_logs.csv", "w")
            self.log_file.write("timestamp,distance,velocity,leader_velocity,desired_v_des,pwm\n")

        rospy.loginfo("MPC controller node started.")

    def __callback_esc_calibrated(self, msg: Bool):
        self.__esc_calibrated = msg.data

    def __callback_distance(self, msg: Range):
        self.__current_distance = msg.range

    def __callback_velocity(self, msg: Float32):
        self.__current_velocity = msg.data

    def __callback_leader_velocity(self, msg: Float32):
        self.__current_leader_velocity = msg.data

    def __callback_has_target(self, msg: Bool):
        self.__has_target = msg.data

class RC_Car_Dynamics:
    def __init__(self, dt, mass, drag, nonlinear_drag=True):
        self.dt = dt
        self.mass = mass
        self.drag = drag
        self.nonlinear_drag = nonlinear_drag
        self.v_history = [0.0, 0.0]  # Initialize velocity history (v(t-1), v(t-2))
        self.u_history = [0.0, 0.0]  # Initialize PWM history (u(t-1), u(t-2))
        # ARX coefficients
        self.a1 = 0.972869359263652
        self.a2 = -0.023137644767055
        self.b1 = -0.000000159414697
        self.b2 = 0.000000239463951

    def system_dynamics(self, pwm):
        # Update the ARX model
        v_t1, v_t2 = self.v_history
        u_t1, u_t2 = self.u_history

        # Compute the next velocity using the ARX model
        v_t = (self.a1 * v_t1 + self.a2 * v_t2 +
               self.b1 * u_t1 + self.b2 * u_t2)

        # Update history
        self.v_history = [v_t, v_t1]
        self.u_history = [pwm, u_t1]

        return v_t

    def mpc_cost(self, v_des_sequence):
        v_des_sequence = np.clip(v_des_sequence, self.v_des_min, self.v_des_max)
        v = self.__current_velocity
        d = self.__current_distance
        cost = 0.0
        prev_v_des = self.__previous_v_des

        for v_des in v_des_sequence:
            # Map desired velocity to PWM
            pwm = self.__velocity_mapper.map_velocity_to_pwm(v_des)
            # System forward simulation
            v = self.system_dynamics(pwm)
            d += v * self.dt

            # Cost on distance error
            cost += self.weight_distance * ((d - self.target_distance)**2)
            # Velocity tracking error
            cost += self.weight_velocity_error * ((v - v_des)**2)
            # Regularization on v_des
            cost += self.weight_v_des * (v_des**2)
            # Penalize changes in v_des
            cost += self.weight_v_des_change * ((v_des - prev_v_des)**2)

            prev_v_des = v_des

        return cost


    def solve_mpc(self):
        v0 = np.full(self.horizon, self.__current_velocity)
        bounds = [(self.v_des_min, self.v_des_max)] * self.horizon
        result = minimize(self.mpc_cost, v0, bounds=bounds, method='SLSQP')

        if result.success:
            return result.x[0]
        else:
            rospy.logwarn("MPC optimization failed! Using fallback velocity.")
            return 0.0

    def __perform_step(self, event):
        if not self.__esc_calibrated:
            return

        # If no target and vehicle should stop if no target is visible,
        # you can add similar logic as PID if desired. For now, we'll just run MPC.
        v_des = self.solve_mpc()
        pwm_float = self.__velocity_mapper.map_velocity_to_pwm(v_des)

        # Convert PWM to int for publishing
        pwm_int = int(pwm_float)
        self.pwm_publisher.publish(pwm_int)
        self.control_publisher.publish(Float32(v_des))
        self.__previous_v_des = v_des

        # Logging
        if not self.__is_leader:
            self.log_file.write(f"{rospy.get_time()},{self.__current_distance},{self.__current_velocity},{self.__current_leader_velocity},{v_des},{pwm_int}\n")
            self.log_file.flush()

    def stop(self):
        if not self.__is_leader:
            self.log_file.close()
        self.pwm_publisher.publish(Int32(self.__idle))


if __name__ == "__main__":
    rospy.init_node("mpc_controller_node", anonymous=True)
    controller = MPCController()
    rospy.on_shutdown(controller.stop)
    rospy.spin()
