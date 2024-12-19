import rospy
import numpy as np
from scipy.optimize import minimize
from std_msgs.msg import Float32
from rcv.velocity_mapper import VelocityMapper

class MPCController:
    def __init__(self):
        # MPC parameters
        self.horizon = rospy.get_param('horizon', 10)  # Prediction horizon
        self.dt = rospy.get_param('dt', 0.1)  # Time step
        # Define a realistic velocity range for your vehicle
        self.v_des_bounds = (0.0, 2.0)  # Example bounds in m/s

        # System parameters
        self.mass = rospy.get_param('mass', 1.8)  # Mass of the vehicle
        self.drag = rospy.get_param('drag', 0.1)  # Drag coefficient
        self.nonlinear_drag = rospy.get_param('nonlinear_drag', False)  # Use nonlinear drag if true

        # Cost weights
        self.weight_distance = rospy.get_param('weight_distance', 1.0)
        self.weight_velocity_error = rospy.get_param('weight_velocity_error', 0.5)
        self.weight_v_des = rospy.get_param('weight_v_des', 0.01)
        self.weight_v_des_change = rospy.get_param('weight_v_des_change', 0.01)

        # Target distance
        self.target_distance = rospy.get_param('target_distance', 1.0)  # Desired distance in meters

        # ROS publishers and subscribers
        self.pwm_pub = rospy.Publisher('/pwm', Float32, queue_size=10)
        rospy.Subscriber('/cmd_distance', Float32, self.cmd_distance_callback)
        rospy.Subscriber('/current_distance', Float32, self.distance_callback)
        rospy.Subscriber('/current_velocity', Float32, self.velocity_callback)

        # State variables
        self.current_distance = 0.0
        self.current_velocity = 0.0
        self.previous_v_des = 0.0

        # Velocity mapper initialization
        # Ensure you have valid parameters set for the mapper
        map_path = rospy.get_param("VELOCITY_PWM_MAP_PATH")
        poly_degree = rospy.get_param("VELOCITY_PWM_MAP_POLYFIT_DEGREE")
        self.__idle = rospy.get_param("VELOCITY_PWM_MAP_IDLE", 0.0)  # Example default
        self.__min_forward = rospy.get_param("VELOCITY_PWM_MAP_MIN_FORWARD", 0.0)
        self.__max_forward = rospy.get_param("VELOCITY_PWM_MAP_MAX_FORWARD", 255.0)

        self.__velocity_mapper = VelocityMapper(
            map_path,
            poly_degree,
            self.__idle,
            self.__min_forward,
            self.__max_forward
        )

    def cmd_distance_callback(self, msg):
        # Update the target distance
        self.target_distance = msg.data

    def distance_callback(self, msg):
        # Update the current distance
        self.current_distance = msg.data

    def velocity_callback(self, msg):
        # Update the current velocity
        self.current_velocity = msg.data

    def system_dynamics(self, v, pwm):
        # Convert PWM (applied force) into acceleration
        # You can treat pwm as force directly or refine this as needed
        force = pwm
        if self.nonlinear_drag:
            dv = (force / self.mass - self.drag * v**2) * self.dt
        else:
            dv = (force / self.mass - self.drag * v) * self.dt
        return v + dv

    def mpc_cost(self, v_des_sequence):
        v_des_sequence = np.clip(v_des_sequence, self.v_des_bounds[0], self.v_des_bounds[1])

        v = self.current_velocity
        d = self.current_distance
        cost = 0.0
        prev_v_des = self.previous_v_des

        for v_des in v_des_sequence:
            # Map desired velocity to PWM
            pwm = self.__velocity_mapper.map_velocity_to_pwm(v_des)

            # Simulate system forward
            v = self.system_dynamics(v, pwm)
            d += v * self.dt

            # Cost on distance error
            cost += self.weight_distance * (d - self.target_distance)**2
            # Cost on velocity tracking error: encourage actual velocity ~ desired velocity
            cost += self.weight_velocity_error * (v - v_des)**2
            # Regularization on v_des itself
            cost += self.weight_v_des * (v_des**2)
            # Penalize changes in desired velocity
            cost += self.weight_v_des_change * ((v_des - prev_v_des)**2)

            prev_v_des = v_des

        return cost

    def solve_mpc(self):
        # Initial guess for desired velocities
        v0 = np.full(self.horizon, self.current_velocity)

        # Define bounds for the control inputs (desired velocity)
        bounds = [(self.v_des_bounds[0], self.v_des_bounds[1])] * self.horizon

        # Solve optimization problem
        result = minimize(self.mpc_cost, v0, bounds=bounds, method='SLSQP')

        if result.success:
            return result.x[0]  # Return the first desired velocity
        else:
            rospy.logwarn("MPC optimization failed! Using fallback velocity.")
            return 0.0  # fallback desired velocity

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz loop rate
        while not rospy.is_shutdown():
            # Solve MPC to get the optimal desired velocity
            v_des = self.solve_mpc()

            # Convert desired velocity to PWM using the mapper
            pwm = self.__velocity_mapper.map_velocity_to_pwm(v_des)

            # Publish the PWM signal
            self.pwm_pub.publish(Float32(pwm))
            self.previous_v_des = v_des

            # Debugging information
            rospy.loginfo(f"Target Distance: {self.target_distance}, Current Distance: {self.current_distance}, Desired Vel: {v_des}, PWM: {pwm}")

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('mpc_controller')
    controller = MPCController()
    controller.run()
