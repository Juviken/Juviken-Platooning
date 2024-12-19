import rospy
import numpy as np
from scipy.optimize import minimize
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rcv.velocity_mapper import VelocityMapper

class MPCController:
    def __init__(self):
        # MPC parameters
        self.horizon = rospy.get_param('~horizon', 10)  # Prediction horizon
        self.dt = rospy.get_param('~dt', 0.1)  # Time step
        self.u_bounds = (-255, 255)  # PWM limits
        self.pwm_offset = rospy.get_param('~pwm_offset', 50)  # Dead zone compensation

        # System parameters (example for distance control)
        self.mass = rospy.get_param('~mass', 1.8)  # Mass of the vehicle
        self.drag = rospy.get_param('~drag', 0.1)  # Drag coefficient
        self.nonlinear_drag = rospy.get_param('~nonlinear_drag', False)  # Use nonlinear drag if true

        # Cost weights
        self.weight_distance = rospy.get_param('~weight_distance', 1.0)
        self.weight_velocity = rospy.get_param('~weight_velocity', 0.1)
        self.weight_pwm = rospy.get_param('~weight_pwm', 0.01)
        self.weight_pwm_change = rospy.get_param('~weight_pwm_change', 0.01)

        # Target distance
        self.target_distance = rospy.get_param('~target_distance', 1.0)  # Desired distance in meters

        # ROS publishers and subscribers
        self.pwm_pub = rospy.Publisher('/pwm', Float32, queue_size=10)
        rospy.Subscriber('/cmd_distance', Float32, self.cmd_distance_callback)
        rospy.Subscriber('/current_distance', Float32, self.distance_callback)
        rospy.Subscriber('/current_velocity', Float32, self.velocity_callback)

        # State variables
        self.current_distance = 0.0
        self.current_velocity = 0.0
        self.previous_pwm = 0.0

        # Initialize the velocity mapper
        self.__velocity_mapper = VelocityMapper(
            rospy.get_param("VELOCITY_PWM_MAP_PATH"),
            rospy.get_param("VELOCITY_PWM_MAP_POLYFIT_DEGREE"),
            self.__idle,
            self.__min_forward,
            self.__max_forward)

    def cmd_distance_callback(self, msg):
        # Update the target distance
        self.target_distance = msg.data

    def distance_callback(self, msg):
        # Update the current distance
        self.current_distance = msg.data

    def velocity_callback(self, msg):
        # Update the current velocity
        self.current_velocity = msg.data

    def system_dynamics(self, v, u):
        # Discrete-time dynamics: v[k+1] = v[k] + dt * (force / mass - drag * v[k])
        force = u
        if self.nonlinear_drag:
            dv = (force / self.mass - self.drag * v ** 2) * self.dt
        else:
            dv = (force / self.mass - self.drag * v) * self.dt
        return v + dv

    def mpc_cost(self, u_sequence):
        # Flatten control inputs into a sequence for optimization
        u_sequence = np.clip(u_sequence, self.u_bounds[0], self.u_bounds[1])
        u_sequence = np.array(u_sequence)

        # Predict state trajectory
        v = self.current_velocity
        d = self.current_distance
        cost = 0.0
        prev_u = self.previous_pwm

        for u in u_sequence:
            v = self.system_dynamics(v, u)
            d += v * self.dt  # Update distance based on velocity
            cost += self.weight_distance * (d - self.target_distance) ** 2  # Quadratic cost on distance error
            cost += self.weight_velocity * (v - 0.0) ** 2  # Penalize unnecessary velocity
            cost += self.weight_pwm * u ** 2  # Quadratic cost on control effort
            cost += self.weight_pwm_change * (u - prev_u) ** 2  # Penalize changes in PWM
            prev_u = u

        return cost

    def solve_mpc(self):
        # Initial guess for control inputs
        u0 = np.zeros(self.horizon)

        # Define bounds for the control inputs
        bounds = [(self.u_bounds[0], self.u_bounds[1])] * self.horizon

        # Solve optimization problem
        result = minimize(self.mpc_cost, u0, bounds=bounds, method='SLSQP')

        if result.success:
            # Return the first control input in the sequence
            return result.x[0]
        else:
            rospy.logwarn("MPC optimization failed! Using fallback PWM.")
            return 0.0

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz loop rate
        while not rospy.is_shutdown():
            # Solve MPC and compute the optimal PWM signal
            pwm = self.solve_mpc()

            # Apply dead zone compensation
            pwm = max(self.pwm_offset, abs(pwm)) * np.sign(pwm)

            # Publish the PWM signal
            self.pwm_pub.publish(Float32(pwm))
            self.previous_pwm = pwm

            # Debugging information
            rospy.loginfo(f"Target Distance: {self.target_distance}, Current Distance: {self.current_distance}, PWM: {pwm}")

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('mpc_controller')
    controller = MPCController()
    controller.run()
