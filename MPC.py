import rospy
import numpy as np
from scipy.optimize import minimize
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class MPCController:
    def __init__(self):
        # MPC parameters
        self.horizon = 10  # Prediction horizon
        self.dt = 0.1  # Time step
        self.u_bounds = (-255, 255)  # PWM limits

        # System parameters (example for velocity control)
        self.mass = 1.8  # Mass of the vehicle
        self.drag = 0.1  # Drag coefficient

        # Target velocity
        self.target_velocity = 0.0

        # ROS publishers and subscribers
        self.pwm_pub = rospy.Publisher('/pwm', Float32, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/current_velocity', Float32, self.velocity_callback)

        # State variables
        self.current_velocity = 0.0

    def cmd_vel_callback(self, msg):
        # Extract the target velocity from the incoming Twist message
        self.target_velocity = msg.linear.x

    def velocity_callback(self, msg):
        # Update the current velocity
        self.current_velocity = msg.data

    def system_dynamics(self, v, u):
        # Discrete-time dynamics: v[k+1] = v[k] + dt * (force / mass - drag * v[k])
        force = u
        dv = (force / self.mass - self.drag * v) * self.dt
        return v + dv

    def mpc_cost(self, u_sequence):
        # Flatten control inputs into a sequence for optimization
        u_sequence = np.clip(u_sequence, self.u_bounds[0], self.u_bounds[1])
        u_sequence = np.array(u_sequence)

        # Predict state trajectory
        v = self.current_velocity
        cost = 0.0

        for u in u_sequence:
            v = self.system_dynamics(v, u)
            cost += (v - self.target_velocity) ** 2  # Quadratic cost on velocity error
            cost += 0.01 * u ** 2  # Quadratic cost on control effort

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
            rospy.logwarn("MPC optimization failed!")
            return 0.0

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz loop rate
        while not rospy.is_shutdown():
            # Solve MPC and compute the optimal PWM signal
            pwm = self.solve_mpc()

            # Publish the PWM signal
            self.pwm_pub.publish(Float32(pwm))

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('mpc_controller')
    controller = MPCController()
    controller.run()