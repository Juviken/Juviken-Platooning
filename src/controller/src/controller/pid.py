class PID:
    def __init__(self, kp, ki, kd, dt: int, pid_min: float = None, pid_max: float = None, max_integral: float = None):
        self.error = 0
        self.error_integral = 0
        self.error_derivative = 0
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.dt = dt

        self.pid_min = pid_min if pid_min is not None else -float('inf')
        self.pid_max = pid_max if pid_max is not None else float('inf')
        
        # Maximum allowable integral value, anti-windup
        self.max_integral = max_integral if max_integral is not None else float('inf')

    def update(self, error: float):
        error_prev = self.error
        self.error = error
        
        # Update integral term with clamping
        self.error_integral += error * self.dt
        self.error_integral = max(min(self.error_integral, self.max_integral), -self.max_integral)
        #lOG THE INTEGRAL in ros
        

        
        # Update derivative term
        self.error_derivative = (error - error_prev) / self.dt

        # Compute the PID output
        u = self.Kp * error + self.Ki * self.error_integral + self.Kd * self.error_derivative

        # Constrain the output to allowable range
        return min(max(u, self.pid_min), self.pid_max)
