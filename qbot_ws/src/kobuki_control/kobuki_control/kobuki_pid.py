class PIDController:
    def __init__(self, kp, ki, kd, output_min=None, output_max=None, integral_min=None, integral_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min
        self.integral_max = integral_max

        self.integral = 0.0
        self.prev_error = 0.0
        self.first_update = True

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_update = True

    def update(self, setpoint, measured, dt):
        if dt <= 0.0:
            return 0.0

        error = setpoint - measured

        # Proportional
        p_term = self.kp * error

        # Integral
        self.integral += error * dt

        if self.integral_min is not None:
            self.integral = max(self.integral_min, self.integral)
        if self.integral_max is not None:
            self.integral = min(self.integral_max, self.integral)

        i_term = self.ki * self.integral

        # Derivative
        if self.first_update:
            derivative = 0.0
            self.first_update = False
        else:
            derivative = (error - self.prev_error) / dt

        d_term = self.kd * derivative

        output = p_term + i_term + d_term

        if self.output_min is not None:
            output = max(self.output_min, output)
        if self.output_max is not None:
            output = min(self.output_max, output)

        self.prev_error = error
        return output