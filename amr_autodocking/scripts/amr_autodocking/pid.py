class PID:
    def __init__(self, kp, ki, kd, out_min: float = 0.0, out_max: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.out_min = out_min
        self.out_max = out_max

    def update(self, setpoint, feedback_value, dt):
        dt_ms = dt*1000
        error = setpoint - feedback_value
        self.integral += error * dt_ms
        derivative = (error - self.prev_error) / dt_ms
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        # print('setpoint: ', setpoint)
        # print('feedback_value: ', feedback_value)
        # print('output: ', output)
        if (self.out_min != 0 or self.out_max != 0):
            return self.clamp(abs(output), self.out_min, self.out_max)
        return output

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def set_tunings(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def clamp(self, value, lower_limit, upper_limit):
        if value > upper_limit:
            return upper_limit
        elif value < lower_limit:
            return lower_limit
        return value