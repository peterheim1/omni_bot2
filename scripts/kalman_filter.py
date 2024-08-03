import numpy as np

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance  # Process (system) noise variance
        self.measurement_variance = measurement_variance  # Measurement noise variance

        self.estimated_error = 1.0  # Initial estimate error
        self.estimate = 0.0  # Initial estimate

    def update(self, measurement):
        kalman_gain = self.estimated_error / (self.estimated_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimated_error = (1 - kalman_gain) * self.estimated_error + self.process_variance
        return self.estimate
