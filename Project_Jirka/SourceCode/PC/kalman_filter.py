# kalman_filter.py
# MEMS 3-axis gyro statistical Kalman filter
# CLIENT SIDE (PC)
# Python 3.x
#
# Designed for:
# - raw MPU6050 gyro streams
# - NO Earth rotation model
# - NO gravity locking
# - NO AHRS
# - NO quaternion stabilization
#
# Purpose:
# - noise reduction
# - bias estimation
# - drift smoothing
#
# Input:
# gx gy gz
#
# Output:
# filtered gyro values
#
# Designed by S.L.O.J. member Jirka Vacek
# in collaboration with Hugo Valmont de AI

import numpy as np


class GyroKalman:

    def __init__(
        self,
        q_angle=0.001,
        q_bias=0.0003,
        r_measure=0.03
    ):

        # =====================================
        # STATE
        # =====================================

        self.angle_x = 0.0
        self.angle_y = 0.0
        self.angle_z = 0.0

        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_z = 0.0

        # =====================================
        # NOISE PARAMETERS
        # =====================================

        self.q_angle = q_angle
        self.q_bias = q_bias
        self.r_measure = r_measure

        # =====================================
        # COVARIANCE MATRICES
        # =====================================

        self.Px = np.zeros((2, 2))
        self.Py = np.zeros((2, 2))
        self.Pz = np.zeros((2, 2))

    # =========================================
    # SINGLE AXIS UPDATE
    # =========================================

    def kalman_update(
        self,
        rate,
        angle,
        bias,
        P,
        dt
    ):

        # -------------------------------------
        # PREDICTION
        # -------------------------------------

        rate_unbiased = rate - bias

        angle += dt * rate_unbiased

        # covariance prediction

        P[0][0] += dt * (
            dt * P[1][1]
            - P[0][1]
            - P[1][0]
            + self.q_angle
        )

        P[0][1] -= dt * P[1][1]

        P[1][0] -= dt * P[1][1]

        P[1][1] += self.q_bias * dt

        # -------------------------------------
        # MEASUREMENT
        # -------------------------------------

        # NO external orientation reference
        # measurement assumed = 0

        innovation = 0.0 - angle

        # innovation covariance

        S = P[0][0] + self.r_measure

        # Kalman gain

        K0 = P[0][0] / S
        K1 = P[1][0] / S

        # -------------------------------------
        # UPDATE
        # -------------------------------------

        angle += K0 * innovation

        bias += K1 * innovation

        # covariance update

        P00 = P[0][0]
        P01 = P[0][1]

        P[0][0] -= K0 * P00
        P[0][1] -= K0 * P01

        P[1][0] -= K1 * P00
        P[1][1] -= K1 * P01

        return angle, bias, P

    # =========================================
    # FULL 3-AXIS UPDATE
    # =========================================

    def update(
        self,
        gx,
        gy,
        gz,
        dt
    ):

        self.angle_x, self.bias_x, self.Px = \
            self.kalman_update(
                gx,
                self.angle_x,
                self.bias_x,
                self.Px,
                dt
            )

        self.angle_y, self.bias_y, self.Py = \
            self.kalman_update(
                gy,
                self.angle_y,
                self.bias_y,
                self.Py,
                dt
            )

        self.angle_z, self.bias_z, self.Pz = \
            self.kalman_update(
                gz,
                self.angle_z,
                self.bias_z,
                self.Pz,
                dt
            )

        return {

            "x": self.angle_x,
            "y": self.angle_y,
            "z": self.angle_z,

            "bias_x": self.bias_x,
            "bias_y": self.bias_y,
            "bias_z": self.bias_z

        }


# =============================================
# EXAMPLE USAGE
# =============================================

if __name__ == "__main__":

    import time
    import random

    kalman = GyroKalman()

    last_time = time.time()

    while True:

        now = time.time()

        dt = now - last_time

        last_time = now

        # simulated gyro noise

        gx = random.uniform(-0.5, 0.5)
        gy = random.uniform(-0.5, 0.5)
        gz = random.uniform(-0.5, 0.5)

        result = kalman.update(
            gx,
            gy,
            gz,
            dt
        )

        print(
            "Filtered:",
            round(result["x"], 5),
            round(result["y"], 5),
            round(result["z"], 5)
        )

        print(
            "Bias:",
            round(result["bias_x"], 5),
            round(result["bias_y"], 5),
            round(result["bias_z"], 5)
        )

        print("--------------------------")

        time.sleep(0.01)
