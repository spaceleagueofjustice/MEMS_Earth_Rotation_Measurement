# calibration.py
# 3x MPU6050 gyro calibration via TCA9548A I2C multiplexer
# MicroPython

import time
from mpu6050 import MPU6050   # standard MicroPython MPU6050 driver

# =========================
# CONFIG
# =========================
MUX_ADDR = 0x70     # TCA9548A default address
SENSORS = 3
SAMPLES = 2000      # increase for better accuracy (5000+ ideal)
DELAY = 0.002       # sampling delay

gyro_bias = [[0.0, 0.0, 0.0] for _ in range(SENSORS)]


# =========================
# MUX CONTROL
# =========================
def mux_select(i2c, channel):
    if channel < 0 or channel > 7:
        raise ValueError("Invalid mux channel")

    i2c.writeto(MUX_ADDR, bytes([1 << channel]))


# =========================
# READ GYRO RAW
# =========================
def read_gyro(mpu):
    # returns degrees/sec (depending on driver scaling)
    g = mpu.get_gyro_data()
    return g["x"], g["y"], g["z"]


# =========================
# CALIBRATE ONE SENSOR
# =========================
def calibrate_sensor(i2c, channel, mpu):
    print("Kalibrace senzoru:", channel)

    sum_x = 0.0
    sum_y = 0.0
    sum_z = 0.0

    # stabilizační pauza
    time.sleep(0.5)

    for i in range(SAMPLES):
        mux_select(i2c, channel)

        gx, gy, gz = read_gyro(mpu)

        sum_x += gx
        sum_y += gy
        sum_z += gz

        time.sleep(DELAY)

    bias = [
        sum_x / SAMPLES,
        sum_y / SAMPLES,
        sum_z / SAMPLES
    ]

    print("Bias", channel, "=", bias)
    return bias


# =========================
# FULL CALIBRATION
# =========================
def calibrate_all(i2c, mpus):
    """
    mpus = list of 3 MPU6050 objects (one per mux channel)
    """

    global gyro_bias

    print("=== START CALIBRATION ===")

    for i in range(SENSORS):
        gyro_bias[i] = calibrate_sensor(i2c, i, mpus[i])

    print("=== CALIBRATION DONE ===")
    return gyro_bias


# =========================
# APPLY CALIBRATION
# =========================
def get_gyro_corrected(i2c, channel, mpu):
    mux_select(i2c, channel)

    gx, gy, gz = read_gyro(mpu)

    bx, by, bz = gyro_bias[channel]

    return (
        gx - bx,
        gy - by,
        gz - bz
    )


# =========================
# SAVE / LOAD (volitelné)
# =========================
def save_bias(filename="gyro_bias.txt"):
    with open(filename, "w") as f:
        for b in gyro_bias:
            f.write("{},{},{}\n".format(b[0], b[1], b[2]))


def load_bias(filename="gyro_bias.txt"):
    global gyro_bias

    with open(filename, "r") as f:
        lines = f.readlines()

    for i, line in enumerate(lines):
        x, y, z = line.strip().split(",")
        gyro_bias[i] = [float(x), float(y), float(z)]

    print("Bias loaded:", gyro_bias)
