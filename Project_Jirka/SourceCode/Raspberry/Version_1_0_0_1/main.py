from machine import Pin, I2C
import time
import dht
from mpu6050 import MPU6050

# =========================
# I2C
# =========================

i2c = I2C(
    0,
    sda=Pin(0),
    scl=Pin(1),
    freq=400000
)

TCA_ADDR = 0x70

# =========================
# DHT11
# =========================

dht_sensor = dht.DHT11(Pin(15))

# =========================
# TCA9548A channel select
# =========================

def select_channel(channel):
    if channel > 7:
        return
    i2c.writeto(TCA_ADDR, bytes([1 << channel]))

# =========================
# MPU instances
# =========================

select_channel(0)
gyro_x = MPU6050(i2c)

select_channel(1)
gyro_y = MPU6050(i2c)

select_channel(2)
gyro_z = MPU6050(i2c)

# =========================
# Main loop
# =========================

while True:

    # --- DHT11 ---
    try:
        dht_sensor.measure()
        temp_dht = dht_sensor.temperature()
        hum = dht_sensor.humidity()
    except:
        temp_dht = -1
        hum = -1

    # --- Gyro X ---
    select_channel(0)
    gx = gyro_x.read_gyro_data()
    ax = gyro_x.read_accel_data()

    # --- Gyro Y ---
    select_channel(1)
    gy = gyro_y.read_gyro_data()
    ay = gyro_y.read_accel_data()

    # --- Gyro Z ---
    select_channel(2)
    gz = gyro_z.read_gyro_data()
    az = gyro_z.read_accel_data()

    # =========================
    # SERIAL OUTPUT
    # =========================

    print(
        "GX:", gx,
        "GY:", gy,
        "GZ:", gz,
        "AX:", ax,
        "AY:", ay,
        "AZ:", az,
        "TEMP:", temp_dht,
        "HUM:", hum
    )

    time.sleep(1)
