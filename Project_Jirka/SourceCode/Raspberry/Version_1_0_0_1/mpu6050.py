from machine import I2C
import time

class MPU6050:

    MPU_ADDR = 0x68

    PWR_MGMT_1 = 0x6B

    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H  = 0x43

    def __init__(self, i2c):

        self.i2c = i2c

        self.i2c.writeto_mem(
            self.MPU_ADDR,
            self.PWR_MGMT_1,
            b'\x00'
        )

        time.sleep(0.1)

    def read_raw_data(self, addr):

        high = self.i2c.readfrom_mem(
            self.MPU_ADDR,
            addr,
            1
        )[0]

        low = self.i2c.readfrom_mem(
            self.MPU_ADDR,
            addr + 1,
            1
        )[0]

        value = (high << 8) | low

        if value > 32768:
            value -= 65536

        return value

    def read_accel_data(self):

        ax = self.read_raw_data(self.ACCEL_XOUT_H)
        ay = self.read_raw_data(self.ACCEL_XOUT_H + 2)
        az = self.read_raw_data(self.ACCEL_XOUT_H + 4)

        return {
            "x": ax,
            "y": ay,
            "z": az
        }

    def read_gyro_data(self):

        gx = self.read_raw_data(self.GYRO_XOUT_H)
        gy = self.read_raw_data(self.GYRO_XOUT_H + 2)
        gz = self.read_raw_data(self.GYRO_XOUT_H + 4)

        return {
            "x": gx,
            "y": gy,
            "z": gz
        }
