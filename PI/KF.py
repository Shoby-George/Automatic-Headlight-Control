#!/usr/bin/python

import smbus
import math
import time
import sys
import numpy as np

class KalmanFilter:
    def __init__(self, angle=0.0, bias=0.0, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        self.Q_angle = Q_angle     # Process Noise
        self.Q_bias = Q_bias
        self.R_measure = R_measure    # Measurement Noise

        self.angle = angle
        self.bias = bias

        self.P = np.zeros((2, 2))

    def getAngle(self, newAngle, newRate, dt):
        rate = newRate - self.bias
        self.angle += dt * rate

        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        y = newAngle - self.angle

        S = self.P[0][0] + self.R_measure

        K0 = self.P[0][0]/S
        K1 = self.P[1][0]/S

        self.angle += K0 * y
        self.bias += K1 * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K0 * P00_temp
        self.P[0][1] -= K0 * P01_temp
        self.P[1][0] -= K1 * P00_temp
        self.P[1][1] -= K1 * P01_temp

        return self.angle

# Power Management Registers
power_mgmt_1 = 0x6B
power_mgmt_2 = 0x6C

def read_byte(adr):
    return bus.read_byte_data(address, adr)


def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr + 1)
    val = (high << 8) + low
    return val


def read_word_2c(adr):
    val = read_word(adr)
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val


def dist(a, b):
    return math.sqrt((a * a) + (b * b))


def get_y_rotation(x, y, z):
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)


def get_x_rotation(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)


bus = smbus.SMBus(1)
address = 0x68

# Wake up 6050 as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

prev_time = time.time()
kfX = KalmanFilter()
kfY = KalmanFilter()

while True:
    gyro_xout = read_word_2c(0x43)
    gyro_yout = read_word_2c(0x45)
    gyro_zout = read_word_2c(0x47)

    accel_xout = read_word_2c(0x3B)
    accel_yout = read_word_2c(0x3D)
    accel_zout = read_word_2c(0x3F)

    accel_xout_scaled = accel_xout / 16384.0
    accel_yout_scaled = accel_yout / 16384.0
    accel_zout_scaled = accel_zout / 16384.0

    accelReadingX = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    gyroReadingX = gyro_xout / 131.0
    accelReadingY = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    gyroReadingY = gyro_yout / 131.0
    cur_time = time.time()

    dt = cur_time - prev_time
    prev_time = cur_time
    angleX = kfX.getAngle(accelReadingX, gyroReadingX, dt)
    angleY = kfY.getAngle(accelReadingY, gyroReadingY, dt)

    print "{},{},{}".format(dt, angleX, angleY)
    sys.stdout.flush()


