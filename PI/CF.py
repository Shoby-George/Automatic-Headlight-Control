#!/usr/bin/python

import smbus
import math
import time
import sys

class ComplementaryFilter:
    def __init__(self, angle=0.0, alpha=0.05):
        self.angle = angle
        self.alpha = alpha

    def getAngle(self, newAngle, newRate, dt):
        self.angle = (1 - self.alpha) * (self.angle + newRate * dt) + self.alpha * newAngle
        return self.angle

#Power Management Registers
power_mgmt_1 = 0x6B
power_mgmt_2 = 0x6C

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

def dist(a, b):
    return math.sqrt((a*a)+(b*b))

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
cfX = ComplementaryFilter();
cfY = ComplementaryFilter();

while True:
    gyro_xout = read_word_2c(0x43)
    gyro_yout = read_word_2c(0x45)
    gyro_zout = read_word_2c(0x47)

    accel_xout = read_word_2c(0x3B)
    accel_yout = read_word_2c(0x3D)
    accel_zout = read_word_2c(0x3F)

    accel_xout_scaled = accel_xout/16384.0
    accel_yout_scaled = accel_yout/16384.0
    accel_zout_scaled = accel_zout/16384.0

    accelReadingX = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    gyroReadingX = gyro_xout/131.0
    accelReadingY = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    gyroReadingY = gyro_yout/131.0
    cur_time = time.time()

    dt = cur_time - prev_time
    prev_time = cur_time
    angleX = cfX.getAngle(accelReadingX, gyroReadingX, dt)
    angleY = cfY.getAngle(accelReadingY, gyroReadingY, dt)

    print "{},{},{}".format(dt, angleX, angleY)
    sys.stdout.flush()
