import gc
from machine import Pin, I2C
from mpu9250 import MPU9250
from fusion import Fusion
import utime as time

gc.enable()

NUM_SAMPLES = 1
NUM_AXES = 3

sample = [[], [], []]

MOTION_THRESHOLD = 1
accel_previous = 0
norm_previous = 0
in_motion = False
motion_count = 0

motion_window = []

i2c = I2C(0, scl=Pin(1), sda=Pin(0))
imu = MPU9250(i2c)
fuse = Fusion()

# Code for external switch
switch = Pin(2, Pin.IN, pull=Pin.PULL_UP)  # Switch to ground on Y7


def sw():
    return not switch.value()


# Code for Pyboard switch
# sw = pyb.Switch()

# Choose test to run
Calibrate = False
Timing = True


def getmag():  # Return (x, y, z) tuple (blocking read)
    return imu.mag.xyz


if Calibrate:
    print("Calibrating. Press switch when done.")
    fuse.calibrate(getmag, sw, lambda: time.sleep_ms(100))
    print(fuse.magbias)

fuse.magbias = (84.65625, -11.23652, 104.5213)

if Timing:
    mag = imu.mag.xyz  # Don't include blocking read in time
    accel = imu.accel.xyz  # or i2c
    gyro = imu.gyro.xyz
    start = time.ticks_us()  # Measure computation time only
    fuse.update(accel, gyro, mag)  # 1.97mS on Pyboard
    t = time.ticks_diff(time.ticks_us(), start)
    print("Update time (uS):", t)


def inmotion():
    global motion_count, in_motion
    # global norm_previous, motion_count
    # global motion_count, norm_previous, in_motion

    # start = time.ticks_us()
    # ax, ay, az = imu.accel.xyz
    # gx, gy, gz = imu.gyro.xyz
    # mx, my, mz = imu.mag.xyz
    # t = time.ticks_diff(time.ticks_us(), start)

    # agmx = 0.33 * ax + 0.33 * gx + 0.33 * mx
    # agmy = 0.33 * ay + 0.33 * gy + 0.33 * my
    # agmz = 0.33 * az + 0.33 * gz + 0.33 * mz

    # norm_current = sqrt(agmx * agmx + agmy * agmy + agmz * agmz)

    # motion_val = abs(norm_current - norm_previous)

    # if motion_val > 0.5:
    #     motion_count += 1
    #     in_motion = True
    # else:
    #     in_motion = False
    #     motion_count = 0

    # print("motion", motion_count, end="      \r")
    # norm_previous = norm_current
    # ax, ay, az = imu.accel.xyz
    # norm = sqrt(ax * ax + ay * ay + az * az)
    # norm *= 10
    # print(norm - norm_previous)
    # if norm - norm_previous > MOTION_THRESHOLD:
    #     in_motion = True
    # else:
    #     motion_count = 0
    #     in_motion = False

    ax, ay, az = imu.accel.xyz

    if ax > MOTION_THRESHOLD or ay > MOTION_THRESHOLD or az > MOTION_THRESHOLD:
        in_motion = True
        motion_count += 1
        print("motion")
    else:
        in_motion = False
        motion_count = 0

    print("motion:", motion_count, end="      \r")
    # norm_previous = norm
    # print(norm, ",", motion_count)
    # motion_count += 1


count = 0
print("100ax,100ay,100az,count")
while True:
    # inmotion()
    fuse.update(imu.accel.xyz, imu.gyro.xyz, imu.mag.xyz)
    time.sleep_ms(20)
    ax, ay, az = imu.accel.xyz
    print(f"{ax*100},{ay*100},{az*100},{count}")
    count += 1
    gc.collect()
