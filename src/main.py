from machine import Pin, I2C
from mpu9250 import MPU9250
from fusion import Fusion
import utime as time


class IMU_Collection:
    def __init__(
        self,
        i2c_id: int = 0,
        i2c_scl: Pin = Pin(1),
        i2c_sda: Pin = Pin(0),
        verbose: bool = False,
        motion_threshold: int = 1,
    ) -> None:
        self.fuse = Fusion()
        i2c = I2C(i2c_id, scl=i2c_scl, sda=i2c_sda)
        self.imu = MPU9250(i2c)

        self.motion_threshold = motion_threshold
        self.__verbose__ = verbose
        self.motion_count = 0
        self.stddev = []
        self.mean = []

        self.ax_prev = 0
        self.ay_prev = 0
        self.az_prev = 0

        self.heading_prev = 0
        self.pitch_prev = 0
        self.roll_prev = 0

    @property
    def inmotion(self) -> bool:
        self.update()
        inmotion = False
        # ax, ay, az = self.imu.accel.xyz
        heading, roll, pitch = self.fuse.heading, self.fuse.roll, self.fuse.pitch

        if (
            abs(heading - self.heading_prev) > self.motion_threshold
            or abs(pitch - self.pitch_prev) > self.motion_threshold
            or abs(roll - self.roll_prev) > self.motion_threshold
        ):
            self.motion_count += 1
            inmotion = True

            if self.__verbose__:
                print("motion:", self.motion_count)

        else:
            self.motion_count = 0
            inmotion = False

        self.heading_prev = heading
        self.pitch_prev = pitch
        self.roll_prev = roll

        return inmotion

    def update(self) -> tuple:
        return self.fuse.update(self.imu.accel.xyz, self.imu.gyro.xyz, self.imu.mag.xyz)


class Gesture:
    def __init__(self) -> None:
        pass


if __name__ == "__main__":
    collect = IMU_Collection(motion_threshold=0.85)
    print("heading,roll,pitch,count")
    count = 0
    while True:
        if collect.inmotion and collect.motion_count > 5:
            print(f"{collect.fuse.heading},{collect.fuse.roll},{collect.fuse.pitch},{count}")
            count += 1
