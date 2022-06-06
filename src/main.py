from machine import Pin, I2C
from mpu9250 import MPU9250
from fusion import Fusion
import utime as time


class IMU:
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

        # self.fuse.beta = 1.33011956

        for _ in range(300):
            self.fuse.update(self.imu.accel.xyz, self.imu.gyro.xyz, self.imu.mag.xyz)

    @property
    def inmotion(self) -> bool:
        self.update()
        _inmotion = False
        # ax, ay, az = self.imu.accel.xyz
        heading, roll, pitch = self.fuse.heading, self.fuse.roll, self.fuse.pitch

        if (
            abs(heading - self.heading_prev) > self.motion_threshold
            or abs(pitch - self.pitch_prev) > self.motion_threshold
            or abs(roll - self.roll_prev) > self.motion_threshold
        ):
            self.motion_count += 1
            _inmotion = True

            if self.__verbose__:
                print("motion:", self.motion_count)

        else:
            self.motion_count = 0
            _inmotion = False

        self.heading_prev = heading
        self.pitch_prev = pitch
        self.roll_prev = roll

        return _inmotion

    def update(self, rate=30) -> tuple:
        time.sleep_ms(rate)
        return self.fuse.update(self.imu.accel.xyz, self.imu.gyro.xyz, self.imu.mag.xyz)


class Motion:
    def __init__(self, imu_: IMU) -> None:
        self.imu: IMU = imu_
        self.samples: list = []
        self.__prev_samples__: list = []

    def update(self, rate=20) -> None:
        self.imu.update(rate=rate)
        if self.imu.inmotion and self.imu.motion_count > 3:
            self.samples.append((self.imu.fuse.heading, self.imu.fuse.roll, self.imu.fuse.pitch))
        else:
            self.samples.clear()


if __name__ == "__main__":
    imu = IMU(motion_threshold=3)
    motion = Motion(imu)

    prev_3 = [False] * 3
    curr = ""

    while True:
        motion.update(rate=0)
        ax, ay, az = imu.imu.accel.xyz
        gx, gy, gz = imu.imu.gyro.xyz
        inmotion = imu.inmotion

        curr = [imu.fuse.heading, imu.fuse.roll, imu.fuse.pitch, ax, ay, az, gx, gy, gz, inmotion]
        prev_3.append(curr[-1])

        if len(prev_3) > 3:
            prev_3.pop(0)

        if not curr[-1] and all(prev_3):
            inmotion = True
            curr[-1] = inmotion

        print("".join([str(v) + ("," if i < len(curr) - 1 else "") for i, v in enumerate(curr)]))
