
from __future__ import annotations

from typing import TYPE_CHECKING

from dataclasses import dataclass

import newton
import numpy as np

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu

from newtonian.modules.plugins.sensors.base import SENSORS, Cache, SensorFixed, noise

if TYPE_CHECKING:
    from newtonian.modules.physics.simulator import NewtonianSimulator


@dataclass
class _Data:
    """
    """
    quat: np.ndarray
    gyro: np.ndarray
    accel: np.ndarray


class _Base(SensorFixed):
    """
    """
    msg_type = Imu

    def on_model(self, sim: NewtonianSimulator) -> None:
        """
        """
        super().on_model(sim=sim)
        p = self.p

        self.imu = newton.sensors.SensorIMU(model=sim.model, sites=[self.site])
        self.accel_std = p.get_float(key="accel_noise_std", default=0.0, at_least=0.0)
        self.gyro_std = p.get_float(key="gyro_noise_std", default=0.0, at_least=0.0)
        self.publish_orientation = p.get_bool(key="orientation", default=True)

    def _imu(self, stamp: Time, data: _Data) -> Imu:
        """
        """
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        if self.publish_orientation:
            msg.orientation.x = float(data.quat[0])
            msg.orientation.y = float(data.quat[1])
            msg.orientation.z = float(data.quat[2])
            msg.orientation.w = float(data.quat[3])
        else:
            msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = float(data.gyro[0])
        msg.angular_velocity.y = float(data.gyro[1])
        msg.angular_velocity.z = float(data.gyro[2])
        msg.linear_acceleration.x = float(data.accel[0])
        msg.linear_acceleration.y = float(data.accel[1])
        msg.linear_acceleration.z = float(data.accel[2])

        msg.angular_velocity_covariance = np.diag([self.gyro_std**2] * 3).flatten().tolist()
        msg.linear_acceleration_covariance = np.diag([self.accel_std**2] * 3).flatten().tolist()

        return msg


@SENSORS.register("imu")
class SensorFixedImu(_Base):
    """
    """

    def to_msg(self, stamp: Time) -> Imu:
        """
        """
        return self._imu(stamp=stamp, data=self.data)

    def update(self, sim: NewtonianSimulator, cache: Cache) -> None:
        """
        """
        self.imu.update(state=sim.state)
        self.data = _Data(
            quat=self.pose(cache=cache)[3:],
            gyro=self.imu.gyroscope.numpy()[0] + noise(sim=sim, std=self.gyro_std, shape=3),
            accel=self.imu.accelerometer.numpy()[0] + noise(sim=sim, std=self.accel_std, shape=3),
        )
