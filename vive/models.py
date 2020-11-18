from pydantic import BaseModel, Field, validator
import math


class ViveTrackerMessage(BaseModel):
    valid: int = Field(default=0)
    x: float = Field(default=0)
    y: float = Field(default=0)
    z: float = Field(default=0)
    roll: float = Field(default=0)
    pitch: float = Field(default=0)
    yaw: float = Field(default=0)
    device_name: str = Field(default="tracker_1")
    vel_x: float = Field(default=0)
    vel_y: float = Field(default=0)
    vel_z: float = Field(default=0)

    def __repr__(self):
        return f"device name: {self.device_name} -> " \
               f"x: {round(self.x, 5)}, y: {round(self.y)}, z: {round(self.z)} | " \
               f"pitch: {round(self.pitch)}, yaw: {round(self.yaw)}, roll: {round(self.roll)} | " \
               f"vel_x: {round(self.vel_x)}, vel_y: {round(self.vel_y)}, vel_z: {round(self.vel_z)}"

    def __str__(self):
        return self.__repr__()


