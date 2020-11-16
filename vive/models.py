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
               f"x: {self.x}, y: {self.y}, z: {self.z} | " \
               f"pitch: {self.pitch}, yaw: {self.yaw}, roll: {self.roll} | " \
               f"vel_x: {self.vel_x}, vel_y: {self.vel_y}, vel_z: {self.vel_z}"

    def __str__(self):
        return self.__repr__()

    @validator('x')
    def round_x(cls, v):
        return int(v*1000)

    @validator('y')
    def round_y(cls, v):
        return int(v*1000)

    @validator('z')
    def round_z(cls, v):
        return int(v*1000)

    @validator('roll')
    def round_roll(cls, v):
        return int(v)

    @validator('pitch')
    def round_pitch(cls, v):
        return int(v)

    @validator('yaw')
    def round_yaw(cls, v):
        return int(v)

    @validator('vel_x')
    def round_vel_x(cls, v):
        return int(v*1000)

    @validator('vel_y')
    def round_vel_y(cls, v):
        return int(v*1000)

    @validator('vel_z')
    def round_vel_z(cls, v):
        return int(v*1000)

    @validator('device_name')
    def cat_devicename(cls, v):
        return v[:9]
