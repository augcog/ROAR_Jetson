class Controller:
    def __init__(self):
        self.new_steering = 0.0
        self.new_throttle = 0.0

    def update(self):
        # update throttle and steering
        pass

    def run_threaded(self, **args):
        return self.new_throttle, self.new_steering


class NaiveController(Controller):
    def __init__(self):
        Controller.__init__(self)

    def update(self):
        while True:
            try:
                x, y = input().split()
                self.new_throttle, self.new_steering = float(x), float(y)
            except:
                pass


# Template of custom Controller class
class MyController(Controller):
    def __init__(self):
        Controller.__init__(self)
        # Other initialization

    # This function lives in another thread
    def update(self):
        # Logic asynchronous with main vehicle loop
        # that you do not want to block main thread
        # goes here
        while True:
            pass

    # This function is called in every vehicle loop
    # to get new throttle and steering value
    def run_threaded(self, image_array, depth_array,
                     imu_acl_x, imu_acl_y, imu_acl_z,
                     imu_gyr_x, imu_gyr_y, imu_gyr_z, **args):
        # Update throttle and steering according to camera data
        return self.new_throttle, self.new_steering

    # If ENABLE_CSIC is True, parameter list is different
    # def run_threaded(self, image_array, image_array_rear, depth_array,
    #                  imu_acl_x, imu_acl_y, imu_acl_z,
    #                  imu_gyr_x, imu_gyr_y, imu_gyr_z, **args):
    #     pass
