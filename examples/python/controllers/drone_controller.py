# Import built pybind module
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../build')))
from _uedsDroneController import DroneController as _DroneController, Coordinates, Rotation
import time


class DroneController(_DroneController):
    def __init__(self, address = "127.0.0.1", port = 4000, retries = 400, retry_interval = 10) -> None:
        self.retries = retries
        self.retry_interval = retry_interval
        super().__init__(address, port)
    
    def GetLidarConfig(self):
        for _ in range(self.retries):
            [res, lidar_config] = _DroneController.GetLidarConfig(self)
            if res is True:
                return [res, lidar_config]
            time.sleep(self.retry_interval)
        return [False, None]

    def ConnectSimple(self):
        for _ in range(self.retries):
            res = _DroneController.ConnectSimple(self)
            if res is True:
                return res
            time.sleep(self.retry_interval)
        return False
    
    def SetLidarConfig(self, lidar_config):
        for _ in range(self.retries):
            res = _DroneController.SetLidarConfig(self, lidar_config)
            if res is True:
                return res
            time.sleep(self.retry_interval)
        return False
    
    def GetLidarData(self):
        for _ in range(self.retries):
            [res, lidar_data, start] = _DroneController.GetLidarData(self)
            if res is True:
                return [res, lidar_data, start]
            time.sleep(self.retry_interval)
        return [False, None, None]
    
    def GetLocation(self):
        for _ in range(self.retries):
            [res, location] = _DroneController.GetLocation(self)
            if res is True:
                return [res, location]
            time.sleep(self.retry_interval)
        return [False, None]

    def SetLocation(self, location, check_collisions):
        for _ in range(self.retries):
            [res, teleported_to, is_hit, impact_point] = _DroneController.SetLocation(self, location, check_collisions)
            if res is True:
                return [res, teleported_to, is_hit, impact_point]
            time.sleep(self.retry_interval)
        return [False, None, None, None]
    
    def SetRotation(self, rotation):
        for _ in range(self.retries):
            [res, teleported_to, is_hit, impact_point] = _DroneController.SetRotation(self, rotation)
            if res is True:
                return [res, teleported_to, is_hit, impact_point]
            time.sleep(self.retry_interval)
        return [False, None, None, None]
    
    def GetCameraData(self):
        for _ in range(self.retries):
            try:
                [res, data, size] = _DroneController.GetCameraData(self)
                if size < 100:
                    print(res, size)
                if res is True:
                    return [res, data, size]
            except:
                pass
            time.sleep(self.retry_interval)
        return [False, None, None]