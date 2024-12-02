# Import built pybind module
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../build')))
from _uedsGameModeController import GameModeController as _GameModeController, CameraCaptureModeEnum
import time

class GameModeController(_GameModeController):
    def __init__(self, address = "127.0.0.1", port = 8000, retries = 400, retry_interval = 10) -> None:
        super().__init__(address, port)
        self.retries = retries
        self.retry_interval = retry_interval
    
    def resetEnvironment(self, verbose=False):
        [res, drones] = self.GetDrones()
        assert res == True

        for drone in drones:
            res = self.RemoveDrone(drone)
            if verbose:
                print('Removed drone at port {}'.format(drone))
    
    def GetFps(self):
        for _ in range(self.retries):
            [res, fps] = _GameModeController.GetFps(self)
            if res is True:
                return [res, fps]
            time.sleep(self.retry_interval)
        return [False, None]