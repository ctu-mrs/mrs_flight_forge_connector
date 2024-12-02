import sys
sys.path.append('../')
sys.path.append('.')
from utils import get_game_mode_controller, get_drone_controller
from controllers import CameraCaptureModeEnum, Coordinates, Rotation
import random
from multiprocessing import Process
import time
import os
import csv

def spawn_drone(game_mode_controller):
    [res, port] = game_mode_controller.SpawnDrone()
    assert res is True

    drone_controller = get_drone_controller(port)

    [res, lidar_config] = drone_controller.GetLidarConfig()
    assert res is True
    lidar_config.beamCount = 0
    lidar_config.showBeams = False
    res = drone_controller.SetLidarConfig(lidar_config)
    assert res is True

    return port, drone_controller

def print_data_to_csv(data, log_path):
    keys = data[0].keys()

    with open(log_path, 'w', newline='') as output_file:
        dict_writer = csv.DictWriter(output_file, keys)
        dict_writer.writeheader()
        dict_writer.writerows(data)

def move(drone_controller, last_move_was_left):
    [res, location] = drone_controller.GetLocation()
    assert res is True

    if last_move_was_left:
        location.y += 10
    else:
        location.y -= 10

    [res, teleported_to, is_hit, impact_point] = drone_controller.SetLocation(location, False)
    assert res is True

BENCHMARK_TIME = 30

def bench_camera(start_time, drone_controller, log_path):
    def camera():
        times = []

        last_move_time = start_time
        last_move_was_left = False

        last_ask_time = start_time

        while time.time() - start_time < BENCHMARK_TIME:
            camera_start_time = None

            
            duration = time.time() - start_time
            # stationary and ask every time
            if duration <= 15:
                camera_start_time = time.time()
                [res, data, size] = drone_controller.GetCameraData()
                camera_end_time = time.time()
                assert res is True
            # move and ask every time
            else:
                # if time.time() - last_move_time > 0.25:
                #     move(drone_controller, last_move_was_left)
                #     last_move_was_left = not last_move_was_left
                #     last_move_time = time.time()

                move(drone_controller, last_move_was_left)
                last_move_was_left = not last_move_was_left

                camera_start_time = time.time()
                [res, data, size] = drone_controller.GetCameraData()
                camera_end_time = time.time()
                assert res is True
            # # ask every 250ms
            # elif time.time() - last_ask_time > 0.25:
            #         camera_start_time = time.time()
            #         [res, data, size] = drone_controller.GetCameraData()
            #         camera_end_time = time.time()
            #         assert res is True
            #         last_ask_time = time.time()

            if camera_start_time is not None:
                times.append({
                    "camera_response_time_ms": (camera_end_time - camera_start_time) * 1000,
                    "time": camera_start_time - start_time,
                })
        print_data_to_csv(times, log_path)
    return camera

def log_fps(start_time, game_mode_controller, log_path):
    def log():
        times = []

        while time.time() - start_time < BENCHMARK_TIME:
            fps_start_time = time.time()
            [res, fps] = game_mode_controller.GetFps()
            assert res is True

            times.append({
                "fps": fps,
                "time": fps_start_time-start_time,
            })
            time.sleep(5/1000)
        print_data_to_csv(times, log_path)
    return log

def benchmark(functions):
    processes = []
    for fn in functions:
        p = Process(target=fn)
        p.start()
        processes.append(p)
    for p in processes:
        p.join()

def setCaputreMode(game_mode_controller, captureMode):
    res = game_mode_controller.SetCameraCaptureMode(captureMode)
    assert res is True

envs_locations = {
    'easy': {
        'loc': Coordinates(-3750, -10420, 1370),
        'rot': Rotation(0, 0, 0),
    },
    'hard': {
        'loc': Coordinates(9930.0, -3200, -20),
        'rot': Rotation(0, 20, 0),
    },
}

def move_to_location(env, drone_controller):
    [res, teleported_to, is_hit, impact_point] = drone_controller.SetLocation(envs_locations[env]['loc'], False)
    assert res is True

    [res, teleported_to, is_hit, impact_point] = drone_controller.SetRotation(envs_locations[env]['rot'])
    assert res is True


if __name__ == "__main__":
    game_mode_controller = get_game_mode_controller()

    data_path = os.path.join(os.path.abspath(os.path.join(os.path.dirname(__file__), './data/camera')))
    
    number_of_drones_list = [1, 2, 4, 8, 16, 24]
    camera_modes = [CameraCaptureModeEnum.CAPTURE_ALL_FRAMES, CameraCaptureModeEnum.CAPTURE_ON_MOVEMENT, CameraCaptureModeEnum.CAPTURE_ON_DEMAND]
    envs = ['easy', 'hard']
    for number_of_drones in number_of_drones_list:
        drone_controllers = [spawn_drone(game_mode_controller) for _ in range(number_of_drones)]
        print(drone_controllers)

        for camera_mode in camera_modes:
            setCaputreMode(game_mode_controller, camera_mode)
            print("Camera mode set to: ", camera_mode)

            for env in envs:
                for i in range(len(drone_controllers)):
                    move_to_location(env, drone_controllers[i][1])
                print("Moved to location: ", env)
                time.sleep(6)

                log_base = os.path.join(data_path, f'./camera_{number_of_drones}_{env}_{str(camera_mode).split(".")[-1]}')
                print(log_base)

                start_time = time.time()
                functions = []
                for i in range(len(drone_controllers)):
                    functions.append(bench_camera(start_time, drone_controllers[i][1], f'{log_base}_drone_{i}.csv'))
                functions.append(log_fps(start_time, game_mode_controller, f'{log_base}_fps.csv'))

                benchmark(functions)

        for [port, drone_controller] in drone_controllers:
            res = game_mode_controller.RemoveDrone(port)
            assert res is True
