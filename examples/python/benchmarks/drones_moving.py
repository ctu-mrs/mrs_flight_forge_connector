import sys
sys.path.append('../')
sys.path.append('.')
from utils import get_game_mode_controller, get_drone_controller
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
    print(lidar_config)
    assert res is True
    lidar_config.beamCount = 0
    lidar_config.showBeams = False
    res = drone_controller.SetLidarConfig(lidar_config)
    print(lidar_config)
    assert res is True

    return port, drone_controller

def print_data_to_csv(data, log_path):
    keys = data[0].keys()

    with open(log_path, 'w', newline='') as output_file:
        dict_writer = csv.DictWriter(output_file, keys)
        dict_writer.writeheader()
        dict_writer.writerows(data)

BENCHMARK_TIME = 60

def move_randomly(start_time, drone_controller, log_path):
    def move():
        times = []

        while time.time() - start_time < BENCHMARK_TIME:
            get_location_start_time = time.time()
            [res, location] = drone_controller.GetLocation()
            get_location_end_time = time.time()
            assert res is True

            location.x += random.uniform(-10, 10)
            location.y += random.uniform(-10, 10)
            location.z += random.uniform(-10, 10)

            set_location_start_time = time.time()
            [res, teleported_to, is_hit, impact_point] = drone_controller.SetLocation(location, True)
            set_location_end_time = time.time()
            assert res is True

            times.append({
                "get_location_response_time_ms": (get_location_end_time - get_location_start_time) * 1000,
                "set_location_response_time_ms": (set_location_end_time - set_location_start_time) * 1000,
                "get_location_start_time": get_location_start_time,
                "set_location_start_time": set_location_start_time,
            })
        print_data_to_csv(times, log_path)
    return move

def log_fps(start_time, game_mode_controller, log_path):
    def log():
        times = []

        while time.time() - start_time < BENCHMARK_TIME:
            fps_start_time = time.time()
            [res, fps] = game_mode_controller.GetFps()
            assert res is True

            times.append({
                "fps": fps,
                "fps_time": fps_start_time,
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

if __name__ == "__main__":
    game_mode_controller = get_game_mode_controller()

    data_path = os.path.join(os.path.abspath(os.path.join(os.path.dirname(__file__), './data/drones_moving')))
    
    number_of_drones_list = [1, 2, 4, 8, 16, 24, 32, 64, 128]
    for number_of_drones in number_of_drones_list:
        drone_controllers = [spawn_drone(game_mode_controller) for _ in range(number_of_drones)]
        print(drone_controllers)
        time.sleep(2)

        log_base = os.path.join(data_path, f'./drones_moving_{number_of_drones}')

        start_time = time.time()
        functions = []
        for i in range(len(drone_controllers)):
            functions.append(move_randomly(start_time, drone_controllers[i][1], f'{log_base}_drone_{i}.csv'))
        functions.append(log_fps(start_time, game_mode_controller, f'{log_base}_fps.csv'))

        benchmark(functions)

        for [port, drone_controller] in drone_controllers:
            res = game_mode_controller.RemoveDrone(port)
            assert res is True
