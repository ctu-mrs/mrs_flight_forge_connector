import sys
sys.path.append('../')
sys.path.append('.')
from utils import get_game_mode_controller, get_drone_controller
from controllers import Coordinates
import random
from multiprocessing import Process
import time
import os
import csv

def spawn_drone(game_mode_controller):
    [res, port] = game_mode_controller.SpawnDrone()
    assert res is True

    drone_controller = get_drone_controller(port)

    [res, teleported_to, is_hit, impact_point] = drone_controller.SetLocation(
        Coordinates(-3740, -10240, 1370),
        False
        )
    assert res is True
    assert is_hit is False

    return port, drone_controller

def set_beam_count(drone_controller, beam_count):
    [res, lidar_config] = drone_controller.GetLidarConfig()
    assert res is True
    lidar_config.beamCount = beam_count
    lidar_config.showBeams = False
    res = drone_controller.SetLidarConfig(lidar_config)

    [res, lidar_config] = drone_controller.GetLidarConfig()
    print(lidar_config)

    assert res is True

def print_data_to_csv(data, log_path):
    keys = data[0].keys()

    with open(log_path, 'w', newline='') as output_file:
        dict_writer = csv.DictWriter(output_file, keys)
        dict_writer.writeheader()
        dict_writer.writerows(data)

BENCHMARK_TIME = 30

def get_lidar_data(start_time, drone_controller, log_path):
    def move():
        times = []

        while time.time() - start_time < BENCHMARK_TIME:
            get_lidar_data_start = time.time()
            [res, lidar_data, start] = drone_controller.GetLidarData()
            assert res is True
            get_lidar_data_end = time.time()

            times.append({
                "get_location_response_time_ms": (get_lidar_data_end - get_lidar_data_start) * 1000,
                "get_lidar_data_start": get_lidar_data_start,
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

    data_path = os.path.join(os.path.abspath(os.path.join(os.path.dirname(__file__), './data/lidar')))

    beam_count = [35, 125, 250, 500, 1000]
    number_of_drones_list = [1, 2, 4, 8, 16, 24, 32, 64, 128]
    for number_of_drones in number_of_drones_list:
        drone_controllers = [spawn_drone(game_mode_controller) for _ in range(number_of_drones)]
        print('Drone controllers length', len(drone_controllers))
        time.sleep(2)

        for beams in beam_count:
            log_base = os.path.join(data_path, f'./drones_{number_of_drones}_beams_{beams}')
            for [port, drone_controller] in drone_controllers:
                set_beam_count(drone_controller, beams)
            time.sleep(2)

            start_time = time.time()
            functions = []
            for i in range(len(drone_controllers)):
                functions.append(get_lidar_data(start_time, drone_controllers[i][1], f'{log_base}_drone_{i}_responses.csv'))
            functions.append(log_fps(start_time, game_mode_controller, f'{log_base}_fps.csv'))

            benchmark(functions)
        
        for [port, drone_controller] in drone_controllers:
            res = game_mode_controller.RemoveDrone(port)
            assert res is True
