from controllers import GameModeController, DroneController, Coordinates, Rotation

import gym
from gym import spaces
import numpy as np
import cv2
from PIL import Image

class UEdsCameraEnv(gym.Env):
    def __init__(
            self,
            game_mode_controller: GameModeController,
            lidar_beam_count=6,
            lidar_beam_length=400,
            lidar_angle=45,
            action_step=10,
            init_location=(1600, -12200, 1000),
            goal_location=(2400, -12500, 1000),
            goal_threshold=50,
            goal_reward=10000,
            max_obstacle_distance = 100,
            max_steps=3000,
            draw_move_line=False,
    ):
        self.last_location = None
        self.game_mode_controller = game_mode_controller
        self.lidar_beam_count = lidar_beam_count
        self.lidar_beam_length = lidar_beam_length
        self.lidar_angle = lidar_angle
        self.action_step = action_step
        self.max_obstacle_distance_normalized = max_obstacle_distance / lidar_beam_length
        self.max_steps = max_steps
        self.draw_move_line = draw_move_line

        self.init_location = np.array([init_location[0], init_location[1], init_location[2]], dtype=np.double)

        goal_location = np.array([goal_location[0], goal_location[1], goal_location[2]], dtype=np.double)
        self.goal = {"loc": goal_location, "reward": goal_reward, "threshold": goal_threshold}

        self.max_free_space_reward = (lidar_beam_count / self.max_obstacle_distance_normalized) - lidar_beam_count if lidar_beam_count > 0 else 1

        self.last_distance = self.calculate_disance(self.init_location, self.goal["loc"])

        self.drone_port = None
        self.drone_controller = None
        self.steps = 0
        self.done = None
        self.last_action = None

        self.action_space = spaces.Discrete(3)
        self._action_to_direction = {
            0: np.array([action_step, 0, 0], dtype=np.double),
            1: np.array([0, action_step, 0], dtype=np.double),
            2: np.array([0, -action_step, 0], dtype=np.double)
        }

        self.observation_space = spaces.Box(low=-1, high=1, shape=(2 + lidar_beam_count,), dtype=np.double)

    def calculate_disance(self, A, B):
        return np.linalg.norm(A - B)
    
    def get_goal_direction(self, camera_data):
        img = cv2.imdecode(np.uint8(camera_data), 1)

        lower = np.array([150, 230, 100])
        upper = np.array([200, 255, 190])

        mask = cv2.inRange(img, lower, upper)
        
        bbox = Image.fromarray(mask).getbbox()
        if bbox is not None:
            bbox_height_mid = (bbox[1] + bbox[3]) / 2
            bbox_width_mid = (bbox[0] + bbox[2]) / 2
            
            height_mid = img.shape[0] / 2
            weight_mid = img.shape[1] / 2

            bbox_height_size = abs(bbox[1] - bbox[3])
            bbox_width_size = abs(bbox[0] - bbox[2])
            
            # return ((bbox_height_mid - height_mid) / height_mid, (bbox_width_mid - weight_mid) / weight_mid)
            return np.array([
                (bbox_height_mid - height_mid) / height_mid,
                (bbox_width_mid - weight_mid) / weight_mid,
                (bbox_height_size * bbox_width_size) / (img.shape[0] * img.shape[1])
            ])
        return None

    def get_observation(self, location):
        observation = self.observation_space.sample()
        
        [res, camera_data, size] = self.drone_controller.GetCameraData()
        assert res is True
        assert size > 0

        directions = self.get_goal_direction(camera_data)
        if directions is not None:
            # observation[:2] = directions
            observation[0] = directions[1]
            observation[1] = directions[2]
        else:
            observation[:2] = np.zeros(2)

        # Lidar data
        if self.lidar_beam_count > 0:
            [res, lidar_data, start] = self.drone_controller.GetLidarData()
            assert res is True
            for i in range(self.lidar_beam_count):
                observation[2 + i] = lidar_data[i].distance / self.lidar_beam_length
        
        return observation

    def get_location(self):
        if self.last_location is None:
            [res, location] = self.drone_controller.GetLocation()
            assert res is True
            self.last_location = np.array([location.x, location.y, location.z], dtype=np.double)
        return self.last_location
    
    def set_location(self, location, check_collisions=True):
        [res, teleported_to, is_hit, impact_point] = self.drone_controller.SetLocation(
            Coordinates(location[0], location[1], location[2]),
            check_collisions
            )
        assert res is True
        teleported_to = np.array([teleported_to.x, teleported_to.y, teleported_to.z], dtype=np.double)
        self.last_location = teleported_to

        return teleported_to, is_hit, impact_point

    def calculate_reward(self, location, is_hit, observation, action):
        is_goal = False

        # Distance reward
        curr_distnace = self.calculate_disance(location, self.goal["loc"])
        distance_reward = self.last_distance - curr_distnace
        self.last_distance = curr_distnace
        if curr_distnace < self.goal["threshold"]:
            distance_reward = self.goal["reward"]
            is_goal = True

        # Outrun penalty
        outrun_penalty = 0
        if location[0] > self.goal["loc"][0] + self.goal["threshold"]:
            outrun_penalty = -1000
            self.done = True

        # Crash penalty
        crash_penalty = 0
        closest = np.min(observation[2:]) * self.lidar_beam_length if observation.shape[0] > 2 else None
        if closest is not None and closest < self.max_obstacle_distance_normalized:
            crash_penalty = closest - self.max_obstacle_distance_normalized

        # Free-space reward
        free_space_reward = 0
        for i in range(2, observation.shape[0]):
            free_space_reward += (observation[i] / self.max_obstacle_distance_normalized) - 1
        free_space_reward /= self.max_free_space_reward

        # Step penalty
        step_penalty = -(self.steps / self.max_steps)

        # Acceleration penalty to prevent shaking
        acceleration_penalty = 0 if action == self.last_action else -1

        if is_hit or is_goal:
            if is_hit:
                crash_penalty -= self.goal["reward"]
            self.done = True

        return distance_reward + outrun_penalty + crash_penalty + free_space_reward + step_penalty + acceleration_penalty

    def step(self, action: int):
        # Process action
        direction = self._action_to_direction[action]
        location = self.get_location()
        [teleported_to, is_hit, impact_point] = self.set_location(location + direction)

        # Get observation
        observation = self.get_observation(teleported_to)

        # Calculate reward
        reward = self.calculate_reward(teleported_to, is_hit, observation, action)

        # Get info
        info = {}

        self.last_action = action
        self.steps += 1

        return observation, reward, self.done, info

    def disconnect_drone(self):
        if self.drone_port is not None:
            res = self.game_mode_controller.RemoveDrone(self.drone_port)
            assert res is True

            self.drone_port = None
            self.drone_controller = None

    def reset_drone(self):
        if self.drone_controller is not None:
            res = self.drone_controller.SetMoveLineVisible(False)
            assert res is True

        if self.drone_controller is None:
            self.disconnect_drone()

            [res, port] = self.game_mode_controller.SpawnDrone()
            assert res is True
            self.drone_port = port 

            self.drone_controller = DroneController(port=port)
            res = self.drone_controller.ConnectSimple()
            assert res is True

            [res, lidar_config] = self.drone_controller.GetLidarConfig()
            assert res is True
            lidar_config.beamCount = self.lidar_beam_count
            lidar_config.showBeams = False
            lidar_config.beamLength = self.lidar_beam_length
            lidar_config.angleFOV = self.lidar_angle
            res = self.drone_controller.SetLidarConfig(lidar_config)
            assert res is True

        [teleported_to, is_hit, impact_point] = self.set_location(self.init_location, False)
        assert is_hit is False

        res = self.drone_controller.SetMoveLineVisible(self.draw_move_line)
        assert res is True

        return self.get_observation(teleported_to)

    def reset(self):
        self.reset_drone()

        self.done = False
        self.steps = 0
        self.last_location = None
        self.last_distance = self.calculate_disance(self.init_location, self.goal["loc"])
        self.last_action = None

        return self.get_observation(self.init_location)

    def render(self, mode=None):
        pass

    def close(self):
        self.disconnect_drone()