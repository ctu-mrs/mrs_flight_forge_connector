from controllers import GameModeController, DroneController, Coordinates, Rotation

import gym
from gym import spaces
import numpy as np


class UEdsLidarEnv(gym.Env):
    def __init__(
            self,
            game_mode_controller: GameModeController,
            lidar_beam_count=6,
            lidar_beam_length=400,
            lidar_angle=45,
            action_step=10,
            init_location=(1600, -12200, 1000),
            waypoint_locations=[],
            goal_location=(2400, -12500, 1000),
            goal_threshold=50,
            goal_reward=10000,
            waypoint_threshold=50,
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
        waypoint_reward = goal_reward / (len(waypoint_locations) * 10) if len(waypoint_locations) > 0 else 0
        self.goals = ([
            {
                "loc": np.array([waypoint_location[0], waypoint_location[1], waypoint_location[2]], dtype=np.double), 
                "reward": waypoint_reward, 
                "threshold": waypoint_threshold
            }
            for waypoint_location in waypoint_locations
        ] if waypoint_locations is not None and len(waypoint_locations) > 0 else [])
        self.goals += [{"loc": goal_location, "reward": goal_reward, "threshold": goal_threshold}]

        self.max_free_space_reward = (lidar_beam_count / self.max_obstacle_distance_normalized) - lidar_beam_count if lidar_beam_count > 0 else 1

        self.current_goal_index = 0

        self.last_distance = self.calculate_disance(self.init_location, self.get_current_goal()["loc"])

        self.drone_port = None
        self.drone_controller = None
        self.steps = 0
        self.done = None
        self.last_action = None

        self.action_space = spaces.Discrete(6)
        self._action_to_direction = {
            0: np.array([action_step, 0, 0], dtype=np.double),
            1: np.array([-action_step, 0, 0], dtype=np.double),
            2: np.array([0, action_step, 0], dtype=np.double),
            3: np.array([0, -action_step, 0], dtype=np.double),
            4: np.array([0, 0, action_step], dtype=np.double),
            5: np.array([0, 0, -action_step], dtype=np.double),
        }

        self.observation_space = spaces.Box(low=-1, high=1, shape=(3 + lidar_beam_count,), dtype=np.double)
    
    def get_current_goal(self):
        return self.goals[self.current_goal_index]
    
    def calculate_disance(self, A, B):
        return np.linalg.norm(A - B)

    def get_observation(self, location):
        observation = self.observation_space.sample()
        
        # Location normalized distance to goal
        observation[:3] = location - self.get_current_goal()["loc"]
        observation[:3] /= np.max(np.abs(observation[:3]))

        # Lidar data
        if self.lidar_beam_count > 0:
            [res, lidar_data, start] = self.drone_controller.GetLidarData()
            assert res is True
            for i in range(self.lidar_beam_count):
                observation[3 + i] = lidar_data[i].distance / self.lidar_beam_length
        
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
        move_goal = False

        current_goal = self.get_current_goal()

        # Distance reward
        curr_distnace = self.calculate_disance(location, current_goal["loc"])
        distance_reward = self.last_distance - curr_distnace
        self.last_distance = curr_distnace
        if curr_distnace < current_goal["threshold"]:
            distance_reward = current_goal["reward"]
            is_goal = self.current_goal_index == len(self.goals) - 1
            move_goal = not is_goal

        # Crash penalty
        crash_penalty = 0
        closest = np.min(observation[3:]) * self.lidar_beam_length if observation.shape[0] > 3 else None
        if closest is not None and closest < self.max_obstacle_distance_normalized:
            crash_penalty = closest - self.max_obstacle_distance_normalized

        # Free-space reward
        free_space_reward = 0
        for i in range(3, observation.shape[0]):
            free_space_reward += (observation[i] / self.max_obstacle_distance_normalized) - 1
        free_space_reward /= self.max_free_space_reward

        # Step penalty
        step_penalty = -(self.steps / self.max_steps)

        # Acceleration penalty to prevent shaking
        acceleration_penalty = 0 if action == self.last_action else -1

        if is_hit or is_goal:
            if is_hit:
                crash_penalty -= self.goals[len(self.goals) - 1]["reward"]
            self.done = True

        return distance_reward + crash_penalty + free_space_reward + step_penalty + acceleration_penalty, move_goal

    def step(self, action: int):
        # Process action
        direction = self._action_to_direction[action]
        location = self.get_location()
        [teleported_to, is_hit, impact_point] = self.set_location(location + direction)

        # Get observation
        observation = self.get_observation(teleported_to)

        # Calculate reward
        reward, move_goal = self.calculate_reward(teleported_to, is_hit, observation, action)

        # Get info
        info = {}

        self.last_action = action
        self.steps += 1
        if move_goal:
            self.current_goal_index += 1

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
        self.last_distance = self.calculate_disance(self.init_location, self.get_current_goal()["loc"])
        self.last_action = None
        self.current_goal_index = 0

        return self.get_observation(self.init_location)

    def render(self, mode=None):
        pass

    def close(self):
        self.disconnect_drone()
