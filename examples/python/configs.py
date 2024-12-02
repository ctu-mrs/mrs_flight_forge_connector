env_config = {
    "simple_follow": {
        "lidar_beam_count": 0,
        "lidar_beam_length": 400,
        "lidar_angle": 45,
        "action_step": 10,
        "init_location": (1600, -12200, 1000),
        "goal_location": (2400, -12500, 1000),
        "goal_threshold": 50,
        "goal_reward": 10000,
        "max_obstacle_distance":  100,
        "max_steps": 3000
    },
    "lidar_obstacles_1" : {
        "lidar_beam_count": 35,
        "lidar_beam_length": 300,
        "lidar_angle": 135,
        "action_step": 10,
        "init_location": (-6320.0, -13730.0, 905.0),
        "waypoint_locations": [
            (-6050.0, -13840.0, 955.0),
            (-5630.0, -13900.0, 1085.0),
        ],
        "goal_location": (-5350.0, -13730.0, 925.0),
        "goal_threshold": 50,
        "goal_reward": 10000,
        "max_obstacle_distance":  60,
        "max_steps": 3000
    },
    "lidar_obstacles_2" : {
        "lidar_beam_count": 35,
        "lidar_beam_length": 300,
        "lidar_angle": 135,
        "action_step": 10,
        "init_location": (-6350.0, -11850.0, 1030.0),
        "waypoint_locations": [
            (-6130.0, -11620.0, 1095.0),
            (-6090.0, -11310.0, 1000.0),
            (-5760.0, -11190.0, 865.0),
            (-5450.0, -11400.0, 955.0),
        ],
        "goal_location": (-5335.0, -11620.0, 1030.0),
        "goal_threshold": 50,
        "goal_reward": 10000,
        "max_obstacle_distance":  60,
        "max_steps": 3000
    },
    "lidar_obstacles_3" : {
        "lidar_beam_count": 35,
        "lidar_beam_length": 300,
        "lidar_angle": 135,
        "action_step": 10,
        "init_location": (-8550.0, -13290.0, 1030.0),
        "waypoint_locations": [
            (-8255.0, -13610.0, 1035.0),
            (-8130.0, -13815.0, 1055.0),
            (-7870.0, -13990.0, 945.0)
        ],
        "goal_location": (-7655.0, -14155.0, 1030.0),
        "goal_threshold": 50,
        "goal_reward": 10000,
        "max_obstacle_distance":  60,
        "max_steps": 3000
    },
    "lidar_obstacles_4" : {
        "lidar_beam_count": 35,
        "lidar_beam_length": 300,
        "lidar_angle": 135,
        "action_step": 10,
        "init_location": (-8450.0, -11695.0, 1030.0),
        "waypoint_locations": [
            (-8260.0, -11730.0, 885.0),
            (-7995.0, -11630.0, 1065.0),
            (-7790.0, -11795.0, 875.0)
        ],
        "goal_location": (-7605.0, -11590.0, 1030),
        "goal_threshold": 50,
        "goal_reward": 10000,
        "max_obstacle_distance":  60,
        "max_steps": 3000
    },
    "lidar_obstacles_5" : {
        "lidar_beam_count": 35,
        "lidar_beam_length": 300,
        "lidar_angle": 135,
        "action_step": 10,
        "init_location": (-10400.0, -13790.0, 1030.0),
        "waypoint_locations": [
            (-10080.0, -13705.0, 1165.0),
            (-9765.0, -13910.0, 1065.0)
        ],
        "goal_location": (-9460.0, -13685.0, 1030),
        "goal_threshold": 50,
        "goal_reward": 10000,
        "max_obstacle_distance":  60,
        "max_steps": 3000
    },
    "lidar_obstacles_6" : {
        "lidar_beam_count": 35,
        "lidar_beam_length": 300,
        "lidar_angle": 135,
        "action_step": 10,
        "init_location": (-12320.0, -8985.0, 1030.0),
        "waypoint_locations": [
            (-11965.0, -9010.0, 1165.0)
        ],
        "goal_location": (-11380.0, -8965.0, 1030),
        "goal_threshold": 50,
        "goal_reward": 10000,
        "max_obstacle_distance":  60,
        "max_steps": 3000
    },
    "lidar_obstacles_7" : {
        "lidar_beam_count": 35,
        "lidar_beam_length": 300,
        "lidar_angle": 135,
        "action_step": 10,
        "init_location": (-12315.0,-6820.0,1080.0),
        "waypoint_locations": [
            (-11995.0, -7120.0, 995.0),
            (-11730.0, -6880.0, 1065.0)
        ],
        "goal_location": (-11375.0, -7120.0, 965.0),
        "goal_threshold": 50,
        "goal_reward": 10000,
        "max_obstacle_distance":  60,
        "max_steps": 3000
    },
    "camera_obstacle_1" : {
        "lidar_beam_count": 20,
        "lidar_beam_length": 300,
        "lidar_angle": 90,
        "action_step": 10,
        "init_location": (-14560, -12780.0, 1080.0),
        "goal_location": (-13620.0, -12640.0, 1080),
        "goal_threshold": 50,
        "goal_reward": 10000,
        "max_obstacle_distance":  60,
        "max_steps": 3000
    },
    "camera_obstacle_2" : {
        "lidar_beam_count": 20,
        "lidar_beam_length": 300,
        "lidar_angle": 90,
        "action_step": 10,
        "init_location": (-14555.0, -10490.0, 1080.0),
        "goal_location": (-13615.0, -10725.0, 1080.0),
        "goal_threshold": 50,
        "goal_reward": 10000,
        "max_obstacle_distance":  60,
        "max_steps": 3000
    },
    "camera_obstacle_3" : {
        "lidar_beam_count": 20,
        "lidar_beam_length": 300,
        "lidar_angle": 90,
        "action_step": 10,
        "init_location": (-16745.0, -10620.0, 1080.0),
        "goal_location": (-15805.0, -10815.0, 1080.0),
        "goal_threshold": 50,
        "goal_reward": 10000,
        "max_obstacle_distance":  60,
        "max_steps": 3000
    },
    "camera_obstacle_4" : {
        "lidar_beam_count": 20,
        "lidar_beam_length": 300,
        "lidar_angle": 90,
        "action_step": 10,
        "init_location": (-16740.0, -8510.0, 1080.0),
        "goal_location": (-15800.0, -8270.0, 1080.0),
        "goal_threshold": 50,
        "goal_reward": 10000,
        "max_obstacle_distance":  60,
        "max_steps": 3000
    },
}