from controllers import GameModeController, DroneController

def get_game_mode_controller():
    _game_mode_controller = GameModeController("127.0.0.1", 8000)
    assert _game_mode_controller.ConnectSimple() is True
    assert _game_mode_controller.Ping() is True
    print("Game mode controller connected")

    return _game_mode_controller

def get_drone_controller(port):
    drone_controller = DroneController(port=port)
    res = drone_controller.ConnectSimple()
    assert res is True
    print("Drone controller connected")

    return drone_controller