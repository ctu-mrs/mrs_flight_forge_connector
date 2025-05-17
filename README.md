# FlightForge connector

This repository contains the API connector for [FlightForge](https://github.com/mrs-ctu/FlightForge), it allows to interact with the FlightForge binary or your own custom build of FlightForge. 


## Building the connector 

To build the C++ connector you can use the provided `build.sh` script. The script will create a build directory, run CMake to configure the project, and then build the connector using `make`. 
An example usage of the API is provided in the `example/cli` directory.
To spawn the UAV in the simulator you can run the 

```bash
./debug_game_mode.sh
2 0
```

This will spawn the UAV in the simulator and you can use the `./debug_cli.sh` script to control the UAV.

## Citing this work

If you use this simulator in your research, please cite the following paper:

```@article{čapek2025flightforge,
  title   = {FlightForge: Advancing UAV Research with Procedural Generation of High-Fidelity Simulation and Integrated Autonomy},
  author  = {David Čapek and Jan Hrnčíř and Tomáš Báča and Jakub Jirkal and Vojtěch Vonásek and Robert Pěnička and Martin Saska},
  year    = {2025},
  journal = {arXiv preprint arXiv: 2502.05038},
  url     = {https://arxiv.org/abs/2502.05038v1},
  pdf     = {https://arxiv.org/pdf/2502.05038.pdf}
}
```
