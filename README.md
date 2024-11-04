# TurtleBot3 Random Navigation Environments

![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-compatible-brightgreen)
![License](https://img.shields.io/github/license/sbremman/turtlebot3_random_navigation_environments)

A ROS package for generating random navigation environments in Gazebo for the TurtleBot3 platform. This package is designed to facilitate testing and training of reinforcement learning algorithms and navigation strategies for autonomous robots by providing diverse, configurable obstacles.

## Features

- **Randomized Navigation Scenarios**: Generate varied and unpredictable environments for TurtleBot3 navigation.
- **Compatible with Gazebo**: Integrates seamlessly with the Gazebo simulator.
- **Ideal for Testing Navigation Algorithms**: Supports testing and benchmarking of obstacle avoidance, path planning, and reinforcement learning models.
- **Configurable Obstacles**: Easily customize obstacle types, sizes, and placements to create unique environments.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Customization](#customization)
- [Contributing](#contributing)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Installation

### 1. Clone the Repository
```bash
git clone https://github.com/sbremman/turtlebot3_random_navigation_environments.git
cd turtlebot3_random_navigation_environments
```

### 2. Install TurtleBot3 ROS Packages
Ensure that the TurtleBot3 ROS packages are installed. You can install them as follows:
```bash
sudo apt update
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-turtlebot3-navigation
```

### 3. Install Python Dependencies
Install required Python packages using `requirements.txt`:
```bash
pip install -r requirements.txt
```

### Using a Virtual Environment (Optional, Recommended)
To avoid conflicts, consider setting up a virtual environment:
1. Create and activate a virtual environment:
   ```bash
   python3 -m venv ~/ros_env
   source ~/ros_env/bin/activate
   ```

2. Install dependencies in the virtual environment:
   ```bash
   pip install -r requirements.txt
   ```

3. Remember to activate this environment before running the package:
   ```bash
   source ~/ros_env/bin/activate
   ```

## Usage

### Launch the Environment
To start Gazebo with a random environment:
```bash
roslaunch turtlebot3_random_navigation_environments random_world.launch
```

### Sending Service Requests
You can send service requests to randomize the environment using the `/world_randomizer` topic. For example:
```bash
rosservice call /world_randomizer "{env_size_x: 10.0, env_size_y: 10.0, num_obstacles: 20, obstacle_max_size: 1.5, obstacle_min_size: 0.1, wall_spawn_chance: 0.7}"
```
This will create a random environment with the specified size, number of obstacles, and wall spawn probability.
To start Gazebo with a random environment:
```bash
roslaunch turtlebot3_random_navigation_environments random_navigation.launch
```

## Customization

You can easily customize the environments and obstacles:

- **Add New Obstacles**: Place additional `.sdf` models in the `sdf_obstacles` directory to increase the variety.
- **Edit Obstacle Properties**: Modify the `.sdf` files in `sdf_obstacles` to adjust size, shape, and other properties of the obstacles.
- **Adjust Environment Parameters**: Update parameters in `random_navigation.launch` to control aspects like obstacle density, spawn randomness, and Gazebo settings.

## Directory Structure

```
turtlebot3_random_navigation_environments/
├── CMakeLists.txt               # Build configuration
├── package.xml                  # Package manifest
├── README.md                    # Project documentation
├── requirements.txt             # Python dependencies
├── launch/                      # Launch files for Gazebo environments
│   └── random_navigation.launch # Main launch file for random environment
├── scripts/                     # Custom scripts for environment management
│   └── random_world.py          # Script to randomize the world layout
└── sdf_obstacles/               # SDF models for obstacles
    ├── Desk/                    # Example obstacle with SDF and mesh files
    └── Wall/                    # Another example obstacle
```

## Contributing

Contributions are welcome! Please follow these steps to contribute:
- Fork the repository.
- Create a new branch (`git checkout -b feature-branch`).
- Commit your changes (`git commit -m 'Add a new feature'`).
- Push to the branch (`git push origin feature-branch`).
- Open a Pull Request on GitHub.

## Troubleshooting

### Gazebo is Not Starting
- Ensure that you have installed `ros-noetic-gazebo-ros` and other necessary dependencies.
- If Gazebo crashes, try restarting it using the provided restart function.

### Cannot Find Random Navigation Launch File
- Make sure you have sourced your workspace: `source ~/catkin_ws/devel/setup.bash`.

### Common ROS Errors
- **Missing Dependencies**: Run `rosdep install --from-paths src --ignore-src -r -y` to install missing dependencies.
- **ROS Master Not Found**: Ensure `roscore` is running before launching any nodes.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
