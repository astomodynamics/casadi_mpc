# CasadiMPCNode ROS2 Package

This package implements a Model Predictive Control (MPC) node using CasADi for optimization in a ROS2 environment.

## Prerequisites

- ROS2 (tested with Humble)
- Python 3.10+
- CasADi
- NumPy

## Installation

1. Create a ROS2 workspace if you haven't already:
   ```
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Clone this package into your workspace:
   ```
   git clone https://github.com/astomodynamics/casadi_mpc.git
   ```

3. Install the required Python packages:
   ```
   cd casadi_mpc
   pip install -r requirements.txt
   ```

4. Build the ROS2 package:
   ```
   cd ~/ros2_ws
   colcon build --packages-select casadi_mpc
   ```

5. Source the setup file:
   ```
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

1. Run the CasadiMPCNode:
   ```
   ros2 run casadi_mpc casadi_mpc
   ```

2. The node subscribes to the `/robot_pose` topic for state updates, the `/goal_pose` topic for goal updates, and publishes control inputs to the `/cmd_vel` topic.


## Customization

- Adjust MPC parameters in the `CasadiMPCNode.__init__` method.
- Modify the system model in `setup_mpc` method to match your specific use case.

## Contributing

Contributions to improve the package are welcome. Please feel free to submit pull requests or open issues for bugs and feature requests.

## License

This project is licensed under the MIT License - see the LICENSE file for details.