# ROS2 Stop and Slow Zone Node

This ROS2 package creates stop and slow zones around a robot and visualizes them in RViz. It is built using Python 3.12 on Ubuntu 22.04.

## Dependencies

To use this package, ensure the following dependencies are installed:

- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `visualization_msgs`
- `std_msgs`

Create the package in your ROS2 workspace using the command:

```bash
ros2 pkg create stop_slow_zones --build_type ament_python --dependencies rclpy sensor_msgs geometry_msgs visualization_msgs std_msgs
```

## Overview

This package creates a node called `stop_node` using the `StopNode` class. The node subscribes to the `/laser_scan` (or `/scan`) and `/cmd_vel` topics to receive the robot's current scan readings and velocity. It publishes markers to visualize stop and slow zones around the robot in RViz. The node also publishes a control signal (`/stop_slow_signal`) of type `String` to indicate if an object has been detected within these zones, with possible values: "stop," "slow_down," and "clear."

### Node Functionality

1. **Zone Control and Visualization**:
    - **Stop and Slow Zones**: The node dynamically adjusts stop and slow-down zones based on the robot's speed and direction. By default, the stop and slow-down distances are set to:
      ```python
      self.stop_threshold_min = 1.5
      self.slow_down_threshold_min = 2.5
      ```
      These thresholds are adjusted dynamically according to the robot's current speed to ensure safe operation.
    - **Marker Visualization**: Visualizes these zones in RViz using polygon markers, adjusting the zones based on both the linear and angular velocity of the robot.

2. **Zone Signals**:
    - Publishes a signal to the `/stop_slow_signal` topic with values "stop," "slow_down," or "clear" depending on the proximity of obstacles to the robot.
    - "Stop" signals take precedence over "Slow" signals to ensure immediate halting if a close obstacle is detected.

3. **Handling Velocities**:
    - Modulates the robot's velocity when a "stop" or "slow_down" command is issued to control the robot’s movement safely. This logic is implemented in the `publish_stop_or_slow` method and can be customized to fit your specific requirements.

### Important Methods

- `scan_callback(self, msg)`: Processes incoming LaserScan messages, divides the scan into sectors, and publishes control signals based on the nearest detected obstacles.

- `divide_scan_into_sectors_360(self, msg)`: Divides the 360-degree LaserScan data into four sectors (front, rear, left, and right) to determine obstacle positions relative to the robot.

- `adjust_zones_based_on_movement(self, msg)`: Adjusts the stop and slow-down zones based on the robot's movement direction (forward, backward, turning left, or turning right).

- `velocity_callback(self, msg)`: Updates the robot's current velocity to adapt the stop and slow-down thresholds based on speed.

- `update_thresholds(self)`: Updates the stop and slow-down thresholds dynamically based on the robot's current speed to scale the safety zones appropriately.

- `publish_stop_or_slow(self, control_command)`: Publishes stop or slow-down commands directly to the `/cmd_vel` topic to control the robot's movement.

- `publish_polygon_marker(self, direction)`: Publishes polygon markers in RViz to visualize the dynamically adjusted stop and slow-down zones based on the robot's movement.

### Customization

- **Thresholds and Zones**: Adjust the stop and slow-down thresholds in the `__init__` method or within the `update_thresholds` method to fit the specific requirements of your robot or vehicle.
- **Zone Partitioning**: The default zone partitioning divides the LaserScan into four sectors (front, rear, left, and right). Adjust the `divide_scan_into_sectors_360` method as needed to suit your scanning setup.
- **Control Logic**: Modify the logic within the `publish_stop_or_slow` method to implement your desired response to stop and slow-down signals, ensuring safe and effective robot control.

## Usage

Run the node with the following command:

```bash
ros2 run stop_slow_zones stop_node
```
