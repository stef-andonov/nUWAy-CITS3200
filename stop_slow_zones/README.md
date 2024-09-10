ROS2 package for creating stop and slow zones around a robot in RVIZ. Built with Python 3.12 in Ubuntu 22.04

Dependencies:

        -rclpy
        -sensor_msgs
        -geometry_msgs
        -visualization_msgs


Create package in your ros2 workspace as:
        ros2 pkg create --build_type ament_python --dependencies rclpy sensor_msgs geometry_msgs visualization_msgs


Creates a node called "stop_node" from the StopNode class that subscribes to LaserScan and Twist to get the current scan readings and velocity of the robot. Publishes Markers of the stop and slow zones around the robot in rviz2. Publishes /stop_slow_signal (String) to denote that an object has been detected within a zone with a corresponding message denoting which zone the object is within. "Stop" signals take precedence over "Slow" signals.

Simple logic for extending the zones as the robot/vehicle accelerates exists and modulating velocity when a "Stop" or "Slow" command is issued, but the user should implement their own scheme. These are located in the "update_threshold" method and "publish_stop_or_slow" method, respectively. Default thresholds for stop and slow zones are initialised as:

  self.stop_threshold_min = 0.5
  
  self.slow_down_threshold_min = 1.5

User should adjust these to fit their robot/vehicle.


stop_node will issue commands depending on the direction the robot/vehicle move in and divides the LaserScan into four sections (by default with starting radians of -2.2, adjust the zones according to your implementation). Zone paritioning implemented in the method "divide_scan_into_sectors".


The method "publish_polygon_marker" publishes points in rviz to visualize the stop and slow zones. Zones will extend in the direction that the robot/vehicle is moving. Currently, the method publishes a square for each zone, although this can be altered by adding more points.
