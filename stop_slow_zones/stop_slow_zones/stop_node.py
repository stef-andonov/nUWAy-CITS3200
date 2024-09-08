import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import String

from rclpy.qos import ReliabilityPolicy, QoSProfile, DurabilityPolicy

import math

### TODO:
### - Extend stop and slow zones to front depending on 

class StopNode(Node):
    def __init__(self):
        super().__init__('stop_node')

        #Publisher for control signals
        self.signal_publisher_ = self.create_publisher(
            String,
            '/stop_slow_signal',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Publisher for RViz marker
        self.marker_publisher_ = self.create_publisher(
            Marker,
            '/visualization_marker',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Subscriber for /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/laser_scan', ### Often just called '/scan'
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        # Subscriber for /cmd_vel topic to get the current velocity
        self.velocity_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.subscription  # prevents unused variable warning
        self.velocity_subscription
        
        self.timer_val = 0.5   # Timer for checking distance to nearest object. Consider increasing with speed and/or when an obstacle has been detected nearby
        #self.timer = self.create_timer(self.timer_val, self.publish_stop_or_slow)
        
        self.stop_required = False
        self.slow_down_required = False
        
        # Default stop and slow zone disatances. Set at your own preferences
        self.stop_threshold_min = 0.5
        self.slow_down_threshold_min = 1.5

        self.stop_threshold = self.stop_threshold_min
        self.slow_down_threshold = self.slow_down_threshold_min

        self.current_velocity = Twist()
        
        self.get_logger().info('Node has been started.')

    def scan_callback(self, msg):
        '''
        Callback function that publishes "stop", "slow", or "clear" strings to
        the /stop_slow_signal topic for /cmd_vel publisher to respond to.
        
        '''

        num_ranges = len(msg.ranges)
        min_distance = min(msg.ranges)
        self.update_thresholds()

        min_distance, direction = self.adjust_zones_based_on_movement(msg)

        # Publish the polygon marker to visualize min_distance
        self.publish_polygon_marker(self.stop_threshold, self.slow_down_threshold, direction)

        control_signal = String()

        if min_distance < self.stop_threshold:
            control_signal.data = "stop"
            self.signal_publisher_.publish(control_signal)
            self.get_logger().info('Publishing stop signal.')
        elif min_distance < self.slow_down_threshold:
            control_signal.data = "slow_down"
            self.signal_publisher_.publish(control_signal)
            self.get_logger().info('Publishing slow down signal.')
        else:
            control_signal.data = "clear"
            self.signal_publisher_.publish(control_signal)
            self.get_logger().info('Publishing clear signal.')

    def divide_scan_into_sectors(self, msg):
        '''
        Divides the LaserScan data into front, left, right, and rear sectors.
        '''
        num_ranges = len(msg.ranges)
        sector_size = num_ranges // 4  # Split into four equal sectors: front, left, right, rear

        front_sector = msg.ranges[sector_size:2 * sector_size]
        left_sector = msg.ranges[2 * sector_size:]  # Left from the middle to end
        right_sector = msg.ranges[:sector_size]  # Right from the beginning
        rear_sector = msg.ranges[3 * sector_size:]  # Rear data

        return front_sector, left_sector, right_sector, rear_sector

    def adjust_zones_based_on_movement(self, msg):
        '''
        Adjusts the stop and slow-down zones based on the robot's movement direction.
        '''
        front_sector, left_sector, right_sector, rear_sector = self.divide_scan_into_sectors(msg)

        # Determine movement direction based on linear and angular velocities
        if self.current_velocity.linear.x > 0:  # Moving forward
            min_front_distance = min(front_sector)
            return min_front_distance, 'front'

        elif self.current_velocity.linear.x < 0:  # Moving backward (reverse)
            min_rear_distance = min(rear_sector)  # Use actual rear sector data
            return min_rear_distance, 'rear'

        elif self.current_velocity.angular.z > 0:  # Turning left
            min_left_distance = min(left_sector)
            return min_left_distance, 'left'

        elif self.current_velocity.angular.z < 0:  # Turning right
            min_right_distance = min(right_sector)
            return min_right_distance, 'right'

        return float('inf'), 'clear'  # No significant movement, assume clear

    def velocity_callback(self, msg):
        '''
        Callback function that updates the current velocity of the robot.
        '''

        self.current_velocity = msg
        self.get_logger().info(f'Current velocity: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')

    def update_thresholds(self):
        '''
        Updates the stop and slow down thresholds based on current velocity
        Current implementation is just for testing, define your own logic here...
        '''
        current_speed = abs(self.current_velocity.linear.x)

        # Linearly scale the thresholds based on speed, with a minimum threshold.
        dynamic_scale = current_speed * 0.5  # Adjust the scaling factor as needed

        # Ensure the stop and slow-down thresholds don't decrease below the minimum
        self.stop_threshold = max(self.stop_threshold_min, self.stop_threshold_min + dynamic_scale)
        self.slow_down_threshold = max(self.slow_down_threshold_min, self.slow_down_threshold_min + dynamic_scale + 0.5)

    def publish_stop_or_slow(self):
        '''
        Timer callback to ensure stop command takes precedence if required.
        '''
        twist = Twist()

        if self.stop_required:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.get_logger().info('Publishing stop command to override other commands.')
        elif self.slow_down_required:
            # Calculate the slow-down command based on current velocity
            twist.linear.x = max(self.current_velocity.linear.x * 0.5, 0.1)  # Slow down by half, minimum speed 0.1
            twist.angular.z = self.current_velocity.angular.z * 0.5  # Reduce angular speed similarly
            self.publisher_.publish(twist)
            self.get_logger().info('Publishing slow-down command to override other commands.')


    def publish_polygon_marker(self, stop_distance, slow_down_distance, direction):
        """
        Publishes polygon markers around the robot in RViz to denote the stop and slow-down distances,
        and adjusts the zones dynamically based on movement direction.
        """
        # Adjust the size of the zones based on the direction
        front_stop = stop_distance if direction == 'front' else self.stop_threshold_min
        rear_stop = stop_distance if direction == 'rear' else self.stop_threshold_min
    
        # Adjust left and right stop zones dynamically based on direction of movement
        left_stop = stop_distance if direction == 'left' else self.stop_threshold_min
        right_stop = stop_distance if direction == 'right' else self.stop_threshold_min
    
        # Adjust slow-down distances similarly
        front_slow = slow_down_distance if direction == 'front' else self.slow_down_threshold_min
        rear_slow = slow_down_distance if direction == 'rear' else self.slow_down_threshold_min
    
        left_slow = slow_down_distance if direction == 'left' else self.slow_down_threshold_min
        right_slow = slow_down_distance if direction == 'right' else self.slow_down_threshold_min
    
        # Stop zone marker
        stop_marker = Marker()
        stop_marker.header.frame_id = "base_link"
        stop_marker.header.stamp = self.get_clock().now().to_msg()
        stop_marker.ns = "stop_distance_polygon"
        stop_marker.id = 0
        stop_marker.type = Marker.LINE_STRIP
        stop_marker.action = Marker.ADD
        stop_marker.scale.x = 0.05  # Line width
        stop_marker.color.a = 1.0
        stop_marker.color.r = 1.0
        stop_marker.color.g = 0.0
        stop_marker.color.b = 0.0
    
        # Define the stop zone shape, adjusting based on direction
        stop_points = [
            Point(x=front_stop, y=left_stop, z=0.0),  # Front-right corner
            Point(x=front_stop, y=-right_stop, z=0.0),  # Front-left corner
            Point(x=-rear_stop, y=-right_stop, z=0.0),  # Rear-left corner
            Point(x=-rear_stop, y=left_stop, z=0.0),  # Rear-right corner
            Point(x=front_stop, y=left_stop, z=0.0)   # Close the loop
        ]
        stop_marker.points = stop_points
        self.marker_publisher_.publish(stop_marker)
    
        # Slow-down zone marker
        slow_marker = Marker()
        slow_marker.header.frame_id = "base_link"
        slow_marker.header.stamp = self.get_clock().now().to_msg()
        slow_marker.ns = "slow_distance_polygon"
        slow_marker.id = 1
        slow_marker.type = Marker.LINE_STRIP
        slow_marker.action = Marker.ADD
        slow_marker.scale.x = 0.05
        slow_marker.color.a = 1.0
        slow_marker.color.r = 0.0
        slow_marker.color.g = 1.0
        slow_marker.color.b = 0.0
    
        # Define the slow-down zone shape, adjusting based on direction
        slow_points = [
            Point(x=front_slow, y=left_slow, z=0.0),
            Point(x=front_slow, y=-right_slow, z=0.0),
            Point(x=-rear_slow, y=-right_slow, z=0.0),
            Point(x=-rear_slow, y=left_slow, z=0.0),
            Point(x=front_slow, y=left_slow, z=0.0)
        ]
        slow_marker.points = slow_points
        self.marker_publisher_.publish(slow_marker)


def main(args=None):
    rclpy.init(args=args)
    
    node = StopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
