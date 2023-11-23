import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

class FollowAprilTagNode(Node):
    def __init__(self):
        super().__init__('follow_april_tag_node')
        
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher for robot movement commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer to update robot position
        self.timer = self.create_timer(0.1, self.update_robot_position)

    def update_robot_position(self):
        try:
            # Get the transform from the robot to the AprilTag
            trans = self.tf_buffer.lookup_transform('camera_link', 'tag36h11:4', rclpy.time.Time())

            # Process the transform and send commands to the robot
            self.process_transform_and_move_robot(trans)

        except Exception as e:
            self.get_logger().info(f'Could not get transform: {e}')

    def process_transform_and_move_robot(self, transform):
        # Extract translation and rotation components
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Log translation and rotation details
        self.get_logger().info(f"Translation: x={translation.x}, y={translation.y}, z={translation.z}")
        self.get_logger().info(f"Rotation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}")

        # Calculate distance and angle to the AprilTag
        distance = (translation.x ** 2 + translation.y ** 2) ** 0.5
        euler_angles = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        angle = euler_angles[2]

        # Log calculated distance and angle
        self.get_logger().info(f"Distance to AprilTag: {distance} meters")
        self.get_logger().info(f"Angle to AprilTag: {angle} radians")

        # Adjust velocities to maintain 50cm distance
        linear_velocity = 0.0
        angular_velocity = 0.0

        if distance > 0.5:  # If more than 50cm away
            linear_velocity = min(0.5 * (distance - 0.5), 1.0)  # Speed capped at 1 m/s
            angular_velocity = -3.0 * angle  # Adjust angular velocity based on angle

        # Log the velocities being set
        self.get_logger().info(f"Setting linear velocity: {linear_velocity} m/s, angular velocity: {angular_velocity} rad/s")

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FollowAprilTagNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
