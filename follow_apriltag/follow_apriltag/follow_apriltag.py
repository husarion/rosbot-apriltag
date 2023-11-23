import rclpy
import math
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from apriltag_msgs.msg import AprilTagDetectionArray


class PIDController:
    def __init__(self, kp, ki, kd, min_output, max_output, integral_max=None, integral_min=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output

        self.integral = 0.0
        self.previous_error = 0.0

        # Integral windup guards
        self.integral_max = integral_max if integral_max is not None else max_output
        self.integral_min = integral_min if integral_min is not None else min_output

    def update(self, error, delta_time):
        self.integral += error * delta_time
        # Clamp integral to prevent windup
        self.integral = max(
            min(self.integral, self.integral_max), self.integral_min)

        derivative = (error - self.previous_error) / delta_time
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error

        # Clamp output to min/max
        return max(min(output, self.max_output), self.min_output)


class FollowAprilTagNode(Node):
    def __init__(self):
        super().__init__('follow_april_tag_node')

        # Subscribe to the detections topic
        self.detection_subscriber = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10)
        self.latest_detections = None

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for robot movement commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer to update robot position
        self.timer = self.create_timer(0.1, self.update_robot_position)

        # PID Controllers
        self.pid_distance = PIDController(
            kp=1.0, ki=0.02, kd=0.05, min_output=-1.0, max_output=1.0, integral_max=0.5, integral_min=-0.5)
        # self.pid_angle = PIDController(
        #     kp=0.5, ki=0.02, kd=0.01, min_output=-1.0, max_output=1.0, integral_max=0.5, integral_min=-0.5)
        self.pid_angle = PIDController(
            kp=1.5, ki=0.05, kd=0.05, min_output=-1.0, max_output=1.0, integral_max=0.5, integral_min=-0.5)

        # Last time for PID calculation
        self.last_time = self.get_clock().now()

    def detection_callback(self, msg):
        # Callback function to handle new data on the detections topic
        self.latest_detections = msg

    def update_robot_position(self):
        try:
            # Get the transform from the robot to the AprilTag
            trans = self.tf_buffer.lookup_transform(
                'camera_link', 'tag36h11:4', rclpy.time.Time())

            # Process the transform and send commands to the robot
            self.process_transform_and_move_robot(trans)

        except Exception as e:
            self.get_logger().info(f'Could not get transform: {e}')
            # Create and publish Twist message
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)

    def process_transform_and_move_robot(self, transform):

        # Check if any detections are available
        if self.latest_detections is not None and len(self.latest_detections.detections) > 0:
            # Extract translation and rotation components
            translation = transform.transform.translation

            # Calculate distance and angle to the AprilTag
            distance = math.sqrt(translation.x ** 2 + translation.y ** 2)
            angle = math.atan2(translation.y, translation.x)

            # Calculate delta time
            current_time = self.get_clock().now()
            # Convert nanoseconds to seconds
            delta_time = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            # PID for Distance
            error_distance = 0.35 - distance  # 35cm is the target distance
            linear_velocity = self.pid_distance.update(
                error_distance, delta_time)

            # PID for Angle
            # Target angle is assumed to be zero (facing the tag directly)
            error_angle = angle
            angular_velocity = self.pid_angle.update(error_angle, delta_time)

            # Create and publish Twist message
            twist = Twist()
            twist.linear.x = - linear_velocity
            twist.angular.z = angular_velocity
            self.publisher.publish(twist)
        else:
            # No detections - don't move the robot
            self.get_logger().info("No AprilTag detected. Robot not moving.")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FollowAprilTagNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
