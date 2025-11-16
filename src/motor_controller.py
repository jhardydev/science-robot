"""
Motor controller for differential drive Duckiebot using ROS
Publishes wheel commands to Duckiebot's motor control node
"""
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Header
import config


class MotorController:
    """Differential drive motor control via ROS"""
    
    def __init__(self):
        """Initialize ROS publisher for motor control"""
        # Note: rospy.init_node should be called in main(), not here
        # But we check if it's already initialized
        if rospy.get_node_uri() is None:
            rospy.logwarn("ROS node not initialized. Call rospy.init_node() in main() first.")
        
        # Publisher for wheel commands
        self.wheels_pub = rospy.Publisher(
            config.MOTOR_TOPIC,
            WheelsCmdStamped,
            queue_size=1
        )
        
        # Wait a moment for publisher to connect
        rospy.sleep(0.1)
        
        self.is_stopped = True
        rospy.loginfo("Motor controller initialized (ROS)")
    
    def _publish_wheel_command(self, left_speed, right_speed):
        """
        Publish wheel command to ROS topic
        
        Args:
            left_speed: Left wheel speed (normalized -1.0 to 1.0)
            right_speed: Right wheel speed (normalized -1.0 to 1.0)
        """
        # Create wheel command message
        msg = WheelsCmdStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        
        # Duckiebot wheels_driver expects speeds in m/s typically
        # Scale by max speed from config
        msg.vel_left = float(left_speed * config.MOTOR_MAX_SPEED)
        msg.vel_right = float(right_speed * config.MOTOR_MAX_SPEED)
        
        self.wheels_pub.publish(msg)
        self.is_stopped = (left_speed == 0.0 and right_speed == 0.0)
    
    def move_forward(self, speed=None):
        """
        Move forward
        
        Args:
            speed: Speed from 0.0 to 1.0 (default from config)
        """
        if speed is None:
            speed = config.MOTOR_BASE_SPEED
        
        self._publish_wheel_command(speed, speed)
    
    def move_backward(self, speed=None):
        """
        Move backward
        
        Args:
            speed: Speed from 0.0 to 1.0 (default from config)
        """
        if speed is None:
            speed = config.MOTOR_BASE_SPEED
        
        self._publish_wheel_command(-speed, -speed)
    
    def turn_left(self, speed=None):
        """
        Turn left (left wheel backward, right wheel forward)
        
        Args:
            speed: Speed from 0.0 to 1.0 (default from config)
        """
        if speed is None:
            speed = config.MOTOR_TURN_SPEED
        
        self._publish_wheel_command(-speed, speed)
    
    def turn_right(self, speed=None):
        """
        Turn right (right wheel backward, left wheel forward)
        
        Args:
            speed: Speed from 0.0 to 1.0 (default from config)
        """
        if speed is None:
            speed = config.MOTOR_TURN_SPEED
        
        self._publish_wheel_command(speed, -speed)
    
    def set_differential_speed(self, left_speed, right_speed):
        """
        Set individual speeds for left and right wheels
        
        Args:
            left_speed: Left wheel speed from -1.0 to 1.0
            right_speed: Right wheel speed from -1.0 to 1.0
        """
        # Clamp speeds to [-1, 1]
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        
        self._publish_wheel_command(left_speed, right_speed)
    
    def stop(self):
        """Stop all motors"""
        self._publish_wheel_command(0.0, 0.0)
    
    def emergency_stop(self):
        """Emergency stop - immediately stop all motors"""
        self.stop()
        rospy.logwarn("EMERGENCY STOP activated")
    
    def cleanup(self):
        """Clean up ROS resources"""
        self.stop()
        rospy.sleep(0.1)
        rospy.loginfo("Motor controller cleaned up")
