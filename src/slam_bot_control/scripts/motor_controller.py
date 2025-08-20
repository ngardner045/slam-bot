#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time
import numpy as np

from slam_bot_control.roboclaw import RoboClaw


class MotorController(Node):
    """ROS2 node for controlling differential drive robot motors"""
    
    def __init__(self):
        super().__init__('motor_controller')
        
        # Get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyUSB0'),
                ('baud_rate', 115200),
                ('left_motor_address', 128),
                ('right_motor_address', 129),
                ('wheel_diameter', 0.065),
                ('wheel_separation', 0.25),
                ('encoder_ticks_per_rev', 1440),
                ('max_linear_velocity', 1.0),
                ('max_angular_velocity', 2.0),
                ('publish_rate', 50.0)
            ]
        )
        
        # Robot parameters
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.encoder_ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        self.max_linear_velocity = self.get_parameter('max_linear_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Calculate wheel circumference and ticks per meter
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.ticks_per_meter = self.encoder_ticks_per_rev / self.wheel_circumference
        
        # Initialize RoboClaw
        try:
            self.roboclaw = RoboClaw(
                port=self.get_parameter('serial_port').value,
                baud_rate=self.get_parameter('baud_rate').value
            )
            self.get_logger().info('RoboClaw initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize RoboClaw: {e}')
            self.roboclaw = None
        
        # Initialize variables
        self.left_encoder_prev = 0
        self.right_encoder_prev = 0
        self.last_time = time.time()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.battery_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing odometry
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_odometry)
        
        # Emergency stop flag
        self.emergency_stop_active = False
        
        # Reset encoders on startup
        if self.roboclaw:
            self.roboclaw.reset_encoder_count(1)
            self.roboclaw.reset_encoder_count(2)
            self.get_logger().info('Encoders reset')
        
        self.get_logger().info('Motor controller node initialized')
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        if self.emergency_stop_active or not self.roboclaw:
            return
        
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Limit velocities
        linear_x = max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_x))
        angular_z = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_z))
        
        # Convert to differential drive
        left_velocity, right_velocity = self.diff_drive_kinematics(linear_x, angular_z)
        
        # Set motor velocities
        try:
            self.roboclaw.set_motor_velocity_differential(left_velocity, right_velocity)
        except Exception as e:
            self.get_logger().error(f'Failed to set motor velocities: {e}')
    
    def diff_drive_kinematics(self, linear_x, angular_z):
        """Convert linear and angular velocities to wheel velocities"""
        # Differential drive equations
        left_velocity = linear_x - (angular_z * self.wheel_separation) / 2.0
        right_velocity = linear_x + (angular_z * self.wheel_separation) / 2.0
        
        # Normalize to [-1, 1] range
        max_velocity = max(abs(left_velocity), abs(right_velocity))
        if max_velocity > 1.0:
            left_velocity /= max_velocity
            right_velocity /= max_velocity
        
        return left_velocity, right_velocity
    
    def publish_odometry(self):
        """Publish odometry and joint states"""
        if not self.roboclaw:
            return
        
        current_time = time.time()
        dt = current_time - self.last_time
        
        try:
            # Read encoder values
            left_encoder = self.roboclaw.get_encoder_count(1)
            right_encoder = self.roboclaw.get_encoder_count(2)
            
            # Calculate encoder deltas
            left_delta = left_encoder - self.left_encoder_prev
            right_delta = right_encoder - self.right_encoder_prev
            
            # Convert to meters
            left_distance = left_delta / self.ticks_per_meter
            right_distance = right_delta / self.ticks_per_rev
            
            # Calculate robot movement
            distance = (left_distance + right_distance) / 2.0
            delta_theta = (right_distance - left_distance) / self.wheel_separation
            
            # Update pose
            self.x += distance * math.cos(self.theta + delta_theta / 2.0)
            self.y += distance * math.sin(self.theta + delta_theta / 2.0)
            self.theta += delta_theta
            
            # Normalize angle to [-pi, pi]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            
            # Calculate velocities
            linear_velocity = distance / dt if dt > 0 else 0.0
            angular_velocity = delta_theta / dt if dt > 0 else 0.0
            
            # Publish odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
            odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
            
            odom_msg.twist.twist.linear.x = linear_velocity
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.angular.z = angular_velocity
            
            # Set covariance (simplified)
            odom_msg.pose.covariance[0] = 0.1  # x
            odom_msg.pose.covariance[7] = 0.1  # y
            odom_msg.pose.covariance[35] = 0.1  # yaw
            
            self.odom_pub.publish(odom_msg)
            
            # Publish joint states
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = ['left_wheel_joint', 'right_wheel_joint']
            joint_msg.position = [left_encoder, right_encoder]
            joint_msg.velocity = [left_distance/dt if dt > 0 else 0.0, 
                                right_distance/dt if dt > 0 else 0.0]
            
            self.joint_state_pub.publish(joint_msg)
            
            # Publish battery voltage
            battery_voltage = self.roboclaw.get_battery_voltage()
            battery_msg = Float32()
            battery_msg.data = battery_voltage
            self.battery_pub.publish(battery_msg)
            
            # Publish TF transform
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = math.sin(self.theta / 2.0)
            transform.transform.rotation.w = math.cos(self.theta / 2.0)
            
            self.tf_broadcaster.sendTransform(transform)
            
            # Update previous values
            self.left_encoder_prev = left_encoder
            self.right_encoder_prev = right_encoder
            self.last_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {e}')
    
    def emergency_stop(self):
        """Emergency stop all motors"""
        if self.roboclaw:
            try:
                self.roboclaw.emergency_stop()
                self.emergency_stop_active = True
                self.get_logger().warn('Emergency stop activated')
                
                # Publish emergency stop status
                stop_msg = Bool()
                stop_msg.data = True
                self.emergency_stop_pub.publish(stop_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to emergency stop: {e}')
    
    def on_shutdown(self):
        """Cleanup on shutdown"""
        if self.roboclaw:
            self.roboclaw.emergency_stop()
            self.roboclaw.close()
        self.get_logger().info('Motor controller shutdown complete')


def main(args=None):
    rclpy.init(args=args)
    
    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.get_logger().info('Keyboard interrupt received')
    except Exception as e:
        motor_controller.get_logger().error(f'Unexpected error: {e}')
    finally:
        motor_controller.on_shutdown()
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
