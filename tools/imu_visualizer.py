#!/usr/bin/env python3
"""
IMU数据可视化转换节点
将IMU数据转换为可在RViz中显示的消息类型
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import Header
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class IMUVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        
        # 订阅IMU数据
        self.imu_sub = self.create_subscription(
            Imu,
            '/unitree_go2/imu/data',
            self.imu_callback,
            10
        )
        
        # 发布转换后的数据
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/unitree_go2/imu/pose',
            10
        )
        
        self.angular_vel_pub = self.create_publisher(
            Vector3Stamped,
            '/unitree_go2/imu/angular_velocity',
            10
        )
        
        self.linear_accel_pub = self.create_publisher(
            Vector3Stamped,
            '/unitree_go2/imu/linear_acceleration',
            10
        )
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('IMU Visualizer Node Started')
        print("\n🚀 IMU Visualizer Node Started Successfully!")
        print("📡 Subscribing to: /unitree_go2/imu/data")
        print("📤 Publishing to:")
        print("   - /unitree_go2/imu/pose")
        print("   - /unitree_go2/imu/angular_velocity") 
        print("   - /unitree_go2/imu/linear_acceleration")
        print("🔧 TF Broadcasting: unitree_go2/base_link → unitree_go2/imu_link")
        print("="*60)
    
    def imu_callback(self, msg):
        """处理IMU数据并转换为可视化消息"""
        
        # 1. 发布姿态数据
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'map'  # 使用map坐标系
        pose_msg.pose.orientation = msg.orientation
        # 位置设为机器人当前位置（这里设为原点，实际应该从odom获取）
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        self.pose_pub.publish(pose_msg)
        
        # 2. 发布角速度数据
        ang_vel_msg = Vector3Stamped()
        ang_vel_msg.header = msg.header
        ang_vel_msg.vector = msg.angular_velocity
        self.angular_vel_pub.publish(ang_vel_msg)
        
        # 3. 发布线加速度数据
        lin_accel_msg = Vector3Stamped()
        lin_accel_msg.header = msg.header
        lin_accel_msg.vector = msg.linear_acceleration
        self.linear_accel_pub.publish(lin_accel_msg)
        
        # 4. 发布TF变换
        self.publish_imu_tf(msg)
        
        # 5. 打印IMU数据摘要
        self.print_imu_summary(msg)
    
    def publish_imu_tf(self, msg):
        """发布IMU的TF变换"""
        t = TransformStamped()
        t.header = msg.header
        t.header.frame_id = 'unitree_go2/base_link'
        t.child_frame_id = 'unitree_go2/imu_link'
        
        # IMU相对于base_link的位置（通常重合）
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # IMU的姿态
        t.transform.rotation = msg.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def print_imu_summary(self, msg):
        """Print IMU data summary with structured format"""
        
        # ===== LINEAR ACCELERATION ANALYSIS =====
        accel_x, accel_y, accel_z = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        accel_magnitude = np.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        
        # ===== ANGULAR VELOCITY ANALYSIS =====
        ang_vel_x, ang_vel_y, ang_vel_z = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ang_vel_magnitude = np.sqrt(ang_vel_x**2 + ang_vel_y**2 + ang_vel_z**2)
        
        # ===== ORIENTATION ANALYSIS =====
        qw, qx, qy, qz = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        
        # Calculate Euler angles (roll, pitch, yaw)
        roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = np.arcsin(2*(qw*qy - qx*qz))
        yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        
        # Convert to degrees
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)
        
        # ===== PRINT STRUCTURED DATA =====
        print("\n" + "="*60)
        print("IMU DATA SUMMARY")
        print("="*60)
        
        # Linear Acceleration Section
        print("📊 LINEAR ACCELERATION:")
        print(f"   Magnitude: {accel_magnitude:.3f} m/s²")
        print(f"   X-axis: {accel_x:8.3f} m/s²")
        print(f"   Y-axis: {accel_y:8.3f} m/s²")
        print(f"   Z-axis: {accel_z:8.3f} m/s²")
        
        # Angular Velocity Section
        print("\n🔄 ANGULAR VELOCITY:")
        print(f"   Magnitude: {ang_vel_magnitude:.3f} rad/s")
        print(f"   X-axis: {ang_vel_x:8.3f} rad/s")
        print(f"   Y-axis: {ang_vel_y:8.3f} rad/s")
        print(f"   Z-axis: {ang_vel_z:8.3f} rad/s")
        
        # Orientation Section
        print("\n🧭 ORIENTATION (Euler Angles):")
        print(f"   Roll:  {roll_deg:8.1f}°")
        print(f"   Pitch: {pitch_deg:8.1f}°")
        print(f"   Yaw:   {yaw_deg:8.1f}°")
        
        # Quaternion Section
        print("\n🔢 QUATERNION (w, x, y, z):")
        print(f"   w: {qw:8.3f}")
        print(f"   x: {qx:8.3f}")
        print(f"   y: {qy:8.3f}")
        print(f"   z: {qz:8.3f}")
        
        # ===== ANALYSIS & WARNINGS =====
        # print("\n⚠️  ANALYSIS:")
        
        # # Check if acceleration is close to gravity
        # if abs(accel_magnitude - 9.81) < 0.5:
        #     print(f"   ⚠️  Acceleration magnitude ({accel_magnitude:.2f} m/s²) is close to gravity!")
        #     print(f"      This may indicate gravity compensation issues.")
        
        # # Check for high angular velocity
        # if ang_vel_magnitude > 1.0:
        #     print(f"   ⚠️  High angular velocity detected: {ang_vel_magnitude:.2f} rad/s")
        
        # # Check for extreme orientation angles
        # if abs(roll_deg) > 45 or abs(pitch_deg) > 45:
        #     print(f"   ⚠️  Extreme orientation angles detected!")
        #     print(f"      Roll: {roll_deg:.1f}°, Pitch: {pitch_deg:.1f}°")
        
        # print("="*60)
        
        # Also log to ROS logger for compatibility
        # self.get_logger().info(
        #     f'IMU Summary - Accel: {accel_magnitude:.2f}m/s² ({accel_x:.3f}, {accel_y:.3f}, {accel_z:.3f}), '
        #     f'AngVel: {ang_vel_magnitude:.3f}rad/s ({ang_vel_x:.3f}, {ang_vel_y:.3f}, {ang_vel_z:.3f}), '
        #     f'Orientation: R={roll_deg:.1f}° P={pitch_deg:.1f}° Y={yaw_deg:.1f}°'
        # )


def main(args=None):
    rclpy.init(args=args)
    node = IMUVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

