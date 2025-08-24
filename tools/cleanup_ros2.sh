#!/bin/bash

echo "🧹 清理ROS2共享内存和进程..."

# 终止所有ROS2相关进程
echo "📋 查找ROS2进程..."
ps aux | grep -E "(ros2|rclpy|fastrtps|rviz2)" | grep -v grep

echo "🛑 终止ROS2进程..."
pkill -f "ros2"
pkill -f "rclpy"
pkill -f "fastrtps"
pkill -f "rviz2"

# 等待进程终止
sleep 2

# 强制终止残留进程
echo "💀 强制终止残留进程..."
pkill -9 -f "ros2" 2>/dev/null
pkill -9 -f "rclpy" 2>/dev/null
pkill -9 -f "fastrtps" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null

# 清理共享内存
echo "🗑️ 清理共享内存..."
sudo rm -f /dev/shm/fastrtps_* 2>/dev/null
sudo rm -f /dev/shm/sem.fastrtps_* 2>/dev/null
sudo rm -f /dev/shm/rtps_* 2>/dev/null
sudo rm -f /dev/shm/sem.rtps_* 2>/dev/null

# 清理临时文件
echo "🧽 清理临时文件..."
rm -rf /tmp/ros2_* 2>/dev/null
rm -rf /tmp/fastrtps_* 2>/dev/null

# 验证清理结果
echo "✅ 验证清理结果..."
echo "共享内存中的fastrtps文件:"
ls -la /dev/shm/ | grep -E "(fastrtps|rtps)" || echo "无残留文件"

echo "ROS2进程:"
ps aux | grep -E "(ros2|rclpy|fastrtps)" | grep -v grep || echo "无残留进程"

echo "🎯 清理完成！"
