#!/usr/bin/env python3
"""
Performance monitoring node for multithreaded Franka system
"""
import time
import psutil
from collections import deque, defaultdict
from typing import Dict, List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Header


class PerformanceMonitor(Node):
    """
    マルチスレッド Franka システムのパフォーマンス監視ノード
    """
    
    def __init__(self):
        super().__init__('performance_monitor')
        
        # パラメータ設定
        self.declare_parameter('monitor_topics', ['/touch_debug_action_pose_rviz'])
        self.declare_parameter('target_hz', 500.0)
        self.declare_parameter('log_interval', 10.0)
        self.declare_parameter('history_size', 1000)
        
        self.monitor_topics = self.get_parameter('monitor_topics').value
        self.target_hz = self.get_parameter('target_hz').value
        self.log_interval = self.get_parameter('log_interval').value
        self.history_size = self.get_parameter('history_size').value
        
        # データ格納用
        self.topic_timestamps: Dict[str, deque] = defaultdict(lambda: deque(maxlen=self.history_size))
        self.topic_counters: Dict[str, int] = defaultdict(int)
        self.last_log_time = time.time()
        
        # サブスクライバー作成
        self._setup_subscribers()
        
        # 定期ログ出力タイマー
        self.create_timer(self.log_interval, self.log_performance)
        
        self.get_logger().info(f"パフォーマンス監視開始: {len(self.monitor_topics)} トピック")
        
    def _setup_subscribers(self):
        """監視対象トピックのサブスクライバーを作成"""
        for topic in self.monitor_topics:
            if 'pose' in topic.lower():
                self.create_subscription(
                    PoseStamped,
                    topic,
                    lambda msg, t=topic: self._topic_callback(t, msg.header),
                    1
                )
            elif 'wrench' in topic.lower():
                self.create_subscription(
                    WrenchStamped,
                    topic,
                    lambda msg, t=topic: self._topic_callback(t, msg.header),
                    1
                )
            else:
                # Generic subscription for any message with header
                self.create_subscription(
                    Header,
                    topic,
                    lambda msg, t=topic: self._topic_callback(t, msg),
                    1
                )
    
    def _topic_callback(self, topic_name: str, header):
        """トピックメッセージ受信時のコールバック"""
        current_time = time.time()
        self.topic_timestamps[topic_name].append(current_time)
        self.topic_counters[topic_name] += 1
    
    def calculate_hz(self, topic_name: str) -> float:
        """指定トピックの周波数を計算"""
        timestamps = self.topic_timestamps[topic_name]
        if len(timestamps) < 2:
            return 0.0
            
        time_window = min(5.0, timestamps[-1] - timestamps[0])  # 最大5秒の窓
        if time_window <= 0:
            return 0.0
            
        # 時間窓内のメッセージ数を数える
        current_time = timestamps[-1]
        window_start = current_time - time_window
        
        count = sum(1 for t in timestamps if t >= window_start)
        return count / time_window if time_window > 0 else 0.0
    
    def get_system_stats(self) -> Dict:
        """システムリソース使用状況を取得"""
        return {
            'cpu_percent': psutil.cpu_percent(interval=0.1),
            'memory_percent': psutil.virtual_memory().percent,
            'cpu_count': psutil.cpu_count(),
            'load_avg': psutil.getloadavg() if hasattr(psutil, 'getloadavg') else (0, 0, 0),
        }
    
    def log_performance(self):
        """パフォーマンス情報をログ出力"""
        current_time = time.time()
        elapsed = current_time - self.last_log_time
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔍 パフォーマンス監視レポート")
        self.get_logger().info("=" * 60)
        
        # システム情報
        sys_stats = self.get_system_stats()
        self.get_logger().info(f"💻 システム:")
        self.get_logger().info(f"   CPU使用率: {sys_stats['cpu_percent']:.1f}%")
        self.get_logger().info(f"   メモリ使用率: {sys_stats['memory_percent']:.1f}%")
        self.get_logger().info(f"   ロードアベレージ: {sys_stats['load_avg'][0]:.2f}")
        
        # トピック統計
        self.get_logger().info(f"📊 トピック統計:")
        for topic in self.monitor_topics:
            hz = self.calculate_hz(topic)
            total_count = self.topic_counters[topic]
            
            # パフォーマンス評価
            performance_ratio = hz / self.target_hz if self.target_hz > 0 else 0
            status_emoji = "🟢" if performance_ratio >= 0.95 else "🟡" if performance_ratio >= 0.80 else "🔴"
            
            self.get_logger().info(f"   {status_emoji} {topic}:")
            self.get_logger().info(f"      周波数: {hz:.1f} Hz (目標: {self.target_hz:.1f} Hz)")
            self.get_logger().info(f"      達成率: {performance_ratio*100:.1f}%")
            self.get_logger().info(f"      総メッセージ数: {total_count}")
            
            if performance_ratio < 0.80:
                self.get_logger().warn(f"⚠️  {topic} の性能が低下しています")
        
        self.get_logger().info("=" * 60)
        self.last_log_time = current_time


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = PerformanceMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()