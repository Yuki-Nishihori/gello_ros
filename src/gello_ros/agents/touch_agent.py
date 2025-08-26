import time
from typing import Dict, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from touch_msgs.msg import TouchButtonEvent, TouchFeedback

import tf2_ros
from gello_ros.agents.agent import Agent

# tf2_geometry_msgsはPoseやWrenchの変換に便利ですが、
# 今回は元のロジックを尊重し、numpyとscipyで計算しています。
# import tf2_geometry_msgs


class TouchAgent(Agent, Node):
    """
    3D Systems Touchデバイスからの入力に基づき、ロボットアームを遠隔操作するエージェント。

    このクラスは、Touchデバイスの姿勢とボタン入力を購読し、
    ロボットのエンドエフェクタの目標姿勢を計算します。
    また、ロボットからの力覚フィードバックをTouchデバイスに送信する機能も持ちます。
    """

    def __init__(self):
        """ノードを初期化し、パラメータ、通信、TFを設定します。"""
        Node.__init__(self, 'touch_agent')
        Agent.__init__(self)

        # メンバ変数の初期化
        self._touch_current_pose: np.ndarray | None = None
        self._touch_start_pose: np.ndarray | None = None
        self._robot_start_pose: np.ndarray | None = None
        self._robot_current_pose: np.ndarray | None = None
        self._last_target_pose: np.ndarray | None = None  # 最後に計算された目標姿勢
        self._pose_initialized = False  # 初期姿勢が設定されたかのフラグ

        # テレオペレーションの状態管理フラグ
        self._is_teleop_active = False  # 白ボタンが押されているか
        self._was_teleop_active = False # 前のステップで白ボタンが押されていたか
        self._is_z_lock_active = False  # 灰ボタンが押されているか
        
        # Z軸下向きの回転（固定値）
        self.z_down_quat = R.from_euler('xyz', [0, np.pi, 0]).as_quat()

        # ROS関連の設定
        self._setup_parameters()
        self._setup_ros_communications()
        self._setup_tf()
        
        # Topic一覧を表示
        self._log_topic_info()

        # Touchデバイスからの最初のポーズメッセージを待つ
        self._wait_for_first_pose()

    def _setup_parameters(self) -> None:
        """ROSパラメータを宣言し、読み込みます。"""
        self.declare_parameter("touch_ee_pose_topic", "/touch/ee_pose")
        self.declare_parameter("touch_button_topic", "/touch/button_event")
        self.declare_parameter("touch_force_feedback_topic", "/touch/force_feedback")
        self.declare_parameter("touch_max_force", 1.5)
        self.declare_parameter("force_scale_to_touch", 1.0)
        self.declare_parameter("teleoperation_mode", "unilateral")
        self.declare_parameter("robot_base_frame", "base_link") # ROS1の実装に合わせてフレーム名を修正・統一
        self.declare_parameter("feedback_wrench_sensor_frame", "tool0")

        self.ee_pose_topic = self.get_parameter("touch_ee_pose_topic").get_parameter_value().string_value
        self.button_topic = self.get_parameter("touch_button_topic").get_parameter_value().string_value
        self.force_feedback_topic = self.get_parameter("touch_force_feedback_topic").get_parameter_value().string_value
        self.touch_max_force = self.get_parameter("touch_max_force").get_parameter_value().double_value
        self.force_scale_to_touch = self.get_parameter("force_scale_to_touch").get_parameter_value().double_value
        self.teleop_mode = self.get_parameter("teleoperation_mode").get_parameter_value().string_value
        self.robot_base_frame = self.get_parameter("robot_base_frame").get_parameter_value().string_value
        self.feedback_wrench_sensor_frame = self.get_parameter("feedback_wrench_sensor_frame").get_parameter_value().string_value

        if self.teleop_mode not in ["unilateral", "bilateral"]:
            self.get_logger().error(f"無効な通信モードです: {self.teleop_mode}。終了します。")
            exit()
        self.get_logger().info(f"{self.teleop_mode} モードでテレオペレーションを開始します。")

    def _setup_ros_communications(self) -> None:
        """PublisherとSubscriberを初期化します。"""
        self.force_feedback_vis_pub = self.create_publisher(
            WrenchStamped, "touch_debug_force_feedback_rviz", 10
        )
        self.force_feedback_pub = self.create_publisher(
            TouchFeedback, self.force_feedback_topic, 10
        )
        self.touch_debug_action_pose_pub = self.create_publisher(
            PoseStamped, "touch_debug_action_pose_rviz", 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, self.ee_pose_topic, self.pose_callback, 10
        )
        self.button_sub = self.create_subscription(
            TouchButtonEvent, self.button_topic, self.button_callback, 10
        )

    def _setup_tf(self) -> None:
        """TF2のBufferとListenerを初期化します。"""
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _log_topic_info(self) -> None:
        """Publisherとsubscriberのtopic一覧をログ出力します。"""
        self.get_logger().info("=== TouchAgent Topic Configuration ===")
        self.get_logger().info(f"  - teleoperation_mode: {self.teleop_mode}")
        self.get_logger().info(f"  - robot_base_frame: {self.robot_base_frame}")
        self.get_logger().info(f"  - feedback_wrench_sensor_frame: {self.feedback_wrench_sensor_frame}")
        self.get_logger().info("=======================================")

    def _wait_for_first_pose(self) -> None:
        """指定したトピックから最初のPoseメッセージが届くまで待機します。"""
        self.get_logger().info(f"トピック '{self.ee_pose_topic}' からのメッセージを待機中...")
        start_time = time.time()
        while self._touch_current_pose is None and rclpy.ok():
            if time.time() - start_time > 5.0:
                self.get_logger().error(f"タイムアウト: トピック '{self.ee_pose_topic}' が利用できません。終了します。")
                exit()
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Touchデバイスのポーズを正常に受信しました。")

    def pose_callback(self, msg: PoseStamped) -> None:
        """Touchデバイスの姿勢を購読し、numpy配列として保存します。"""
        quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                        msg.pose.orientation.z, msg.pose.orientation.w])
        if np.linalg.norm(quat) < 1e-6:
            quat = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            quat = quat / np.linalg.norm(quat)
        self._touch_current_pose = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            quat[0], quat[1], quat[2], quat[3]
        ])

    def button_callback(self, msg: TouchButtonEvent) -> None:
        """Touchデバイスのボタン状態を購読し、フラグを更新します。"""
        self._is_teleop_active = (msg.white_button == 1)
        self._is_z_lock_active = (msg.grey_button == 1)
        
    def transform_wrench(
        self, wrench_array: np.ndarray, transform: tf2_ros.TransformStamped
    ) -> Tuple[WrenchStamped, TouchFeedback]:
        """
        レンチ（力とトルク）を指定された座標系に変換し、フィードバックメッセージを作成します。
        """
        # 1. 観測されたレンチをスケーリングし、最大値でクリッピングする
        scaled_wrench = wrench_array * self.force_scale_to_touch
        force_in_tool = np.clip(scaled_wrench[:3], -self.touch_max_force, self.touch_max_force)
        torque_in_tool = np.clip(scaled_wrench[3:], -self.touch_max_force, self.touch_max_force)

        # 2. base_link 座標系への変換に必要な回転と並進を取得
        rotation = R.from_quat([
            transform.transform.rotation.x, transform.transform.rotation.y,
            transform.transform.rotation.z, transform.transform.rotation.w
        ]).as_matrix()
        
        translation_vector = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        # 3. レンチを base_link 座標系に変換
        force_in_base = rotation @ force_in_tool
        torque_in_base = rotation @ torque_in_tool + np.cross(translation_vector, force_in_base)

        # 4. RViz可視化用メッセージ (base_link 座標系) を作成
        wrench_vis_msg = WrenchStamped()
        wrench_vis_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_vis_msg.header.frame_id = self.robot_base_frame
        wrench_vis_msg.wrench.force.x, wrench_vis_msg.wrench.force.y, wrench_vis_msg.wrench.force.z = force_in_base
        wrench_vis_msg.wrench.torque.x, wrench_vis_msg.wrench.torque.y, wrench_vis_msg.wrench.torque.z = torque_in_base

        # 5. Touchデバイスへのフィードバック用メッセージを作成
        feedback_msg = TouchFeedback()
        
        # ======================= ここを修正 =======================
        # ROS1の正しい実装に基づき、フィードバックする力を tool 座標系から base 座標系に変更します。
        # これにより、RVizでの可視化とHapticsデバイスへのフィードバックが同じ座標系の力になります。
        feedback_msg.force.x, feedback_msg.force.y, feedback_msg.force.z = force_in_base
        # ==========================================================

        return wrench_vis_msg, feedback_msg
    
    def calculate_pose_difference(
        self, touch_start: np.ndarray, touch_current: np.ndarray, robot_start: np.ndarray
    ) -> np.ndarray:
        """
        Touchデバイスの移動量からロボットの目標姿勢を計算します。
        """
        pos_diff = touch_current[:3] - touch_start[:3]
        start_rot = R.from_quat(touch_start[3:])
        current_rot = R.from_quat(touch_current[3:])
        relative_rot = current_rot * start_rot.inv()
        robot_start_rot = R.from_quat(robot_start[3:])
        new_robot_rot = relative_rot * robot_start_rot
        new_robot_pos = robot_start[:3] + pos_diff
        return np.concatenate((new_robot_pos, new_robot_rot.as_quat()))

    def act(self, obs: Dict[str, np.ndarray], force_pose_update: bool = False) -> Dict:
        """
        観測(obs)に基づいて行動(action)を決定します。
        """
        current_ee_pose = np.concatenate((obs["ee_pos"], obs["ee_quat"]))
        
        if self.teleop_mode == "bilateral":
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_base_frame, 
                    self.feedback_wrench_sensor_frame, 
                    tf2_ros.Time(), 
                    timeout=Duration(seconds=1.0)
                )
                wrench_vis, wrench_feedback = self.transform_wrench(obs["ee_wrench"], transform)
                
                self.force_feedback_vis_pub.publish(wrench_vis)
                self.force_feedback_pub.publish(wrench_feedback)
            except tf2_ros.TransformException as ex:
                self.get_logger().debug(f"力覚フィードバック処理でTF変換に失敗: {ex}")

        if self._is_teleop_active and not self._was_teleop_active:
            # ボタンを押した時の開始姿勢は、保持されている目標姿勢（なければ現在の実際の姿勢）
            if self._robot_current_pose is not None:
                self._robot_start_pose = self._robot_current_pose.copy()
            else:
                self._robot_start_pose = current_ee_pose
            
            self._touch_start_pose = self._touch_current_pose
            self.get_logger().info("テレオペレーション開始")
        elif not self._is_teleop_active and self._was_teleop_active:
            # ボタンを離した時点で、最後に計算された目標姿勢を保存（current_ee_poseではなく）
            if self._last_target_pose is not None:
                self._robot_current_pose = self._last_target_pose.copy()
            else:
                self._robot_current_pose = current_ee_pose
            self.get_logger().info("テレオペレーション終了")
        
        self._was_teleop_active = self._is_teleop_active

        target_pose = np.zeros(7)
        if self._is_teleop_active:
            if self._touch_start_pose is not None and self._robot_start_pose is not None:
                if self._is_z_lock_active:
                    target_pose = self.calculate_pose_difference(
                        self._touch_start_pose, self._touch_current_pose, self._robot_start_pose
                    )
                    target_pose[3:] = self.z_down_quat
                    self._last_target_pose = target_pose.copy()  # 計算した目標姿勢を保存
                else:
                    target_pose = self.calculate_pose_difference(
                        self._touch_start_pose, self._touch_current_pose, self._robot_start_pose
                    )
                    self._last_target_pose = target_pose.copy()  # 計算した目標姿勢を保存
            else:
                target_pose = self._robot_current_pose if self._robot_current_pose is not None else current_ee_pose
        else:
            # テレオペ非アクティブ時の姿勢維持ロジック
            if self._robot_current_pose is None or not self._pose_initialized:
                self._robot_current_pose = current_ee_pose
                self._pose_initialized = True
            elif force_pose_update:
                self._robot_current_pose = current_ee_pose
            
            target_pose = self._robot_current_pose

        target_quat = target_pose[3:]
        action_dict = {
            "joint_positions": np.zeros(6),
            "ee_pos": target_pose[:3],
            "ee_quat": target_quat,
            "ee_rot_matrix": R.from_quat(target_quat).as_matrix(),
            "ee_euler": R.from_quat(target_quat).as_euler('xyz')
        }
        
        debug_pose_msg = PoseStamped()
        debug_pose_msg.header.stamp = self.get_clock().now().to_msg()
        debug_pose_msg.header.frame_id = self.robot_base_frame
        debug_pose_msg.pose.position.x = float(target_pose[0])
        debug_pose_msg.pose.position.y = float(target_pose[1])
        debug_pose_msg.pose.position.z = float(target_pose[2])
        debug_pose_msg.pose.orientation.x = float(target_quat[0])
        debug_pose_msg.pose.orientation.y = float(target_quat[1])
        debug_pose_msg.pose.orientation.z = float(target_quat[2])
        debug_pose_msg.pose.orientation.w = float(target_quat[3])
        self.touch_debug_action_pose_pub.publish(debug_pose_msg)
        
        return action_dict

def main(args=None):
    rclpy.init(args=args)
    touch_agent = TouchAgent()
    # このエージェントは外部ループから`act`が呼び出される想定
    touch_agent.get_logger().info("TouchAgent node has been initialized.")
    # rclpy.spin(touch_agent) # 独立して動作させる場合はspinが必要
    touch_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()