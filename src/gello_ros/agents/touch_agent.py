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
        self.declare_parameter("touch_base_frame", "touch_base")
        self.declare_parameter("feedback_wrench_sensor_frame", "tool0")

        self.ee_pose_topic = self.get_parameter("touch_ee_pose_topic").get_parameter_value().string_value
        self.button_topic = self.get_parameter("touch_button_topic").get_parameter_value().string_value
        self.force_feedback_topic = self.get_parameter("touch_force_feedback_topic").get_parameter_value().string_value
        self.touch_max_force = self.get_parameter("touch_max_force").get_parameter_value().double_value
        self.force_scale_to_touch = self.get_parameter("force_scale_to_touch").get_parameter_value().double_value
        self.teleop_mode = self.get_parameter("teleoperation_mode").get_parameter_value().string_value
        self.touch_base_frame = self.get_parameter("touch_base_frame").get_parameter_value().string_value
        self.feedback_wrench_sensor_frame = self.get_parameter("feedback_wrench_sensor_frame").get_parameter_value().string_value

        if self.teleop_mode not in ["unilateral", "bilateral"]:
            self.get_logger().error(f"無効な通信モードです: {self.teleop_mode}。終了します。")
            # rclpy.shutdown() や raise を使う方がより堅牢
            exit()
        self.get_logger().info(f"{self.teleop_mode} モードでテレオペレーションを開始します。")

    def _setup_ros_communications(self) -> None:
        """PublisherとSubscriberを初期化します。"""
        # Publisher
        self.force_feedback_vis_pub = self.create_publisher(
            WrenchStamped, "force_feedback_vis", 10
        )
        self.force_feedback_pub = self.create_publisher(
            TouchFeedback, self.force_feedback_topic, 10
        )
        self.touch_debug_action_pose_pub = self.create_publisher(
            PoseStamped, "touch_debug_action_pose", 10
        )
        # Subscriber
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

    def _wait_for_first_pose(self) -> None:
        """指定したトピックから最初のPoseメッセージが届くまで待機します。"""
        self.get_logger().info(f"トピック '{self.ee_pose_topic}' からのメッセージを待機中...")
        start_time = time.time()
        while self._touch_current_pose is None and rclpy.ok():
            if time.time() - start_time > 5.0:  # 5秒のタイムアウト
                self.get_logger().error(f"タイムアウト: トピック '{self.ee_pose_topic}' が利用できません。終了します。")
                exit()
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Touchデバイスのポーズを正常に受信しました。")

    def pose_callback(self, msg: PoseStamped) -> None:
        """Touchデバイスの姿勢を購読し、numpy配列として保存します。"""
        # クォータニオンを検証
        quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                        msg.pose.orientation.z, msg.pose.orientation.w])
        quat_norm = np.linalg.norm(quat)
        
        if quat_norm < 1e-6:  # ゼロノルムの場合
            self.get_logger().warn("受信したクォータニオンがゼロノルムです。デフォルト値を設定します。")
            quat = np.array([0.0, 0.0, 0.0, 1.0])  # 単位クォータニオン
        else:
            # 正規化
            quat = quat / quat_norm
            
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

        Args:
            wrench_array: [fx, fy, fz, tx, ty, tz] 形式のレンチ配列 (tool0座標系)。
            transform: base_linkからtool0への座標変換。

        Returns:
            RViz可視化用のWrenchStampedと、Touchデバイス用のTouchFeedbackのタプル。
        """
        # 1. 観測されたレンチをスケーリングし、最大値でクリッピングする
        scaled_wrench = wrench_array * self.force_scale_to_touch
        force_in_tool = np.clip(scaled_wrench[:3], -self.touch_max_force, self.touch_max_force)
        torque_in_tool = np.clip(scaled_wrench[3:], -self.touch_max_force, self.touch_max_force)

        # 2. RVizでの可視化のために、レンチを base_link 座標系に変換する
        # 変換用の回転行列と並進ベクトルを取得
        rotation_matrix = R.from_quat([
            transform.transform.rotation.x, transform.transform.rotation.y,
            transform.transform.rotation.z, transform.transform.rotation.w
        ]).as_matrix()
        
        # lookup_transform("base_link", "tool0")で得られる並進ベクトルは、
        # base_linkから見たtool0の位置ベクトル
        translation_vector_in_base = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        # 座標変換の計算
        force_in_base = rotation_matrix @ force_in_tool
        # トルクの変換: T_base = R @ T_tool + P_base_tool x F_base
        torque_in_base = rotation_matrix @ torque_in_tool + np.cross(translation_vector_in_base, force_in_base)

        # 3. RViz可視化用メッセージ (base_link 座標系) を作成
        wrench_vis_msg = WrenchStamped()
        wrench_vis_msg.header.stamp = self.get_clock().now().to_msg()
        # frame_idは可視化する座標系に合わせるのが一般的
        wrench_vis_msg.header.frame_id = "base_link"
        wrench_vis_msg.wrench.force.x, wrench_vis_msg.wrench.force.y, wrench_vis_msg.wrench.force.z = force_in_base
        wrench_vis_msg.wrench.torque.x, wrench_vis_msg.wrench.torque.y, wrench_vis_msg.wrench.torque.z = torque_in_base

        # 4. Touchデバイスへのフィードバック用メッセージを作成
        feedback_msg = TouchFeedback()
        
        # --- ここが重要な変更点 ---
        # 力覚フィードバックは、ユーザーが感じる力の方向と一致させるため、
        # 座標変換前の「ツール座標系」の力 (force_in_tool) を使用します。
        # これにより、ロボットの先端が受けた力をそのまま直感的に感じることができます。
        feedback_msg.force.x, feedback_msg.force.y, feedback_msg.force.z = force_in_tool

        return wrench_vis_msg, feedback_msg
    
    def calculate_pose_difference(
        self, touch_start: np.ndarray, touch_current: np.ndarray, robot_start: np.ndarray
    ) -> np.ndarray:
        """
        Touchデバイスの移動量からロボットの目標姿勢を計算します。

        Args:
            touch_start: テレオペ開始時のTouchデバイスの姿勢 [x,y,z,qx,qy,qz,qw]。
            touch_current: 現在のTouchデバイスの姿勢。
            robot_start: テレオペ開始時のロボットのEE姿勢。

        Returns:
            ロボットの新しい目標姿勢 [x,y,z,qx,qy,qz,qw]。
        """
        # 並進の差分を計算
        pos_diff = touch_current[:3] - touch_start[:3]
        
        # 回転の差分を計算
        start_rot = R.from_quat(touch_start[3:])
        current_rot = R.from_quat(touch_current[3:])
        relative_rot = current_rot * start_rot.inv()

        # ロボットの開始姿勢に差分を適用
        robot_start_rot = R.from_quat(robot_start[3:])
        new_robot_rot = relative_rot * robot_start_rot
        
        new_robot_pos = robot_start[:3] + pos_diff
        
        return np.concatenate((new_robot_pos, new_robot_rot.as_quat()))

    def act(self, obs: Dict[str, np.ndarray], force_pose_update: bool = False) -> Dict:
        """
        観測(obs)に基づいて行動(action)を決定します。

        Args:
            obs: ロボットからの観測データ（EE姿勢、レンチなど）。
            force_pose_update: 現在のEE姿勢を強制的に目標値とするフラグ。

        Returns:
            行動指令を含む辞書。
        """
        current_ee_pose = np.concatenate((obs["ee_pos"], obs["ee_quat"]))
        
        # --- 力覚フィードバックの計算と送信 ---
        if self.teleop_mode == "bilateral":
            try:
                # パラメータで指定されたフレーム名を使用してTF変換を実行
                transform = self.tf_buffer.lookup_transform(
                    self.touch_base_frame, 
                    self.feedback_wrench_sensor_frame, 
                    tf2_ros.Time(), 
                    timeout=Duration(seconds=1.0)
                )
                wrench_vis, wrench_feedback = self.transform_wrench(obs["ee_wrench"], transform)
                
                self.force_feedback_vis_pub.publish(wrench_vis)
                self.force_feedback_pub.publish(wrench_feedback)
            except tf2_ros.TransformException as ex:
                # TF変換に失敗した場合（ダミーコントローラー等）は、単位変換を使用
                self.get_logger().debug(f"TF変換に失敗しました（{self.touch_base_frame} -> {self.feedback_wrench_sensor_frame}）、単位変換を使用します: {ex}")
                try:
                    # 単位変換を作成
                    from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
                    identity_transform = TransformStamped()
                    identity_transform.header.frame_id = self.touch_base_frame
                    identity_transform.child_frame_id = self.feedback_wrench_sensor_frame
                    identity_transform.header.stamp = self.get_clock().now().to_msg()
                    identity_transform.transform.translation = Vector3(x=0.0, y=0.0, z=0.0)
                    identity_transform.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    
                    wrench_vis, wrench_feedback = self.transform_wrench(obs["ee_wrench"], identity_transform)
                    
                    self.force_feedback_vis_pub.publish(wrench_vis)
                    self.force_feedback_pub.publish(wrench_feedback)
                except Exception as e:
                    self.get_logger().debug(f"力覚フィードバック処理でエラーが発生しました: {e}")
        
        # --- テレオペレーションの状態遷移 ---
        # 白ボタンが押された瞬間
        if self._is_teleop_active and not self._was_teleop_active:
            self._robot_start_pose = current_ee_pose
            self._touch_start_pose = self._touch_current_pose
            self.get_logger().info("テレオペレーション開始")
        # 白ボタンが離された瞬間
        elif not self._is_teleop_active and self._was_teleop_active:
            self._robot_current_pose = current_ee_pose
            self.get_logger().info("テレオペレーション終了")
        
        self._was_teleop_active = self._is_teleop_active

        # --- 目標姿勢の計算 ---
        target_pose = np.zeros(7)
        if self._is_teleop_active:
            if self._touch_start_pose is not None and self._robot_start_pose is not None:
                # Z軸固定モード（白と灰の両方が押されている）
                if self._is_z_lock_active:
                    target_pose = self.calculate_pose_difference(
                        self._touch_start_pose, self._touch_current_pose, self._robot_start_pose
                    )
                    target_pose[3:] = self.z_down_quat # 回転をZ軸下向きで上書き
                # 通常のテレオペモード
                else:
                    target_pose = self.calculate_pose_difference(
                        self._touch_start_pose, self._touch_current_pose, self._robot_start_pose
                    )
            else:
                # 開始ポーズが未設定の場合は現在のポーズを維持
                target_pose = self._robot_current_pose if self._robot_current_pose is not None else current_ee_pose
        else:
            # テレオペ中でない場合は、最後の姿勢を維持
            if self._robot_current_pose is None or force_pose_update:
                self._robot_current_pose = current_ee_pose
            target_pose = self._robot_current_pose

        # --- 行動指令の生成 ---
        target_quat = target_pose[3:]
        action_dict = {
            "joint_positions": np.zeros(6),  # IKソルバーが計算するためダミー値
            "ee_pos": target_pose[:3],
            "ee_quat": target_quat,
            "ee_rot_matrix": R.from_quat(target_quat).as_matrix(),
            "ee_euler": R.from_quat(target_quat).as_euler('xyz')
        }
        
        # デバッグ用にtouch_debug_action_poseをpublish
        debug_pose_msg = PoseStamped()
        debug_pose_msg.header.stamp = self.get_clock().now().to_msg()
        debug_pose_msg.header.frame_id = "base_link"
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
    """メイン関数：ノードを初期化し、実行します。"""
    rclpy.init(args=args)
    touch_agent = TouchAgent()
    
    # このエージェントは外部のループから `act` が呼び出されることを想定しているため、
    # ここでは spin せずにシャットダウンするのが適切かもしれません。
    # もしノードとして独立して動作させる場合は spin が必要です。
    # rclpy.spin(touch_agent)

    # この例では、初期化が完了したら終了します。
    touch_agent.get_logger().info("TouchAgent node has been initialized.")
    # 必要に応じてクリーンアップ
    touch_agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()