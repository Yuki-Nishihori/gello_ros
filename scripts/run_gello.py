#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# GelloPublisherがROS2用に修正されていることを想定
from gello_ros.agents.gello_publisher import GelloPublisher

def main(args=None):
    """
    ROS2ノードを初期化し、GelloPublisherを実行するメイン関数。
    """
    # 1. ROS2クライアントライブラリを初期化
    rclpy.init(args=args)

    # 2. ノードを作成
    node = Node('gello_publish_node')

    # 3. パラメータを宣言し、値を取得
    # ROS2では、まずパラメータを宣言する必要があります。
    node.declare_parameter('gello_port', '')  # デフォルト値を空文字列に設定
    port_param = node.get_parameter('gello_port').get_parameter_value().string_value
    
    # パラメータが設定されていない場合、元のコードの'None'と同じ挙動にする
    gello_port = port_param if port_param else None

    # 4. エージェントのインスタンスを作成
    # ROS2では、Publisherなどを持つクラスはNodeオブジェクトを必要とするのが一般的です。
    # そのため、GelloPublisherのコンストラクタがnodeを引数に取ると仮定しています。
    agent = GelloPublisher(node=node, port=gello_port)

    # 5. メイン処理の実行とシャットダウン処理
    try:
        node.get_logger().info('Gello publisherを開始します...')
        # このメソッドが内部でループし、Ctrl+Cまで実行され続けることを想定
        agent.publish_joint_states()
    except KeyboardInterrupt:
        # Ctrl+Cが押されたときの処理
        node.get_logger().info('プログラムを終了します。')
    finally:
        # 6. ノードを破棄し、rclpyをシャットダウン
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()