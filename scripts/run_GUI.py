#!/usr/bin/env python3
import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class ROSButtonApp(Node):
    """
    tkinterのGUIを持ち、ボタン操作でROS2トピックにメッセージを送信するノード。
    """
    def __init__(self):
        # 1. ROS2ノードを初期化
        super().__init__('button_publisher_node')
        
        # 2. ROS2パブリッシャーを作成
        self.pub = self.create_publisher(String, '/button_state', 10)
        
        # 3. TkinterのGUIをセットアップ
        self.root = tk.Tk()
        self.root.title("ROS2 Button Publisher")
        self.root.geometry("400x300")

        button_font = ("Helvetica", 24, "bold")
        
        # Startボタン
        start_button = tk.Button(self.root, text="Start", font=button_font, 
                                 width=10, height=2, 
                                 command=lambda: self.publish_message("start"))
        start_button.pack(pady=20, expand=True)
        
        # Quitボタン
        quit_button = tk.Button(self.root, text="Quit", font=button_font, 
                                width=10, height=2, 
                                command=self.quit_application)
        quit_button.pack(pady=20, expand=True)
        
        # ウィンドウの「x」ボタンが押されたときの処理を登録
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def publish_message(self, message: str):
        """ROS2トピックにメッセージを送信する関数"""
        # ROS2ではメッセージオブジェクトを作成して渡す必要がある
        msg = String()
        msg.data = message
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.pub.publish(msg)

    def quit_application(self):
        """Quitボタンが押されたときの動作"""
        self.get_logger().info('Quit button pressed. Shutting down...')
        self.publish_message("quit")
        self.on_close()

    def on_close(self):
        """アプリケーション終了時のクリーンアップ処理"""
        self.get_logger().info("Closing GUI window.")
        # tkinterのメインループを終了させる
        self.root.quit()

    def run_gui(self):
        """tkinterのメインループを開始する"""
        self.root.mainloop()

def main(args=None):
    # ROS2クライアントライブラリを初期化
    rclpy.init(args=args)
    
    # アプリケーション（ノード）のインスタンスを作成
    app = ROSButtonApp()

    # ROS2ノードのスピン（コールバック処理など）を別スレッドで実行
    # これにより、GUIのメインループとROS2の処理が互いにブロックしなくなる
    ros_spin_thread = threading.Thread(target=rclpy.spin, args=(app,), daemon=True)
    ros_spin_thread.start()

    try:
        # メインスレッドでGUIを実行
        app.run_gui()
    except KeyboardInterrupt:
        app.get_logger().info('Keyboard interrupt detected, shutting down.')
    finally:
        # アプリケーション終了時にノードを破棄し、rclpyをシャットダウン
        app.get_logger().info('Shutting down ROS2 node.')
        app.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()