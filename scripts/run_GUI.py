#!/usr/bin/env python3
import tkinter as tk
import rospy
from std_msgs.msg import String
import threading
import signal
import sys

class ROSButtonApp:
    def __init__(self, root):
        # ROSの初期化
        rospy.init_node('button_publisher_node', anonymous=True)
        self.pub = rospy.Publisher('/button_state', String, queue_size=10)
        
        # TkinterのGUIセットアップ
        self.root = root
        self.root.title("ROS Button Publisher")
        self.root.geometry("400x300")  # ウィンドウサイズの設定

        # フォント設定
        button_font = ("Helvetica", 24, "bold")
        
        # Startボタン
        self.start_button = tk.Button(root, text="Start", font=button_font, width=10, height=2, 
                                       command=lambda: self.publish_message("start"))
        self.start_button.pack(pady=20, expand=True)
        
        # Quitボタン
        self.quit_button = tk.Button(root, text="Quit", font=button_font, width=10, height=2, 
                                      command=self.quit_application)
        self.quit_button.pack(pady=20, expand=True)
        
        # Quit処理を閉じる際にもROSノードをシャットダウン
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def publish_message(self, message):
        """ROSトピックにメッセージを送信する関数"""
        rospy.loginfo(f"Publishing: {message}")
        self.pub.publish(message)

    def quit_application(self):
        """Quitボタンを押したときの動作"""
        self.publish_message("quit")
        self.on_close()

    def on_close(self):
        """アプリケーション終了時の処理"""
        rospy.loginfo("Shutting down ROS node.")
        rospy.signal_shutdown("GUI Closed")
        self.root.quit()  # Tkinterアプリケーションを終了

def ros_spin_thread():
    """ROSのspinを別スレッドで実行"""
    rospy.spin()

def signal_handler(sig, frame):
    """Ctrl + C をキャッチして終了処理を実行"""
    rospy.loginfo("Ctrl + C detected. Shutting down.")
    rospy.signal_shutdown("Keyboard Interrupt")
    sys.exit(0)

if __name__ == "__main__":
    # SIGINT (Ctrl + C) シグナルハンドラを登録
    signal.signal(signal.SIGINT, signal_handler)

    root = tk.Tk()
    app = ROSButtonApp(root)

    # ROSのスピンを別スレッドで実行
    ros_thread = threading.Thread(target=ros_spin_thread)
    ros_thread.daemon = True  # スレッドが終了時に強制終了されるように設定
    ros_thread.start()

    # Tkinterのイベントループを開始
    try:
        root.mainloop()
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard Interrupt received. Shutting down.")
        rospy.signal_shutdown("Keyboard Interrupt")
        root.destroy()
