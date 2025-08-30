#!/usr/bin/env python3
import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import signal
import sys

# Global shutdown flag
shutdown_flag = False


class ROSButtonApp(Node):
    """
    tkinterのGUIを持ち、ボタン操作でROS2トピックにメッセージを送信するノード。
    ComponentManager対応版
    """
    def __init__(self, node_name='button_publisher_node', **kwargs):
        # 1. ROS2ノードを初期化
        super().__init__(node_name, **kwargs)
        
        # Component mode detection
        self.component_mode = kwargs.get('component_mode', False)
        self.shutdown_requested = False
        
        # 2. ROS2パブリッシャーを作成
        self.pub = self.create_publisher(String, '/button_state', 10)
        
        # 3. Initialize GUI in component mode
        if self.component_mode:
            self.setup_component_gui()
        else:
            self.setup_standalone_gui()

    def setup_component_gui(self):
        """Component mode GUI setup with proper threading"""
        self.get_logger().info("Initializing GUI in component mode")
        
        # Create GUI in a separate thread for component mode
        self.gui_thread = threading.Thread(target=self._create_gui_thread, daemon=True)
        self.gui_thread.start()
        
    def setup_standalone_gui(self):
        """Standalone mode GUI setup"""
        self.get_logger().info("Initializing GUI in standalone mode")
        self._create_gui()
        
    def _create_gui_thread(self):
        """Create GUI in a separate thread"""
        try:
            self._create_gui()
            # Start GUI mainloop with proper shutdown handling
            while not self.shutdown_requested and not shutdown_flag and hasattr(self, 'root'):
                try:
                    self.root.update()
                    # Small sleep to prevent high CPU usage
                    import time
                    time.sleep(0.001)
                except tk.TclError:
                    # GUI window was closed
                    break
                except KeyboardInterrupt:
                    # Handle Ctrl+C in GUI thread
                    self.shutdown_requested = True
                    break
                
        except Exception as e:
            self.get_logger().error(f"GUI thread error: {e}")
        finally:
            self.get_logger().info("GUI thread finishing")
    
    def _create_gui(self):
        """Create the tkinter GUI"""
        # 3. TkinterのGUIをセットアップ
        self.root = tk.Tk()
        self.root.title("ROS2 Button Publisher (Component)")
        self.root.geometry("400x300")

        button_font = ("Helvetica", 24, "bold")
        
        # Startボタン
        start_button = tk.Button(self.root, text="Start", font=button_font, 
                                 width=10, height=2, 
                                 command=lambda: self.publish_message("start"),
                                 bg="#4CAF50", fg="white")
        start_button.pack(pady=20, expand=True)
        
        # Passボタン (追加)
        pass_button = tk.Button(self.root, text="Pass", font=button_font, 
                                width=10, height=2, 
                                command=lambda: self.publish_message("pass"),
                                bg="#FF9800", fg="white")
        pass_button.pack(pady=10, expand=True)
        
        # Quitボタン
        quit_button = tk.Button(self.root, text="Quit", font=button_font, 
                                width=10, height=2, 
                                command=self.quit_application,
                                bg="#f44336", fg="white")
        quit_button.pack(pady=20, expand=True)
        
        # Status display
        self.status_var = tk.StringVar()
        self.status_var.set("Status: Ready")
        status_label = tk.Label(self.root, textvariable=self.status_var, 
                               font=("Helvetica", 12), fg="blue")
        status_label.pack(pady=10)
        
        # ウィンドウの「x」ボタンが押されたときの処理を登録
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def publish_message(self, message: str):
        """ROS2トピックにメッセージを送信する関数"""
        try:
            # ROS2ではメッセージオブジェクトを作成して渡す必要がある
            msg = String()
            msg.data = message
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.pub.publish(msg)
            
            # Update status display
            if hasattr(self, 'status_var'):
                self.status_var.set(f"Status: {message.capitalize()}")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing message: {e}")

    def quit_application(self):
        """Quitボタンが押されたときの動作"""
        self.get_logger().info('Quit button pressed. Shutting down...')
        self.publish_message("quit")
        self.on_close()

    def on_close(self):
        """アプリケーション終了時のクリーンアップ処理"""
        self.get_logger().info("Closing GUI window.")
        self.shutdown_requested = True
        
        try:
            # tkinterのメインループを終了させる
            if hasattr(self, 'root'):
                self.root.quit()
                if not self.component_mode:
                    self.root.destroy()
        except Exception as e:
            self.get_logger().error(f"Error during GUI cleanup: {e}")

    def run_gui(self):
        """tkinterのメインループを開始する (standalone mode)"""
        if not self.component_mode and hasattr(self, 'root'):
            try:
                # Use update loop instead of mainloop for better control
                while not self.shutdown_requested and not shutdown_flag and hasattr(self, 'root'):
                    try:
                        self.root.update()
                        # Small sleep to prevent high CPU usage
                        import time
                        time.sleep(0.001)
                    except tk.TclError:
                        # GUI window was closed
                        break
            except KeyboardInterrupt:
                self.get_logger().info("Keyboard interrupt in GUI loop")
                self.shutdown_requested = True
            
    def cleanup(self):
        """Clean up resources before shutdown"""
        self.get_logger().info("Starting GUI cleanup...")
        try:
            self.on_close()
            if hasattr(self, 'gui_thread') and self.gui_thread.is_alive():
                self.gui_thread.join(timeout=1.0)
            self.get_logger().info("GUI cleanup completed")
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")


def signal_handler(sig, frame):
    """Handle shutdown signals gracefully"""
    print("\nGUI shutdown signal received, cleaning up...")
    # Set global flag for GUI threads to check
    global shutdown_flag
    shutdown_flag = True
    

def main(args=None):
    """Main function for standalone execution"""
    # Set up signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # ROS2クライアントライブラリを初期化
    rclpy.init(args=args)
    
    app = None
    ros_spin_thread = None
    
    try:
        # アプリケーション（ノード）のインスタンスを作成
        app = ROSButtonApp()

        # ROS2ノードのスピン（コールバック処理など）を別スレッドで実行
        # これにより、GUIのメインループとROS2の処理が互いにブロックしなくなる
        ros_spin_thread = threading.Thread(target=rclpy.spin, args=(app,), daemon=True)
        ros_spin_thread.start()

        # メインスレッドでGUIを実行
        app.run_gui()
        
    except KeyboardInterrupt:
        global shutdown_flag
        shutdown_flag = True
        if app:
            app.get_logger().info('Keyboard interrupt detected, shutting down.')
            app.shutdown_requested = True
    except Exception as e:
        if app:
            app.get_logger().error(f'Unexpected error: {e}')
    finally:
        # アプリケーション終了時にノードを破棄し、rclpyをシャットダウン
        if app:
            app.get_logger().info('Shutting down ROS2 GUI node.')
            app.cleanup()
            app.destroy_node()
            
        if ros_spin_thread and ros_spin_thread.is_alive():
            ros_spin_thread.join(timeout=1.0)
            
        rclpy.shutdown()
        print("GUI shutdown complete")


# Component entry point for ROS2 ComponentManager
def get_gui_node_component():
    """Entry point for component manager"""
    def _create_node(*args, **kwargs):
        kwargs['component_mode'] = True
        return ROSButtonApp(**kwargs)
    return _create_node


if __name__ == '__main__':
    main()