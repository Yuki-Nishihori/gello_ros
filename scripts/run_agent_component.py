#!/usr/bin/env python3
import os
import signal
import sys
import glob
import time
from typing import List
from functools import partial

import numpy as np
from policy_config import (
    POLICY_CONFIG,
    TASK_CONFIG,
    TRAIN_CONFIG,
)  # must import first
from gello_ros.agents.agent import DummyAgent
from gello_ros.agents.gello_agent import GelloAgent
from gello_ros.agents.touch_agent import TouchAgent
from gello_ros.agents.act_agent import ACTAgent
from gello_ros.data_utils.save_episode import save_episode
from gello_ros.env import RobotEnv
from gello_ros.robots.robot import PrintRobot
from gello_ros.policy.utils import *

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter, ParameterType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Wrench, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
import threading
import asyncio



class AgentNode(Node):
    def __init__(self, node_name='agent_node', **kwargs):
        super().__init__(node_name, **kwargs)
        
        # Initialize variables
        self.agent = None
        self.camera_images = {}
        self.button_state = "pass"
        self.bridge = CvBridge()
        self.shutdown_requested = False
        self.component_mode = kwargs.get('component_mode', False)
        
        # Component execution state
        self.start_time = time.time()
        self.current_episode_number = 0
        self.current_save_thread = None
        self.obs = None
        
        # Declare parameters with default values
        self._declare_parameters()
        
        # Get parameters
        self.get_parameters()
        
        # Initialize subscribers if needed
        if self.use_save_interface:
            self.start_button_subscriber()
            self.get_logger().info(f"Save interface enabled, listening to /GUI_button")
        
        # Initialize camera subscribers
        if not self.mock and self.camera_names:
            self.start_camera_subscribers()
        # Initialize robot and agent
        self.initialize_robot()
        self.initialize_agent()
        

    def _declare_parameters(self):
        """Declare all ROS2 parameters with default values"""
        self.declare_parameter("agent_type", "gello")
        self.declare_parameter("camera_names", [""])
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("wrench_dim", 6)
        self.declare_parameter("control_hz", 100)
        self.declare_parameter("robot_home_joints_with_gello", [0.0])
        self.declare_parameter("robot_home_pose_with_touch", [0.0])
        self.declare_parameter("robot_start_pose_with_touch", [0.0])
        self.declare_parameter("controller_type", "cartesian_impedance_controller")
        self.declare_parameter("control_mode", "cartesian")
        self.declare_parameter("use_gripper", False)
        self.declare_parameter("use_FT_sensor", False)
        self.declare_parameter("skip_initial_move", True)
        self.declare_parameter("save_episode", False)
        self.declare_parameter("gello_port", "")
        self.declare_parameter("number_of_episodes", 1)
        self.declare_parameter("number_of_steps", 10000)
        self.declare_parameter("eval_ckpt_dir", "policy_last.ckpt")
        self.declare_parameter("save_episode_dir", "./episode_data")
        self.declare_parameter("task_name", "default")
        self.declare_parameter("robot_description", "")

    def get_parameters(self):
        """Get all parameters from ROS2 parameter server"""
        self.get_logger().info(f"Getting ROS parameters...")
        self.agent_type = self.get_parameter("agent_type").get_parameter_value().string_value
        self.camera_names = self.get_parameter("camera_names").get_parameter_value().string_array_value
        self.hz = self.get_parameter("control_hz").get_parameter_value().integer_value
        self.robot_home_joints_with_gello = self.get_parameter("robot_home_joints_with_gello").get_parameter_value().double_array_value
        self.robot_home_pose_with_touch = self.get_parameter("robot_home_pose_with_touch").get_parameter_value().double_array_value
        self.robot_start_pose_with_touch = self.get_parameter("robot_start_pose_with_touch").get_parameter_value().double_array_value
        self.controller_type = self.get_parameter("controller_type").get_parameter_value().string_value
        self.control_mode = self.get_parameter("control_mode").get_parameter_value().string_value
        self.use_gripper = self.get_parameter("use_gripper").get_parameter_value().bool_value
        self.use_FT_sensor = self.get_parameter("use_FT_sensor").get_parameter_value().bool_value
        self.skip_initial_move = self.get_parameter("skip_initial_move").get_parameter_value().bool_value
        self.use_save_interface = self.get_parameter("save_episode").get_parameter_value().bool_value
        gello_port_param = self.get_parameter("gello_port").get_parameter_value().string_value
        self.gello_port = gello_port_param if gello_port_param else None
        self.number_of_episodes = self.get_parameter("number_of_episodes").get_parameter_value().integer_value
        self.number_of_steps = self.get_parameter("number_of_steps").get_parameter_value().integer_value
        self.eval_ckpt_dir = self.get_parameter("eval_ckpt_dir").get_parameter_value().string_value
        self.robot_description = self.get_parameter("robot_description").get_parameter_value().string_value
        
        # Mock is hardcoded to False for now
        self.mock = False

    def _camera_color_callback(self, camera_name, msg: Image):
        """Callback for the color image."""
        try:
            # self.get_logger().debug(f"Received image for {camera_name}") # ログが多すぎる場合はコメントアウト
            self.camera_images[camera_name] = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image for {camera_name}: {e}")

    def start_camera_subscribers(self):
        """Start camera subscribers for all camera names"""
        for camera_name in self.camera_names:
            self.create_subscription(
                Image,
                f"/{camera_name}/color/image_raw",
                partial(self._camera_color_callback, camera_name),
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"Subscribed to /{camera_name}/color/image_raw")

    def _button_callback(self, msg: String):
        """Callback for button state"""
        self.get_logger().info(f"Button state changed to: {msg.data}")
        self.button_state = msg.data

    def start_button_subscriber(self):
        """Start button state subscriber"""
        self.create_subscription(
            String,
            "/GUI_button",
            self._button_callback,
            1  # QoS depth
        )

    def initialize_robot(self):
        """Initialize robot based on controller type"""
        try:
            if self.mock:
                self.robot = PrintRobot(8, dont_print=True)
                self.camera_clients = {}
            else:
                self.camera_clients = {}

                if self.controller_type == "joint_trajectory_controller":
                    self.get_logger().info(f"Using controller: {self.controller_type}")
                    from gello_ros.robots.ros_joint_trajectory_control_robot import (
                        JointTrajectoryControlRobot,
                    )
                    self.robot = JointTrajectoryControlRobot(self.use_gripper, self.use_FT_sensor)
                elif self.controller_type == "cartesian_compliance_controller":
                    self.get_logger().info(f"Using controller: {self.controller_type}")
                    from gello_ros.robots.ros_cartesian_compliance_control_robot import (
                        CartesianComplianceControlRobot,
                    )
                    self.robot = CartesianComplianceControlRobot(self.use_gripper)
                elif self.controller_type == "cartesian_impedance_controller":
                    self.get_logger().info(f"Using controller: {self.controller_type}")
                    from gello_ros.robots.ros_cartesian_impedance_control_robot import (
                        CartesianImpedanceControlRobot,
                    )
                    self.robot = CartesianImpedanceControlRobot(
                        use_gripper=self.use_gripper, robot_description=self.robot_description
                    )
                elif self.controller_type == "cartesian_motion_controller":
                    self.get_logger().info(f"Using controller: {self.controller_type}")
                    from gello_ros.robots.ros_cartesian_motion_control_robot import (
                        CartesianMotionControlRobot,
                    )
                    self.robot = CartesianMotionControlRobot(self.use_gripper)
                elif self.controller_type == "dummy_controller":
                    self.get_logger().info(f"Using controller: {self.controller_type}")
                    from gello_ros.robots.ros_dummy_control_robot import (
                        DummyControlRobot,
                    )
                    self.robot = DummyControlRobot(self.use_gripper, self.use_FT_sensor)
                else:
                    raise NotImplementedError(
                        f"Controller {self.controller_type} not implemented"
                    )
        except Exception as e:
            self.get_logger().fatal(f"Failed to initialize robot, shutting down. Error: {e}")
            sys.exit(1)

    def initialize_agent(self):
        """Initialize agent based on agent type"""
        try:
            if self.agent_type == "gello":
                assert self.control_mode in "joint"
                self.env = RobotEnv(self.robot, control_rate_hz=self.hz, camera_dict=self.camera_clients, control_mode=self.control_mode)
                self.get_logger().info("Using Gello agent")
                
                if self.gello_port is None:
                    usb_ports = glob.glob("/dev/serial/by-id/*")
                    self.get_logger().info(f"Found {len(usb_ports)} ports")
                    if len(usb_ports) > 0:
                        self.gello_port = usb_ports[0]
                        self.get_logger().info(f"using port {self.gello_port}")
                    else:
                        raise ValueError(
                            "No gello port found, please specify one or plug in gello"
                        )

                gello_reset_joints = np.array(self.robot_home_joints_with_gello)
                self.agent = GelloAgent()
                time.sleep(1)

                # Start the gello agent
                obs = self.env.get_obs()
                robot_joints = obs["joint_positions"]
                self.get_logger().info(f"Robot joints: {robot_joints}")

                gello_curr_joints = np.array(self.env.get_obs()["joint_positions"])

                if not self.skip_initial_move and gello_reset_joints.shape == gello_curr_joints.shape:
                    max_delta = (np.abs(gello_curr_joints - gello_reset_joints)).max()
                    steps = min(int(max_delta / 0.01), 100)
                    for jnt in np.linspace(gello_curr_joints, gello_reset_joints, steps):
                        self.env.step(jnt)
                        time.sleep(0.001)

                # preprocess the agent start position
                agent_start_pos = self.agent.act(self.env.get_obs())
                self.get_logger().info(f"Gello agent start pos: {agent_start_pos}")

                # check if the joints are close
                abs_deltas = np.abs(agent_start_pos - robot_joints)
                id_max_joint_delta = np.argmax(abs_deltas)
                self.get_logger().info(f"Agent start pos: {agent_start_pos}, Robot joints: {robot_joints}")
                max_joint_delta = 0.8
                if abs_deltas[id_max_joint_delta] > max_joint_delta:
                    self.get_logger().warn("Joint deltas are too big, please check the following joints")

                    id_mask = abs_deltas > max_joint_delta
                    ids = np.arange(len(id_mask))[id_mask]
                    for i, delta, joint, current_j in zip(
                        ids,
                        abs_deltas[id_mask],
                        agent_start_pos[id_mask],
                        robot_joints[id_mask],
                    ):
                        self.get_logger().warn(
                            f"joint[{i}]: 	 delta: {delta:4.3f} , leader: 	{joint:4.3f} , follower: 	{current_j:4.3f}"
                        )
                    return

                assert len(agent_start_pos) == len(
                    robot_joints
                ), f"agent output dim = {len(agent_start_pos)}, but env dim = {len(robot_joints)}"

                # soft startup
                max_delta = 0.003
                startup_iterations = 500
                for i in range(startup_iterations):
                    obs = self.env.get_obs()
                    command_joints = self.agent.act(obs)
                    current_joints = obs["joint_positions"]
                    delta = command_joints - current_joints
                    max_joint_delta = np.abs(delta).max()
                    if max_joint_delta > max_delta:
                        delta = delta / max_joint_delta * max_delta
                    self.env.step(current_joints + delta)

                # check if the joints are close
                obs = self.env.get_obs()
                joints = obs["joint_positions"]
                action = self.agent.act(obs)
                if (action - joints > 0.5).any():
                    self.get_logger().warn("Action is too big")
                    self.get_logger().warn(f"action: {action}")
                    self.get_logger().warn(f"joints: {joints}")

                    # print which joints are too big
                    joint_index = np.where(action - joints > 0.8)
                    for j in joint_index:
                        self.get_logger().warn(
                            f"Joint [{j}], leader: {action[j]}, follower: {joints[j]}, diff: {action[j] - joints[j]}"
                        )
                    return
                self._update_full_obs()

            elif self.agent_type == "touch":
                self._initialize_touch_agent_sequence()

                
            elif self.agent_type == "dummy" or self.agent_type == "none":
                self.env = RobotEnv(self.robot, control_rate_hz=self.hz, camera_dict=self.camera_clients, control_mode="joint")
                self.agent = DummyAgent(num_dofs=self.robot.num_dofs())
                self.obs = self.env.get_obs()
                
            elif self.agent_type == "act":
                self.env = RobotEnv(self.robot, control_rate_hz=self.hz, camera_dict=self.camera_clients, control_mode=self.control_mode)
                # load config
                cfg = TASK_CONFIG
                policy_config = POLICY_CONFIG
                train_cfg = TRAIN_CONFIG
                device = os.environ["DEVICE"]
                # load the policy
                policy = make_policy(policy_config["policy_class"], policy_config)
                eval_ckpt_file = os.path.join(self.eval_ckpt_dir, "policy_last.ckpt")
                self.get_logger().info(f"Loading checkpoint: {eval_ckpt_file}")
                loading_status = policy.load_state_dict(
                    torch.load(eval_ckpt_file, map_location=torch.device(device))
                )
                self.get_logger().info(f"Loading status: {loading_status}")
                policy.to(device)
                policy.eval()
                self.get_logger().info("ACT policy loaded")
                if self.camera_names is None:
                    raise ValueError("Camera names not provided")
                self.agent = ACTAgent(
                    policy, self.camera_names, train_cfg, policy_config, task_cfg=cfg, device=device
                )

                # Initialize obs
                self._update_full_obs()
                
            elif self.agent_type == "policy":
                raise NotImplementedError("add your imitation policy here if there is one")
            else:
                raise ValueError("Invalid agent type: %s" % self.agent_type)
        except Exception as e:
            self.get_logger().fatal(f"Failed to initialize agent, shutting down. Error: {e}")
            sys.exit(1)
    

    def _update_full_obs(self):
        """Gets the latest observation from the environment and populates it with camera images."""
        # For standalone mode, spin_once is needed to process callbacks
        rclpy.spin_once(self, timeout_sec=0.001)
        rclpy.spin_once(self.robot, timeout_sec=0.001)
        if isinstance(self.agent, Node):
            rclpy.spin_once(self.agent, timeout_sec=0.001)
        
        self.obs = self.env.get_obs()
        self._add_images_to_obs()

    def _get_obs(self, repetitions=100):
        """Continuously spins to get the latest observation from the environment, waiting for data to be updated."""
        for _ in range(repetitions):
            rclpy.spin_once(self, timeout_sec=0.001)
            rclpy.spin_once(self.robot, timeout_sec=0.001)
            if isinstance(self.agent, Node):
                rclpy.spin_once(self.agent, timeout_sec=0.001)
            self.obs = self.env.get_obs()
        self.get_logger().info(f"Obs ee_pos: {self.obs['ee_pos']}")
        
    def _add_images_to_obs(self):
        """Populates self.obs with the latest camera images."""
        if self.obs is None:
            return
        for camera_name in self.camera_names:
            self.obs[f"{camera_name}_rgb"] = self.camera_images.get(camera_name)

    def _initialize_touch_agent_sequence(self):
        """Initializes the TouchAgent and its pose, handling the initial move logic."""
        assert self.control_mode in "cartesian"
        self.env = RobotEnv(self.robot, control_rate_hz=self.hz, camera_dict=self.camera_clients, control_mode=self.control_mode)
        self.get_logger().info("Using 3D Systems Touch agent")

        self.agent = TouchAgent(
            robot_description=self.robot_description,
        )

        # Move the robot to the configured home pose if not skipping
        if not self.skip_initial_move:
            self.get_logger().info("Moving to the home pose...")
            action = {"ee_pos": self.robot_home_pose_with_touch[:3], "ee_quat": self.robot_home_pose_with_touch[3:]}
            self.env.step(action)
            time.sleep(5)

        # Initialize obs for the main loop
        self._get_obs()
        self.agent.act(self.obs)

    def save_episode_thread(self, episode_number, obs_replay, action_replay):
        """Thread function for saving episodes"""
        save_episode(self, episode_number, obs_replay, action_replay)
    
    def cleanup(self):
        """Clean up resources before shutdown"""
        self.get_logger().info("Starting cleanup...")
        try:
            if hasattr(self, 'robot') and self.robot is not None:
                self.get_logger().info("Stopping robot...")
                
            if hasattr(self, 'agent') and self.agent is not None:
                self.get_logger().info("Cleaning up agent...")
                if hasattr(self.agent, 'cleanup'):
                    self.agent.cleanup()
                    
            self.get_logger().info("Cleanup completed")
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")

        
    def run(self):
        """
        Main execution loop for standalone mode.
        NOTE: This method is NOT called when running as a component.
        The spin_once calls here are necessary for standalone execution.
        """
        self.get_logger().info("Start 🚀🚀🚀")
        start_time = time.time()
        current_episode_number = 0
        current_save_thread = None
        message = ""
        
        try:
            while rclpy.ok() and not self.shutdown_requested:
                if self.use_save_interface:
                    if self.button_state == "start":                                   
                        self.get_logger().info("Moving to the home pose...")
                        action = {"ee_pos": self.robot_home_pose_with_touch[:3], "ee_quat": self.robot_home_pose_with_touch[3:]}
                        self.env.step(action)
                        time.sleep(5)
                        self._get_obs()
                        self.agent.act(self.obs)

                        obs_replay = []
                        action_replay = []
                        if current_save_thread is not None and current_save_thread.is_alive():
                            self.get_logger().warn("Can't start new episode, current episode is still saving")
                            self.button_state = "pass"
                            continue
                        if (current_episode_number + 1) > self.number_of_episodes:
                            self.get_logger().info("All episodes done")
                            break
                        st_episode = time.time()
                        for i in range(self.number_of_steps):

                            # Check if user wants to quit mid-episode
                            if self.button_state == "quit":
                                self.get_logger().info("Quit command received during episode, stopping.")
                                break
                            
                            step_st = time.time()
                            action = self.agent.act(self.obs)
                            self.obs = self.env.step(action)
                            self._add_images_to_obs()
                            action_replay.append(action)
                            obs_replay.append(self.obs)
                            message = f"Episode number: {current_episode_number} Time passed: {round(time.time() - st_episode, 2)}, Time for step: {round((time.time() - step_st)*1000,1)} ms"
                            self.get_logger().info(message)

                        self.get_logger().info("Episode done, saving now")
                        current_save_thread = threading.Thread(target=self.save_episode_thread, args=(current_episode_number, obs_replay, action_replay))
                        current_save_thread.start()


                        self.button_state = "pass"
                        current_episode_number += 1

                    elif self.button_state == "pass":
                        step_st = time.time()
                        action = self.agent.act(self.obs)
                        self.obs = self.env.step(action)
                        message = f"Waiting for the next episode. Time for step: {round((time.time() - step_st)*1000,1)} ms"
                        # self.get_logger().info(message)
                        self._update_full_obs()                        
                    elif self.button_state == "quit":
                        self.get_logger().info("Quit episode recording")
                        break
                    else:
                        raise ValueError(f"Invalid state {self.button_state}")
                elif self.agent_type == "act":
                    # For standalone mode, spin_once is needed to process camera callbacks
                    self._get_obs()
                    # Run the agent
                    for t in range(self.number_of_steps):
                        rclpy.spin_once(self, timeout_sec=0.001)
                        time_passed = time.time() - start_time
                        step_st = time.time()
                        action = self.agent.act(self.obs, t)
                        self.obs = self.env.step(action)
                        self._add_images_to_obs()
                        message = f"Time passed: {round(time_passed, 2)} Step: {t} Time for step: {round((time.time() - step_st)*1000,1)} ms"
                        self.get_logger().info(message)
                else:
                    # For standalone mode, spin_once is needed to process callbacks
                    step_st = time.time()
                    
                    act_start = time.time()
                    action = self.agent.act(self.obs)
                    act_time = (time.time() - act_start) * 1000
                    
                    step_start = time.time()
                    self.obs = self.env.step(action)
                    step_time = (time.time() - step_start) * 1000
                    
                    total_time = (time.time() - step_st) * 1000
                    # self.get_logger().info(f"Timing: act={act_time:.1f}ms, step={step_time:.1f}ms, total={total_time:.1f}ms")
        except KeyboardInterrupt:
            self.get_logger().info("ROS node interrupted by Ctrl+C")
            self.shutdown_requested = True
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
        finally:
            if not self.component_mode:
                self.cleanup()


def signal_handler(sig, frame):
    """Handle shutdown signals gracefully"""
    print("\nShutdown signal received, cleaning up...")
    
def main():
    # Set up signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    rclpy.init()
    agent_node = None
    
    try:
        agent_node = AgentNode()
        agent_node.run()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    finally:
        if agent_node is not None:
            agent_node.cleanup()
        rclpy.shutdown()
        print("Shutdown complete")


# Component entry point for ROS2 ComponentManager
def get_agent_node_component():
    """Entry point for component manager"""
    def _create_node(*args, **kwargs):
        kwargs['component_mode'] = True
        return AgentNode(**kwargs)
    return _create_node

if __name__ == "__main__":
    main()