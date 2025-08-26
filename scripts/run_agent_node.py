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
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import threading
import asyncio


def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


class AgentNode(Node):
    def __init__(self):
        super().__init__('agent_node')
        
        # Initialize variables
        self.agent = None
        self.camera_images = {}
        self.button_state = "pass"
        self.bridge = CvBridge()
        
        # Declare parameters with default values
        self._declare_parameters()
        
        # Get parameters
        self.get_parameters()
        
        # Initialize subscribers if needed
        if self.use_save_interface:
            self.start_button_subscriber()
        
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
        self.declare_parameter("control_hz", 100)
        self.declare_parameter("gello_home_joints", [0.0])
        self.declare_parameter("touch_home_pose", [0.0])
        self.declare_parameter("touch_start_pose", [0.0])
        self.declare_parameter("controller_type", "cartesian_impedance_controller")
        self.declare_parameter("control_mode", "cartesian")
        self.declare_parameter("use_gripper", False)
        self.declare_parameter("use_FT_sensor", False)
        self.declare_parameter("skip_initial_move", False)
        self.declare_parameter("save_episode", False)
        self.declare_parameter("gello_port", "")
        self.declare_parameter("number_of_episodes", 1)
        self.declare_parameter("number_of_steps", 1000)
        self.declare_parameter("eval_ckpt_dir", "policy_last.ckpt")

    def get_parameters(self):
        """Get all parameters from ROS2 parameter server"""
        self.agent_type = self.get_parameter("agent_type").get_parameter_value().string_value
        self.camera_names = self.get_parameter("camera_names").get_parameter_value().string_array_value
        self.hz = self.get_parameter("control_hz").get_parameter_value().integer_value
        self.gello_home_joints = self.get_parameter("gello_home_joints").get_parameter_value().double_array_value
        self.touch_home_pose = self.get_parameter("touch_home_pose").get_parameter_value().double_array_value
        self.touch_start_pose = self.get_parameter("touch_start_pose").get_parameter_value().double_array_value
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
        
        # Mock is hardcoded to False for now
        self.mock = False

    def _camera_color_callback(self, camera_name, msg: Image):
        """Callback for the color image."""
        try:
            self.camera_images[camera_name] = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image for {camera_name}: {e}")

    def start_camera_subscribers(self):
        """Start camera subscribers for all camera names"""
        for camera_name in self.camera_names:
            self.create_subscription(
                Image,
                f"/{camera_name}_camera/color/image_raw",
                partial(self._camera_color_callback, camera_name),
                1  # QoS depth
            )

    def _button_callback(self, msg: String):
        """Callback for button state"""
        self.button_state = msg.data

    def start_button_subscriber(self):
        """Start button state subscriber"""
        self.create_subscription(
            String,
            "/button_state",
            self._button_callback,
            1  # QoS depth
        )

    def initialize_robot(self):
        """Initialize robot based on controller type"""
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
                self.robot = CartesianImpedanceControlRobot(self.use_gripper)
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

    def initialize_agent(self):
        """Initialize agent based on agent type"""
        if self.agent_type == "gello":
            assert self.control_mode in "joint"
            self.env = RobotEnv(self.robot, control_rate_hz=self.hz, camera_dict=self.camera_clients, control_mode=self.control_mode)
            print("Using Gello agent")
            
            if self.gello_port is None:
                usb_ports = glob.glob("/dev/serial/by-id/*")
                print(f"Found {len(usb_ports)} ports")
                if len(usb_ports) > 0:
                    self.gello_port = usb_ports[0]
                    print(f"using port {self.gello_port}")
                else:
                    raise ValueError(
                        "No gello port found, please specify one or plug in gello"
                    )

            gello_reset_joints = np.array(self.gello_home_joints)
            self.agent = GelloAgent()
            time.sleep(1)

            # Start the gello agent
            obs = self.env.get_obs()
            robot_joints = obs["joint_positions"]
            print(f"Robot joints: {robot_joints}")

            gello_curr_joints = np.array(self.env.get_obs()["joint_positions"])

            if not self.skip_initial_move and gello_reset_joints.shape == gello_curr_joints.shape:
                max_delta = (np.abs(gello_curr_joints - gello_reset_joints)).max()
                steps = min(int(max_delta / 0.01), 100)
                for jnt in np.linspace(gello_curr_joints, gello_reset_joints, steps):
                    self.env.step(jnt)
                    time.sleep(0.001)

            # preprocess the agent start position
            agent_start_pos = self.agent.act(self.env.get_obs())
            print(f"Gello agent start pos: {agent_start_pos}")

            # check if the joints are close
            abs_deltas = np.abs(agent_start_pos - robot_joints)
            id_max_joint_delta = np.argmax(abs_deltas)
            print(f"Agent start pos: {agent_start_pos}", f"Robot joints: {robot_joints}")
            max_joint_delta = 0.8
            if abs_deltas[id_max_joint_delta] > max_joint_delta:
                print("Joint deltas are too big, please check the following joints")

                id_mask = abs_deltas > max_joint_delta
                ids = np.arange(len(id_mask))[id_mask]
                for i, delta, joint, current_j in zip(
                    ids,
                    abs_deltas[id_mask],
                    agent_start_pos[id_mask],
                    robot_joints[id_mask],
                ):
                    print(
                        f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}"
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
                print("Action is too big")
                print("action", action)
                print("joints", joints)

                # print which joints are too big
                joint_index = np.where(action - joints > 0.8)
                for j in joint_index:
                    print(
                        f"Joint [{j}], leader: {action[j]}, follower: {joints[j]}, diff: {action[j] - joints[j]}"
                    )
                return
            # Initialize camera images
            for camera_name in self.camera_names:
                obs[f"{camera_name}_rgb"] = self.camera_images.get(camera_name)
            
            self.obs = obs

        elif self.agent_type == "touch":
            assert self.control_mode in "cartesian"
            self.env = RobotEnv(self.robot, control_rate_hz=self.hz, camera_dict=self.camera_clients, control_mode=self.control_mode)
            print("Using 3D Systems Touch agent")
            # Initialize the touch agent
            self.agent = TouchAgent()
            # Move the robot towards the touch_home_pose until it's close enough
            if not self.skip_initial_move:
                action = {"ee_pos": self.touch_home_pose[:3], "ee_quat": self.touch_home_pose[3:]}
                self.env.step(action)
                time.sleep(5)
            # Initialize obs
            obs = self.env.get_obs()
            for camera_name in self.camera_names:
                obs[f"{camera_name}_rgb"] = self.camera_images.get(camera_name)
            self.obs = obs
            
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
            print("Loading checkpoint: ", eval_ckpt_file)
            loading_status = policy.load_state_dict(
                torch.load(eval_ckpt_file, map_location=torch.device(device))
            )
            print(loading_status)
            policy.to(device)
            policy.eval()
            print("ACT policy loaded")
            if self.camera_names is None:
                raise ValueError("Camera names not provided")
            self.agent = ACTAgent(
                policy, self.camera_names, train_cfg, policy_config, task_cfg=cfg, device=device
            )

            # Initialize obs
            obs = self.env.get_obs()
            for camera_name in self.camera_names:
                obs[f"{camera_name}_rgb"] = self.camera_images.get(camera_name)
            self.obs = obs
            
        elif self.agent_type == "policy":
            raise NotImplementedError("add your imitation policy here if there is one")
        else:
            raise ValueError("Invalid agent type: %s" % self.agent_type)

    def save_episode_thread(self, episode_number, obs_replay, action_replay):
        """Thread function for saving episodes"""
        save_episode(episode_number, obs_replay, action_replay)

    def run(self):
        """Main execution loop"""
        print_color("\nStart 🚀🚀🚀", color="green", attrs=("bold",))
        start_time = time.time()
        current_episode_number = 0
        current_save_thread = None
        message = ""
        
        try:
            while rclpy.ok():
                if self.use_save_interface:
                    if self.button_state == "start":
                        print("\nMoving to the start pose")
                        if self.agent_type == "gello":
                            pass
                        elif self.agent_type == "touch":
                            if not self.skip_initial_move:
                                action = {"ee_pos": self.touch_start_pose[:3], "ee_quat": self.touch_start_pose[3:]}
                                self.obs = self.env.step(action)
                                time.sleep(5)
                                self.obs = self.env.step(action)
                            action = self.agent.act(self.obs, force_pose_update=True)
                        elif self.agent_type == "act":
                            pass

                        obs_replay = []
                        action_replay = []
                        if current_save_thread is not None and current_save_thread.is_alive():
                            print("Can't start new episode, current episode is still saving")
                            self.button_state = "pass"
                            continue
                        if (current_episode_number + 1) > self.number_of_episodes:
                            print("All episodes done")
                            break
                        st_episode = time.time()
                        for i in range(self.number_of_steps):
                            step_st = time.time()
                            action = self.agent.act(self.obs)
                            self.obs = self.env.step(action)
                            for camera_name in self.camera_names:
                                self.obs[f"{camera_name}_rgb"] = self.camera_images.get(camera_name)
                            action_replay.append(action)
                            obs_replay.append(self.obs)
                            message = f"\rEpisode number: {current_episode_number} Time passed: {round(time.time() - st_episode, 2)},\tTime for step: {round((time.time() - step_st)*1000,1)} ms   "
                            print_color(
                                message,
                                color="white",
                                attrs=("bold",),
                                end="",
                                flush=True,
                            )

                        print("Episode done, saving now")
                        current_save_thread = threading.Thread(target=self.save_episode_thread, args=(current_episode_number, obs_replay, action_replay))
                        current_save_thread.start()

                        self.button_state = "pass"
                        current_episode_number += 1

                    elif self.button_state == "pass":
                        step_st = time.time()
                        action = self.agent.act(self.obs)
                        print("ee_euler", self.obs["ee_euler"])
                        self.obs = self.env.step(action)
                        message = f"\rWaiting for the next episode.\tTime for step: {round((time.time() - step_st)*1000,1)} ms   "
                        print_color(
                            message,
                            color="white",
                            attrs=("bold",),
                            end="",
                            flush=True,
                        )
                    elif self.button_state == "quit":
                        print("Quit episode recording")
                        break
                    else:
                        raise ValueError(f"Invalid state {self.button_state}")
                elif self.agent_type == "act":
                    # Initialize position and observation
                    action = {"ee_pos": self.touch_start_pose[:3], "ee_quat": self.touch_start_pose[3:]}
                    self.obs = self.env.step(action)
                    time.sleep(5)
                    self.obs = self.env.step(action)
                    for camera_name in self.camera_names:
                        self.obs[f"{camera_name}_rgb"] = self.camera_images.get(camera_name)
                    # Run the agent
                    for t in range(self.number_of_steps):
                        time_passed = time.time() - start_time
                        step_st = time.time()
                        action = self.agent.act(self.obs, t)
                        self.obs = self.env.step(action)
                        for camera_name in self.camera_names:
                            self.obs[f"{camera_name}_rgb"] = self.camera_images.get(camera_name)
                        message = f"\rTime passed: {round(time_passed, 2)} Step: {t} Time for step: {round((time.time() - step_st)*1000,1)} ms   "
                        print_color(
                            message,
                            color="white",
                            attrs=("bold",),
                            end="",
                            flush=True,
                        )
                else:
                    step_st = time.time()
                    
                    # TouchAgentの場合はspinを実行してコールバックを処理
                    if self.agent_type == "touch":
                        rclpy.spin_once(self.agent, timeout_sec=0.001)
                    
                    # agent.act() timing
                    act_start = time.time()
                    action = self.agent.act(self.obs)
                    act_time = (time.time() - act_start) * 1000
                    
                    # env.step() timing
                    step_start = time.time()
                    self.obs = self.env.step(action)
                    step_time = (time.time() - step_start) * 1000
                    
                    total_time = (time.time() - step_st) * 1000
                    print(f"Timing: act={act_time:.1f}ms, step={step_time:.1f}ms, total={total_time:.1f}ms")
                    message = f"\rTime passed: {round(time.time() - start_time, 2)},\tTime for step: {round((time.time() - step_st)*1000,1)} ms   "
                    print_color(
                        message,
                        color="white",
                        attrs=("bold",),
                        end="",
                        flush=True,
                    )
        except KeyboardInterrupt:
            print("ROS node interrupted")
        except Exception as e:
            print(f"Unexpected error: {e}")


def main():
    rclpy.init()
    
    try:
        agent_node = AgentNode()
        agent_node.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
