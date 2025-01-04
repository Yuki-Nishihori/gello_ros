#!/usr/bin/env python3
import os
import signal
import sys
import glob
import time
from typing import List

import numpy as np
from policy_config import (
    POLICY_CONFIG,
    TASK_CONFIG,
    TRAIN_CONFIG,
)  # must import first
from gello_ros.agents.agent import DummyAgent
from gello_ros.agents.gello_agent import GelloAgent
from gello_ros.agents.act_agent import ACTAgent
from gello_ros.data_utils.save_episode import save_episode
from gello_ros.env import RobotEnv
from gello_ros.robots.robot import PrintRobot
from gello_ros.policy.utils import *

import rospy
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import threading
import asyncio

agent = None

def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


def _color_callback_base(msg: Image):
    """Callback for the color image."""
    global base_image
    bridge = CvBridge()
    try:
        base_image = bridge.imgmsg_to_cv2(msg, "rgb8")
    except Exception as e:
        rospy.logerr(f"Failed to convert image: {e}")


def _color_callback_side(msg: Image):
    """Callback for the color image."""
    global side_image
    bridge = CvBridge()
    try:
        side_image = bridge.imgmsg_to_cv2(msg, "rgb8")
    except Exception as e:
        rospy.logerr(f"Failed to convert image: {e}")


def start_camera_subscriber():
    global base_camera_subscriber, side_camera_subscriber
    base_camera_subscriber = rospy.Subscriber(
        "/base_camera/color/image_raw",
        Image,
        _color_callback_base,
        queue_size=1,
    )
    side_camera_subscriber = rospy.Subscriber(
        "/side_camera/color/image_raw",
        Image,
        _color_callback_side,
        queue_size=1,
    )
def _button_callback(msg: String):
    global button_state
    button_state = msg.data


def start_button_subscriber():
    global button_subscriber
    button_subscriber = rospy.Subscriber(
        "/button_state",
        String,
        _button_callback,
        queue_size=1,
    )
    
def save_episode_thread(episode_number, obs_replay, action_replay):
    save_episode(episode_number, obs_replay, action_replay)

def main():
    rospy.init_node("agent_node", anonymous=True)

    agent_name: str = rospy.get_param("~agent_name", "gello")
    hostname: str = rospy.get_param("~default_hostname", "127.0.0.1")
    robot_port: int = rospy.get_param("~default_robot_port", 6001)
    camera_port: int = rospy.get_param("~default_camera_port", 7001)
    camera_names: List[str] = rospy.get_param("~camera_names", ["base"])

    robot_type: str = None  # only needed for quest agent or spacemouse agent
    hz: int = rospy.get_param("~control_hz", 100)
    start_joints: List[float] = rospy.get_param("~gello_start_joints")
    gello_mode: str = rospy.get_param("~gello_mode")
    controller_type: str = rospy.get_param("~controller_type")
    use_gripper: bool = rospy.get_param("~use_gripper")
    use_FT_sensor: bool = rospy.get_param("~use_FT_sensor")

    print(f"start_joints: {start_joints}")

    mock: bool = False
    use_save_interface: bool = rospy.get_param("~save_episode", False)
    verbose: bool = False
    gello_port: str = rospy.get_param("~gello_port", None)
    number_of_episodes: int = rospy.get_param("~number_of_episodes", 1)
    number_of_steps: int = rospy.get_param("~number_of_steps", 1000)
    task_name: str = rospy.get_param("~task_name", "cup_push")
    eval_ckpt_dir: str = rospy.get_param("~eval_ckpt_dir", "policy_last.ckpt")

    if use_save_interface:
        start_button_subscriber()
        global button_state
        button_state = "pass"


    if mock:
        robot_client = PrintRobot(8, dont_print=True)
        camera_clients = {}
    else:
        camera_clients = {}
        global base_image
        global side_image
        base_image = np.zeros((480, 640, 3), dtype=np.uint8)
        side_image = np.zeros((480, 640, 3), dtype=np.uint8)
        start_camera_subscriber()

        if controller_type == "joint_trajectory_controller":
            from gello_ros.robots.ros_joint_trajectory_control_robot import (
                JointTrajectoryControlRobot,
            )

            robot = JointTrajectoryControlRobot(use_gripper, use_FT_sensor)
        elif controller_type == "cartesian_compliance_controller":
            from gello_ros.robots.ros_cartesian_compliance_control_robot import (
                CartesianComplianceControlRobot,
            )

            robot = CartesianComplianceControlRobot(use_gripper)
        elif controller_type == "cartesian_motion_controller":
            from gello_ros.robots.ros_cartesian_motion_control_robot import (
                CartesianMotionControlRobot,
            )

            robot = CartesianMotionControlRobot(use_gripper)

        elif robot == "none" or robot == "print":
            robot = PrintRobot(8)

        else:
            raise NotImplementedError(
                f"Robot {robot} not implemented, choose one of: sim_ur, xarm, ur, bimanual_ur, none"
            )
    env = RobotEnv(robot, control_rate_hz=hz, camera_dict=camera_clients)

    if agent_name == "gello":
        print("Using Gello agent")
        gello_port = gello_port
        if gello_port is None:
            usb_ports = glob.glob("/dev/serial/by-id/*")
            print(f"Found {len(usb_ports)} ports")
            if len(usb_ports) > 0:
                gello_port = usb_ports[0]
                print(f"using port {gello_port}")
            else:
                raise ValueError(
                    "No gello port found, please specify one or plug in gello"
                )

        gello_reset_joints = np.array(start_joints)
        # agent = GelloAgent(
        #     port=gello_port, start_joints=gello_reset_joints, mode=gello_mode
        # )
        agent = GelloAgent()
        time.sleep(1)

        # Start the gello agent

        obs = env.get_obs()
        robot_joints = obs["joint_positions"]
        print(f"Robot joints: {robot_joints}")

        gello_curr_joints = np.array(env.get_obs()["joint_positions"])

        if gello_reset_joints.shape == gello_curr_joints.shape:
            max_delta = (np.abs(gello_curr_joints - gello_reset_joints)).max()
            steps = min(int(max_delta / 0.01), 100)
            for jnt in np.linspace(gello_curr_joints, gello_reset_joints, steps):
                env.step(jnt)
                time.sleep(0.001)

        # preprocess the agent start position
        agent_start_pos = agent.act(env.get_obs())
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
            obs = env.get_obs()
            command_joints = agent.act(obs)
            current_joints = obs["joint_positions"]
            delta = command_joints - current_joints
            max_joint_delta = np.abs(delta).max()
            if max_joint_delta > max_delta:
                delta = delta / max_joint_delta * max_delta
            env.step(current_joints + delta)
            # print("start up iteration", i, "/", startup_iterations)
        
        # check if the joints are close
        obs = env.get_obs()
        joints = obs["joint_positions"]
        action = agent.act(obs)
        if (action - joints > 0.5).any():
            print("Action is too big")
            print("action", action)
            print("jpints", joints)

            # print which joints are too big
            joint_index = np.where(action - joints > 0.8)
            for j in joint_index:
                print(
                    f"Joint [{j}], leader: {action[j]}, follower: {joints[j]}, diff: {action[j] - joints[j]}"
                )
            exit()

    elif agent_name == "quest":
        from gello_ros.agents.quest_agent import SingleArmQuestAgent

        agent = SingleArmQuestAgent(robot_type=robot_type, which_hand="l")
    elif agent_name == "spacemouse":
        from gello_ros.agents.spacemouse_agent import SpacemouseAgent

        agent = SpacemouseAgent(robot_type=robot_type, verbose=verbose)
    elif agent_name == "dummy" or agent_name == "none":
        agent = DummyAgent(num_dofs=robot_client.num_dofs())
    elif agent_name == "act":
        # load config
        cfg = TASK_CONFIG
        policy_config = POLICY_CONFIG
        train_cfg = TRAIN_CONFIG
        device = os.environ["DEVICE"]
        # load the policy
        policy = make_policy(policy_config["policy_class"], policy_config)
        eval_ckpt_file = os.path.join(eval_ckpt_dir, "policy_last.ckpt")
        print("Loading checkpoint: ", eval_ckpt_file)
        loading_status = policy.load_state_dict(
            torch.load(eval_ckpt_file, map_location=torch.device(device))
        )
        print(loading_status)
        policy.to(device)
        policy.eval()
        print("ACT policy loaded")
        agent = ACTAgent(
            policy, camera_names, train_cfg, policy_config, task_cfg=cfg, device=device
        )
        obs = env.get_obs()
        obs["base_rgb"] = base_image
        obs["side_rgb"] = side_image
    elif agent_name == "policy":
        raise NotImplementedError("add your imitation policy here if there is one")
    else:
        raise ValueError("Invalid agent name")

    print_color("\nStart 🚀🚀🚀", color="green", attrs=("bold",))
    start_time = time.time()
    current_episode_number = 0
    current_save_thread = None
    message=""
    try:
        while not rospy.is_shutdown():
            if use_save_interface:
                if button_state == "start":
                    obs_replay = []
                    action_replay = []
                    if current_save_thread is not None and current_save_thread.is_alive():
                        print("Can't start new episode, current episode is still saving")
                        button_state = "pass"
                        continue
                    if (current_episode_number + 1) > number_of_episodes:
                        print("All episodes done")
                        break
                    st_episode = time.time()
                    for i in range(number_of_steps):
                        step_st = time.time()
                        action = agent.act(obs)
                        obs = env.step(action)
                        obs["base_rgb"] = base_image
                        obs["side_rgb"] = side_image
                        action_replay.append(action)
                        obs_replay.append(obs)
                        message = f"\rEpisode number: {current_episode_number} Time passed: {round(time.time() - st_episode, 2)},\tTime for step: {round((time.time() - step_st)*1000,1)} ms   "
                        print_color(
                        message,
                        color="white",
                        attrs=("bold",),
                        end="",
                        flush=True,
                        )

                    print("Episode done, saving now")
                    current_save_thread = threading.Thread(target=save_episode_thread, args=(current_episode_number, obs_replay, action_replay))
                    current_save_thread.start()
                    
                    button_state="pass"
                    current_episode_number += 1
                  
                elif button_state == "pass":
                    step_st = time.time()
                    action = agent.act(obs)
                    obs = env.step(action)
                    message = f"\rWaiting for the next episode.\tTime for step: {round((time.time() - step_st)*1000,1)} ms   "
                    print_color(
                            message,
                            color="white",
                            attrs=("bold",),
                            end="",
                            flush=True,
                    )
                elif button_state == "quit":
                    print("Quit episode recording")
                    break
                else:
                    raise ValueError(f"Invalid state {button_state}")
            elif agent_name == "act":
                for t in range(number_of_steps):
                    time_passed = time.time() - start_time
                    step_st = time.time()
                    action = agent.act(obs, t)
                    obs = env.step(action)
                    obs["base_rgb"] = base_image
                    obs["side_rgb"] = side_image
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
                action = agent.act(obs)
                obs = env.step(action)
                message = f"\rTime passed: {round(time.time() - start_time, 2)},\tTime for step: {round((time.time() - step_st)*1000,1)} ms   "
                print_color(
                            message,
                            color="white",
                            attrs=("bold",),
                            end="",
                            flush=True,
                        )
    except rospy.ROSInterruptException:
        print("ROS node interrupted")
    except Exception as e:
        print(f"Unexpected error: {e}")
    

if __name__ == "__main__":
    main()
