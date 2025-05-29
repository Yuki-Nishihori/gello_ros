import numpy as np
import os
import pickle
import torch
from typing import Any, Dict
from einops import rearrange
from gello_ros.agents.agent import Agent
import tf_transformations  # ROS 2用
from rclpy.node import Node  # ROS 2のNodeクラス

class ACTAgent(Agent):
    def __init__(
        self,
        policy,
        camera_names: list,
        train_cfg: dict,
        policy_config: dict,
        task_cfg: dict,
        device: str,
        node: Node  # ROS 2 Node を渡す
    ):
        self.policy = policy
        self.camera_names = camera_names
        self.device = device
        self.node = node

        # パラメータ取得（ROS 2方式）
        stats_path = os.path.join(
            node.get_parameter("eval_ckpt_dir").get_parameter_value().string_value,
            "dataset_stats.pkl",
        )

        try:
            with open(stats_path, "rb") as f:
                self.stats = pickle.load(f)
        except FileNotFoundError:
            raise ValueError(f"Statistics file not found at: {stats_path}")

        self.pre_process = (
            lambda s_qpos: (s_qpos - self.stats["obs_mean"]) / self.stats["obs_std"]
        )
        self.post_process = (
            lambda a: a * self.stats["action_std"] + self.stats["action_mean"]
        )

        self.query_frequency = policy_config["num_queries"]
        num_steps = node.get_parameter("number_of_steps").get_parameter_value().integer_value

        if policy_config["temporal_agg"]:
            self.query_frequency = 1
            self.all_time_actions = torch.zeros(
                [num_steps, num_steps, policy_config["num_queries"], task_cfg["state_dim"]],
                device=device,
            )

    def _image_converter(self, obs: Dict[str, Any]) -> torch.Tensor:
        curr_images = []
        for cam_name in self.camera_names:
            curr_image = rearrange(obs[f"{cam_name}_rgb"], "h w c -> c h w")
            curr_images.append(curr_image)

        curr_image = np.stack(curr_images, axis=0)
        curr_image = torch.from_numpy(curr_image / 255.0).float().to(self.device).unsqueeze(0)
        return curr_image

    def act(self, obs: Dict[str, Any], t: int) -> Dict[str, np.ndarray]:
        action_dict = {}
        obs_pos_rot_matrix = np.append(obs["ee_pos"], obs["ee_rot_matrix"].flatten())

        with torch.inference_mode():
            img = self._image_converter(obs)
            processed_pos_rot_matrix = self.pre_process(obs_pos_rot_matrix)
            processed_pos_rot_matrix = torch.from_numpy(processed_pos_rot_matrix).float().to(self.device).unsqueeze(0)

            if t % self.query_frequency == 0:
                self.all_actions = self.policy(processed_pos_rot_matrix, img)

            if self.query_frequency == 1:
                self.all_time_actions[[t], t : t + self.query_frequency] = self.all_actions
                actions_for_curr_step = self.all_time_actions[:, t]
                actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                actions_for_curr_step = actions_for_curr_step[actions_populated]

                k = 0.01
                exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                exp_weights = exp_weights / exp_weights.sum()
                exp_weights = (
                    torch.from_numpy(exp_weights.astype(np.float32))
                    .to(self.device)
                    .unsqueeze(dim=1)
                )
                raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
            else:
                raw_action = self.all_actions[:, t % self.query_frequency]

            raw_action = raw_action.squeeze(0).cpu().numpy()
            action = self.post_process(raw_action)

            pos = action[:3]
            rot_matrix = action[3:].reshape((3, 3))
            rot_hom = np.eye(4)
            rot_hom[:3, :3] = rot_matrix
            quat = tf_transformations.quaternion_from_matrix(rot_hom)
            euler = tf_transformations.euler_from_matrix(rot_hom)

            action_dict["joint_positions"] = np.zeros(6)
            action_dict["ee_pos"] = pos
            action_dict["ee_rot_matrix"] = rot_matrix
            action_dict["ee_quat"] = quat
            action_dict["ee_euler"] = euler

            return action_dict
