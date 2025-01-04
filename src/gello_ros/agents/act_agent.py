from typing import Any, Dict
import numpy as np
import os
import pickle
import rospy
from einops import rearrange
import torch
from gello_ros.agents.agent import Agent


class ACTAgent(Agent):
    def __init__(
        self,
        policy,
        camera_names: list,
        train_cfg: dict,
        policy_config: dict,
        task_cfg: dict,
        device: str,
    ):
        """
        Initializes the ACTAgent with policy, camera names, and configurations.

        Args:
            policy: The policy model to be used for action prediction.
            camera_names (list): List of camera names.
            train_cfg (dict): Configuration for training.
            policy_config (dict): Configuration for the policy.
            task_cfg (dict): Task-specific configurations.
            device (str): Device to run the model on (e.g., 'cpu' or 'cuda').
        """
        self.policy = policy
        self.camera_names = camera_names
        self.device = device

        # Load dataset statistics
        stats_path = os.path.join(
            rospy.get_param("~eval_ckpt_dir"),
            "dataset_stats.pkl",
        )

        # Error handling for loading statistics
        try:
            with open(stats_path, "rb") as f:
                self.stats = pickle.load(f)
        except FileNotFoundError:
            raise ValueError(f"Statistics file not found at: {stats_path}")

        # Preprocess and postprocess lambdas
        self.pre_process = (
            lambda s_qpos: (s_qpos - self.stats["qpos_mean"]) / self.stats["qpos_std"]
        )
        self.post_process = (
            lambda a: a * self.stats["action_std"] + self.stats["action_mean"]
        )

        # Query frequency configuration
        self.query_frequency = policy_config["num_queries"]
        num_steps=rospy.get_param("~number_of_steps")
        if policy_config["temporal_agg"]:
            self.query_frequency = 1
            self.all_time_actions = torch.zeros(
                [
                    num_steps,
                    num_steps,
                    +policy_config["num_queries"],
                    task_cfg["state_dim"],
                ],
                device=device,
            )

    def _image_converter(self, obs: Dict[str, Any]) -> torch.Tensor:
        """
        Convert images from observations to a tensor format.

        Args:
            obs (dict): Dictionary containing observed data.

        Returns:
            torch.Tensor: A tensor of shape (1, num_images, channels, height, width).
        """
        curr_images = []

        # Iterate through the camera names
        for cam_name in self.camera_names:
            # Retrieve and rearrange the image for the current camera
            curr_image = rearrange(obs[f"{cam_name}_rgb"], "h w c -> c h w")
            curr_images.append(curr_image)

        # Stack images into a single numpy array
        curr_image = np.stack(curr_images, axis=0)

        # Convert to a PyTorch tensor, normalize and move to the specified device
        curr_image = (
            torch.from_numpy(curr_image / 255.0).float().to(self.device).unsqueeze(0)
        )

        return curr_image

    def act(self, obs: Dict[str, Any], t: int) -> np.ndarray:
        """
        Generate actions based on observations and current time step.

        Args:
            obs (dict): Dictionary containing observed data.
            t (int): Current time step.

        Returns:
            np.ndarray: The generated action.
        """
        with torch.inference_mode():
            # Convert observations to image tensor
            img = self._image_converter(obs)

            # Preprocess joint positions
            processed_joint_positions = self.pre_process(obs["joint_positions"])

            # Process joint positions
            processed_joint_positions = (
                torch.from_numpy(processed_joint_positions)
                .float()
                .to(self.device)
                .unsqueeze(0)
            )

            # Call the policy to get actions at specific time steps
            if t % self.query_frequency == 0:
                self.all_actions = self.policy(processed_joint_positions, img)

            if self.query_frequency == 1:  # Temporal aggregation
                self.all_time_actions[[t], t : t + self.query_frequency] = (
                    self.all_actions
                )
                actions_for_curr_step = self.all_time_actions[:, t]
                actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                actions_for_curr_step = actions_for_curr_step[actions_populated]

                # Exponential weights for actions
                k = 0.01
                exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                exp_weights = exp_weights / exp_weights.sum()
                exp_weights = (
                    torch.from_numpy(exp_weights.astype(np.float32))
                    .to(self.device)
                    .unsqueeze(dim=1)
                )

                raw_action = (actions_for_curr_step * exp_weights).sum(
                    dim=0, keepdim=True
                )
            else:
                raw_action = self.all_actions[:, t % self.query_frequency]

            # Post-process actions
            raw_action = raw_action.squeeze(0).cpu().numpy()
            action = self.post_process(raw_action)

            return action
