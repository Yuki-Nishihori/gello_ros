import os
import datetime
import h5py
import numpy as np
import rclpy
from rclpy.node import Node
import tf_transformations


class EpisodeSaver(Node):
    def __init__(self):
        super().__init__('episode_saver')

        self.declare_parameter("camera_names", ["base"])
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("wrench_dim", 6)
        self.declare_parameter("save_episode_dir", "./episode_data")
        self.declare_parameter("task_name", "default")
        self.declare_parameter("use_FT_sensor", False)

        self.timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    def save_episode(self, episode_number: int, obs_replay: list, action_replay: list):
        cfg = {
            "camera_names": self.get_parameter("camera_names").value,
            "cam_width": self.get_parameter("camera_width").value,
            "cam_height": self.get_parameter("camera_height").value,
            "wrench_dim": self.get_parameter("wrench_dim").value,
            "save_episode_dir": self.get_parameter("save_episode_dir").value,
            "task_name": self.get_parameter("task_name").value,
            "use_FT_sensor": self.get_parameter("use_FT_sensor").value,
        }

        self.get_logger().info(f"Saving episode {episode_number} to {cfg['save_episode_dir']}")

        data_dict = {
            "/action/joint_positions": [],
            "/action/ee_pos": [],
            "/action/ee_quat": [],
            "/action/ee_rot_matrix": [],
            "/action/ee_euler": [],
            "/observations/joint_positions": [],
            "/observations/joint_velocities": [],
            "/observations/joint_torques": [],
            "/observations/ee_pos": [],
            "/observations/ee_quat": [],
            "/observations/ee_rot_matrix": [],
            "/observations/ee_euler": [],
        }

        for cam_name in cfg["camera_names"]:
            img_name = f"{cam_name}_rgb"
            data_dict[f"/observations/images/{img_name}"] = []

        if cfg["use_FT_sensor"]:
            data_dict["/observations/wrench"] = []

        for o, a in zip(obs_replay, action_replay):
            data_dict["/action/joint_positions"].append(a["joint_positions"])
            data_dict["/action/ee_pos"].append(a["ee_pos"])
            data_dict["/action/ee_quat"].append(a["ee_quat"])
            data_dict["/action/ee_rot_matrix"].append(a["ee_rot_matrix"])
            data_dict["/action/ee_euler"].append(a["ee_euler"])
            data_dict["/observations/joint_positions"].append(o["joint_positions"])
            data_dict["/observations/joint_velocities"].append(o["joint_velocities"])
            data_dict["/observations/joint_torques"].append(o["joint_torques"])
            data_dict["/observations/ee_pos"].append(o["ee_pos"])
            data_dict["/observations/ee_quat"].append(o["ee_quat"])
            data_dict["/observations/ee_rot_matrix"].append(o["ee_rot_matrix"])
            data_dict["/observations/ee_euler"].append(o["ee_euler"])
            if cfg["use_FT_sensor"]:
                data_dict["/observations/wrench"].append(o["ee_wrench"])
            for cam_name in cfg["camera_names"]:
                img_name = f"{cam_name}_rgb"
                data_dict[f"/observations/images/{img_name}"].append(o[img_name])

        max_timesteps = len(data_dict["/observations/joint_positions"])

        data_dir = os.path.join(cfg["save_episode_dir"], self.timestamp + "_" + cfg["task_name"])
        os.makedirs(data_dir, exist_ok=True)
        dataset_path = os.path.join(data_dir, f"episode_{episode_number}.hdf5")

        with h5py.File(dataset_path, "w", rdcc_nbytes=1024**2 * 2) as root:
            root.attrs["sim"] = True
            obs = root.create_group("observations")
            image = obs.create_group("images")

            for cam_name in cfg["camera_names"]:
                img_name = f"{cam_name}_rgb"
                image.create_dataset(
                    img_name,
                    (max_timesteps, cfg["cam_height"], cfg["cam_width"], 3),
                    dtype="uint8",
                    chunks=(1, cfg["cam_height"], cfg["cam_width"], 3),
                )

            obs.create_dataset("joint_positions", (max_timesteps, 6))
            obs.create_dataset("joint_velocities", (max_timesteps, 6))
            obs.create_dataset("joint_torques", (max_timesteps, 6))
            obs.create_dataset("ee_pos", (max_timesteps, 3))
            obs.create_dataset("ee_quat", (max_timesteps, 4))
            obs.create_dataset("ee_rot_matrix", (max_timesteps, 3, 3))
            obs.create_dataset("ee_euler", (max_timesteps, 3))
            if cfg["use_FT_sensor"]:
                obs.create_dataset("wrench", (max_timesteps, cfg["wrench_dim"]))

            action = root.create_group("action")
            action.create_dataset("joint_positions", (max_timesteps, 6))
            action.create_dataset("ee_pos", (max_timesteps, 3))
            action.create_dataset("ee_quat", (max_timesteps, 4))
            action.create_dataset("ee_rot_matrix", (max_timesteps, 3, 3))
            action.create_dataset("ee_euler", (max_timesteps, 3))

            for name, array in data_dict.items():
                root[name][:] = array

        self.get_logger().info(f"Data saved successfully at {dataset_path}")


def main():
    rclpy.init()
    node = EpisodeSaver()

    # テスト用: 擬似データで save_episode 呼び出し（実際には他のノードから呼び出される想定）
    obs = [{
        "joint_positions": np.zeros(6),
        "joint_velocities": np.zeros(6),
        "joint_torques": np.zeros(6),
        "ee_pos": np.zeros(3),
        "ee_quat": np.array([0, 0, 0, 1]),
        "ee_rot_matrix": np.eye(3),
        "ee_euler": np.zeros(3),
        "base_rgb": np.zeros((480, 640, 3), dtype=np.uint8)
    }] * 10
    act = [{
        "joint_positions": np.zeros(6),
        "ee_pos": np.zeros(3),
        "ee_quat": np.array([0, 0, 0, 1]),
        "ee_rot_matrix": np.eye(3),
        "ee_euler": np.zeros(3),
    }] * 10

    node.save_episode(1, obs, act)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
