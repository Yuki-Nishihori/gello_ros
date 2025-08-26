import os
import datetime
import h5py
import rospy
import numpy as np
import tf.transformations

# Get the current timestamp
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

def save_episode(episode_number, obs_replay, action_replay):
    """
    Save observations and actions to an HDF5 file.

    Parameters:
        episode_number (int): the episode number.
        obs_replay (list): list of observations.
        action_replay (list): list of actions.
    """
    # Configuration dictionary
    cfg = {
        "camera_names": rospy.get_param("~camera_names", ["base"]),
        "cam_width": rospy.get_param("~camera_width", 640),
        "cam_height": rospy.get_param("~camera_height", 480),
        "wrench_dim": rospy.get_param("~wrench_dim", 6),
        "save_episode_dir": rospy.get_param("~save_episode_dir", "./episode_data"),
        "task_name": rospy.get_param("~task_name", "default"),
        "use_FT_sensor": rospy.get_param("~use_FT_sensor", False),
    }

    print(f"Saving episode {episode_number} to {cfg['save_episode_dir']}")

    # Create a dictionary to store the data
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

    # Add camera images to the data dictionary
    for cam_name in cfg["camera_names"]:
        img_name = f"{cam_name}_rgb"
        data_dict[f"/observations/images/{img_name}"] = []

    # Add FT sensor data if used
    if cfg["use_FT_sensor"]:
        data_dict["/observations/wrench"] = []

    # Store the observations and actions
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
        # Store the images
        for cam_name in cfg["camera_names"]:
            img_name = f"{cam_name}_rgb"
            data_dict[f"/observations/images/{img_name}"].append(o[img_name])

    max_timesteps = len(data_dict["/observations/joint_positions"])

    # Create data directory if it doesn't exist
    data_dir = os.path.join(cfg["save_episode_dir"], timestamp + "_" + cfg["task_name"])
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    # Define the dataset path
    dataset_path = os.path.join(data_dir, f"episode_{episode_number}")

    # Save the data to an HDF5 file
    with h5py.File(dataset_path + ".hdf5", "w", rdcc_nbytes=1024**2 * 2) as root:
        root.attrs["sim"] = True
        obs = root.create_group("observations")
        image = obs.create_group("images")

        # Create datasets for each camera's images
        for cam_name in cfg["camera_names"]:
            img_name = f"{cam_name}_rgb"
            _ = image.create_dataset(
                img_name,
                (max_timesteps, cfg["cam_height"], cfg["cam_width"], 3),
                dtype="uint8",
                chunks=(1, cfg["cam_height"], cfg["cam_width"], 3),
            )

        # Create HDF5 datasets for the new observations
        _ = obs.create_dataset("joint_positions", (max_timesteps, 6))  # Assuming 6 joints
        _ = obs.create_dataset("joint_velocities", (max_timesteps, 6))  # Assuming 6 joints
        _ = obs.create_dataset("joint_torques", (max_timesteps, 6))  # Assuming 6 joints
        _ = obs.create_dataset("ee_pos", (max_timesteps, 3))  # Assuming 3D position
        _ = obs.create_dataset("ee_quat", (max_timesteps, 4))  # Assuming 4-element quaternion
        _ = obs.create_dataset("ee_rot_matrix", (max_timesteps, 3, 3))  # Assuming 3x3 rotation matrix
        _ = obs.create_dataset("ee_euler", (max_timesteps, 3))  # Assuming 3-element Euler angles

        if cfg["use_FT_sensor"]:
            _ = obs.create_dataset("wrench", (max_timesteps, cfg["wrench_dim"]))
        # Create datasets for actions
        action = root.create_group("action")
        _ = action.create_dataset("joint_positions", (max_timesteps, 6))  # Assuming 6 joints
        _ = action.create_dataset("ee_pos", (max_timesteps, 3))  # Assuming 3D position
        _ = action.create_dataset("ee_quat", (max_timesteps, 4))  # Assuming 4-element quaternion
        _ = action.create_dataset("ee_rot_matrix", (max_timesteps, 3, 3))  # Assuming 3x3 rotation matrix
        _ = action.create_dataset("ee_euler", (max_timesteps, 3))  # Assuming 3-element Euler angles

        # Store the data in the corresponding dataset
        for name, array in data_dict.items():
            root[name][:] = array

    print(f"Data saved successfully in {dataset_path}.hdf5")
