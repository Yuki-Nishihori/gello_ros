import os
import datetime
import h5py
import rospy

timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def save_episode(episode_number, obs_replay, action_replay):
    """
    Save observations and actions to an HDF5 file.

    Parameters:
        episode_number (int): the episode number.
        obs_replay (list): list of observations.
        action_replay (list): list of actions.
    """
    # configration dictionary
    cfg = {
        "camera_names": rospy.get_param("~camera_names", ["base"]),
        "cam_width": rospy.get_param("~camera_width", 640),
        "cam_height": rospy.get_param("~camera_height", 480),
        "state_dim": rospy.get_param("~state_dim", 6),
        "action_dim": rospy.get_param("~action_dim", 6),
        "save_episode_dir": rospy.get_param("~save_episode_dir", "./episode_data"),
        "task_name": rospy.get_param("~task_name", "default"),
        "use_FT_sensor": rospy.get_param("~use_FT_sensor", False),
    }

    # create a dictionary to store the data
    data_dict = {
        "/observations/qpos": [],
        "/observations/qvel": [],
        "/action": [],
    }
    # there may be more than one camera
    for cam_name in cfg["camera_names"]:
        img_name = f"{cam_name}_rgb"
        data_dict[f"/observations/images/{img_name}"] = []
    # add FT sensor data
    if cfg["use_FT_sensor"]:
        data_dict["/observations/wrench"] = []

    # store the observations and actions
    for o, a in zip(obs_replay, action_replay):
        data_dict["/observations/qpos"].append(o["joint_positions"])
        data_dict["/observations/qvel"].append(o["joint_velocities"])
        data_dict["/action"].append(a)
        if cfg["use_FT_sensor"]:
            data_dict["/observations/wrench"].append(o["ee_wrench"])
        # store the images
        for cam_name in cfg["camera_names"]:
            img_name = f"{cam_name}_rgb"
            data_dict[f"/observations/images/{img_name}"].append(o[img_name])

    max_timesteps = len(data_dict["/observations/qpos"])

    # create data dir if it doesn't exist
    data_dir = os.path.join(cfg["save_episode_dir"], timestamp + "_" + cfg["task_name"])
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    # count number of files in the directory to avoid overwriting
    dataset_path = os.path.join(data_dir, f"episode_{episode_number}")

    # save the data
    with h5py.File(dataset_path + ".hdf5", "w", rdcc_nbytes=1024**2 * 2) as root:
        root.attrs["sim"] = True
        obs = root.create_group("observations")
        image = obs.create_group("images")

        # create datasets for each camera's images
        for cam_name in cfg["camera_names"]:
            img_name = f"{cam_name}_rgb"
            _ = image.create_dataset(
                img_name,
                (max_timesteps, cfg["cam_height"], cfg["cam_width"], 3),
                dtype="uint8",
                chunks=(1, cfg["cam_height"], cfg["cam_width"], 3),
            )

        # create datasets for qpos, qvel, and actions
        qpos = obs.create_dataset("qpos", (max_timesteps, cfg["state_dim"]))
        qvel = obs.create_dataset("qvel", (max_timesteps, cfg["state_dim"]))
        action = root.create_dataset("action", (max_timesteps, cfg["action_dim"]))
        if cfg["use_FT_sensor"]:
            wrench = obs.create_dataset("wrench", (max_timesteps, cfg["state_dim"]))

        # store the data in the corresponding dataset
        for name, array in data_dict.items():
            root[name][...] = array

    print(f"Data saved successfully in {dataset_path}.hdf5")
