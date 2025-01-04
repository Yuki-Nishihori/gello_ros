import os

# fallback to cpu if mps is not available for specific operations
os.environ["PYTORCH_ENABLE_MPS_FALLBACK"] = "1"
import torch

# data directory
DATA_DIR = "data/"

# device
device = "cpu"
if torch.cuda.is_available():
    device = "cuda"
# if torch.backends.mps.is_available(): device = 'mps'
os.environ["DEVICE"] = device


# task config (you can add new tasks)
TASK_CONFIG = {
    "dataset_dir": "/root/onolab/catkin_ws/src/gello_software/gello_ros/scripts/episode_data/20241202_184842_cup_push",
    "state_dim": 6,
    "action_dim": 6,
    "camera_names": ["base_rgb", "side_rgb"],
    "camera_port": 0,
}


# policy config
POLICY_CONFIG = {
    "lr": 1e-5,
    "device": device,
    "num_queries": 100,  # get the last 100 actions from the policy
    "kl_weight": 10,
    "hidden_dim": 512,
    "dim_feedforward": 3200,
    "lr_backbone": 1e-5,
    "backbone": "resnet18",
    "enc_layers": 4,
    "dec_layers": 7,
    "nheads": 8,
    "camera_names": ["base_rgb", "side_rgb"],
    "policy_class": "ACT",
    "temporal_agg": False,
}

# training config
TRAIN_CONFIG = {
    "seed": 42,
    "num_epochs": 2000,  # number of training epochs
    "batch_size_val": 8,
    "batch_size_train": 8,
    "checkpoint_dir": "/root/onolab/catkin_ws/src/gello_software/gello_ros/scripts/checkpoints",
}
