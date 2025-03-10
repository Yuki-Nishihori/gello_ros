import time
from typing import Any, Dict, Optional

import numpy as np

from gello_ros.cameras.camera import CameraDriver
from gello_ros.robots.robot import Robot


class Rate:
    def __init__(self, rate: float):
        self.last = time.time()
        self.rate = rate

    def sleep(self) -> None:
        while self.last + (1.0 / self.rate) > time.time():
            time.sleep(0.00001)
        self.last = time.time()


class RobotEnv:
    def __init__(
        self,
        robot: Robot,
        control_rate_hz: float = 100.0,
        camera_dict: Optional[Dict[str, CameraDriver]] = None,
        control_mode: str = "cartesian",
    ) -> None:
        assert control_mode in ["joint", "cartesian"]
        self._robot = robot
        self._rate = Rate(control_rate_hz)
        self._camera_dict = {} if camera_dict is None else camera_dict
        self._control_mode = control_mode

    def robot(self) -> Robot:
        """Get the robot object.

        Returns:
            robot: the robot object.
        """
        return self._robot

    def __len__(self):
        return 0

    def step(self, action: np.ndarray) -> Dict[str, Any]:
        
        if self._control_mode == "cartesian":
            assert action["ee_quat"] is not np.zeros(4)
            assert action["ee_pos"] is not np.zeros(3)
            pos_quat = np.concatenate([action["ee_pos"], action["ee_quat"]])
            self._robot.command_pose(pos_quat)
        elif self._control_mode == "joint":
            assert action["joint_positions"] is not np.zeros(6)
            self._robot.command_joint_state(action["joint_positions"])
        else:
            raise ValueError("Invalid control mode")
        
        self._rate.sleep()
        return self.get_obs()
    
    def get_obs(self) -> Dict[str, Any]:
        """Get observation from the environment.

        Returns:
            obs: observation from the environment.
        """
        observations = {}
        # for name, camera in self._camera_dict.items():
        #     image = camera.read()
        #     observations[f"{name}_rgb"] = image

        robot_obs = self._robot.get_observations()
        assert "joint_positions" in robot_obs
        assert "joint_velocities" in robot_obs
        assert "ee_pos" in robot_obs
        observations["joint_positions"] = robot_obs["joint_positions"]
        observations["joint_velocities"] = robot_obs["joint_velocities"]
        observations["joint_torques"] = robot_obs["joint_torques"]
        observations["ee_pos"] = robot_obs["ee_pos"]
        observations["ee_quat"] = robot_obs["ee_quat"]
        observations["ee_rot_matrix"] = robot_obs["ee_rot_matrix"]
        observations["ee_euler"] = robot_obs["ee_euler"]
        observations["gripper_position"] = robot_obs["gripper_position"]
        observations["ee_wrench"] = robot_obs["ee_wrench"]
        observations["jacobian"] = robot_obs["jacobian"]
        

        return observations


def main() -> None:
    pass


if __name__ == "__main__":
    main()
