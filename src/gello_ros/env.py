import time
from typing import Any, Dict, List
from functools import partial

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from gello_ros.robots.robot import Robot



class Rate:
    def __init__(self, rate: float):
        self.last = time.time()
        self.rate = rate

    def sleep(self) -> None:
        while self.last + (1.0 / self.rate) > time.time():
            time.sleep(0.00001)
        self.last = time.time()


class RobotEnv(Node):
    def __init__(
        self,
        robot: Robot,
        control_rate_hz: float = 100.0,
        control_mode: str = "cartesian",
        node_name: str = "robot_env",
    ) -> None:
        super().__init__(node_name)
        assert control_mode in ["joint", "cartesian"]
        self._robot = robot
        self._rate = Rate(control_rate_hz)
        self._control_mode = control_mode
        

    def robot(self) -> Robot:
        """Get the robot object.

        Returns:
            robot: the robot object.
        """
        return self._robot

    def __len__(self):
        return 0

    def step(self, action: Dict[str, np.ndarray]) -> Dict[str, Any]:
        
        if self._control_mode == "cartesian":
            # Ensure keys exist before accessing
            if "ee_pos" in action and "ee_quat" in action:
                pos_quat = np.concatenate([action["ee_pos"], action["ee_quat"]])
                self._robot.command_pose(pos_quat)
        elif self._control_mode == "joint":
            if "joint_positions" in action:
                self._robot.command_joint_state(action["joint_positions"])
        else:
            raise ValueError(f"Invalid control mode: {self._control_mode}")
        
        self._rate.sleep()
        return self.get_obs()
    
    def get_obs(self) -> Dict[str, Any]:
        """Get observation from the environment."""
        # Only spin robot node to update robot states
        # Camera updates are handled by the main spin loop in component mode
        rclpy.spin_once(self._robot, timeout_sec=0.001)

        observations = {}
        
        robot_obs = self._robot.get_observations()
        # It's safer to merge dictionaries
        observations.update(robot_obs)
        
        # Ensure essential keys are present
        assert "joint_positions" in observations
        assert "joint_velocities" in observations
        assert "ee_pos" in observations
        
        return observations


def main() -> None:
    pass


if __name__ == "__main__":
    main()