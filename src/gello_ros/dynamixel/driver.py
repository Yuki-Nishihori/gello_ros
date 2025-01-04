import time
from threading import Event, Lock, Thread
from concurrent.futures import ThreadPoolExecutor
from typing import Protocol, Sequence

import numpy as np
from dynamixel_sdk.group_sync_read import GroupSyncRead
from dynamixel_sdk.group_sync_write import GroupSyncWrite
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.robotis_def import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
)

# Constants
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
ADDR_GOAL_CURRENT = 102
LEN_GOAL_CURRENT = 2
ADDR_PRESENT_CURRENT = 126
LEN_PRESENT_CURRENT = 2

CONTROL_MODES = {
    "CURRENT_MODE": 0,
    "VELOCITY_MODE": 1,
    "PWM_MODE": 2,
    "POSITION_MODE": 3,
    "CURRENT_BASED_POSITION_MODE": 5,
}


class DynamixelDriverProtocol(Protocol):
    def set_joints(self, joint_angles: Sequence[float]):
        """Set the joint angles for the Dynamixel servos.

        Args:
            joint_angles (Sequence[float]): A list of joint angles.
        """
        ...

    def torque_enabled(self) -> bool:
        """Check if torque is enabled for the Dynamixel servos.

        Returns:
            bool: True if torque is enabled, False if it is disabled.
        """
        ...

    def set_torque_mode(self, enable: bool):
        """Set the torque mode for the Dynamixel servos.

        Args:
            enable (bool): True to enable torque, False to disable.
        """
        ...

    def get_joints(self) -> np.ndarray:
        """Get the current joint angles in radians.

        Returns:
            np.ndarray: An array of joint angles.
        """
        ...

    def close(self):
        """Close the driver."""


class FakeDynamixelDriver(DynamixelDriverProtocol):
    def __init__(self, ids: Sequence[int]):
        self._ids = ids
        self._joint_angles = np.zeros(len(ids), dtype=int)
        self._joint_current_goals = np.zeros(len(ids), dtype=int)
        self._torque_enabled = False

    def set_joints(self, joint_angles: Sequence[float]):
        if len(joint_angles) != len(self._ids):
            raise ValueError(
                "The length of joint_angles must match the number of servos"
            )
        if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled to set joint angles")
        self._joint_angles = np.array(joint_angles)

    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def set_torque_mode(self, enable: bool):
        self._torque_enabled = enable

    def get_joints(self) -> np.ndarray:
        return self._joint_angles.copy()

    def close(self):
        pass


class DynamixelDriver(DynamixelDriverProtocol):
    def __init__(
        self,
        ids: Sequence[int],
        port: str = "/dev/ttyUSB0",
        baudrate: int = 2000000,
        read_only: bool = False,
    ):
        """Initialize the DynamixelDriver class.

        Args:
            ids (Sequence[int]): A list of IDs for the Dynamixel servos.
            port (str): The USB port to connect to the arm.
            baudrate (int): The baudrate for communication.
        """
        self.read_only = read_only
        self._ids = ids
        self._joint_angles = None
        self._joint_current_goals = np.zeros(len(ids), dtype=float)
        self._lock = Lock()

        # Initialize the port handler, packet handler, and group sync read/write
        self._portHandler = PortHandler(port)
        self._packetHandler = PacketHandler(2.0)
        self._groupSyncReadPosition = GroupSyncRead(
            self._portHandler,
            self._packetHandler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )
        self._groupSyncReadCurrent = GroupSyncRead(
            self._portHandler,
            self._packetHandler,
            ADDR_PRESENT_CURRENT,
            LEN_PRESENT_CURRENT,
        )
        self._groupSyncWriteCurrent = GroupSyncWrite(
            self._portHandler,
            self._packetHandler,
            ADDR_GOAL_CURRENT,
            LEN_GOAL_CURRENT,
        )

        # Open the port and set the baudrate
        if not self._portHandler.openPort():
            raise RuntimeError("Failed to open the port")

        if not self._portHandler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to change the baudrate, {baudrate}")

        # Add parameters for each Dynamixel servo to the group sync read
        for dxl_id in self._ids:
            if not self._groupSyncReadPosition.addParam(dxl_id):
                raise RuntimeError(
                    f"Failed to add parameter for Dynamixel with ID {dxl_id}"
                )
            if not self._groupSyncReadCurrent.addParam(dxl_id):
                raise RuntimeError(
                    f"Failed to add parameter for Dynamixel with ID {dxl_id}"
                )
            # Set Return Delay Time to 0μs
            dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(
                self._portHandler, dxl_id, 9, 0
            )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print(f"Failed to set Return Delay Time for Dynamixel with ID {dxl_id}")
            print(f"dxl_comm_result: {dxl_comm_result}, dxl_error: {dxl_error}")

        # Error check for group sync write
        if len(self._joint_current_goals) != len(self._ids):
            raise ValueError(
                "The length of joint_angles must match the number of servos"
            )
            # if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled to set joint angles")

        self._pause_event = Event()
        self._pause_event.set()
        self._stop_event = Event()
        self._start_thread()

        # Disable torque for each Dynamixel servo
        self._torque_enabled = False
        try:
            self.set_torque_mode(self._torque_enabled)
        except Exception as e:
            print(f"port: {port}, {e}")

    def pause_thread(self):
        self._pause_event.clear()
        time.sleep(0.1)

    def resume_thread(self):
        self._pause_event.set()
        time.sleep(0.1)

    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def set_torque_mode(self, enable: bool):
        self.pause_thread()
        torque_value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        with self._lock:
            for dxl_id in self._ids:
                dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(
                    self._portHandler, dxl_id, ADDR_TORQUE_ENABLE, torque_value
                )
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    print(dxl_comm_result)
                    print(dxl_error)
                    raise RuntimeError(
                        f"Failed to set torque mode for Dynamixel with ID {dxl_id}"
                    )
        self._torque_enabled = enable
        self.resume_thread()
        print(f"torque enabled: {self._torque_enabled}")

    def set_control_mode(self, mode_name: str):
        self.pause_thread()
        mode = CONTROL_MODES[mode_name]
        with self._lock:
            for dxl_id in self._ids:
                dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(
                    self._portHandler, dxl_id, ADDR_OPERATING_MODE, mode
                )
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    print(dxl_comm_result)
                    print(dxl_error)
                    raise RuntimeError(
                        f"Failed to set control mode for Dynamixel with ID {dxl_id}"
                    )
        self._control_mode = mode
        self.resume_thread()
        print(f"control mode: {self._control_mode}")

    def set_read_only(self, read_only: bool):
        self.read_only = read_only

    def _start_thread(self):
        self._read_and_write_thread = Thread(target=self._read_and_write)
        self._read_and_write_thread.daemon = True
        self._read_and_write_thread.start()

    def _write_joint_currents(self):
        for dxl_id, current in zip(self._ids, self._joint_current_goals):
            current_value = int(current)
            param_current = [
                DXL_LOBYTE(current_value),
                DXL_HIBYTE(current_value),
            ]
            # Add current value to the Syncwrite parameter storage
            dxl_addparam_result = self._groupSyncWriteCurrent.addParam(
                dxl_id, param_current
            )
            if not dxl_addparam_result:
                raise RuntimeError(
                    f"Failed to set current for Dynamixel with ID {dxl_id}. "
                    f"dxl_id: {dxl_id}, param_current: {param_current}"
                )

        # Syncwrite goal current
        dxl_comm_result = self._groupSyncWriteCurrent.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError("Failed to syncwrite goal current")
        # Clear syncwrite parameter storage
        self._groupSyncWriteCurrent.clearParam()

    def _read_joint_currents(self):
        joint_currents = np.zeros(len(self._ids), dtype=int)
        dxl_comm_result = self._groupSyncReadCurrent.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"warning, comm failed: {dxl_comm_result}")
        for i, dxl_id in enumerate(self._ids):
            if self._groupSyncReadCurrent.isAvailable(
                dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT
            ):
                current = self._groupSyncReadCurrent.getData(
                    dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT
                )
                current = np.int16(np.uint16(current))
                joint_currents[i] = current
            else:
                raise RuntimeError(
                    f"Failed to get joint currents for Dynamixel with ID {dxl_id}"
                )
        return joint_currents

    def _read_joint_angles(self):
        joint_angles = np.zeros(len(self._ids), dtype=int)
        dxl_comm_result = self._groupSyncReadPosition.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"warning, comm failed: {dxl_comm_result}")
        for i, dxl_id in enumerate(self._ids):
            if self._groupSyncReadPosition.isAvailable(
                dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            ):
                angle = self._groupSyncReadPosition.getData(
                    dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                )
                angle = np.int32(np.uint32(angle))
                joint_angles[i] = angle
            else:
                raise RuntimeError(
                    f"Failed to get joint angles for Dynamixel with ID {dxl_id}"
                )
        return joint_angles

    def _read_joint_angles_async(self):
        with ThreadPoolExecutor() as executor:
            future = executor.submit(self._read_joint_angles)
            return future.result()

    def _read_and_write(self):
        while not self._stop_event.is_set():
            # st = time.time()
            self._pause_event.wait()
            self._joint_angles = self._read_joint_angles_async()
            # print(f"Time to communicate with  Dynamixel: {time.time() - st} seconds")
            # with self._lock:
            # if self.read_only == False:
            #     self._write_joint_currents()
            # self._joint_angles = self._read_joint_angles_async()
            # self._joint_currents = self._read_joint_currents()
            # print(f"joint angles: {self._joint_angles}")
            # print(f"joint currents: {self._joint_currents}")

    def get_joints(self) -> np.ndarray:
        # Return a copy of the joint_angles array to avoid race conditions
        while self._joint_angles is None:
            time.sleep(0.1)
        # with self._lock:
        _j = self._joint_angles.copy()
        return _j / 2048.0 * np.pi

    def set_joint_current_goals(self, values: Sequence[float]):
        self._joint_current_goals = values.copy()

    def close(self):
        self._stop_event.set()
        self._pause_event.set()
        self._read_and_write_thread.join()
        self._portHandler.closePort()


def main():
    # Set the port, baudrate, and servo IDs
    ids = [1]

    # Create a DynamixelDriver instance
    try:
        driver = DynamixelDriver(ids)
    except FileNotFoundError:
        driver = DynamixelDriver(ids, port="/dev/cu.usbserial-FT7WBMUB")

    # Test setting torque mode
    driver.set_torque_mode(True)
    driver.set_torque_mode(False)

    # Test reading the joint angles
    try:
        while True:
            joint_angles = driver.get_joints()
            print(f"Joint angles for IDs {ids}: {joint_angles}")
            # print(f"Joint angles for IDs {ids[1]}: {joint_angles[1]}")
    except KeyboardInterrupt:
        driver.close()


if __name__ == "__main__":
    main()  # Test the driver
