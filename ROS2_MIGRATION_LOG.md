# ROS2移植ログ

## 概要
gello_rosパッケージをROS1からROS2へ移植するための作業ログです。

## 分析結果: ROS依存関係のある/ないファイル分類

### ROSに依存するファイル (13ファイル)

#### Robot Controllers (ROS Controllers)
- `src/gello_ros/robots/ros_cartesian_impedance_control_robot.py`
- `src/gello_ros/robots/ros_cartesian_compliance_control_robot.py` 
- `src/gello_ros/robots/ros_cartesian_motion_control_robot.py`
- `src/gello_ros/robots/ros_joint_trajectory_control_robot.py`

#### Agents (ROS Publishers/Subscribers)
- `src/gello_ros/agents/touch_agent.py`
- `src/gello_ros/agents/act_agent.py`
- `src/gello_ros/agents/gello_agent.py`
- `src/gello_ros/agents/gello_publisher.py`

#### Data and Camera System
- `src/gello_ros/data_utils/save_episode.py`
- `src/gello_ros/cameras/realsense_ros.py`

#### Main Scripts
- `scripts/run_agent_node.py` (メインROSノード)
- `scripts/run_gello.py` 
- `scripts/run_GUI.py`


### ROSに依存しないファイル (36ファイル)

#### Core Architecture (移植不要)
- `src/gello_ros/agents/agent.py` (ベースプロトコル)
- `src/gello_ros/agents/spacemouse_agent.py`
- `src/gello_ros/agents/quest_agent.py`
- `src/gello_ros/env.py` (環境抽象化)

#### Hardware Drivers (移植不要)
- `src/gello_ros/dynamixel/driver.py`
- `src/gello_ros/dynamixel/__init__.py`
- `src/gello_ros/dynamixel/tests/test_driver.py`

#### Robot Controllers (Non-ROS)
- `src/gello_ros/robots/robot.py` (ベースクラス)
- `src/gello_ros/robots/panda.py`
- `src/gello_ros/robots/ur.py`
- `src/gello_ros/robots/xarm_robot.py`
- `src/gello_ros/robots/sim_robot.py`
- `src/gello_ros/robots/dynamixel.py`
- `src/gello_ros/robots/cobotta_gripper.py`
- `src/gello_ros/robots/robotiq_gripper.py`

#### COBOTTA Interface
- `src/gello_ros/robots/pybcapclient/__init__.py`
- `src/gello_ros/robots/pybcapclient/bcapclient.py`
- `src/gello_ros/robots/pybcapclient/variant.py`
- `src/gello_ros/robots/pybcapclient/orinexception.py`

#### Camera System (Non-ROS)
- `src/gello_ros/cameras/camera.py` (ベースクラス)
- `src/gello_ros/cameras/realsense_camera.py`

#### Policy/Learning System (移植不要)
- `src/gello_ros/policy/policy.py`
- `src/gello_ros/policy/utils.py`
- `src/gello_ros/policy/detr/` (ACT実装全体 - 10ファイル)

#### Data Utilities (一部移植必要)
- `src/gello_ros/data_utils/conversion_utils.py` (移植不要)
- `src/gello_ros/data_utils/demo_to_gdict.py` (移植不要)
- `src/gello_ros/data_utils/keyboard_interface.py` (移植不要)
- `src/gello_ros/data_utils/plot_utils.py` (移植不要)

#### Configuration and Training Scripts (移植不要)
- `scripts/gello_get_offset.py`
- `scripts/policy_config.py` 
- `scripts/train.py`
- `setup.py`

## 移植戦略

### Phase 1: Build System Migration
- [ ] `package.xml` をROS2フォーマットに更新 (format="3")
- [ ] `CMakeLists.txt` をament_cmakeベースに変更
- [ ] `setup.py` の削除 (ament_cmakeのみ使用)

### Phase 2: Core ROS Files Migration
#### 優先度: 高
- [ ] `scripts/run_agent_node.py` (メインノード)
- [ ] ROS Controllers (4ファイル)
- [ ] ROSベースAgents (4ファイル)

#### 優先度: 中  
- [ ] `src/gello_ros/cameras/realsense_ros.py`
- [ ] `src/gello_ros/data_utils/save_episode.py`

### Phase 3: Launch Files Migration
- [ ] `.launch` → `.launch.py` 変換 (9ファイル)

### Phase 4: Configuration Migration  
- [ ] `config/*.yaml` パラメータ形式の確認・調整

## ROS1 → ROS2 主な変更点

### Import文の変更
```python
# ROS1
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Wrench

# ROS2  
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Wrench
```

### Node初期化の変更
```python
# ROS1
rospy.init_node('node_name')
pub = rospy.Publisher('/topic', String, queue_size=10)

# ROS2
rclpy.init()
node = Node('node_name') 
pub = node.create_publisher(String, '/topic', 10)
```

## 作業ステータス
- [x] ROS依存関係分析完了
- [x] ファイル分類完了  
- [x] Build system移植
  - [x] `package.xml` をROS2フォーマット（format="3"）に更新
  - [x] `CMakeLists.txt` をament_cmakeベースに変更  
  - [x] `setup.py` の削除（ament_cmakeのみ使用）
- [ ] Core ROS files移植
- [ ] Launch files移植
- [ ] Testing & Validation

## ROS2でのKinematics（運動学計算）対応

### 1. 背景と課題

ROS1で広く使われていた`urdf_parser_py`や`trac_ik_python`といった運動学計算ライブラリは、ROS2の標準ディストリビューションには含まれていません。そのため、ROS2へ移行するにあたり、これらの代替となるライブラリを再選定し、運動学計算（順運動学: FK、逆運動学: IK、ヤコビアン計算）の実装方法を再構築する必要がありました。

### 2. 採用ライブラリとセットアップ

本プロジェクトでは、実績と性能を考慮し、以下の2つの外部ライブラリを採用します。

-   **FK / ヤコビアン計算:** `kdl_parser_py`
    -   **役割:** ROS2環境で`PyKDL`を容易に扱うためのラッパーライブラリ。URDFファイルから直接KDLツリーを構築し、FKやヤコビアン計算をシンプルに実行できます。
    -   **出典:** `powder_grinding_ros2`プロジェクトで利用されている実装を参考にしています。
-   **IK計算:** `pytracik`
    -   **役割:** KDLベースでありながら、標準のIKソルバーよりも高速かつロバストな解を見つけることができる`Trac-IK`のPythonバインディングです。関節リミットを考慮した計算が可能です。

これらのライブラリは、`third_party.repos`に定義されており、`vcs import`コマンドまたは以下の`git clone`コマンドで手動でワークスペースに導入する必要があります。

```bash
# kdl_parser_py (powder_grinding_ros2内に含まれる)
git clone https://github.com/quantumbeam/powder_grinding_ros2.git src/third_party/powder_grinding_ros2

# pytracik
git clone https://github.com/YusakuNakajima/pytracik.git src/third_party/pytracik
```

### 3. 詳細な実装方法

#### 3.1. `kdl_parser_py`によるFKとヤコビアン計算

`kdl_helper.py`は、PyKDLの複雑な処理をカプセル化し、直感的なインターフェースを提供します。

**`KDLHelper`クラスの初期化と使用例**
```python
import PyKDL
from kdl_parser_py.kdl_helper import KDLHelper

# URDFパス、ベースリンク、エンドエフェクタリンクを指定して初期化
urdf_path = "/path/to/your/robot.urdf"
base_link = "base_link"
ee_link = "tool0"
kdl_helper = KDLHelper(urdf_path=urdf_path, base_link=base_link, ee_link=ee_link)

# 現在の関節角度 (サンプル)
joint_states = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

# 順運動学 (FK) の計算
# 戻り値: [x, y, z], [x, y, z, w] (位置とクォータニオン)
fk_pos, fk_rot = kdl_helper.fk(joint_states)
print(f"FK Position: {fk_pos}, Rotation: {fk_rot}")

# ヤコビアンの計算
# 戻り値: 6xNのnumpy配列 (Nは関節数)
jacobian_matrix = kdl_helper.jacobian(joint_states)
print(f"Jacobian Matrix:\n{jacobian_matrix}")
```

#### 3.2. `pytracik`によるIK計算

`pytracik`は、目標姿勢に対してロバストなIK解を提供します。

**`TracIK`ソルバーの初期化と使用例**
```python
from pytracik.trac_ik import TracIK
import numpy as np

# URDFパス、ベースリンク、エンドエフェクタリンクを指定して初期化
urdf_path = "/path/to/your/robot.urdf"
ik_solver = TracIK(
    base_link_name="base_link",
    tip_link_name="tool0",
    urdf_path=urdf_path
)

# 目標姿勢 (位置と回転行列)
target_pos = [0.5, 0.2, 0.4]
target_rotmat = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0]
])

# 計算の初期値となるシード関節角度
seed_jnt = [0.0] * ik_solver.number_of_joints

# 逆運動学 (IK) の計算
# 戻り値: 関節角度のリスト、または解が見つからない場合はNone
result_ik = ik_solver.ik(target_pos, target_rotmat, seed_jnt_values=seed_jnt)

if result_ik:
    print(f"IK Solution: {result_ik}")
else:
    print("IK solution not found.")
```

### 4. `gello_ros`移植での推奨アプローチ

`gello_ros`パッケージのROS1実装からROS2へ移行するにあたり、運動学計算は以下の通り置き換えることを推奨します。これにより、`moveit_commander`への依存をなくし、コンポーネントの独立性と計算速度を向上させます。

| ROS1での実装 | → | ROS2での推奨実装 | 目的 |
| :--- | :--- | :--- | :--- |
| `trac_ik_python` | → | **`pytracik`** | 高速・ロバストなIK計算 |
| `ur_pykdl` | → | **`kdl_helper`** | FK計算の簡素化 |
| `move_group.get_jacobian_matrix` | → | **`kdl_helper.jacobian()`** | MoveIt非依存のヤコビアン計算 |



## 備考
- 全体の約26% (13/49ファイル) がROS依存
- Policy/Learning関連は完全にROS非依存のため移植不要
- Hardware drivers (Dynamixel, UR, Franka等) は移植不要
