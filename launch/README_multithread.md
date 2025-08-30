# マルチスレッド対応 Franka Launch ファイル

## 概要

Franka FR3ロボットとTouchデバイスのマルチスレッド統合システム用launchファイルです。

## 用意されたlaunchファイル

### 1. `run_multithread_franka.launch.py`
基本的なマルチスレッド対応統合システム

### 2. `run_multithread_franka_advanced.launch.py`  
高性能・リアルタイム対応統合システム（推奨）

## 使用方法

### 基本的な使用法

```bash
# 基本版の起動
ros2 launch gello_ros run_multithread_franka.launch.py

# 高性能版の起動  
ros2 launch gello_ros run_multithread_franka_advanced.launch.py
```

### パラメータ指定例

```bash
# TouchAgentでカルテシアンインピーダンス制御
ros2 launch gello_ros run_multithread_franka_advanced.launch.py \
    agent_type:=touch \
    controller_type:=cartesian_impedance_controller \
    robot_ip:=192.168.1.100 \
    control_hz:=500

# GelloAgentでジョイント制御
ros2 launch gello_ros run_multithread_franka_advanced.launch.py \
    agent_type:=gello \
    controller_type:=joint_trajectory_controller \
    control_hz:=100

# ACT推論モード
ros2 launch gello_ros run_multithread_franka_advanced.launch.py \
    agent_type:=act \
    save_episode:=false \
    skip_initial_move:=false
```

### リアルタイム最適化

```bash
# リアルタイム優先度とCPU親和性設定
ros2 launch gello_ros run_multithread_franka_advanced.launch.py \
    use_rt_priority:=true \
    cpu_affinity:=2,3 \
    control_hz:=1000

# パフォーマンス監視有効
ros2 launch gello_ros run_multithread_franka_advanced.launch.py \
    enable_performance_monitoring:=true
```

## 利用可能パラメータ

| パラメータ | デフォルト | 説明 |
|------------|------------|------|
| `agent_type` | `touch` | エージェント種別 (touch/gello/act) |
| `controller_type` | `cartesian_impedance_controller` | ロボットコントローラ |
| `robot_ip` | `192.168.1.1` | Frankaロボット IP アドレス |
| `control_hz` | `500` | 制御周波数 [Hz] |
| `save_episode` | `false` | エピソード保存の有効化 |
| `skip_initial_move` | `true` | 初期移動のスキップ |
| `use_rt_priority` | `true` | リアルタイム優先度使用 |
| `cpu_affinity` | `2,3` | CPU親和性 (コア指定) |

## システム要件

### 推奨システム設定

1. **リアルタイムカーネル**
   ```bash
   # RT kernel確認
   uname -r | grep rt
   ```

2. **CPU Governor設定**
   ```bash
   # Performance modeに設定
   sudo cpupower frequency-set -g performance
   ```

3. **ネットワーク最適化**
   ```bash
   # Franka用のネットワーク設定
   sudo ethtool -G <interface> rx 4096 tx 4096
   ```

4. **メモリロック**
   ```bash
   # /etc/security/limits.conf に追加
   * soft memlock unlimited
   * hard memlock unlimited
   ```

## トラブルシューティング

### 1. パフォーマンスが出ない場合

- CPU governorを確認: `cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor`
- システム負荷を確認: `htop`, `iotop`
- リアルタイムカーネルの使用を検討

### 2. TouchAgent接続エラー

- Touchデバイスの接続確認: `lsusb`
- 権限確認: デバイスファイルの読み書き権限
- ドライバ確認: OpenHapticsライブラリのインストール

### 3. マルチスレッドエラー

- ROS2 executorログの確認
- メモリ不足の確認: `free -h`
- スレッド制限の確認: `ulimit -u`

## パフォーマンス監視

パフォーマンス監視を有効にすると、以下の情報が定期的に出力されます：

- CPU/メモリ使用率
- トピック周波数
- 目標周波数に対する達成率
- システムロードアベレージ

```bash
# ログ例
🔍 パフォーマンス監視レポート
💻 システム:
   CPU使用率: 45.2%
   メモリ使用率: 62.1%
📊 トピック統計:
   🟢 /touch_debug_action_pose_rviz:
      周波数: 498.5 Hz (目標: 500.0 Hz)
      達成率: 99.7%
```

## 注意事項

- 高周波数制御（500Hz以上）には十分なハードウェア性能が必要
- リアルタイム優先度設定には管理者権限が必要な場合があります
- CPU親和性設定は使用可能コア数に応じて調整してください