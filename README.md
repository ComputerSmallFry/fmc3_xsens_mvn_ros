# xsens_mvn_ros

`xsens_mvn_ros` 是一个将 Xsens MVN 实时 UDP 数据接入 ROS 2 的仓库。

它的核心作用是：

- 从 MVN Studio 发出的 UDP datagram 中接收人体运动数据
- 解析关节角、刚体位姿、线速度学、角速度学和质心信息
- 发布为 ROS 2 话题与 TF，方便机器人、可视化和算法模块直接使用

当前仓库还包含一个独立的 Python 解析与调试目录 `xsens_mvn_ros_python/`，可用于排查 UDP 收包、查看原始关节信息，以及在不启动 ROS 节点的情况下验证网络流是否正常。

## 功能概览

- ROS 2 节点 `xsens_client`
- 自定义消息包 `xsens_mvn_ros_msgs`
- Xsens MVN UDP parser SDK 子模块 `xsens_mvn_sdk`
- 发布关节角、链节状态、质心和 TF
- 附带 Python 版 UDP 解析与调试工具

## 仓库结构

```text
.
├── xsens_mvn_ros/              # 主功能包，C++ ROS 2 节点
│   ├── include/
│   ├── launch/
│   ├── rviz/
│   └── src/
├── xsens_mvn_ros_msgs/         # 自定义消息定义
├── xsens_mvn_ros_python/       # Python 版 UDP 解析与调试工具
├── docs/                       # 文档资源（历史内容）
└── .gitmodules                 # 子模块定义
```

## 数据流

```text
MVN Studio (Windows)
        |
        | UDP
        v
    Socket
        v
  ParserManager
        v
 HumanDataHandler
        v
ROS 2 topics + TF
```

## 主要组件

### `xsens_mvn_ros`

主包位于 [xsens_mvn_ros](/home/phl/workspace/xsens_mvn_ros/xsens_mvn_ros)。

关键文件：

- [xsens_client_node.cpp](/home/phl/workspace/xsens_mvn_ros/xsens_mvn_ros/src/xsens_client_node.cpp)：ROS 2 节点入口，负责发布话题和 TF
- [XSensClient.cpp](/home/phl/workspace/xsens_mvn_ros/xsens_mvn_ros/src/xsens_client/XSensClient.cpp)：UDP 数据采集与人体状态更新
- [Socket.cpp](/home/phl/workspace/xsens_mvn_ros/xsens_mvn_ros/src/xsens_client/Socket.cpp)：基础 UDP socket 封装
- [HumanDataHandler.cpp](/home/phl/workspace/xsens_mvn_ros/xsens_mvn_ros/src/xsens_client/HumanDataHandler.cpp)：内存中的人体状态容器
- [xsens_client.launch.py](/home/phl/workspace/xsens_mvn_ros/xsens_mvn_ros/launch/xsens_client.launch.py)：ROS 2 启动文件

### `xsens_mvn_ros_msgs`

消息包位于 [xsens_mvn_ros_msgs](/home/phl/workspace/xsens_mvn_ros/xsens_mvn_ros_msgs)。

包含：

- [LinkState.msg](/home/phl/workspace/xsens_mvn_ros/xsens_mvn_ros_msgs/msg/LinkState.msg)
- [LinkStateArray.msg](/home/phl/workspace/xsens_mvn_ros/xsens_mvn_ros_msgs/msg/LinkStateArray.msg)

### `xsens_mvn_ros_python`

Python 调试工具位于 [xsens_mvn_ros_python](/home/phl/workspace/xsens_mvn_ros/xsens_mvn_ros_python)。

适合以下场景：

- 想确认某个 UDP 端口上到底有没有 Xsens 数据
- 不启动 ROS 2，只看原始 joint angle datagram
- 排查 MVN Studio 配置、IP、端口和 datagram 类型

入口文件：

- [demo.py](/home/phl/workspace/xsens_mvn_ros/xsens_mvn_ros_python/demo.py)
- [protocol.py](/home/phl/workspace/xsens_mvn_ros/xsens_mvn_ros_python/protocol.py)

## 依赖

### 系统与 ROS 2

该仓库当前使用 `ament_cmake` 和 `colcon`，面向 ROS 2。

主包依赖包括：

- `rclcpp`
- `geometry_msgs`
- `sensor_msgs`
- `std_msgs`
- `tf2_ros`
- `tf2_eigen`
- `urdf`
- `Eigen3`

消息包依赖包括：

- `rosidl_default_generators`
- `geometry_msgs`
- `std_msgs`

### Python 调试工具

`xsens_mvn_ros_python/` 只依赖 Python 标准库，不需要额外 `pip install`。

## 获取代码

建议连同子模块一起克隆：

```bash
git clone --recursive <your-repo-url>
cd xsens_mvn_ros
```

如果已经克隆过仓库，再补一次子模块：

```bash
git submodule update --init --recursive
```

## 构建

在 ROS 2 工作空间中执行，例如：

```bash
mkdir -p ~/xsens_ws/src
cd ~/xsens_ws/src
git clone --recursive <your-repo-url> xsens_mvn_ros
cd ..
colcon build --symlink-install
source install/setup.bash
```

如果缺少 ROS 依赖，可先执行：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 运行

### 启动 ROS 2 节点

```bash
ros2 launch xsens_mvn_ros xsens_client.launch.py
```

自定义端口：

```bash
ros2 launch xsens_mvn_ros xsens_client.launch.py udp_port:=9765
```

关闭 RViz：

```bash
ros2 launch xsens_mvn_ros xsens_client.launch.py launch_rviz:=false
```

可用参数：

- `model_name`，默认 `skeleton`
- `reference_frame`，默认 `world`
- `udp_port`，默认 `8001`
- `launch_rviz`，默认 `true`
- `rviz_config_file`，默认使用包内 RViz 配置

### 启动 Python 调试工具

查看端口上收到的 datagram 类型：

```bash
python -m xsens_mvn_ros_python.demo --port 8001 --bootstrap-debug
```

直接打印关节角 datagram：

```bash
python -m xsens_mvn_ros_python.demo --port 8001 --print-joints
```

打印完整内部状态 JSON：

```bash
python -m xsens_mvn_ros_python.demo --port 8001 --dump-json --summary-every 1
```

## ROS 2 输出

节点启动后会发布以下内容：

- `joint_states`：`sensor_msgs/msg/JointState`
- `link_states`：`xsens_mvn_ros_msgs/msg/LinkStateArray`
- `com`：`geometry_msgs/msg/Point`
- TF：每个 link 的位姿变换

说明：

- `joint_states.name` 中的名字会带上 `model_name` 前缀，并拆分为 `_x/_y/_z`
- `joint_states.position` 中的角度单位为弧度
- `link_states` 中包含 pose、twist 和 accel
- TF 的 `child_frame_id` 也会带上 `model_name` 前缀

## 快速检查

查看话题列表：

```bash
ros2 topic list
```

查看关节状态：

```bash
ros2 topic echo /joint_states
```

查看链节状态：

```bash
ros2 topic echo /link_states
```

查看质心：

```bash
ros2 topic echo /com
```

## MVN Studio 配置建议

为了让节点正常启动，MVN Studio 侧至少需要满足以下条件：

- 使用 UDP 实时流
- 目标 IP 填写为运行 ROS 2 节点这台机器的 IP
- 目标端口与 `udp_port` 参数一致
- 启用节点初始化所需的 datagram 类型

当前 C++ 客户端在初始化阶段会等待：

- Quaternion datagram
- Joint angles datagram

如果这两类数据没有发到目标端口，节点或调试脚本会一直停在等待初始化的阶段。

## 常见问题

### 1. `Address already in use`

说明目标 UDP 端口已经被别的进程占用。

典型原因：

- `xsens_client` 已经在监听该端口
- Python 调试脚本和 ROS 2 节点同时用了同一个端口
- 上一次运行的进程没有完全退出

可以用下面的命令查看占用者：

```bash
lsof -nP -iUDP:8001
ss -lunp | rg ':8001\\b'
```

### 2. 一直停在 `Waiting for bootstrap packets`

通常表示：

- 当前端口没有收到 UDP 数据
- 收到的数据类型不包含 quaternion / joint angles
- MVN Studio 的目标 IP 或端口配置不对

建议先用 Python 工具检查：

```bash
python -m xsens_mvn_ros_python.demo --port 8001 --bootstrap-debug
```

### 3. Python 工具没有任何输出

如果端口监听成功，但一直没有任何 datagram 类型打印，通常就是该端口没有数据到达。优先检查：

- Windows 端发送目标 IP
- 发送端口
- 防火墙
- 两台设备是否在同一网段

## 示例

仓库中提供了一个关节状态示例输出：

- [example_msg.md](/home/phl/workspace/xsens_mvn_ros/xsens_mvn_ros/example_msg.md)

## 开发说明

该仓库当前已经是 ROS 2 版本，构建系统为 `ament_cmake`，运行方式为 `colcon + ros2 launch`。

如果你在阅读旧资料时看到 `catkin`、`roslaunch` 或 ROS 1 API，请以当前仓库源码为准。

## License

本项目采用 BSD License，详见 [LICENSE](/home/phl/workspace/xsens_mvn_ros/LICENSE)。
