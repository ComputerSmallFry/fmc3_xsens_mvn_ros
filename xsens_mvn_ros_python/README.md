# xsens_mvn_ros_python

这个目录是对 `xsens_mvn_ros` 里 UDP 解析链路的 Python 重写版。

目标很直接:

- 保留和现有 C++ 实现接近的数据流: `UDP socket -> parser -> human data`
- 把协议字段、字节偏移、单位和映射关系写清楚
- 做成一个独立目录，方便你单独阅读、调试和二次开发

## 目录结构

- `protocol.py`: Xsens UDP datagram 的协议定义和二进制解析
- `data_types.py`: 轻量级人体状态数据结构
- `model.py`: Xsens 链接名和关节名常量
- `client.py`: UDP 客户端，负责收包并更新人体状态
- `demo.py`: 一个可直接运行的监听示例

## 运行方式

先确认 MVN Studio 正在向本机的 UDP 端口发数据，然后执行:

```bash
python -m xsens_mvn_ros_python.demo --port 8001
```

如果你只想直接看关节包，不想等待完整模型初始化:

```bash
python -m xsens_mvn_ros_python.demo --port 8001 --print-joints
```

如果你怀疑没有数据，或者不知道当前收到的是哪种包:

```bash
python -m xsens_mvn_ros_python.demo --port 8001 --bootstrap-debug
```

如果只想跑测试:

```bash
python -m unittest discover -s xsens_mvn_ros_python/tests -t .
```

## 说明

这个 Python 版本目前没有直接接 ROS topic，而是先把“收包 + 解析 + 状态更新”这层独立出来。
这样你可以先验证协议和数据，再决定是否接 `rospy` 或 `rclpy`。
