# ROS2 室内导盲机器人技术总结

## 1. 项目目标

本项目实现了一个面向室内导盲场景的 ROS2 智能导航机器人系统。系统运行在 Gazebo 仿真环境中，使用 TurtleBot3 作为移动机器人平台，结合 AWS RoboMaker small house 室内场景、Nav2 导航栈、语义指令解析、任务记忆和导盲式障碍物预警，实现从自然语言指令到自主导航执行的完整闭环。

演示目标包括：

- 用户输入自然语言目的地或生活意图。
- Agent 模块解析用户意图并映射到室内目标点。
- Nav2 根据地图、定位和代价地图规划路径。
- 机器人在 Gazebo 室内环境中移动到目标地点。
- 靠近障碍物时给出方向、距离和风险等级提示。
- 到达目的地后通过 TTS 进行语音反馈。

## 2. 系统总体架构

系统采用 ROS2 多节点解耦架构，各节点通过 topic、action 和参数文件协作。

```text
用户语音/文本
    |
    v
/raw_text
    |
    v
center_node ---------------> /task_status
    |                         /agent_memory
    | /navigation_command
    v
nav_client_node -----------> Nav2 /navigate_to_pose action
    |                         /navigation_status
    v
Nav2 Planner + Controller
    |
    v
/cmd_vel_nav -> velocity_smoother -> /cmd_vel -> Gazebo TurtleBot3

/scan
    |
    v
obstacle_warning_monitor -> /avoidance_status
                         -> /tts_text

/tts_text
    |
    v
tts_node -> 控制台输出 / espeak-ng 语音播报
```

核心模块如下：

| 模块 | 主要文件 | 功能 |
|---|---|---|
| 语音/文本输入 | `asr_node.py` | 将语音识别或文本兜底输入发布到 `/raw_text` |
| Agent 意图解析 | `center_node.py` | 解析自然语言、维护任务记忆、生成导航命令 |
| 导航客户端 | `nav_client_node.py` | 将目标点转换为 Nav2 action goal |
| 语音反馈 | `tts_node.py` | 订阅 `/tts_text` 并播报 |
| 导盲预警 | `reactive_avoidance_node.py` | 根据激光雷达判断障碍方向、距离和风险 |
| 演示启动 | `guide_robot_demo.launch.py` | 启动 Agent、导航客户端、TTS、ASR、日志节点 |
| 预警启动 | `obstacle_warning_monitor.launch.py` | 以 monitor 模式启动障碍预警，不抢占 Nav2 控制 |

## 3. 仿真环境与地图

项目使用 AWS RoboMaker small house world 作为室内环境，相比 TurtleBot3 官方 house world，场景更加接近真实住宅，包括卧室、客厅、厨房、餐厅等区域和家具障碍物。

仿真环境包含：

- Gazebo Classic 11
- ROS2 Humble
- TurtleBot3 Waffle
- AWS small house world
- 预构建 occupancy grid map
- Nav2 AMCL 定位

系统中配置了 small house 场景的语义目标点：

| 语义地点 | 坐标 |
|---|---|
| 门口 | `(0.0, 0.0)` |
| 卧室 | `(-4.811442, 1.789502)` |
| 客厅沙发 | `(0.336492, -1.760747)` |
| 厨房 | `(8.418441, -3.376843)` |
| 餐厅 | `(5.289403, 1.399966)` |
| 书房/书桌 | `(-7.944181, -3.683770)` |

这些坐标保存在：

```text
src/guide_robot_assistant/config/small_house_locations.yaml
```

## 4. Agent 意图理解与记忆系统

项目中的 `center_node` 不是简单的关键词转发节点，而是一个轻量级 Agent 调度中心。它负责从 `/raw_text` 接收用户输入，输出结构化任务命令到 `/navigation_command`。

### 4.1 意图解析

系统支持以下导航意图：

| 意图 | 说明 |
|---|---|
| `navigate` | 前往单个目标点 |
| `multi_navigate` | 按顺序前往多个目标点 |
| `cancel_navigation` | 取消当前导航 |
| `query_location` | 查询当前位置或任务状态 |
| `unknown` | 无法解析的输入 |

示例：

```text
去厨房 -> navigate(kitchen)
带我去卧室 -> navigate(bedroom)
停下 -> cancel_navigation
我现在在哪里 -> query_location
```

### 4.2 生活意图到地点的推理

为了让系统更接近 Agent，而不是机械命令匹配，项目增加了生活语义推理规则：

| 用户表达 | Agent 推理 | 目标点 |
|---|---|---|
| 我想做饭 / 倒水 / 找冰箱 | 做饭或取用食物 | 厨房 |
| 我想吃饭 | 用餐需求 | 餐厅 |
| 我想休息一下 / 看电视 | 休息需求 | 客厅沙发 |
| 我要学习 / 办公 / 用电脑 | 学习办公需求 | 书桌 |
| 我想睡觉 / 躺一会 | 休息睡眠需求 | 卧室 |
| 我要出门 | 离开房间需求 | 门口 |

例如，输入：

```text
我想做饭
```

Agent 输出的回复是：

```text
你提到做饭或取用食物，我判断目的地是厨房，现在开始规划路线。
```

这体现了从“用户意图”到“导航目标”的推理过程。

### 4.3 任务记忆

系统维护以下记忆状态：

- `current_location`：当前已知位置或正在前往的地点
- `current_destination`：当前导航目标
- `last_requested`：最近请求过的目标
- `last_visited`：最近到达过的目标
- `last_reason`：最近一次意图推理原因

这些信息发布到：

```text
/agent_memory
```

记忆系统支持上下文指令：

```text
再去一次
刚才那里
我现在在哪里
```

例如用户先说“我想休息一下”，系统推理到客厅沙发；之后用户说“再去一次”，系统会根据 `last_requested` 自动复用最近目标。

### 4.4 LLM 扩展接口

项目保留了 LLM 解析接口，可通过 `use_llm:=true` 启用大模型解析；在无网络或演示稳定性要求较高时，默认使用本地规则 Agent。这样既保证可复现性，也为后续接入更复杂语言理解能力留下扩展空间。

## 5. 路线规划与自主导航

机器人导航使用 ROS2 Nav2 导航栈，主要流程如下：

1. Gazebo 发布 `/odom`、`/scan` 和 TF。
2. `map_server` 加载 small house occupancy grid map。
3. `amcl` 根据激光雷达和地图完成定位，建立 `map -> odom` 变换。
4. `planner_server` 根据目标点计算全局路径。
5. `controller_server` 根据局部代价地图和 DWB 控制器生成速度命令。
6. `velocity_smoother` 平滑速度后发布 `/cmd_vel`。
7. Gazebo 中的 TurtleBot3 差速驱动插件执行运动。

实际速度链路为：

```text
controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> turtlebot3_diff_drive
```

项目中重点解决了仿真时间一致性问题。Nav2、项目节点和 Gazebo 必须统一使用 `use_sim_time:=true`，否则会出现 action 成功但机器人不动的问题。最终系统在 `guide_robot_demo.launch.py` 中为项目节点显式设置 `use_sim_time`，并在 Nav2 参数文件中固化 `use_sim_time: true`。

## 6. 导盲式障碍物预警

项目中的障碍预警不是简单的“遇到障碍就停”，而是面向导盲场景设计了更细粒度的安全提示。

### 6.1 雷达扇区划分

`reactive_avoidance_node` 订阅 `/scan`，将激光雷达数据划分为：

- 正前方区域
- 左侧区域
- 右侧区域

节点计算三个区域内的最小距离：

```text
min_front_distance
min_left_distance
min_right_distance
```

### 6.2 多级风险判断

系统设置了多级距离阈值：

| 等级 | 条件 | 含义 |
|---|---|---|
| `clear` | 前方安全 | 正常通行 |
| `warning` | 前方障碍进入预警距离 | 提醒用户放慢脚步 |
| `avoid` | 前方障碍进入避让距离 | 提醒方向并建议绕行 |
| `critical` | 距离过近 | 提示立即停下 |
| `side_warning` | 左右侧距离过近 | 提醒靠左或靠右保持安全距离 |

### 6.3 导盲提示内容

预警节点会生成更贴近导盲场景的语音提示，包括：

- 障碍方向
- 障碍距离
- 风险等级
- 建议动作

示例：

```text
注意，正前方约0.8米处有障碍物，请放慢脚步。
右前方约0.5米有障碍，左侧空间较大，正在向左绕行。
危险，正前方约0.3米有障碍，请立即停下。
左侧约0.4米有障碍，请稍微靠右。
```

同时，节点发布结构化状态到：

```text
/avoidance_status
```

关键字段包括：

```text
hazard_level
hazard_direction
guidance_advice
min_front_distance
min_left_distance
min_right_distance
```

### 6.4 Monitor 模式避免控制冲突

在 Nav2 导航过程中，障碍预警节点使用 monitor 模式：

```text
publish_cmd_vel: False
```

这意味着该节点只负责安全提示和状态发布，不直接抢占 `/cmd_vel` 控制权。机器人运动仍由 Nav2 完成，避免多个节点同时发布速度命令导致控制冲突。

## 7. 语音交互与反馈

系统包含完整的语音交互链路：

```text
ASR/text input -> /raw_text -> Agent -> /navigation_command -> Nav2
TTS output <- /tts_text <- Agent / NavClient / ObstacleMonitor
```

由于 WSL 环境中麦克风映射不稳定，演示中使用文本发布 `/raw_text` 作为 ASR 结果模拟。系统架构仍保留 `asr_node`，支持：

- 麦克风识别模式
- 文本兜底模式
- 发布统一 `/raw_text` 接口

TTS 侧使用 `tts_node` 订阅 `/tts_text`，支持：

- 控制台输出
- `espeak-ng` 中文语音播报

语音反馈覆盖三个阶段：

- 指令理解反馈：说明 Agent 判断结果
- 导航执行反馈：开始导航、到达目标、任务失败
- 安全提示反馈：障碍物方向、距离、风险和建议动作

## 8. 工程实现亮点

### 8.1 模块解耦

系统通过 ROS2 topic 和 action 解耦各功能模块。Agent 不直接控制机器人，只发布结构化任务；导航客户端只负责与 Nav2 action 对接；预警节点只负责安全状态和播报。

### 8.2 可解释 Agent

每次意图解析不仅输出目标，还输出 `reason` 字段说明推理依据。例如：

```json
{
  "intent": "navigate",
  "targets": ["kitchen"],
  "reason": "你提到做饭或取用食物，我判断目的地是厨房"
}
```

这让系统行为更容易展示和解释，也方便后续接入更复杂的 LLM 推理。

### 8.3 面向导盲场景的安全设计

障碍物预警不只是用于机器人避障，也考虑到“陪伴盲人移动”的信息表达需求，因此提示内容更接近用户可理解的空间语言：

- 正前方、左前方、右前方
- 约 0.8 米
- 请放慢脚步、请稍微靠右、请立即停下

### 8.4 稳定可复现

项目在仿真中固定：

- 小房子世界模型
- 地图文件
- 目标点坐标
- Nav2 参数
- 仿真时间

这样便于录制视频、调试和课堂展示。

## 9. 演示流程总结

推荐课堂演示流程：

1. Gazebo 展示 small house 室内环境和 TurtleBot3。
2. 发送自然语言意图：

   ```bash
   ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '我想做饭'}"
   ```

3. Agent 推理到厨房并通过 TTS 解释原因。
4. Nav2 规划路径并驱动机器人前往厨房。
5. 靠近墙体、门框或家具时，导盲预警播报方向和距离。
6. 到达厨房后播报完成。
7. 再发送：

   ```bash
   ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '我想休息一下'}"
   ```

8. Agent 推理到客厅沙发并继续导航。
9. 发送：

   ```bash
   ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '再去一次'}"
   ```

10. 展示记忆能力：系统根据最近请求复用目标。

可同时展示：

```bash
ros2 topic echo /agent_memory
ros2 topic echo /avoidance_status
```

## 10. 当前限制与后续改进

当前系统已经完成从自然语言意图到导航执行的闭环，但仍有进一步提升空间：

- WSL 麦克风映射不稳定，真实语音识别在演示环境中暂以 `/raw_text` 模拟。
- 当前 Agent 以规则推理为主，后续可接入本地或云端 LLM，提高复杂表达理解能力。
- 障碍预警目前基于 2D 激光雷达，可进一步结合 RGB-D 相机识别障碍类别。
- 目标点为手工标定，后续可通过 RViz 点击或地图标注工具自动维护。
- 可加入路径可视化、任务统计和用户偏好记忆，例如“用户常去厨房”“偏好较宽路线”等。

## 11. 总结

本项目实现了一个完整的 ROS2 室内导盲机器人演示系统。相比普通 TurtleBot3 导航示例，本项目的重点不只是“让机器人移动”，而是把导航能力包装成面向用户的智能助理：

- 通过 Agent 模块理解自然语言和生活意图。
- 通过记忆系统支持多轮上下文任务。
- 通过 Nav2 实现可靠的路径规划和移动控制。
- 通过导盲式障碍预警提供方向、距离和行动建议。
- 通过 TTS 形成可听见的交互反馈。

因此，系统具备较完整的人机交互闭环，也更贴近“室内导盲机器人”这一应用场景。
