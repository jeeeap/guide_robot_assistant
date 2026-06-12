# ROS2 Guide Robot Demo Runbook

目标：录制一个演示视频，内容包括：语言或文本输入目的地、Nav2 规划路线、机器人在 AWS small house 中移动、靠近障碍物时语音提示、到达目的地后语音播报。

以下命令默认在 WSL Ubuntu-22.04 中执行，工作区为 `~/guide_robot_ws`。

## 0. 每次开始前清理

如果刚才启动失败、目标秒成功、窗口卡死，先清理旧进程。不要用 `pkill -f ros2`，它会把 ROS2 CLI daemon 也打坏。

```bash
pkill -f gzserver
pkill -f gzclient
pkill -f rviz2
pkill -f nav2
pkill -f component_container
pkill -f robot_state_publisher
pkill -f spawn_entity.py
pkill -f topic_tools
ros2 daemon stop
sleep 2
```

如果 ROS2 命令开始报 `!rclpy.ok()`，在 Windows PowerShell 里执行：

```powershell
wsl --shutdown
```

然后重新打开 Ubuntu。

## 1. 确认项目已编译

终端任意一个，先编译一次：

```bash
cd ~/guide_robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. 确认地点坐标

地点文件应为：

```bash
~/guide_robot_ws/src/guide_robot_assistant/config/small_house_locations.yaml
```

检查：

```bash
cat ~/guide_robot_ws/src/guide_robot_assistant/config/small_house_locations.yaml
```

建议内容：

```yaml
locations:
  entrance:
    name: 门口
    x: 0.0
    y: 0.0
    yaw: 0.0

  bedroom:
    name: 卧室
    x: -4.811442
    y: 1.789502
    yaw: 0.0

  living_room:
    name: 沙发
    x: 0.336492
    y: -1.760747
    yaw: 0.0

  kitchen:
    name: 厨房
    x: 8.418441
    y: -3.376843
    yaw: 0.0

  dining_room:
    name: 餐厅
    x: 5.289403
    y: 1.399966
    yaw: 0.0

  study:
    name: 书房
    x: -7.944181
    y: -3.683770
    yaw: 0.0

  desk:
    name: 书桌
    x: -7.944181
    y: -3.683770
    yaw: 0.0

  service_desk:
    name: 服务台
    x: 0.336492
    y: -1.760747
    yaw: 0.0
```

## 3. 修复 Nav2 仿真时间参数

这是关键项。之前机器人“目标成功但不动”的根因是 Nav2 节点没有使用仿真时间。

先给参数文件加上 `use_sim_time: true`：

```bash
cd ~/guide_robot_ws
cp nav2_params/waffle_small_house.yaml nav2_params/waffle_small_house.yaml.bak.$(date +%Y%m%d_%H%M%S)

python3 - <<'PY'
from pathlib import Path
import yaml

p = Path.home() / "guide_robot_ws/nav2_params/waffle_small_house.yaml"
data = yaml.safe_load(p.read_text())

def walk(obj):
    if isinstance(obj, dict):
        params = obj.get("ros__parameters")
        if isinstance(params, dict):
            params["use_sim_time"] = True
        for v in obj.values():
            walk(v)

walk(data)
p.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
PY

grep -n "use_sim_time" nav2_params/waffle_small_house.yaml
```

如果以后发现 Nav2 仍然发 `/cmd_vel_nav`，这是正常链路：

```text
controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> Gazebo
```

不需要 relay。只有 `/cmd_vel_nav` 有速度但 `/cmd_vel` 没速度时，才需要检查 `velocity_smoother` 和 `use_sim_time`。

## 4. 终端 1：启动 Gazebo 小房子

```bash
source /opt/ros/humble/setup.bash
export SMALL_HOUSE=$HOME/gazebo_worlds/aws-robomaker-small-house-world
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$SMALL_HOUSE/models:/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_DATABASE_URI=""
export LIBGL_ALWAYS_SOFTWARE=1

gazebo --verbose $SMALL_HOUSE/worlds/small_house.world \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so
```

Gazebo 俯视角操作：

- 鼠标滚轮缩放。
- 按住中键拖动平移。
- 右键或左键拖动旋转视角。
- 工具栏里选择正交/俯视不稳定时，直接用鼠标调到俯视即可。

## 5. 终端 2：启动机器人 TF

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle

ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=true
```

## 6. 终端 3：生成 TurtleBot3

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle

ros2 run gazebo_ros spawn_entity.py \
  -entity waffle \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf \
  -x 0.0 -y 0.0 -z 0.01
```

检查传感器：

```bash
ros2 topic echo --once /odom
ros2 topic echo --once /scan
```

## 7. 终端 4：启动 Nav2

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export SMALL_HOUSE=$HOME/gazebo_worlds/aws-robomaker-small-house-world
export MAP=$SMALL_HOUSE/maps/turtlebot3_waffle_pi/map.yaml
export PARAMS=$HOME/guide_robot_ws/nav2_params/waffle_small_house.yaml

ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=$MAP \
  params_file:=$PARAMS \
  autostart:=true
```

## 8. 终端 5：设置初始位姿

等 Nav2 日志里出现 `Managed nodes are active` 后执行：

```bash
source /opt/ros/humble/setup.bash

ros2 topic pub --times 3 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}"
```

确认所有 Nav2 节点都在用仿真时间：

```bash
for n in /amcl /map_server /controller_server /planner_server /bt_navigator /behavior_server /waypoint_follower /velocity_smoother; do
  echo "=== $n ==="
  ros2 param get $n use_sim_time
done
```

应该全部是：

```text
Boolean value is: True
```

确认定位：

```bash
ros2 run tf2_ros tf2_echo map odom
```

能连续输出 Translation / Rotation 即可，按 `Ctrl+C` 停掉。

## 9. 终端 6：可选 RViz

如果需要看地图、路径、点击目标点：

```bash
source /opt/ros/humble/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_XCB_GL_INTEGRATION=none

rviz2
```

RViz 里如果 Map 不显示：

- Add -> Map
- Topic 选 `/map`
- Map 的 Durability Policy 设为 `Transient Local`
- Reliability Policy 设为 `Reliable`
- Fixed Frame 设为 `map`

## 10. 先做 Nav2 裸测试

在终端 5 执行：

```bash
source /opt/ros/humble/setup.bash

ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 8.418441, y: -3.376843, z: 0.0}, orientation: {w: 1.0}}}}"
```

机器人应移动到厨房。

如果出现 `Goal succeeded` 但机器人不动，立刻查：

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
```

正常情况下 `/cmd_vel_nav` 和 `/cmd_vel` 在导航时都应出现非零速度。

## 11. 终端 7：启动课程项目主程序

文本输入模式，适合录制时更稳定：

```bash
cd ~/guide_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch guide_robot_assistant guide_robot_demo.launch.py \
  locations_file:=$HOME/guide_robot_ws/src/guide_robot_assistant/config/small_house_locations.yaml \
  use_sim_time:=true \
  use_microphone:=false \
  use_espeak:=true \
  use_llm:=false
```

启动后 `asr_node` 会提示文本输入兜底模式。WSL/ROS launch 中多节点同时运行时，键盘输入有时不会稳定传给 `asr_node`。录制时推荐在新终端直接发布 `/raw_text`，这等价于 ASR 节点识别出一句话。

新开一个“指令终端”：

```bash
source /opt/ros/humble/setup.bash
```

发送明确地点指令：

```bash
ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '去厨房'}"
```

Agent 意图推理演示：

```bash
ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '我想做饭'}"
ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '我想休息一下'}"
ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '我要学习'}"
ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '我想吃饭'}"
ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '再去一次'}"
ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '我现在在哪里'}"
ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '停下'}"
```

对应推理关系：

```text
我想做饭       -> 厨房
我想吃饭       -> 餐厅
我想休息一下   -> 客厅沙发
我要学习       -> 书桌
再去一次       -> 最近一次请求的目的地
我现在在哪里   -> 查询 Agent 记忆
```

可以打开状态监听，展示 Agent 解析和记忆：

```bash
ros2 topic echo /task_status
ros2 topic echo /agent_memory
```

如果 `locations_file` 参数报错，说明 WSL 里的代码还没同步到新版。先重新编译：

```bash
cd ~/guide_robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果仍然报错，临时把 `small_house_locations.yaml` 复制覆盖默认文件：

```bash
cp ~/guide_robot_ws/src/guide_robot_assistant/config/small_house_locations.yaml \
   ~/guide_robot_ws/src/guide_robot_assistant/config/task_locations.yaml

colcon build --symlink-install
source install/setup.bash

ros2 launch guide_robot_assistant guide_robot_demo.launch.py \
  use_sim_time:=true \
  use_microphone:=false \
  use_espeak:=true \
  use_llm:=false
```

## 12. 终端 8：启动障碍物语音提示

推荐用 monitor 版本，只提示，不抢 `/cmd_vel` 控制权：

```bash
cd ~/guide_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch guide_robot_assistant obstacle_warning_monitor.launch.py
```

不要在 Nav2 同时运行时使用旧的：

```bash
ros2 launch guide_robot_assistant reactive_avoidance.launch.py
```

旧版本会发布 `/cmd_vel`，可能和 Nav2 抢控制。

障碍提示验证：

```bash
ros2 topic echo /avoidance_status
ros2 topic echo /tts_text
```

机器人靠近墙、桌子、门框、家具时，应在 `/tts_text` 看到带方向、距离和行动建议的提示，并由 TTS 播报。例如：

```text
注意，正前方约0.8米处有障碍物，请放慢脚步。
右前方约0.5米有障碍，左侧空间较大，正在向左绕行。
危险，正前方约0.3米有障碍，请立即停下。
```

`/avoidance_status` 中会包含：

```text
hazard_level
hazard_direction
guidance_advice
min_front_distance
min_left_distance
min_right_distance
```

## 13. 录制推荐流程

建议录制画面包含 Gazebo 俯视图和项目终端。RViz 可选，如果电脑卡就不录 RViz。

推荐演示脚本：

1. Gazebo 俯视显示小房子和机器人。
2. 指令终端发送 Agent 意图指令：

   ```bash
   ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '我想做饭'}"
   ```

3. TTS 播报“你提到做饭或取用食物，我判断目的地是厨房，现在开始规划路线。”
4. Nav2 规划并移动，Gazebo 中机器人移动。
5. 接近门框或家具时，障碍提示节点播报带方向和距离的安全提醒。
6. 到达厨房后，TTS 播报“已到达厨房”。
7. 再发送：

   ```bash
   ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '我想休息一下'}"
   ```

8. 机器人推理到客厅沙发并导航。
9. 再发送：

   ```bash
   ros2 topic pub --once /raw_text std_msgs/msg/String "{data: '再去一次'}"
   ```

10. 展示 Agent 记忆：它会重复最近一次请求的目的地。

如果要模拟“语音输入”，可以先用文本模式稳定录一版。真正麦克风模式用：

```bash
ros2 launch guide_robot_assistant guide_robot_demo.launch.py \
  locations_file:=$HOME/guide_robot_ws/src/guide_robot_assistant/config/small_house_locations.yaml \
  use_sim_time:=true \
  use_microphone:=true \
  use_espeak:=true \
  use_llm:=false
```

麦克风模式对 WSL 音频和识别依赖更敏感，今晚录制建议优先用文本输入兜底模式，并在视频旁白里说明这是语音识别节点的文本兜底输入。

## 14. 常见问题速查

### Goal accepted 后立刻 succeeded，但机器人不动

先查仿真时间：

```bash
for n in /amcl /map_server /controller_server /planner_server /bt_navigator /behavior_server /waypoint_follower /velocity_smoother; do
  echo "=== $n ==="
  ros2 param get $n use_sim_time
done
```

如果有 `False`，重新检查 `nav2_params/waffle_small_house.yaml` 里是否有 `use_sim_time: true`。

### Goal rejected

通常是初始位姿没设好，重新执行第 8 步。

### Gazebo 有世界但没有机器人

确认终端 3 的 spawn 输出：

```text
SpawnEntity: Successfully spawned entity [waffle]
```

### `/cmd_vel_nav` 有速度，`/cmd_vel` 没速度

多半是 `velocity_smoother` 时间源或状态问题。先查：

```bash
ros2 lifecycle get /velocity_smoother
ros2 param get /velocity_smoother use_sim_time
```

### `/cmd_vel` 有速度但 Gazebo 不动

确认 Gazebo 机器人订阅：

```bash
ros2 topic info -v /cmd_vel
```

应看到 subscriber：

```text
Node name: turtlebot3_diff_drive
```

### RViz 地图不显示

把 Map 的 Durability Policy 改成 `Transient Local`。

## 15. 关机收尾

录完后在各终端按 `Ctrl+C`，或者执行：

```bash
pkill -f gzserver
pkill -f gzclient
pkill -f rviz2
pkill -f nav2
pkill -f component_container
pkill -f robot_state_publisher
pkill -f spawn_entity.py
ros2 daemon stop
```
