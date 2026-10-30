# holydrone_identification

用于两件事：

1. 用实飞数据辨识 Gazebo 飞机参数。
2. 用 Gazebo 数据辨识训练所需的一阶响应。

## 编译

```bash
cd /home/holy/QUAD_RL/deployment
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 实飞采集

```bash
roslaunch holydrone_identification record.launch \
  environment:=real output:=/data/flight_01
```

该命令只记录数据，不会解锁或控制飞机。

辨识物理参数：

```bash
rosrun holydrone_identification identify.py physics \
  --train /data/train.csv --validation /data/validation.csv \
  --survey /data/survey.yaml --output /data/physics.yaml
```

## Gazebo 辨识

```bash
roslaunch holydrone_identification sim.launch \
  output:=/data/id_train auto_takeoff:=true
```

换一个输出目录和随机种子，再采集一次验证数据。然后拟合一阶响应：

```bash
rosrun holydrone_identification identify.py response \
  --train /data/id_train/record/samples.csv \
  --validation /data/id_validation/record/samples.csv \
  --output /data/response.yaml
```

训练数据和验证数据不能相同。详细状态可查看：

```bash
rostopic echo /identification/state
```
