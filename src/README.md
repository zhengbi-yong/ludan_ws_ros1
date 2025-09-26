## bipedal-robot

该程序是没有手臂版本。

该程序跑在ubuntu20.04上，使用ROS1-Noetic，需要x86架构电脑，**比较吃cpu**。

该程序使用OCS2和ros-control框架，控制方法是非线性模型预测控制，模型采用质心动力学。

该程序是基于[livelybot_dynamic_control](https://github.com/HighTorque-Robotics/livelybot_dynamic_control)、[hunter_bipedal_control](https://bridgedp.github.io/hunter_bipedal_control)以及[legged_control](https://github.com/qiayuanl/legged_control)上进行一些改进，主要是对**约束**进行了修改。

## 统一的机器人描述与命名空间

`wanren_arm/urdf/wanren_arm.urdf` 现在同时包含身体、左右两条腿、左右两条手臂以及颈部关节链，无需再维护多份 URDF。与之配套的控制命名空间按照 `wanren/legs`、`wanren/arms`、`wanren/neck` 进行划分，可通过 `simple_hybrid_joint_controller/launch/bringup_real.launch` 一次性加载所有硬件接口与控制器。各硬件包会从参数 `joint_names` 中读取明确的关节顺序，避免再根据字符串前缀做模糊匹配。

## 模块化开关与配置

- **URDF/Xacro 开关**：`wanren_arm.urdf` 为每个部位提供布尔开关（`enable_waist`、`enable_leg_left`、`enable_leg_right`、`enable_arm_left`、`enable_arm_right`、`enable_neck_left`、`enable_neck_right`），通过 `xacro:if` 组合最终模型，便于按需裁剪硬件。【F:wanren_arm/urdf/wanren_arm.urdf†L4-L13】
- **关节模块参数**：腿部与颈部硬件配置文件使用 `joint_modules` 列表描述每条链路的关节名、方向与启用状态，驱动程序会自动扁平化为 `joint_names`/`joint_directions`，扩展其它部位时只需追加一个模块即可。【F:legged_examples/legged_dm_hw/config/dm.yaml†L1-L21】【F:legged_examples/legged_dm_hw/src/DmHW.cpp†L121-L208】

得益于上述设计，腰、左右臂、左右腿、左右颈部都可以独立启停，后续要拓展新的链路（如手指、视觉云台）仅需复用相同模式配置。

## 更清晰的工作空间结构

为减少无用依赖，左臂相关的功能包（`dmbot_serial_left_arm`、`simple_hybrid_joint_controller_left_arm`、`legged_examples/left_arm_dm_hw`）已被彻底移除，只保留腿部、颈部与右臂桥接的实现。统一的 URDF 与命名空间仍然按照 `wanren/legs`、`wanren/neck`（以及可选的 `wanren/right_arm`）进行划分，避免了重复的模型维护工作。

## 快速使用

1. **编译并配置工作空间**
   ```bash
   cd ~/ludan_ws_ros1
   catkin build          # 或使用 catkin_make，视个人习惯选择
   source devel/setup.bash
   ```
2. **准备串口别名**（见下文“串口命名与硬件准备”）。确保腿、颈、腰等控制板已经映射为 `/dev/mcu_leg`、`/dev/mcu_neck`、`/dev/mcu_waist`，右臂若使用则映射为 `/dev/mcu_rightarm`。
3. **启动腿部 + 颈部硬件与控制器**
   ```bash
   roslaunch simple_hybrid_joint_controller bringup_real.launch robot_type:=dm base_ns:=wanren
   ```
   该入口会加载统一 URDF，启动 `legged_dm_hw` 与 `neck_dm_hw`，并在 `wanren/legs`、`wanren/neck` 命名空间下自动装载对应的混合关节控制器。
4. **右臂（可选）** 若需要右臂的 MoveJ 桥接，可单独启动：
   ```bash
   roslaunch right_arm_hw bringup.launch
   ```
   然后向 `/all_joints_hjc/command_moveJ` 或自定义话题发送命令。

### 调试与排错

- 启动硬件节点时报 `could not retrieve one of the required parameters: loop_hz or cycle_time_error_threshold or thread_priority`
  的错误，多半是因为节点启动时清空了私有命名空间内的参数。Launch 文件已移除 `clear_params="true"`，只要在启动节点前通过
  `<rosparam ... ns="legged_dm_hw"/>` / `<rosparam ... ns="neck_dm_hw"/>` 加载配置，就能正确读取 `loop_frequency` 等关键参数。
  【F:legged_examples/legged_dm_hw/launch/legged_dm_hw.launch†L13-L17】【F:legged_examples/neck_dm_hw/launch/neck_dm_hw.launch†L7-L11】
- 建议的本地测试流程：
  1. `cd ~/ludan_ws_ros1 && catkin build`（或 `catkin_make`），然后 `source devel/setup.bash`。
  2. 仅验证腿部链路：`roslaunch legged_dm_hw legged_dm_hw.launch robot_type:=dm`。
  3. 仅验证颈部链路：`roslaunch neck_dm_hw neck_dm_hw.launch robot_type:=dm`。
  4. 若需要同时检查控制器工作流，可继续使用 `simple_hybrid_joint_controller/launch/bringup_real.launch`。
  这样可以确认参数加载、控制器装载及话题命名是否与硬件代码保持一致。

如需调试腿部或颈部的混合控制器，可直接向 `wanren/legs`、`wanren/neck` 命名空间下的控制话题发布命令；硬件接口会根据 `wanren_arm/config/wanren_controllers.yaml` 中定义的关节顺序进行映射。【F:wanren_arm/config/wanren_controllers.yaml†L1-L41】

### 串口命名与硬件准备

实机运行前请统一各控制板的串口设备名，便于 Launch 文件直接引用：

1. **确认设备信息**：插入控制板后，使用下面的命令查看其 `idVendor`、`idProduct` 以及 `serial` 等属性。
   ```bash
   udevadm info -a -n /dev/ttyACM0 | grep -E "(idVendor|idProduct|serial)"
   ```
2. **编写 udev 规则**：根据实际硬件信息在 `/etc/udev/rules.d/99-wanren-mcu.rules` 中写入类似规则（请将示例值替换为自己的设备 ID）。
   ```
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="6015", ATTRS{serial}=="LEG",   SYMLINK+="mcu_leg",     MODE="0666"
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="6015", ATTRS{serial}=="NECK",  SYMLINK+="mcu_neck",    MODE="0666"
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="6015", ATTRS{serial}=="WAIST", SYMLINK+="mcu_waist",   MODE="0666"
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="6015", ATTRS{serial}=="RARM",  SYMLINK+="mcu_rightarm", MODE="0666"
   ```
3. **刷新规则并验证**：
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ls -l /dev/mcu_*
   ```
   如无权限问题，请确保当前用户加入 `dialout` 组：`sudo usermod -a -G dialout $USER`。

系统默认的 Launch/节点已经使用上述别名：腿部硬件默认 `/dev/mcu_leg`，颈部硬件默认 `/dev/mcu_neck`，腰部默认 `/dev/mcu_waist`，右臂桥接默认 `/dev/mcu_rightarm`。【F:simple_hybrid_joint_controller_leg/launch/bringup_real.launch†L3-L4】【F:simple_hybrid_joint_controller_neck/launch/bringup_real.launch†L3-L4】【F:simple_hybrid_joint_controller_waist/launch/bringup_real.launch†L3-L4】【F:dmbot_serial/src/robot_connect.cpp†L9-L96】【F:dmbot_serial_neck/src/robot_connect.cpp†L9-L96】确保符号链接正确后，即可直接运行上述 Launch 文件。

## 学习
1. 理论框架

该程序的数学原理框架可看[基于NMPC和WBC的双足机器人控制框架简介](https://mcpocket.blog.csdn.net/article/details/136541630?fromshare=blogdetail&sharetype=blogdetail&sharerId=136541630&sharerefer=PC&sharesource=sleeer_zzZZ&sharefrom=from_link)

2. 程序框架

该程序框架是采用ros-control标准格式，对ros-control不太了解的兄弟可先参考用ros-control控制达妙电机的例程[dm-control](https://gitee.com/xauter/dm-control)

## 安装
***安装需要ros基础，这里使用catkin build编译，而不是catkin_make***
### 安装依赖

- [OCS2](https://leggedrobotics.github.io/ocs2/installation.html#prerequisites)

- [ROS1-Noetic](http://wiki.ros.org/noetic)

### 安装 ROS1-Noetic
 这个网上安装教程很多，很简单。
 
### 安装 OCS2
***注意：安装OCS2要保证网络好***

- C++ compiler with C++11 support

- Eigen (v3.3)
```shell
sudo apt-get update
sudo apt-get install libeigen3-dev
```

- Boost C++ (v1.71)
```shell
wget https://boostorg.jfrog.io/artifactory/main/release/1.71.0/source/boost_1_71_0.tar.bz2
tar --bzip2 -xf boost_1_71_0.tar.bz2
cd boost_1_71_0
./bootstrap.sh
sudo ./b2 install
```
如果解压boost_1_71\_0.tar.bz2失败，可以自己去官网下载boost_1_71_0.tar.bz2,官网链接如下：

[https://www.boost.org/users/history/](https://www.boost.org/users/history/)


- 安装剩余依赖
```shell
sudo apt install ros-noetic-catkin
sudo apt install libglpk-dev libglfw3-dev
sudo apt install ros-noetic-pybind11-catkin
sudo apt install python3-catkin-tools
sudo apt install doxygen doxygen-latex
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
sudo apt-get install ros-noetic-rqt-multiplot
sudo apt install ros-noetic-grid-map-msgs
sudo apt install ros-noetic-octomap-msgs
sudo apt install libreadline-dev
sudo apt install libcgal-dev
sudo apt update && sudo apt install checkinstall
sudo apt-get install libserialport0 libserialport-dev
sudo apt install ros-noetic-serial
sudo apt install ros-noetic-joy
sudo apt install ros-noetic-joy-teleop
```

- 安装相关库
1. 针对ocs2单独创建一个工作空间
```shell
mkdir -p ~/ocs2_ws/src
cd ~/ocs2_ws/src
```
2. 下载功能包***（先不编译）***

在~/ocs2_ws/src目录下，打开终端，输入：
```shell
 git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
 git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
 git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
 git clone --depth 1 https://github.com/raisimTech/raisimLib.git -b v1.1.01
 git clone https://github.com/leggedrobotics/elevation_mapping_cupy.git
 git clone https://github.com/ANYbotics/grid_map.git
 git clone https://github.com/leggedrobotics/ocs2.git
```
![](./src/docs/catalog_folder.png)

3. 编译Raisim
```shell
cd ~/ocs2_ws/src/raisimLib
mkdir build
cd build
cmake .. 
make -j4 && sudo checkinstall
```

4. ONNX Runtime 依赖
```shell
cd /tmp
wget https://github.com/microsoft/onnxruntime/releases/download/v1.7.0/onnxruntime-linux-x64-1.7.0.tgz
tar xf onnxruntime-linux-x64-1.7.0.tgz
mkdir -p ~/.local/bin ~/.local/include/onnxruntime ~/.local/lib ~/.local/share/cmake/onnxruntime
rsync -a /tmp/onnxruntime-linux-x64-1.7.0/include/ ~/.local/include/onnxruntime
rsync -a /tmp/onnxruntime-linux-x64-1.7.0/lib/ ~/.local/lib
rsync -a ~/ocs2_ws/src/ocs2/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/ ~/.local/share/cmake/onnxruntime
mkdir -p ~/.local/share/onnxruntime/cmake/
rsync -a ~/ocs2_ws/src/ocs2/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/ ~/.local/share/onnxruntime/cmake/
```
然后需要在.bashrc手动设置环境变量，否则在后续安装ocs2时，容易找不到该软件包
打开终端，输入：
```shell
cd
gedit .bashrc
```
然后在最后一行输入：
```shell
export onnxruntime_DIR=~/.local/
export LD_LIBRARY_PATH=~/.local/lib:${LD_LIBRARY_PATH}
```

5. 编译（最关键的步骤来了，**编译需要在ocs2_ws目录下进行**）

首先，优先编译elevation_mapping_cupy

***注意：一定要添加catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo，如下所示***
```shell
cd ~/ocs2_ws/
catkin init
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build elevation_mapping_cupy
```
没报错的话，编译grid_map
```shell
catkin build grid_map
```
没报错的话，编译hpp-fcl
```shell
catkin build hpp-fcl
```
没报错的话，编译pinocchio
```shell
catkin build pinocchio
```
没报错的话，编译ocs2_robotic_assets
```shell
catkin build ocs2_robotic_assets
```
没报错的话，编译ocs2（这一步编译要等很长时间）
```shell
catkin build ocs2
```

6. 测试

编译完成之后需要在.bashrc手动设置环境变量
打开终端，输入：
```shell
cd
gedit .bashrc
```
然后在.bashrc文件里最后一行输入：
```shell
source ~/ocs2_ws/devel/setup.bash
```
然后在终端输入：
```shell
source .bashrc
```
然后启动ocs2的一个四足例程（首次运行需要等一下，ocs2有个预先计算，等待rviz出现图形）
```shell
roslaunch ocs2_legged_robot_ros legged_robot_ddp.launch
```
<img src="./src/docs/leg_example.png" width="500" height="auto">

OCS2安装成功！！！！

### 安装和编译双足控制程序

首先打开终端，输入：
```shell
mkdir -p ~/catkin_ws
cd ~/catkin_ws
```
然后把gitee上的src文件夹放到catkin_ws目录下，如下所示（不能有其他东西）
![](./src/docs/src.png)

打开终端，输入：
```shell
cd ~/catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build
```
然后在.bashrc文件里最后一行输入：
```
source ~/catkin_ws/devel/setup.bash
```
然后打开终端，输入：
```
cd
source .bashrc
```

## 仿真与实机的运行
***注意：一开始运行实机前最好先运行仿真，检查仿真没问题后，再运行实机***
### 仿真，分两种情况，一种是有遥控器，一种是没有遥控器
#### 没有遥控器
1. 运行gazebo仿真并且载入控制器:

```shell
roslaunch legged_controllers one_start_gazebo.launch
```
<img src="./src/docs/sim.png" width="600" height="auto">

2. 在rqt里面设置***kp_position=100***, ***kd_position=1***，然后在gazebo里按住键盘***Ctrl+Shift+R***让机器人站起来。

3. 新建一个终端，发布/reset_estimation话题，重置状态:
```shell
rostopic pub --once /reset_estimation std_msgs/Float32 "data: 0.0" 
```
4. 新建一个终端，发布/cmd_vel话题，**这里速度不能为0**，不然话题无法发布出去:
```bash
rosrun rqt_robot_steering rqt_robot_steering 
```
<img src="./src/docs/vel.png" width="300" height="auto">

5. 新建一个终端，开启控制器：
```shell
rostopic pub --once /load_controller std_msgs/Float32 "data: 0.0" 
```
6. 最后发布/set_walk话题:
```shell
rostopic pub --once /set_walk std_msgs/Float32 "data: 0.0" 
```


rostopic pub --once /controllers/legged_controller/cmd_joint_velocity std_msgs/Float64MultiArray '{
  "layout": {
    "dim": [
      {"label": "joints", "size": 12, "stride": 12}
    ],
    "data_offset": 0
  },
  "data": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
}'

#### 有遥控器（能支持ros的遥控器都能直接用）

1. 运行gazebo仿真并且载入控制器:
```shell
roslaunch legged_controllers one_start_gazebo.launch
```
2. 在rqt里面设置***kp_position=100***, ***kd_position=1***，然后在gazebo里按住键盘***Ctrl+Shift+R***让机器人站起来。

后面的步骤就使用遥控器按键，每一个按键对应一个话题，对应关系如下图所示：

<img src="./src/docs/handle.png" width="500" height="auto">

### 实机

1. 给stm32h7开发板烧录下位机程序，上电，此时所有电机应该全亮绿灯

2. 在你的电脑里运行上位机程序
首先检查开发板和陀螺仪的连接
```shell
cd
ls /dev/ttyACM*
```
<img src="./src/docs/dev2.png" width="450" height="auto">

***注意：/dev/ttyACM0是单片机设备，/dev/ttyACM1是IMU设备，两者要各自对应***

然后给用usb连接的开发板和陀螺仪设置权限
```shell
sudo chmod -R 777 /dev/ttyACM*
```
接着运行上位机程序
```shell
roslaunch legged_controllers one_start_real.launch
```
此时rviz中的机器人模型不动，并且姿势和实机一样

2. 在rqt里面设置***kp_position=100***, ***kd_position=1***，然后在现实世界里扶正机器人，机器人此时稳定站立。

后面的步骤和上面仿真一样，另外遥控器连接后打开遥控器开关，它会自动发布cmd\_cel话题，不需要再运行rosrun rqt\_robot\_steering rqt_robot\_steering 




## 参考

### 代码参考

https://github.com/bridgedp/hunter_bipedal_control

https://github.com/HighTorque-Robotics/livelybot_dynamic_control

https://github.com/qiayuanl/legged_control

### 文献参考
[OCS2安装参考1](https://blog.csdn.net/Study__ing_/article/details/140177463?fromshare=blogdetail&sharetype=blogdetail&sharerId=140177463&sharerefer=PC&sharesource=sleeer_zzZZ&sharefrom=from_link)

[OCS2安装参考2](https://blog.csdn.net/m0_52545777/article/details/140276558?fromshare=blogdetail&sharetype=blogdetail&sharerId=140276558&sharerefer=PC&sharesource=sleeer_zzZZ&sharefrom=from_link)

[基于NMPC和WBC的双足机器人控制框架简介](https://mcpocket.blog.csdn.net/article/details/136541630?fromshare=blogdetail&sharetype=blogdetail&sharerId=136541630&sharerefer=PC&sharesource=sleeer_zzZZ&sharefrom=from_link)

[OCS2代码解析：Quadrotor]https://zhuanlan.zhihu.com/p/687952253

```
# State Estimation

[1] Flayols T, Del Prete A, Wensing P, et al. Experimental evaluation of simple estimators for humanoid robots[C]//2017 IEEE-RAS 17th International Conference on Humanoid Robotics (Humanoids). IEEE, 2017: 889-895.

[2] Bloesch M, Hutter M, Hoepflinger M A, et al. State estimation for legged robots-consistent fusion of leg kinematics and IMU[J]. Robotics, 2013, 17: 17-24.

# MPC

[3] Di Carlo J, Wensing P M, Katz B, et al. Dynamic locomotion in the mit cheetah 3 through convex model-predictive control[C]//2018 IEEE/RSJ international conference on intelligent robots and systems (IROS). IEEE, 2018: 1-9.

[4] Grandia R, Jenelten F, Yang S, et al. Perceptive Locomotion Through Nonlinear Model-Predictive Control[J]. IEEE Transactions on Robotics, 2023.

[5] Sleiman J P, Farshidian F, Minniti M V, et al. A unified mpc framework for whole-body dynamic locomotion and manipulation[J]. IEEE Robotics and Automation Letters, 2021, 6(3): 4688-4695.

# WBC

[6] Bellicoso C D, Gehring C, Hwangbo J, et al. Perception-less terrain adaptation through whole body control and hierarchical optimization[C]//2016 IEEE-RAS 16th International Conference on Humanoid Robots (Humanoids). IEEE, 2016: 558-564.

[7] Kim D, Di Carlo J, Katz B, et al. Highly dynamic quadruped locomotion via whole-body impulse control and model predictive control[J]. arXiv preprint arXiv:1909.06586, 2019.
```
# ludan_ws_ros1
