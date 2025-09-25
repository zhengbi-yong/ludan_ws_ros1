
# start env
source /opt/ros/noetic/setup.bash
source ~/ludan_ws/devel/setup.bash

# start roscore
roscore

# strat legged_dm_hw
rosparam set /port /dev/ttyACM0
rosparam set /baud 921600
rosparam get /port
rosparam get /baud

sudo chmod a+rw /dev/ttyACM0
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager
sudo lsof /dev/ttyACM0

roslaunch legged_dm_hw legged_dm_hw.launch

# start joint_state_controller
rosparam set /all_joints_hjc/type "simple_hjc/AllJointsHybridController"
rosparam set /all_joints_hjc/default_kp 0.0
rosparam set /all_joints_hjc/default_kd 0.0
rosparam set /all_joints_hjc/default_vel 0.0
rosparam set /all_joints_hjc/default_ff  0.0
rosparam set /all_joints_hjc/joints "['leg_l1_joint','leg_l2_joint','leg_l3_joint','leg_l4_joint','leg_l5_joint','leg_r1_joint','leg_r2_joint','leg_r3_joint','leg_r4_joint','leg_r5_joint']"

rosrun controller_manager spawner joint_state_controller all_joints_hjc


##### check
A. 核对控制链是否“活着”
1) 确认硬件与控制器在跑
rosservice call /controller_manager/list_controllers

理想输出里应看到：

 joint_state_controller: running


 all_joints_hjc: running


2) 确认控制器订阅到了你发的 topic
rostopic list | grep all_joints_hjc
rostopic info /all_joints_hjc/command_same

Subscribers: 应该 ≥ 1（就是你的控制器）。如果是 0，说明名字/命名空间不对或控制器没在跑。
3) 关节反馈是否在更新
rostopic echo -n 1 /joint_states

能看到 10 个 name/position/velocity/effort，且 header.stamp 在刷新。


1) 写入控制器参数（让 controller_manager 知道它是什么）
rosparam set /all_joints_hjc/type "simple_hjc/AllJointsHybridController"
rosparam set /all_joints_hjc/joints "['leg_l1_joint','leg_l2_joint','leg_l3_joint','leg_l4_joint','leg_l5_joint','leg_r1_joint','leg_r2_joint','leg_r3_joint','leg_r4_joint','leg_r5_joint']"
rosparam set /all_joints_hjc/default_kp 0.0
rosparam set /all_joints_hjc/default_kd 0.0
rosparam set /all_joints_hjc/default_vel 0.0
rosparam set /all_joints_hjc/default_ff  0.0

 想一次性加载也可以先保存成文件再 rosparam load，但上面这样最快。
 
2) 用服务“加载并启动”控制器（比 spawner 更直观）
# 加载
rosservice call /controller_manager/load_controller "name: 'all_joints_hjc'"

# 启动
rosservice call /controller_manager/switch_controller "{start_controllers: ['all_joints_hjc'], stop_controllers: [], strictness: 2}"

# 查看状态（应出现 running）
rosservice call /controller_manager/list_controllers

期望你能看到：
name: "joint_state_controller" state: "running"
name: "all_joints_hjc"        state: "running"


4) 让“所有电机以同一速度转起来”（速度模式 + 阻尼/前馈）
先给小阻尼、小前馈，确认能动再加大：
# 速度模式：vel=+0.20rad/s, kp=0, kd=2.0, ff=0.8
rostopic pub /all_joints_hjc/command_same std_msgs/Float64MultiArray \
"data: [0.0, 0.20, 0.0, 2.0, 0.8]" -1



emergency stop
rostopic pub /all_joints_hjc/command_same std_msgs/Float64MultiArray "data: [0,0,0,0,0]" -1

rostopic pub /all_joints_hjc/command_same std_msgs/Float64MultiArray "data: [0.0, 0.50, 0.0, 1.0, 1.0]" -1