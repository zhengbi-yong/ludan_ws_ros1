
# new
# 打开端口的方法
rosparam set /port /dev/mcu_rightarm
rosparam set /baud 921600
rosparam get /port
rosparam get /baud

sudo chmod a+rw /dev/mcu_rightarm
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager
#sudo lsof /dev/mcu_rightarm


# 运行运动功能
roslaunch simple_hybrid_joint_controller bringup_real.launch


roslaunch simple_hybrid_joint_controller bringup_new.launch

# 成功后你应该能看到：
rosservice call /controller_manager/list_controllers
# joint_state_controller: running
# all_joints_hjc:        running

# 所有电机一起动
# all motor control together
rostopic pub -r 20 /all_joints_hjc/command_same std_msgs/Float64MultiArray "data: [0.0, 0.20, 0.0, 1.0, 1.0]"

# 急停
# EMERGENCTY STOP
rostopic pub -r 20 /all_joints_hjc/command_same std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0]"

# 单个电机运动
# single motor control

# 速度控制 speed control
# example id=12 DM4340
rostopic pub /all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [12, 0, 0.2, 0, 1, 1]"

# 位置控制 pos control
# example id=12 DM4340
rostopic pub /all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [12, 1.0, 0.0, 1.0, 1.0, 0.0]"


# test
rostopic pub rightarm/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [13, 1.0, 0.0, 2.0, 1.0, 0.0]"

rostopic pub neck/all_joints_hjc_neck/command_one std_msgs/Float64MultiArray "data: [10, 1.0, 0.0, 2.0, 1.0, 0.0]"

# 自定义电机 ID 映射（示例：仅启用 9 号电机控制第 10 个关节，其余禁用）
rosparam set /neck/motor_id_map "[-1, -1, -1, -1, -1, -1, -1, -1, -1, 9, -1, -1, -1, -1]"

# 注意：列表长度必须为 14，对应 14 个关节；值为 -1 表示忽略该关节。

# 高级控制
# MoveJ
# all motor input:position Kp=10 Kd=1 ff=0 vel=0
rostopic pub /all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0]"

# MoveIt
# 符合ros_control 标准定义的东西 配置文件
# moveit  收到末端xyz和关节角 输出关节角 给ros_control  

# start MoveJ
roslaunch simple_hybrid_joint_controller bringup_real.launch 

# start bridge
roslaunch right_arm_hw bringup.launch 

# start moveit
roslaunch right_arm_moveit_config move_group.launch

# start rviz
rosrun rviz rviz

# set
    # add Robotstate
    # add MotionPlanning
    # Joints set angle
    # plan
    # execute



 


