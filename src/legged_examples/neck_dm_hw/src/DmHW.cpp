#include "neck_dm_hw/DmHW.h"

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <dmbot_serial/robot_connect.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <XmlRpcValue.h>

namespace legged
{

static void writedata2file(float pos,float vel,float tau,const std::string& path)
{
  std::ofstream f(path, std::ios::app);
  if (!f.is_open()) return;
  f << pos << " " << vel << " " << tau << "\n";
}

bool DmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  // 为了避免外部控制器切换误操作，置一个无效值
  root_nh.setParam("gsmp_controller_switch", "null");

  // 订阅外部IMU与手动覆盖话题
  odom_sub_       = root_nh.subscribe("/imu/data",   1, &DmHW::OdomCallBack, this);
  hybrid_cmd_sub_ = root_nh.subscribe("/hybrid_cmd", 1, &DmHW::hybridCmdCb,  this);
  emg_sub_        = root_nh.subscribe("/emergency_stop", 1, &DmHW::emgCb,    this);

  // 允许在 launch 里配置是否启用手动覆盖和超时
  root_nh.param("manual_topic_override", manual_override_, false);
  root_nh.param("manual_cmd_timeout",    manual_cmd_timeout_, 0.2);

  // 先跑基类（载入URDF等，注册接口，构造 motorsInterface）
  if (!LeggedHW::init(root_nh, robot_hw_nh))
    return false;

  robot_hw_nh.getParam("power_limit", powerLimit_);

  if (!loadJointConfiguration(robot_hw_nh))
    return false;

  loadMotorIdMap(robot_hw_nh);

  // 注册关节、IMU句柄
  if (!setupJoints())
    return false;
  setupImu();

  // 可选的状态与命令回显（不强制）
  cmd_pos_pub_  = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_pos", 10);
  cmd_vel_pub_  = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_vel", 10);
  cmd_ff_pub_   = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_ff", 10);

  read_pos_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_pos", 10);
  read_vel_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_vel", 10);
  read_ff_pub_  = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_ff", 10);

  last_cmd_stamp_ = ros::Time(0);
  emergency_stop_ = false;

  return true;
}

void DmHW::read(const ros::Time & /*time*/, const ros::Duration & /*period*/)
{
  // 从下位机读取关节
  double pos, vel, tau;
  for (size_t i = 0; i < numJoints_; ++i)
  {
    const int motor_idx = motorIdMap_[i];
    if (motor_idx < 0 || static_cast<size_t>(motor_idx) >= numJoints_)
    {
      jointData_[i].pos_ = 0.0;
      jointData_[i].vel_ = 0.0;
      jointData_[i].tau_ = 0.0;
      continue;
    }

    motorsInterface->get_motor_data(pos, vel, tau, motor_idx);
    jointData_[i].pos_ = pos * directionMotor_[i];
    jointData_[i].vel_ = vel * directionMotor_[i];
    jointData_[i].tau_ = tau * directionMotor_[i];
  }

  // IMU 直接用外部 /imu/data
  imuData_.ori[0] = yesenceIMU_.orientation.x;
  imuData_.ori[1] = yesenceIMU_.orientation.y;
  imuData_.ori[2] = yesenceIMU_.orientation.z;
  imuData_.ori[3] = yesenceIMU_.orientation.w;

  imuData_.angular_vel[0] = yesenceIMU_.angular_velocity.x;
  imuData_.angular_vel[1] = yesenceIMU_.angular_velocity.y;
  imuData_.angular_vel[2] = yesenceIMU_.angular_velocity.z;

  imuData_.linear_acc[0] = yesenceIMU_.linear_acceleration.x;
  imuData_.linear_acc[1] = yesenceIMU_.linear_acceleration.y;
  imuData_.linear_acc[2] = yesenceIMU_.linear_acceleration.z;
}

void DmHW::write(const ros::Time& time, const ros::Duration& /*period*/)
{
  bool use_manual = false;

  if (manual_override_)
  {
    const bool alive = (time - last_cmd_stamp_).toSec() < manual_cmd_timeout_;
    use_manual = alive && !emergency_stop_;
  }

  // 生成要下发的 N 路命令
  for (size_t i = 0; i < numJoints_; ++i)
  {
    double pos_des, vel_des, kp, kd, ff;

    if (use_manual)
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      pos_des = cmd_pos_[i];
      vel_des = cmd_vel_[i];
      kp      = cmd_kp_[i];
      kd      = cmd_kd_[i];
      ff      = cmd_tau_[i];
    }
    else
    {
      pos_des = jointData_[i].pos_des_;
      vel_des = jointData_[i].vel_des_;
      kp      = jointData_[i].kp_;
      kd      = jointData_[i].kd_;
      ff      = jointData_[i].ff_;
    }

    const int motor_idx = motorIdMap_[i];
    if (motor_idx < 0 || static_cast<size_t>(motor_idx) >= numJoints_)
    {
      ROS_WARN_THROTTLE(5.0,
        "[Neck] Joint %zu disabled by motor_id_map, skip command.", i);
      dmSendcmd_[i].pos_des_ = 0.0;
      dmSendcmd_[i].vel_des_ = 0.0;
      dmSendcmd_[i].kp_      = 0.0;
      dmSendcmd_[i].kd_      = 0.0;
      dmSendcmd_[i].ff_      = 0.0;
      continue;
    }

    // ⭐ 在这里加日志，打印 controller 写进来的数据
    ROS_INFO_THROTTLE(1.0,
      "[Neck] Pre-map Joint[%zu->motor %d]: pos=%.3f vel=%.3f kp=%.2f kd=%.2f ff=%.3f (manual=%d)",
      i, motor_idx, pos_des, vel_des, kp, kd, ff, use_manual);

    // 关节→电机 方向映射
    dmSendcmd_[i].pos_des_ = pos_des * directionMotor_[i];
    dmSendcmd_[i].vel_des_ = vel_des * directionMotor_[i];
    dmSendcmd_[i].kp_      = kp;
    dmSendcmd_[i].kd_      = kd;
    dmSendcmd_[i].ff_      = ff * directionMotor_[i];

    // ⭐ 再加一层，确认映射后的数据
    ROS_INFO_THROTTLE(1.0,
      "[Neck] Mapped  Joint[%zu->motor %d]: pos=%.3f vel=%.3f kp=%.2f kd=%.2f ff=%.3f",
      i, motor_idx, dmSendcmd_[i].pos_des_, dmSendcmd_[i].vel_des_, dmSendcmd_[i].kp_,
      dmSendcmd_[i].kd_, dmSendcmd_[i].ff_);
  }

  // 写入下位机
  for (size_t i = 0; i < numJoints_; ++i)
  {
    const int motor_idx = motorIdMap_[i];
    if (motor_idx < 0 || static_cast<size_t>(motor_idx) >= numJoints_)
      continue;

    motorsInterface->fresh_cmd_motor_data(dmSendcmd_[i].pos_des_,
                                          dmSendcmd_[i].vel_des_,
                                          dmSendcmd_[i].ff_,
                                          dmSendcmd_[i].kp_,
                                          dmSendcmd_[i].kd_, motor_idx);
  }
  motorsInterface->send_motor_data();

  std_msgs::Float64MultiArray a;
  a.data.resize(numJoints_);
  for (size_t i = 0; i < numJoints_; ++i)
    a.data[i] = dmSendcmd_[i].pos_des_;
  cmd_pos_pub_.publish(a);
}

bool DmHW::loadJointConfiguration(const ros::NodeHandle& robot_hw_nh)
{
  jointNames_.clear();
  directionMotor_.clear();

  if (!flattenJointModules(robot_hw_nh, jointNames_, directionMotor_))
  {
    robot_hw_nh.getParam("joint_names", jointNames_);
    robot_hw_nh.getParam("joint_directions", directionMotor_);
  }

  if (jointNames_.empty())
  {
    ROS_ERROR("[DmHW] Parameters 'joint_modules' or 'joint_names' must provide at least one joint.");
    return false;
  }

  numJoints_ = jointNames_.size();

  if (!directionMotor_.empty() && directionMotor_.size() != numJoints_)
  {
    ROS_ERROR("[DmHW] joint_directions size (%zu) does not match joint count (%zu).",
              directionMotor_.size(), numJoints_);
    return false;
  }

  if (directionMotor_.empty())
    directionMotor_.assign(numJoints_, 1);

  jointData_.assign(numJoints_, {});
  dmSendcmd_.assign(numJoints_, {});
  cmd_pos_.assign(numJoints_, 0.0);
  cmd_vel_.assign(numJoints_, 0.0);
  cmd_kp_.assign(numJoints_, 0.0);
  cmd_kd_.assign(numJoints_, 0.0);
  cmd_tau_.assign(numJoints_, 0.0);

  ROS_INFO_STREAM("[DmHW] Loaded joint configuration with " << numJoints_ << " entries.");
  return true;
}

bool DmHW::flattenJointModules(const ros::NodeHandle& robot_hw_nh,
                               std::vector<std::string>& names,
                               std::vector<int>& directions) const
{
  XmlRpc::XmlRpcValue modules;
  if (!robot_hw_nh.getParam("joint_modules", modules))
    return false;

  if (modules.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("[DmHW] Parameter 'joint_modules' must be a list.");
    return false;
  }

  for (int i = 0; i < modules.size(); ++i)
  {
    if (modules[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("[DmHW] joint_modules[%d] is not a struct, skipping.", i);
      continue;
    }

    const auto& module = modules[i];
    bool enabled = true;
    if (module.hasMember("enabled"))
    {
      try
      {
        enabled = static_cast<bool>(module["enabled"]);
      }
      catch (const XmlRpc::XmlRpcException&)
      {
        ROS_WARN("[DmHW] joint_modules[%d].enabled is not boolean, defaulting to true.", i);
      }
    }
    if (!enabled)
      continue;

    if (!module.hasMember("joints"))
    {
      ROS_WARN("[DmHW] joint_modules[%d] missing 'joints', skipping.", i);
      continue;
    }

    const auto& joint_list = module["joints"];
    if (joint_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN("[DmHW] joint_modules[%d].joints must be a list, skipping.", i);
      continue;
    }

    std::vector<int> moduleDirections;
    if (module.hasMember("directions"))
    {
      const auto& dir_list = module["directions"];
      if (dir_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_WARN("[DmHW] joint_modules[%d].directions must be a list, ignoring.", i);
      }
      else
      {
        moduleDirections.reserve(dir_list.size());
        for (int j = 0; j < dir_list.size(); ++j)
        {
          try
          {
            moduleDirections.push_back(static_cast<int>(dir_list[j]));
          }
          catch (const XmlRpc::XmlRpcException&)
          {
            ROS_WARN("[DmHW] joint_modules[%d].directions[%d] not integer, using 1.", i, j);
            moduleDirections.push_back(1);
          }
        }
      }
    }

    for (int j = 0; j < joint_list.size(); ++j)
    {
      try
      {
        const std::string joint_name = static_cast<std::string>(joint_list[j]);
        names.push_back(joint_name);
        if (!moduleDirections.empty())
        {
          if (static_cast<size_t>(j) < moduleDirections.size())
            directions.push_back(moduleDirections[j]);
          else
            directions.push_back(moduleDirections.back());
        }
        else
        {
          directions.push_back(1);
        }
      }
      catch (const XmlRpc::XmlRpcException&)
      {
        ROS_WARN("[DmHW] joint_modules[%d].joints[%d] not a string, skipping entry.", i, j);
      }
    }
  }

  if (directions.size() != names.size())
    directions.resize(names.size(), 1);

  return !names.empty();
}

void DmHW::loadMotorIdMap(const ros::NodeHandle& robot_hw_nh)
{
  motorIdMap_.resize(numJoints_);
  for (size_t i = 0; i < numJoints_; ++i)
    motorIdMap_[i] = static_cast<int>(i);

  std::vector<int> motor_ids;
  if (!robot_hw_nh.getParam("motor_id_map", motor_ids))
  {
    ROS_INFO("[DmHW] '~motor_id_map' not set, using default 0..%zu", numJoints_ ? numJoints_ - 1 : 0);
    return;
  }

  if (motor_ids.size() != numJoints_)
  {
    ROS_WARN("[DmHW] '~motor_id_map' size=%zu, expected %zu. Keep default mapping.",
             motor_ids.size(), numJoints_);
    return;
  }

  std::vector<bool> used(numJoints_, false);
  for (size_t i = 0; i < motor_ids.size(); ++i)
  {
    int id = motor_ids[i];
    if (id == kInvalidMotor)
    {
      motorIdMap_[i] = kInvalidMotor;
      continue;
    }

    if (id < 0 || static_cast<size_t>(id) >= numJoints_)
    {
      ROS_WARN("[DmHW] motor_id_map[%zu]=%d out of range [0,%zu]. Revert to default mapping.",
               i, id, numJoints_ ? numJoints_ - 1 : 0);
      for (size_t j = 0; j < numJoints_; ++j)
        motorIdMap_[j] = static_cast<int>(j);
      return;
    }

    if (used[id])
    {
      ROS_WARN("[DmHW] motor_id_map contains duplicate id %d. Revert to default mapping.", id);
      for (size_t j = 0; j < numJoints_; ++j)
        motorIdMap_[j] = static_cast<int>(j);
      return;
    }

    used[id] = true;
    motorIdMap_[i] = id;
  }

  std::ostringstream oss;
  for (size_t i = 0; i < numJoints_; ++i)
  {
    if (i)
      oss << ", ";
    oss << motorIdMap_[i];
  }
  ROS_INFO_STREAM("[DmHW] Using motor_id_map: [" << oss.str() << "]");
}

bool DmHW::setupJoints()
{
  if (jointNames_.empty())
  {
    ROS_ERROR("[DmHW] No joints configured. Set 'joint_modules' or 'joint_names'.");
    return false;
  }

  for (size_t index = 0; index < jointNames_.size(); ++index)
  {
    const auto& name = jointNames_[index];
    if (!urdfModel_->getJoint(name))
    {
      ROS_ERROR("[DmHW] Joint '%s' from configuration not found in URDF", name.c_str());
      return false;
    }

    hardware_interface::JointStateHandle state_handle(
      name, &jointData_[index].pos_, &jointData_[index].vel_, &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);

    hybridJointInterface_.registerHandle(HybridJointHandle(
      state_handle,
      &jointData_[index].pos_des_, &jointData_[index].vel_des_,
      &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  }

  ROS_INFO_STREAM("[DmHW] Registered " << jointNames_.size() << " joints.");
  return true;
}

bool DmHW::setupImu()
{
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
      "imu_link", "imu_link",
      imuData_.ori, imuData_.ori_cov,
      imuData_.angular_vel, imuData_.angular_vel_cov,
      imuData_.linear_acc,  imuData_.linear_acc_cov));

  imuData_.ori_cov[0] = 0.0012; imuData_.ori_cov[4] = 0.0012; imuData_.ori_cov[8] = 0.0012;
  imuData_.angular_vel_cov[0] = 0.0004; imuData_.angular_vel_cov[4] = 0.0004; imuData_.angular_vel_cov[8] = 0.0004;
  return true;
}

void DmHW::hybridCmdCb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if (msg->data.size() != 5 * numJoints_) {
    ROS_WARN_THROTTLE(1.0, "[DmHW] /hybrid_cmd 长度应为%zu(%zupos+%zuvel+%zukp+%zukd+%zutau)，当前=%zu",
                    5ull * numJoints_, numJoints_, numJoints_, numJoints_, numJoints_, numJoints_,
                    msg->data.size());
    return;
  }
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  const auto& d = msg->data;
  for (size_t i = 0; i < numJoints_; ++i) cmd_pos_[i] = d[0 * numJoints_ + i];
  for (size_t i = 0; i < numJoints_; ++i) cmd_vel_[i] = d[1 * numJoints_ + i];
  for (size_t i = 0; i < numJoints_; ++i) cmd_kp_[i]  = d[2 * numJoints_ + i];
  for (size_t i = 0; i < numJoints_; ++i) cmd_kd_[i]  = d[3 * numJoints_ + i];
  for (size_t i = 0; i < numJoints_; ++i) cmd_tau_[i] = d[4 * numJoints_ + i];

  last_cmd_stamp_ = ros::Time::now();
}

void DmHW::emgCb(const std_msgs::Float32::ConstPtr& msg)
{
  emergency_stop_ = (msg->data > 0.5);
  if (emergency_stop_) ROS_ERROR("[DmHW] 接收到急停！");
}

} // namespace legged
