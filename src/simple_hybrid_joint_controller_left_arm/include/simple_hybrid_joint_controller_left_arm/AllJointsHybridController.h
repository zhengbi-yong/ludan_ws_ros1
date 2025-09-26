#pragma once
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_interface/controller.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

namespace simple_hjc {

class AllJointsHybridController
  : public controller_interface::Controller<legged::HybridJointInterface> {
public:
  bool init(legged::HybridJointInterface* hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  std::vector<legged::HybridJointHandle> jointHandles_;
  std::vector<std::string> jointNames_;
  size_t jointCount_{0};  // 关节数 = joints.size()

  // 命令缓存：N x 5 (pos, vel, kp, kd, ff)
  std::vector<std::array<double,5>> jointCommandBuffer_;

  // 默认参数
  double defaultPositionGain_{100.0};
  double defaultDampingGain_{2.0};
  double defaultVelocityCommand_{0.0};
  double defaultFeedforwardTorque_{0.0};

  // 订阅者
  ros::Subscriber sameCommandSub_;
  ros::Subscriber positionArraySub_;
  ros::Subscriber matrixCommandSub_;
  ros::Subscriber singleJointSub_;
  ros::Subscriber moveJointSub_;

  void sameCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void posAllCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void matrixCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void oneCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void moveJCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
};

} // namespace simple_hjc
