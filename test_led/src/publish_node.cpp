#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "test_led/test_led.h"

uint8_t send_value = '0'; // 默认灯带颜色

void ledCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    send_value = msg->data;        // 接收 ROS topic 发布的值
    writeSpeed(static_cast<uint8_t>(send_value + '0'));        // 发送给 STM32
    ROS_INFO("recieve data: %d", send_value);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "public_node");
    ros::NodeHandle nh("~");

    // 串口初始化
    serialInit();

    // 订阅 /led_mode 话题
    ros::Subscriber sub = nh.subscribe("/led_mode", 10, ledCallback);

    ros::spin();
    return 0;
}
