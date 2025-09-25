#ifndef TEST_LED_H
#define TEST_LED_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>

extern void serialInit();
extern void writeSpeed(uint8_t value);
extern bool readSpeed(uint8_t &mode);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);

#endif