/**
 * @file    publisher.cpp
 * @author  nr-parikh
 * @copyright MIT License (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Class implementation for Publisher
 *
 */
#include "publisher.hpp"
#include <string>
/**
 * @brief      Constructs the publisher object.
 */
Publisher::Publisher() {
  publisher_ = nh_.advertise<std_msgs::String>("chatter", 1000);
}
/**
 * @brief      Method to publish the string message
 */
void Publisher::publish(const std::string& str) {
  // Set the rate at which messages are to be published
  ros::Rate loop_rate(10);
  // Create a message variable of std_msgs;:String type
  std_msgs::String msg;
  // Continuously publish messages
  while (ros::ok()) {
    // Assign the string to message
    msg.data = str;
    // Publish the message
    publisher_.publish(msg);
    // Spin in the callbacks
    ros::spinOnce();
    // Sleep for some time
    loop_rate.sleep();
  }
}
