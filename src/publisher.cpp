/**
 * @file    publisher.cpp
 * @author  nr-parikh
 * @copyright MIT License (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Class implementation for Publisher
 *
 */
#include "beginner_tutorials/change_text.h"
#include <string>
#include "publisher.hpp"

/**
 * @brief      Constructs the publisher object.
 */
Publisher::Publisher(const std::string& str) {
  publisher_ = nh_.advertise<std_msgs::String>("chatter", 1000);
  setText(str);
}
/**
 * @brief      Sets the string to be published
 *
 */
auto Publisher::setText(const std::string& str) -> void { 
  text_ = str; 
  ROS_INFO_STREAM(">>>>>>>"<<text_);
}
/**
 * @brief      Method to publish the string
 */
auto Publisher::publish() -> void {
  // Set the rate at which messages are to be published
  ros::Rate loop_rate(10);
  // Create a message variable of std_msgs;:String type
  std_msgs::String msg;
  // Continuously publish messages
  while (ros::ok()) {
    // Check the number of subscribers and if it is 0, generate warning
    if (publisher_.getNumSubscribers() == 0) {
      ROS_WARN_STREAM("No subscribers on the topic.");
    } else {
      ROS_INFO_STREAM(
          "Number of subscribers are: " << publisher_.getNumSubscribers());
    }
    // Assign the string to message
    msg.data = text_;
    // Publish the message
    publisher_.publish(msg);
    // Sleep for some time
    loop_rate.sleep();
    // Spin in the callbacks
    ros::spinOnce();
  }
}

auto Publisher::changeText(beginner_tutorials::change_text::Request& request,
                           beginner_tutorials::change_text::Response& resp)
    -> bool {
  // Assign the string received from the service
  text_ = request.text;
  // Response of the service
  resp.response = true;
  // Warn that the message being published is changed
  ROS_WARN_STREAM("Changing the message being published...");

  return true;
}
