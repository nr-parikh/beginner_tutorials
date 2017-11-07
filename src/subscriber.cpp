/**
 * @file    subscriber.cpp
 * @author  nr-parikh
 * @copyright MIT License (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Class implementation for Subscriber
 *
 */

#include <string>
#include "subscriber.hpp"
/**
 * @brief      Constructs the object of the class.
 */
Subscriber::Subscriber() {
  // Subscribe to the "chatter" topic
  subscriber_ =
      nh_.subscribe("chatter", 1000, &Subscriber::subscriberCallback, this);
}
/**
 * @brief      Subsriber callback
 *
 */
auto Subscriber::subscriberCallback(const std_msgs::String::ConstPtr& msg)
    -> void {
  // Log the incoming string
  ROS_INFO_STREAM("The incoming stream is: " << std::endl << msg->data);
}
