/**
 * @file    subscriber.hpp
 * @author  nr-parikh
 * @copyright MIT License (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Class for subscribing to chatter topic.
 *
 */

#ifndef INCLUDE_SUBSCRIBER_HPP_
#define INCLUDE_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <beginner_tutorials/change_text.h>

/**
 * @brief      Class for subscribing to chatter topic
 */
class Subscriber {
 public:
  /**
   * @brief      Constructor of the class
   */
  Subscriber();
  /**
   * @brief      Callback function for the subscriber
   *
   * @param[in]  msg:   String which is being read
   *
   * @return     void:  Return nothing
   */
  auto subscriberCallback(const std_msgs::String::ConstPtr& msg) -> void;

 private:
  ros::NodeHandle nh_;          ///< Node handle for the subscriber node
  ros::Subscriber subscriber_;  ///< Subscriber object
};
#endif  // INCLUDE_SUBSCRIBER_HPP_
