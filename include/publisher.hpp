/**
 * @file    publisher.hpp
 * @author  nr-parikh
 * @copyright MIT License (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Class for publishing to chatter topic.
 *
 */

#ifndef INCLUDE_PUBLISHER_HPP_
#define INCLUDE_PUBLISHER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include "beginner_tutorials/change_text.h"

/**
 * @brief      Class for publishing to chatter topic
 */
class Publisher {
 public:
  /**
   * @brief      Constructor of the class
   */
  Publisher(const std::string& str);
  /**
   * @brief      Method to publish the passed string to chatter topic
   *
   * @param[in]  str:   String to be published
   *
   * @return     void: Return nothing
   */
  auto publish() -> void;
  /**
   * @brief      Function for service callback. Change the text being published
   * on chatter topic
   *
   * @param      request:  Request of Service
   * @param      resp:     Response of Service
   *
   * @return     bool:     Return true if service callback was successful
   */
  auto changeText(beginner_tutorials::change_text::Request& request,
                  beginner_tutorials::change_text::Response& resp) -> bool;

 private:
  ros::NodeHandle nh_;        ///< Node handle for the publisher node
  ros::Publisher publisher_;  ///< Publisher object
  std::string text_;          ///< Message to be published
};
#endif  // INCLUDE_PUBLISHER_HPP_
