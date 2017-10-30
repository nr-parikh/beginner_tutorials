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

#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief      Class for publishing to chatter topic
 */
class Publisher {
 public:
  /**
   * @brief      Constructor of the class
   */
  Publisher();

  void publish(const std::string& str);

 private:
  ros::NodeHandle nh_;        ///< Node handle for the publisher node
  ros::Publisher publisher_;  ///< Publisher object
  std::string text_;          ///< Message to be published
};
#endif  // INCLUDE_PUBLISHER_HPP_
