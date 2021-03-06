// MIT License

// Copyright (c) 2017 Neel Parikh

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @file    publisher.hpp
 * @author  nr-parikh
 * @copyright MIT license (c) 2017 Neel Parikh
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
  explicit Publisher(const std::string& str);

  auto setText(const std::string& str) -> void;
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
