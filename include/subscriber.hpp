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
 * @file    subscriber.hpp
 * @author  nr-parikh
 * @copyright MIT license (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Class for subscribing to chatter topic.
 *
 */

#ifndef INCLUDE_SUBSCRIBER_HPP_
#define INCLUDE_SUBSCRIBER_HPP_

#include <beginner_tutorials/change_text.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

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
