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
 * @file    publisher.cpp
 * @author  nr-parikh
 * @copyright MIT License (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Class implementation for Publisher
 *
 * Publish the transform between world frame and publisher frame.
 *
 */

#include "publisher.hpp"
#include <string>
#include "beginner_tutorials/change_text.h"
#include "tf/transform_broadcaster.h"

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
auto Publisher::setText(const std::string& str) -> void { text_ = str; }
/**
 * @brief      Method to publish the string
 */
auto Publisher::publish() -> void {
  // Set the rate at which messages are to be published
  ros::Rate loop_rate(10);
  // Create a message variable of std_msgs;:String type
  std_msgs::String msg;
  // Create a TF broadcaster object
  static tf::TransformBroadcaster tf_br;
  // Create transform object to set transform
  tf::Transform transform;
  // Create quaternion
  tf::Quaternion quaternion;
  // Set origin of the publisher frame
  transform.setOrigin(tf::Vector3(1.0, 1.0, 1.0));
  // Set rotation value
  quaternion.setRPY(0, 0, 3.14);
  // Set the transform
  transform.setRotation(quaternion);

  // Continuously publish messages
  while (ros::ok()) {
    // Check the number of subscribers and if it is 0, generate warning
    if (publisher_.getNumSubscribers() == 0) {
      ROS_WARN_STREAM("No subscribers on the topic.");
    } else {
      ROS_INFO_STREAM(
          "Number of subscribers are: " << publisher_.getNumSubscribers());
    }

    // Broadcast transform between world and publisher frames
    tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                             "world", "publisher"));
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
