// MIT License

// Copyright (c) 2017 Neel Parikh

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @file    publisher_main.cpp
 * @author  nr-parikh
 * @copyright MIT License (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Main file for the publisher node. Initializes node, initializes the string
 * being published, starts the server for the service change_text, and publish
 * the string.
 *
 */

#include <string>
#include "beginner_tutorials/change_text.h"
#include "publisher.hpp"

int main(int argc, char **argv) {
  // Create publisher node
  ros::init(argc, argv, "publisher");
  // Inform that the publisher node is started
  ROS_INFO_STREAM("Initializing node: publisher...");
  // Initiate the string to be published
  std::string str = "Welcome to ENPM808X, Fall 2017!";
  if (argc > 1) {
    // Initialize the message with string coming from launch file
    str = argv[1];
  }
  // Node handle for publisher node
  ros::NodeHandle nh;
  // Create the Publisher object
  Publisher pub(str);
  // Create server for service change_text
  ros::ServiceServer server =
      nh.advertiseService("change_text", &Publisher::changeText, &pub);
  // Inform that service is started
  ROS_INFO_STREAM("Service started: change_text");

  // Publish the message
  pub.publish();

  return 0;
}
