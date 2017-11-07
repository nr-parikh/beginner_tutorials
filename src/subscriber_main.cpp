/**
 * @file    subscriber_main.cpp
 * @author  nr-parikh
 * @copyright MIT License (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Main code for the subscriber node. Initialize the node, log the information,
 * and set callbacks.
 *
 */
#include "beginner_tutorials/change_text.h"
#include <string>
#include "subscriber.hpp"

int main(int argc, char **argv) {
  // Create publisher node
  ros::init(argc, argv, "subscriber");
  // Inform that node is initialized
  ROS_INFO_STREAM("Initializing node: subscriber...");
  // Node handle for the subscriber node
  ros::NodeHandle nh;
  // Create service client for service change_text
  ros::ServiceClient client =
      nh.serviceClient<beginner_tutorials::change_text>("change_text");
  // Log fatal error if client is not created
  if (!client.exists()) {
    ROS_FATAL_STREAM("Service is not available!");
  } else {
    ROS_DEBUG_STREAM("Service is running.");
  }
  // Log information until server is not available
  if (ros::service::waitForService("change_text", 10000)) {
    ROS_DEBUG_STREAM("Server is available.");
  } else {
    ROS_ERROR_STREAM("Waiting for server to run.");
  }

  // Create the Publisher object
  Subscriber sub;
  // Spin in the callback
  ros::spin();

  return 0;
}