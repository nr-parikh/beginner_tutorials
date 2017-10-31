/**
 * @file    main.cpp
 * @author  nr-parikh
 * @copyright MIT License (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Main node file. This file initializes ROS and creates the objects for the
 * Publisher and Subscriber classes.
 *
 */

#include <string>
#include "publisher.hpp"
#include "subscriber.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "main_node");
  // Initiate the string to be published
  std::string str = "Welcome to ENPM808X, Fall 2017!";
  // Create the Subscriber object
  Subscriber sub;
  // Create the Publisher object
  Publisher pub;
  // Publish the message
  pub.publish(str);

  return 0;
}
