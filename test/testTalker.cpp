/**
 * BSD 3-Clause License
 * 
 * Copyright (c) 2018, Ashwin Goyal
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** @file testTalker.cpp
 * @brief It defines a basic rostest and gtest for "talker" node.
 *
 * Copyright [2018] Ashwin Goyal
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include "beginner_tutorials/changeString.h"
#include "beginner_tutorials/setMaxCount.h"

/**
 * @brief This is a basic test which checks if the "editString" service exists
 *        or not. If it exists, check if it changes the string correctly.
 */
TEST(TalketTest, testStringService) {
  // Create NodeHandle
  ros::NodeHandle n;

  // Create a service object for service editString
  ros::service::waitForService("editString");
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::changeString>
                                                                ("editString");

  // Create an object of editString
  beginner_tutorials::changeString srv;

  // Assign input to desired string
  srv.request.str = "The new count is: ";

  // Check if the service call worked properly
  EXPECT_TRUE(client.call(srv));

  // Compare the input and output string of the service
  EXPECT_STREQ("The new count is: ", srv.response.str.c_str());
}

/**
 * @brief This is a basic test which checks if the "setCount" service exists or
 *        not. If it exists, check if it sets the maximum count correctly.
 */
TEST(TalketTest, testCountService) {
  // Create NodeHandle
  ros::NodeHandle n;

  // Create a service object for service setCount
  ros::service::waitForService("setCount");
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::setMaxCount>
                                                                 ("setCount");

  // Create an object of setCount
  beginner_tutorials::setMaxCount srv;

  // Assign input to desired maximum count
  srv.request.count = 100;

  // Check if the service call worked properly
  EXPECT_TRUE(client.call(srv));

  // Compare the input and output count of the service
  EXPECT_EQ(100, srv.response.count);
}

/**
 * @brief This is a basic test which checks if the broadcaster is working.
 */
TEST(TalketTest, testBroadcaster) {
  // Create NodeHandle
  ros::NodeHandle n;

  // Initialize tf parameters
  tf::TransformListener listener;
  tf::StampedTransform transform;

  // Check for the broadcast
  listener.waitForTransform("/world", "/talk", ros::Time(0),
                            ros::Duration(10.0));
  listener.lookupTransform("/world", "/talk", ros::Time(0), transform);

  // Check the position
  tf::Vector3 pos = transform.getOrigin();
  EXPECT_LE(pos.x(), 1);
  EXPECT_LE(pos.y(), 1);
  EXPECT_GE(pos.x(), -1);
  EXPECT_GE(pos.y(), -1);
}