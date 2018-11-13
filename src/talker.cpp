/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 /**
 * @file talker.cpp
 * @author Ashwin Goyal [Ghost1995]
 * @copyright 2018 BSD License
 * 
 * @brief Implementing publisher node
 */

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/changeString.h"
#include "beginner_tutorials/setMaxCount.h"

// Define global variables to be able to call the callback function
extern std::string str = "The count is: ";
extern int maxCount = 0;

/**
 * @brief A call back function which changes the string being published
 * 
 * @param The first parameter is the request to the service
 * @param The second parameter is the response of the service
 * 
 * @return The output is a boolean which is true if there were no errors
 */
bool editString(beginner_tutorials::changeString::Request &req,
                beginner_tutorials::changeString::Response &res) {
  res.str = req.str;
  str = res.str;
  if (str.length() < 1) {
    ROS_ERROR_STREAM("The string should contain at least 1 character.");
    str = "The count is: ";
    ROS_INFO_STREAM("String to be published has not been modified.");
  } else {
    ROS_INFO_STREAM("String to be published has been modified.");
  }
  return true;
}

/**
 * @brief A call back function which sets the maximum number of messages to be published
 * 
 * @param The first parameter is the request to the service
 * @param The second parameter is the response of the service
 * 
 * @return The output is a boolean which is true if there were no errors
 */
bool setCount(beginner_tutorials::setMaxCount::Request &req,
              beginner_tutorials::setMaxCount::Response &res) {
  res.count = req.count;
  maxCount = res.count;
  if (maxCount < 1) {
    ROS_ERROR_STREAM("The maximum count should be greater than 0.");
    maxCount = 0;
    ROS_INFO_STREAM("Maximum count of messages to be published has not been set.");
  } else {
    ROS_INFO_STREAM("Maximum count of messages to be published has been set.");
  }
  return true;
}

/**
 * @brief It is the main function. The program demonstrates simple sending of messages over the ROS system.
 *
 * @param First parameter is the number of inputs
 * @param Second parameter is the input
 *
 * @result It gives out an int just to avoid a warning
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Define service calls
  ros::ServiceServer srvStr = n.advertiseService("editString", editString);
  ros::ServiceServer srvCount = n.advertiseService("setCount", setCount);

  // Define the frequency of publishing the message
  double f = 10.0;
  if (argc > 1) {
    std::string::size_type sz;
    f = std::stod(argv[1], &sz);
    // Warning if the frequency is less than 0
    if (f <= 0) {
      ROS_ERROR_STREAM("Frequency need to be greater than 0.");
      f = 10.0;
      ROS_INFO_STREAM("Frequency of publishing is set as 10 Hz.");
    } else {
      ROS_INFO_STREAM("Frequency of publishing is set as " << f << " Hz.");
    }
  }

  // Set frequency
  ros::Rate loop_rate(f);

  /**
   * A count of how many messages we have sent. This is used to create a
   * unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std::stringstream ss;
    ss << str << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    ros::spinOnce();

    // Check if maximum count of messages has reached
    if (maxCount > 0) {
      if ((maxCount%2 == 0)&&(count == maxCount/2)) {
        ROS_WARN_STREAM("Only " << count << " messages left to be published.");
      } else if ((maxCount%2 == 1)&&(count == (maxCount-1)/2)) {
        ROS_WARN_STREAM("Only " << count+1 << " messages left to be published.");
      } else if (maxCount == count) {
        ROS_WARN_STREAM(count << " messages have been published.");
        ROS_INFO_STREAM("Closing the talker and listener.");
        system("rosnode kill /listener");
        system("rosnode kill /talker");
        break;
      }
    }

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
