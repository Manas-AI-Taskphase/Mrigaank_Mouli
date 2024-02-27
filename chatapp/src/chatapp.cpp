#include "ros/ros.h"
#include "std_msgs/String.h"


std::string username;

void chatCallback(const std_msgs::String::ConstPtr & msg) {
  ROS_INFO("%s", msg -> data.c_str()); // Printing messages from the subscribed 'chat' topic
}

int main(int argc, char ** argv) {
  std::cout << "Enter your username: ";
  std::getline(std::cin, username);
  // Initialing a User Node
  ros::init(argc, argv, username);
  ros::NodeHandle nh;
  // Creating an Asynchronous Spin object 
  ros::AsyncSpinner spinner(1);
  // Starting  a thread to handle Callbacks concurrently
  spinner.start();
  // Creating a Publisher object to publish the message input by user to the 'chat topic'
  ros::Publisher pub = nh.advertise < std_msgs::String > ("chat", 1000);
  // Creating a Subscribe object to recive messages published  to the 'chat topic'
  ros::Subscriber sub = nh.subscribe("chat", 1000, chatCallback);

  while (ros::ok()) {
    std_msgs::String msg;
    std::string message;
    std::getline(std::cin, message);
    msg.data = username + ":" + message;
    pub.publish(msg); // Publishing the message to the chat topic
    ros::spinOnce();
  }
  spinner.stop();
  return 0;
}
