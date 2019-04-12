#include "ros/ros.h" 
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "stdint.h"
#include "math.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Vector3.h"

// Initializing messages
geometry_msgs::Vector3 joints;
bool move_done = false;

// Callbacks 
void move_done_cb(std_msgs::Bool msg) {
  move_done = msg.data;
}


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "master");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  ros::Subscriber move_subscriber = n.subscribe("move_done", 100, move_done_cb);
  //ros::Subscriber even_move_subscriber = n.subscribe("even_done", 100, even_done_cb);

  ros::Publisher move_publisher = n.advertise<geometry_msgs::Vector3>("move", 100);
  //ros::Publisher even_move_publisher = n.advertise<std_msgs::Bool>("even_move", 100);

  int i = 0; 

  while(ros::ok())
  { 
    // Execute tripod walking 
    if (move_done) {
      if (i % 2) {
        // Tell even to move
        joints.x = 0; 
        joints.y = 2; 
        joints.z = 4; 
        move_publisher.publish(joints);
      } else {
        // Tell odd to move 
        joints.x = 1; 
        joints.y = 3; 
        joints.z = 5; 
        move_publisher.publish(joints);
      }
      i++;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
