#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "stdint.h"
#include "math.h"
#include <visualization_msgs/Marker.h>
#include "hebiros/AddGroupFromNamesSrv.h"
#include "geometry_msgs/Polygon.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/SizeSrv.h"
#include <vector>
#include <algorithm>

using namespace std;
using namespace hebiros;

std_msgs::Bool moving;

std_msgs::Float64MultiArray standing_queue;
int leg_components = 18;

sensor_msgs::JointState feedback;       // The actuator feedback struccture
volatile int            feedbackvalid = 0;

void movingCallback(std_msgs::Bool data) { 
  moving = data;
}

/*
**   Feedback Subscriber Callback
*/
void feedbackCallback(sensor_msgs::JointState data)
{
  feedback = data;
  //ROS_INFO("FBK pos [%f]", feedback.position[0]);
  feedbackvalid = 1;
}

void stand() { 
  // Create a queue of standing positions.
  for (int i = 0; i < leg_components; i++) { 
    if ((i % 3) == 0) {
      standing_queue.data.push_back(0);
    } else if ((i % 3) == 1) { 
      if ((i % 2) == 0) { 
        standing_queue.data.push_back(-M_PI/2);
      } else { 
        standing_queue.data.push_back(M_PI/2);
      }
    } else {
      standing_queue.data.push_back(0.);
    }
  } 
}

void walkTripod() { 
  // Create a queue of tripod gait positions.

}

int main(int argc, char **argv)
{ 
  // Initialize the basic ROS node, run at 200Hz.
  ros::init(argc, argv, "tripod_brain");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);

  standing_queue.data.reserve(leg_components);

  // Ask the Hebi node to list the modules.  Create a client to their
  // service, instantiate a service class, and call.  This has no
  // input or output arguments.
  ros::ServiceClient entry_list_client = n.serviceClient<EntryListSrv>("/hebiros/entry_list");
  EntryListSrv entry_list_srv;
  entry_list_client.call(entry_list_srv);
  ROS_INFO("%d", entry_list_srv.response.entry_list.size);

  // Create a new "group of actuators".  This has input arguments,
  // which are the names of the actuators.
  std::string group_name = "me110";
  ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromNamesSrv>("/hebiros/add_group_from_names");
  AddGroupFromNamesSrv add_group_srv;
  add_group_srv.request.group_name = group_name;
  add_group_srv.request.names = {"hip1", "femur1", "tibia1", "hip2", "femur2", "tibia2",
                                 "hip3", "femur3", "tibia3", "hip4", "femur4", "tibia4",
                                 "hip5", "femur5", "tibia5", "hip6", "femur6", "tibia6"};
  add_group_srv.request.families = {"leg1", "leg1", "leg1", "leg2", "leg2", "leg2", 
                                    "leg3", "leg3", "leg3", "leg4", "leg4", "leg4",
                                    "leg5", "leg5", "leg5", "leg6", "leg6", "leg6"};
  // Repeatedly call the service until it succeeds.
  while(!add_group_client.call(add_group_srv));

  // Check the size of this group.  This has an output argument.
  ros::ServiceClient size_client = n.serviceClient<SizeSrv>("/hebiros/"+group_name+"/size");
  SizeSrv size_srv;
  size_client.call(size_srv);
  ROS_INFO("%s has been created and has size %d", group_name.c_str(), size_srv.response.size);

  // Subscriber to listen to whether the robot is moving.
  ros::Subscriber movingSubscriber = n.subscribe("/moving", 100, movingCallback);


  // Create a subscriber to receive feedback from the actuator group.
  ros::Subscriber feedback_subscriber = n.subscribe("/hebiros/"+group_name+"/feedback/joint_state", 100, feedbackCallback);

  feedback.position.reserve(leg_components);
  feedback.velocity.reserve(leg_components);
  feedback.effort.reserve(leg_components);

  /*initial_pos.position.reserve(leg_components);
  initial_pos.velocity.reserve(leg_components);
  initial_pos.effort.reserve(leg_components);*/

  // Create a publisher to send commands to the actuator group.
  // ros::Publisher command_publisher = n.advertise<sensor_msgs::JointState>("/hebiros/"+group_name+"/command/joint_state", 100);

  // GoalPos publisher
  ros::Publisher goal_publisher = n.advertise<std_msgs::Float64MultiArray>("goal", 100);

  sensor_msgs::JointState command_msg;
  command_msg.name.push_back("leg1/hip1");
  command_msg.name.push_back("leg1/femur1");
  command_msg.name.push_back("leg1/tibia1");
  command_msg.name.push_back("leg2/hip2");
  command_msg.name.push_back("leg2/femur2");
  command_msg.name.push_back("leg2/tibia2");
  command_msg.name.push_back("leg3/hip3");
  command_msg.name.push_back("leg3/femur3");
  command_msg.name.push_back("leg3/tibia3");
  command_msg.name.push_back("leg4/hip4");
  command_msg.name.push_back("leg4/femur4");
  command_msg.name.push_back("leg4/tibia4");
  command_msg.name.push_back("leg5/hip5");
  command_msg.name.push_back("leg5/femur5");
  command_msg.name.push_back("leg5/tibia5");
  command_msg.name.push_back("leg6/hip6");
  command_msg.name.push_back("leg6/femur6");
  command_msg.name.push_back("leg6/tibia6");
  command_msg.position.resize(leg_components);
  command_msg.velocity.resize(leg_components);
  command_msg.effort.resize(leg_components);

  // Wait until we have some feedback from the actuator.
  ROS_INFO("Waiting for initial feedback");
  while (!feedbackvalid) {
      ros::spinOnce();
      loop_rate.sleep();
  }
  ROS_INFO("Initial feedback received");

  // Prep the servo loop.
  double  dt = loop_rate.expectedCycleTime().toSec();
  double  speed = 1.0;          // Speed to reach goal.
  double  cmdpos = feedback.position[0];
  double  cmdvel = 0.0;

  while (ros::ok) {

    if (!moving.data) {
      // Send over next target position
      stand();
      goal_publisher.publish(standing_queue);
      standing_queue.data.clear();
    } 

    ros::spinOnce();
    loop_rate.sleep();

  }

}
