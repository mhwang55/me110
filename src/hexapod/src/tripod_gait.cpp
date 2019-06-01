#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "hexapod/Instruction.h"
#include "sensor_msgs/JointState.h"
#include "stdint.h"
#include "math.h"
#include <visualization_msgs/Marker.h>
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/FeedbackMsg.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/SizeSrv.h"
#include <vector>
#include <algorithm>

sensor_msgs::JointState rviz_command_msg;
std_msgs::Bool haveGoal;

using namespace std;
using namespace hebiros;

//int leg_components = 18;
int leg_components = 6;
vector<vector<float>> queue;
vector<float> next_set;
vector<double> goalPositions(leg_components);        // The goal position
vector<double> goalEfforts(leg_components);     // The efforts
double d = 10;
// max seconds to move between points
double moving_time = 5;
std_msgs::Bool moving;
bool newGoal;
bool standing = false;
double current_time, timeSince;
string status;
volatile int feedbackvalid = 0;
//hexapod::InstructionQueue positionQueue;

/*
**   Position Subscriber Callback
*/
void goalCallback(sensor_msgs::JointState data)
{
  //ROS_INFO("Goal Callback");
  goalPositions = data.position;
  goalEfforts = data.effort;
  feedbackvalid = 1;
}

int main(int argc, char **argv)
{
  // Initialize the basic ROS node, run at 200Hz.
  ros::init(argc, argv, "tripod_gait");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);
  
  current_time = ros::Time::now().toSec(); 
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
  add_group_srv.request.names = {
                                 ///*
                                 "hip1", "femur1", "tibia1",
                                 "hip2", "femur2", "tibia2",
                                 "hip3", "femur3", "tibia3",
                                 "hip4", "femur4", "tibia4",
                                 "hip5", "femur5", "tibia5",
                                 "hip6", "femur6", "tibia6"
                                 //*/
                                };

  add_group_srv.request.families = {
                                    ///*
                                    "leg1", "leg1", "leg1",
                                    "leg2", "leg2", "leg2", 
                                    "leg3", "leg3", "leg3",
                                    "leg4", "leg4", "leg4",
                                    "leg5", "leg5", "leg5",
                                    "leg6", "leg6", "leg6"
                                    //*/
                                   };

  // Repeatedly call the service until it succeeds.
  while(!add_group_client.call(add_group_srv)) ;

  // Check the size of this group.  This has an output argument.
  ros::ServiceClient size_client = n.serviceClient<SizeSrv>("/hebiros/"+group_name+"/size");
  SizeSrv size_srv;
  size_client.call(size_srv);
  ROS_INFO("%s has been created and has size %d", group_name.c_str(), size_srv.response.size);

  // Create a subscriber to listen for a goal pos.
  ros::Subscriber goalSubscriber = n.subscribe("go_to_goal", 200, goalCallback);

  // Create publisher to send moving commands. 
  //ros::Publisher moving_publisher = n.advertise<std_msgs::Bool>("/moving", 100);

  // Create a publisher to send commands to the actuator group.
  ros::Publisher command_publisher = n.advertise<sensor_msgs::JointState>("/hebiros/"+group_name+"/command/joint_state", 100);

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
  while (!feedbackvalid)  {
      ros::spinOnce();
      loop_rate.sleep();
  }
  ROS_INFO("Initial feedback received");

  // Prep the servo loop.
  double  dt = loop_rate.expectedCycleTime().toSec();
  double  speed = 1.0;          // Speed to reach goal.
  bool    nextStance = false;

  while(ros::ok())
  {
    //while (timeSince < moving_time && !goalpos.empty() && !nextStance)
    if (true)
    {
      // Keep moving towards the new point using min-jerk trajectory
      for (int i = 0; i < leg_components; i++)
      {
        command_msg.position[i] = goalPositions[i];
        command_msg.effort[i] = goalEfforts[i];

        //ROS_INFO("****CMD: %f", command_msg.position[1]);
      }

      command_publisher.publish(command_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
