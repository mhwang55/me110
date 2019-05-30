#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "stdint.h"
#include "math.h"
#include <visualization_msgs/Marker.h>
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/SizeSrv.h"
#include <vector>
#include <algorithm>

sensor_msgs::JointState rviz_command_msg;

using namespace std;
using namespace hebiros;

vector<vector<float>> queue; 
vector<float> next_set;
std_msgs::Float64MultiArray goalpos;        // The goal position
volatile int            feedbackvalid = 0;
sensor_msgs::JointState feedback;       // The actuator feedback struccture
sensor_msgs::JointState initial_pos;
double d = 5;
<<<<<<< HEAD
// max seconds to move between points
double moving_time = 5; 
std_msgs::Bool moving;
bool newGoal;
double current_time, timeSince; 
=======

float position (float initialPos, float desiredPos, int i)
{
  return initialPos + 
         (desiredPos - initialPos) * 
         (10*((i/10)*d*(i/10)*d*(i/10)*d) - 
         15*((i/10)*d)*((i/10)*d*(i/10)*d*(i/10)*d) + 
         6*((i/10)*d*(i/10)*d*(i/10)*d*(i/10)*d*(i/10)*d));
}

>>>>>>> ba106d279555a2d4c51cc477bdf7bbe7197777f8

/*
**   Feedback Subscriber Callback
*/
void feedbackCallback(sensor_msgs::JointState data)
{
  feedback = data;
  //ROS_INFO("FBK pos [%f]", feedback.position[0]);
  feedbackvalid = 1;
}

/*
**   Goal Subscriber Callback
*/
void goalCallback(const std_msgs::Float64MultiArray data)
{
  goalpos = data;
  newGoal = true;
}


int main(int argc, char **argv)
{ 
  // Initialize the basic ROS node, run at 200Hz.
  ros::init(argc, argv, "tripod_gait");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);
  moving.data = false;
  
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
  add_group_srv.request.names = {"hip1", "femur1", "tibia1",
                                 "hip2", "femur2", "tibia2",
                                 "hip3", "femur3", "tibia3",
                                 "hip4", "femur4", "tibia4",
                                 "hip5", "femur5", "tibia5",
                                 "hip6", "femur6", "tibia6"
                                };

  add_group_srv.request.families = {"leg1", "leg1", "leg1",
                                    "leg2", "leg2", "leg2",
                                    "leg3", "leg3", "leg3",
                                    "leg4", "leg4", "leg4",
                                    "leg5", "leg5", "leg5",
                                    "leg6", "leg6", "leg6"
                                   };

  // Repeatedly call the service until it succeeds.
  while(!add_group_client.call(add_group_srv)) ;

  // Check the size of this group.  This has an output argument.
  ros::ServiceClient size_client = n.serviceClient<SizeSrv>("/hebiros/"+group_name+"/size");
  SizeSrv size_srv;
  size_client.call(size_srv);
  ROS_INFO("%s has been created and has size %d", group_name.c_str(), size_srv.response.size);

  // Create a subscriber to listen for a goal pos.
  ros::Subscriber goalSubscriber = n.subscribe("goal", 100, goalCallback);

  int leg_components = 18;

  // Create a subscriber to receive feedback from the actuator group.
  ros::Subscriber feedback_subscriber = n.subscribe("/hebiros/"+group_name+"/feedback/joint_state", 100, feedbackCallback);

  feedback.position.reserve(leg_components);
  feedback.velocity.reserve(leg_components);
  feedback.effort.reserve(leg_components);

  initial_pos.position.reserve(leg_components);
  initial_pos.velocity.reserve(leg_components);
  initial_pos.effort.reserve(leg_components);

<<<<<<< HEAD
  // Create publisher to send moving commands. 
  ros::Publisher moving_publisher = n.advertise<std_msgs::Bool>("/moving", 100);
=======
  // Create publisher to send done commands. 
  //ros::Publisher done_publisher = n.advertise<sensor_msgs::Boolean>("")
>>>>>>> ba106d279555a2d4c51cc477bdf7bbe7197777f8

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

  moving_publisher.publish(moving);
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
  double  cmdpos = feedback.position[0];
  double  cmdvel = 0.0;
  double  rad1, rad2, effort1, effort2, effortTibiaLeft, effortTibiaRight;
  rad1 = M_PI/2;
  rad2 = -M_PI/2;
  ///*
  rad1 = 0.7;
  rad2 = -0.7;
  //*/

<<<<<<< HEAD
  while(ros::ok())  { 
    timeSince = ros::Time::now().toSec() - current_time;

    if (newGoal) { 
      // Got a new target
      initial_pos = feedback;
      newGoal = false;
      current_time = ros::Time::now().toSec(); 
      timeSince = 0.; 
      moving.data = true;
      moving_publisher.publish(moving);
    } 

    if (timeSince < moving_time && !goalpos.data.empty()) {
      // Keep moving towards the new point using min-jerk trajectory
      for (int i = 0; i < leg_components; i++) {
        // Test the first leg first.
        if (i == 0 || i == 1 || i == 2) { 
        command_msg.position[i] = initial_pos.position[i] + 
                                  (goalpos.data[i] - initial_pos.position[i]) * 
                                  (10*(timeSince/10)*moving_time*(timeSince/10)*
                                   moving_time*(timeSince/10) - 
                                   15*moving_time*(timeSince/10)*moving_time*(timeSince/10)*
                                   moving_time*(timeSince/10)*moving_time*(timeSince/10) + 
                                   6*moving_time*(timeSince/10)*moving_time*(timeSince/10)*
                                   moving_time*(timeSince/10)*moving_time*(timeSince/10)*
                                   moving_time*(timeSince/10)); 
        } else {
        command_msg.position[i] = feedback.position[i];
        }
      }

      timeSince = ros::Time::now().toSec() - current_time;
      moving.data = true;
      moving_publisher.publish(moving);
      command_publisher.publish(command_msg);
    } else { 
      // Ready to receive new position command
      moving.data = false; 
      moving_publisher.publish(moving);
    } 
=======
  effort1 = -1;
  effort2 = 1;
  ///*
  effortTibiaLeft = 0;
  effortTibiaRight = -0;
  //*/

  bool prep = false;

  while(ros::ok())
  {
    initial_pos = feedback;

    if (!prep)
    {
      for (int i=0; i<10; i++)
      {
        command_msg.position[0] = 0;
        command_msg.position[1] = position(initial_pos.position[1], rad1, i);
        command_msg.position[2] = 0;
        command_msg.effort[0] = 0;
        command_msg.effort[1] = effort1;
        command_msg.effort[2] = effortTibiaRight;
        command_msg.position[3] = 0;
        command_msg.position[4] = position(initial_pos.position[4], rad2, i);
        command_msg.position[5] = 0;
        command_msg.effort[3] = 0;
        command_msg.effort[4] = effort2;
        command_msg.effort[5] = effortTibiaLeft;
        command_msg.position[6] = 0;
        command_msg.position[7] = position(initial_pos.position[7], rad1, i);
        command_msg.position[8] = 0;
        command_msg.effort[6] = 0;
        command_msg.effort[7] = effort1;
        command_msg.effort[8] = effortTibiaRight;
        command_msg.position[9] = 0;
        command_msg.position[10] = position(initial_pos.position[10], rad2, i);
        command_msg.position[11] = 0;
        command_msg.effort[9] = 0;
        command_msg.effort[10] = effort2;
        command_msg.effort[11] = effortTibiaLeft;
        command_msg.position[12] = 0;
        command_msg.position[13] = position(initial_pos.position[13], rad1, i);
        command_msg.position[14] = 0;
        command_msg.effort[12] = 0;
        command_msg.effort[13] = effort1;
        command_msg.effort[14] = effortTibiaRight;
        command_msg.position[15] = 0;
        command_msg.position[16] = position(initial_pos.position[16], rad2, i);
        command_msg.position[17] = 0;
        command_msg.effort[15] = 0;
        command_msg.effort[16] = effort2;
        command_msg.effort[17] = effortTibiaLeft;
  
        command_publisher.publish(command_msg);
      }
    }
    else
    {
  rad1 = M_PI/2;
  rad2 = -M_PI/2;
      for (int i=0; i<10; i++)
      {
        command_msg.position[0] = 0;
        command_msg.position[1] = position(initial_pos.position[1], rad1, i);
        command_msg.position[2] = 0;
        command_msg.effort[0] = 0;
        command_msg.effort[1] = 8;
        command_msg.effort[2] = -5;
        command_msg.position[3] = 0;
        command_msg.position[4] = position(initial_pos.position[4], rad2, i);
        command_msg.position[5] = 0;
        command_msg.effort[3] = 0;
        command_msg.effort[4] = -8;
        command_msg.effort[5] = 5;
        command_msg.position[6] = 0;
        command_msg.position[7] = position(initial_pos.position[7], rad1, i);
        command_msg.position[8] = 0;
        command_msg.effort[6] = 0;
        command_msg.effort[7] = 8;
        command_msg.effort[8] = -5;
        command_msg.position[9] = 0;
        command_msg.position[10] = position(initial_pos.position[10], rad2, i);
        command_msg.position[11] = 0;
        command_msg.effort[9] = 0;
        command_msg.effort[10] = -8;
        command_msg.effort[11] = 5;
        command_msg.position[12] = 0;
        command_msg.position[13] = position(initial_pos.position[13], rad1, i);
        command_msg.position[14] = 0;
        command_msg.effort[12] = 0;
        command_msg.effort[13] = 8;
        command_msg.effort[14] = -5;
        command_msg.position[15] = 0;
        command_msg.position[16] = position(initial_pos.position[16], rad2, i);
        command_msg.position[17] = 0;
        command_msg.effort[15] = 0;
        command_msg.effort[16] = -8;
        command_msg.effort[17] = 5;
  
        command_publisher.publish(command_msg);
      }
    }
>>>>>>> ba106d279555a2d4c51cc477bdf7bbe7197777f8

    ros::spinOnce();
    loop_rate.sleep();

  }


}
