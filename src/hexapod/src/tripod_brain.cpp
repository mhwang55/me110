#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "stdint.h"
#include "math.h"
#include <visualization_msgs/Marker.h>
#include "hexapod/Instruction.h"
#include "hexapod/InstructionQueue.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "geometry_msgs/Polygon.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/SizeSrv.h"
#include <vector>
#include <algorithm>

using namespace std;
using namespace hebiros;

std_msgs::Bool moving;
std_msgs::Bool alive;

hexapod::InstructionQueue standing_queue;
hexapod::Instruction standing_pos;
hexapod::Instruction current_pos;
//int leg_components = 18;
int leg_components = 6;

sensor_msgs::JointState feedback;       // The actuator feedback struccture
volatile bool           getGoal;
volatile int            feedbackvalid = 0;

double startTime, currTime;

void movingCallback(std_msgs::Bool data)
{
  moving = data;
  //ROS_INFO("moving callback: %d", moving.data);
}

void getGoalCallback(std_msgs::Bool data)
{
  currTime = ros::Time::now().toSec();
  if (data.data == true && currTime - startTime > 1)
    getGoal = data.data;
  else
    getGoal = false;

  ROS_INFO("new goal callback: %d", getGoal);
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

void stand()
{
  double effortFemurDown, effortFemurDownFinal, effortTibiaDown;
  effortFemurDown = 8;
  effortFemurDownFinal = 6;
  effortTibiaDown = 2.8;
  for (int i = 0; i < 3; i++)
  {
    standing_pos.positions.clear();
    standing_pos.efforts.clear();

    // Create a queue of standing positions.
    if (i == 0)
    //if (true)
    {
      for (int j = 0; j < leg_components; j++)
      {
        standing_pos.positions.push_back(0.);
        /*
        if (j == 0)
        {
          standing_pos.positions.push_back(0.3);
        }
        else if (j == 3)
        {
          standing_pos.positions.push_back(-0.3);
        }
        else if (j == 12)
        {
          standing_pos.positions.push_back(-0.3);
        }
        else if (j == 15)
        {
          standing_pos.positions.push_back(0.3);
        }
        else
        {
          standing_pos.positions.push_back(0.);
        }
        //*/
      }

      standing_pos.state = 0;
    }
    /*
    else if (i == 1)
    {
      for (int j = 0; j < leg_components; j++)
      {
        standing_pos.positions.push_back(0.);
      }
    }
    //*/
 
    //else if (false)
    else if (i == 1)
    {
      for (int j = 0; j < leg_components; j++)
      {
        if (j % 3 == 1)
        {
          if (j == 16)
          {
            standing_pos.positions.push_back(-M_PI / 2);
            standing_pos.efforts.push_back(-effortFemurDown);
          }
          else if (j == 1)
          {
            standing_pos.positions.push_back(M_PI / 2);
            standing_pos.efforts.push_back(11);
          }
          else if (j == 4)
          {
            standing_pos.positions.push_back(-M_PI / 2);
            standing_pos.efforts.push_back(-10);
          }
          else if (j % 2 == 0)
          {
            standing_pos.positions.push_back(-M_PI / 2);
            standing_pos.efforts.push_back(-effortFemurDown - 1);
          }
          else
          {
            standing_pos.positions.push_back(M_PI / 2);
            standing_pos.efforts.push_back(effortFemurDown);
          }
        }
        else if (j % 3 == 2)
        {
          if (j % 2 == 0)
          {
            standing_pos.positions.push_back(0.);
            standing_pos.efforts.push_back(-0.2);
          }
          else
          {
            standing_pos.positions.push_back(0.);
            standing_pos.efforts.push_back(0.2);
          }
        }
        else
        {
          standing_pos.positions.push_back(0.);
          standing_pos.efforts.push_back(0.);
          /*
          if (j == 0)
          {
            standing_pos.positions.push_back(0.3);
            standing_pos.efforts.push_back(0.);
          }
          else if (j == 3)
          {
            standing_pos.positions.push_back(-0.3);
            standing_pos.efforts.push_back(-0.);
          }
          else if (j == 12)
          {
            standing_pos.positions.push_back(-0.3);
            standing_pos.efforts.push_back(-0.);
          }
          else if (j == 15)
          {
            standing_pos.positions.push_back(0.3);
            standing_pos.efforts.push_back(0.);
          }
          else
          {
            standing_pos.positions.push_back(0.);
            standing_pos.efforts.push_back(0.);
          }
          //*/
        }
      }

      standing_pos.state = 1;
    }
    else
    //else if (false)
    {
      for (int j = 0; j < leg_components; j++)
      {
        if (j % 3 == 1)
        {
          if (j == 16)
          {
            standing_pos.positions.push_back(-M_PI / 2);
            standing_pos.efforts.push_back(-effortFemurDownFinal);
          }
          else if (j == 1)
          {
            standing_pos.positions.push_back(M_PI / 2);
            standing_pos.efforts.push_back(effortFemurDownFinal + 3);
          }
          else if (j == 4)
          {
            standing_pos.positions.push_back(-M_PI / 2);
            standing_pos.efforts.push_back(-effortFemurDownFinal - 2);
          }
          else if (j % 2 == 0)
          {
            standing_pos.positions.push_back(-M_PI / 2);
            standing_pos.efforts.push_back(-effortFemurDownFinal - 1);
          }
          else
          {
            standing_pos.positions.push_back(M_PI / 2);
            standing_pos.efforts.push_back(effortFemurDownFinal);
          }
 
          /*
          if (j % 2 == 0)
          {
            standing_pos.positions.push_back(-M_PI / 2);
            standing_pos.efforts.push_back(-effortFemurDown);
          }
          else
          {
            standing_pos.positions.push_back(M_PI / 2);
            standing_pos.efforts.push_back(effortFemurDown);
          }
          //*/
        }
        else if (j % 3 == 2)
        {
          if (j % 2 == 0)
          {
            standing_pos.positions.push_back(0.);
            standing_pos.efforts.push_back(-effortTibiaDown + 0.0);
          }
          else
          {
            standing_pos.positions.push_back(0.);
            standing_pos.efforts.push_back(effortTibiaDown - 0.2);
          }
        }
        else
        {
          standing_pos.positions.push_back(0.);
          standing_pos.efforts.push_back(0.);
          /*
          if (j == 0)
          {
            standing_pos.positions.push_back(0.3);
            standing_pos.efforts.push_back(0.);
          }
          else if (j == 3)
          {
            standing_pos.positions.push_back(-0.3);
            standing_pos.efforts.push_back(-0.);
          }
          else if (j == 12)
          {
            standing_pos.positions.push_back(-0.3);
            standing_pos.efforts.push_back(-0.);
          }
          else if (j == 15)
          {
            standing_pos.positions.push_back(0.3);
            standing_pos.efforts.push_back(0.);
          }
          else
          {
            standing_pos.positions.push_back(0.);
            standing_pos.efforts.push_back(0.);
          }
          //*/
        }
        /*
        else
        {
          standing_pos.positions.push_back(0.);
          standing_pos.efforts.push_back(0.);
        }
        */
      }

      standing_pos.state = 2;
    }

    if (i == 2)
    {
      standing_pos.header = "final";
    }
    else
    {
      standing_pos.header = "standing";
    }

    standing_queue.data.push_back(standing_pos);
  }

  reverse(standing_queue.data.begin(), standing_queue.data.end());

  ROS_INFO("done");
  ROS_INFO("stand3: %zd", standing_queue.data.size());
  /*
  for (int i = 0; i < sizeof(standing_queue.data) / sizeof(standing_queue.data[0]); i++)
  {
    ROS_INFO("stand: %f", standing_queue.data[i].positions[1]);
  }
  */
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
  add_group_srv.request.names = {
                                 "hip1", "femur1", "tibia1",
                                 "hip2", "femur2", "tibia2",
                                 "hip3", "femur3", "tibia3",
                                 "hip4", "femur4", "tibia4",
                                 "hip5", "femur5", "tibia5",
                                 "hip6", "femur6", "tibia6"
                                };

  add_group_srv.request.families = {
                                    "leg1", "leg1", "leg1",
                                    "leg2", "leg2", "leg2", 
                                    "leg3", "leg3", "leg3",
                                    "leg4", "leg4", "leg4",
                                    "leg5", "leg5", "leg5",
                                    "leg6", "leg6", "leg6"
                                   };

  // Repeatedly call the service until it succeeds.
  while(!add_group_client.call(add_group_srv));

  // Check the size of this group.  This has an output argument.
  ros::ServiceClient size_client = n.serviceClient<SizeSrv>("/hebiros/"+group_name+"/size");
  SizeSrv size_srv;
  size_client.call(size_srv);
  ROS_INFO("%s has been created and has size %d", group_name.c_str(), size_srv.response.size);

  // Subscriber to listen to whether the robot is moving.
  ros::Subscriber movingSubscriber = n.subscribe("/moving", 100, movingCallback);

  ros::Subscriber getGoalSubscriber = n.subscribe("/getGoal", 100, getGoalCallback);


  // Create a subscriber to receive feedback from the actuator group.
  ros::Subscriber feedback_subscriber = n.subscribe("/hebiros/"+group_name+"/feedback/joint_state", 100, feedbackCallback);

  feedback.position.reserve(leg_components);
  feedback.velocity.reserve(leg_components);
  feedback.effort.reserve(leg_components);

  /*
  initial_pos.position.reserve(leg_components);
  initial_pos.velocity.reserve(leg_components);
  initial_pos.effort.reserve(leg_components);*/

  // Create a publisher to send commands to the actuator group.
  // ros::Publisher command_publisher = n.advertise<sensor_msgs::JointState>("/hebiros/"+group_name+"/command/joint_state", 100);

  // GoalPos publisher
  ros::Publisher goal_publisher = n.advertise<hexapod::Instruction>("/goal", 100);

  ros::Publisher alivePublisher = n.advertise<std_msgs::Bool>("/alive", 100);

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
  while (!feedbackvalid)
  {
    alive.data = false;
    alivePublisher.publish(alive);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Brain initial feedback received");

  moving.data = false;

  // Prep the servo loop.
  double  dt = loop_rate.expectedCycleTime().toSec();
  double  speed = 1.0;          // Speed to reach goal.
  bool go = false;
  bool published = false;

  current_pos.header = "NULL";
  getGoal = false;

  startTime = ros::Time::now().toSec();

  while (ros::ok)
  {
    alive.data = true;
    alivePublisher.publish(alive);
    if (moving.data)
    {
      // Send over next target position
      if (!go)
      {
        stand();
        go = true;
      }
  
      if (!published)
      {
        ROS_INFO("size before: %zd", standing_queue.data.size());
        current_pos = standing_queue.data.back();
        standing_queue.data.pop_back(); 
        published = true;
        ROS_INFO("published: %f", current_pos.positions[1]);
        ROS_INFO("size after: %zd", standing_queue.data.size());
        //goal_publisher.publish(current_pos);
      }
      else if (getGoal)
      {
        //ROS_INFO("*************get goal");

        startTime = ros::Time::now().toSec();

        ROS_INFO("size get before: %zd", standing_queue.data.size());
        current_pos = standing_queue.data.back();
        standing_queue.data.pop_back(); 
        getGoal = false;
        //goal_publisher.publish(current_pos);
        ROS_INFO("published get: %f", current_pos.positions[1]);

        ROS_INFO("size get after: %zd", standing_queue.data.size());
      }
      ///*
      if (current_pos.header != "NULL")
        goal_publisher.publish(current_pos);
      //*/
    }


    /*
    ROS_INFO("Is moving? %d", moving.data);
    if (!moving.data)
    {
      // Send over next target position
      //if (standing_queue.data.empty())
      if (!go)
      {
        stand();
        go = true;
      }


      if (!published)
      {
        current_pos = standing_queue.data.back();
        standing_queue.data.pop_back(); 
        goal_publisher.publish(current_pos);
        published = true;
      }

      moving.data = true;
      ROS_INFO("********data positions: %f", current_pos.positions[1]);
    }
    if (getGoal.data)
    {
      published = false;
    }
    */

    ros::spinOnce();
    loop_rate.sleep();
  }
}
