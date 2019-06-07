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

hexapod::InstructionQueue traj_queue;
hexapod::Instruction standing_pos;
hexapod::Instruction current_pos;
hexapod::Instruction walking_pos;
int leg_components = 18;
//int leg_components = 6;

sensor_msgs::JointState feedback;       // The actuator feedback struccture
volatile bool           getGoal;
volatile int            feedbackvalid = 0;

double startTime, currTime;

double pos1 = M_PI / 2;
double pos2 = M_PI / 2;

double init_hip_pos = 0.5;

//double eff1 = M_PI / 2;
int state = 0;

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

  //ROS_INFO("new goal callback: %d", getGoal);
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


void walk()
{
  ROS_INFO("walk_called");

  int x = 2;
  double add = 5.0;
  for (int i = 0; i < 4; i++)
  {
    walking_pos.positions.clear();
    walking_pos.velocities.clear();
    walking_pos.efforts.clear();
    // 2 by 2 by 2 gait
    if (i == 0)
    {
      for (int j = 0; j < leg_components; j++)
      {
        walking_pos.velocities.push_back(NULL);
        if (j == 1)
        {
          walking_pos.positions.push_back(0.7);
          walking_pos.efforts.push_back(-5.);
        }
        else if (j == 16)
        {
          walking_pos.positions.push_back(-0.7);
          walking_pos.efforts.push_back(5.);
        }
        else
        {
          walking_pos.positions.push_back(feedback.position[j]);
          walking_pos.efforts.push_back(feedback.effort[j]);
        }
      }
    }
    else if (i == 1)
    {
      for (int j = 0; j < leg_components; j++)
      {
        if (j == 0 || j == 1)
          walking_pos.velocities.push_back(2.0);
        else
          walking_pos.velocities.push_back(NULL);

        if (j == 1)
        {
          walking_pos.positions.push_back(pos1);
          walking_pos.efforts.push_back(-5.);
        }
        else if (j == 16)
        {
          walking_pos.positions.push_back(-pos1);
          walking_pos.efforts.push_back(5.);
        }
        else if (j == 0)
        {
          // hip 1
          walking_pos.positions.push_back(init_hip_pos + 0.3);
          walking_pos.efforts.push_back(-3.);
        }
        else if (j == 3)
        {
          // hip 2
          walking_pos.positions.push_back(init_hip_pos - 0.3);
          walking_pos.efforts.push_back(1.);
        }
        else if (j == 6)
        {
          // hip 3
          //walking_pos.positions.push_back(-0.6);
          walking_pos.positions.push_back(0.25 - 0.3);
          walking_pos.efforts.push_back(-1.);
        }
        else if (j == 9)
        {
          // hip 4
          //walking_pos.positions.push_back(0.3);
          walking_pos.positions.push_back(-0.25 + 0.3);
          walking_pos.efforts.push_back(1.5);
        }
        else if (j == 12)
        {
          // hip 5
          walking_pos.positions.push_back(-init_hip_pos - 0.3);
          walking_pos.efforts.push_back(-1.);
        }
        else if (j == 15)
        {
          // hip 6
          walking_pos.positions.push_back(init_hip_pos - 0.3);
          walking_pos.efforts.push_back(3.);
        }
        else if (j == 2 || j == 17)
        {
          // tibias 1 and 6
          walking_pos.positions.push_back(0.0);
          walking_pos.efforts.push_back(0.);
        }
        else if (j == 4 || j == 10)
        {
          // femurs 2 and 4
          walking_pos.positions.push_back(feedback.position[j]);
          walking_pos.efforts.push_back(feedback.effort[j] + add);
        }
        else if (j == 7)
        {
          // femur 3
          walking_pos.positions.push_back(feedback.position[j]);
          walking_pos.efforts.push_back(feedback.effort[j] + add);
        }
        else if (j == 13) {
          // femur 5
          walking_pos.positions.push_back(feedback.position[j]);
          walking_pos.efforts.push_back(feedback.effort[j] + add);
        }
        else
        {
          walking_pos.positions.push_back(feedback.position[j]);
          walking_pos.efforts.push_back(feedback.effort[j]);
        }
      }
    }
    else if (i == 2)
    {
      for (int j = 0; j < leg_components; j++)
      {
        walking_pos.velocities.push_back(NULL);
        if (j == 7)
        {
          walking_pos.positions.push_back(0.7);
          walking_pos.efforts.push_back(-5.);
        }
        else if (j == 10)
        {
          walking_pos.positions.push_back(-0.7);
          walking_pos.efforts.push_back(5.);
        }
        else
        {
          walking_pos.positions.push_back(feedback.position[j]);
          walking_pos.efforts.push_back(feedback.effort[j]);
        }
      }
    }
    else if (i == 3)
    {
      for (int j = 0; j < leg_components; j++)
      {
        //if (j == 0 || j == 1)
        if (false)
          walking_pos.velocities.push_back(2.0);
        else
          walking_pos.velocities.push_back(NULL);

        if (j == 1)
        {
          walking_pos.positions.push_back(pos1);
          walking_pos.efforts.push_back(-5.);
        }
        else if (j == 16)
        {
          walking_pos.positions.push_back(-pos1);
          walking_pos.efforts.push_back(5.);
        }
        else if (j == 0)
        {
          // hip 1
          walking_pos.positions.push_back(init_hip_pos + 0.0);
          walking_pos.efforts.push_back(-3.);
        }
        else if (j == 3)
        {
          // hip 2
          walking_pos.positions.push_back(init_hip_pos - 0.6);
          walking_pos.efforts.push_back(1.);
        }
        else if (j == 6)
        {
          // hip 3
          //walking_pos.positions.push_back(-0.6);
          walking_pos.positions.push_back(0.25 - 0.3);
          walking_pos.efforts.push_back(-1.);
        }
        else if (j == 9)
        {
          // hip 4
          //walking_pos.positions.push_back(0.3);
          walking_pos.positions.push_back(-0.25 + 0.3);
          walking_pos.efforts.push_back(1.5);
        }
        else if (j == 12)
        {
          // hip 5
          walking_pos.positions.push_back(-init_hip_pos - 0.6);
          walking_pos.efforts.push_back(-1.);
        }
        else if (j == 15)
        {
          // hip 6
          walking_pos.positions.push_back(init_hip_pos - 0.0);
          walking_pos.efforts.push_back(3.);
        }
        else if (j == 2 || j == 17)
        {
          // tibias 1 and 6
          walking_pos.positions.push_back(0.0);
          walking_pos.efforts.push_back(0.);
        }
        else if (j == 4)
        {
          // femur 2
          walking_pos.positions.push_back(feedback.position[j]);
          walking_pos.efforts.push_back(feedback.effort[j] + add);
        }
        else if (j == 7)
        {
          // femur 3
          walking_pos.positions.push_back(pos1);
          walking_pos.efforts.push_back(-5.0);
        }
        else if (j == 10)
        {
          // femur 4
          walking_pos.positions.push_back(-pos1);
          walking_pos.efforts.push_back(5.0);
        }
        else if (j == 13) {
          // femur 5
          walking_pos.positions.push_back(feedback.position[j]);
          walking_pos.efforts.push_back(feedback.effort[j] + add);
        }
        else
        {
          walking_pos.positions.push_back(feedback.position[j]);
          walking_pos.efforts.push_back(feedback.effort[j]);
        }
      }
    }
 

    walking_pos.state = 3 + i;
    traj_queue.data.push_back(walking_pos);
  }

  reverse(traj_queue.data.begin(), traj_queue.data.end());


  //ROS_INFO("TRAJ 1: %f", traj_queue.data[0].positions[1]);



  /*
  for (int i = 0; i < 1; i++)
  {
    walking_pos.positions.clear();
    walking_pos.efforts.clear();
    if (i == 0)
    {
      for (int j = 0; j < leg_components; j++)
      {
        if (j == 10 || j == 1 || j == 13) {// || j == 10 || j == 13) { 
          walking_pos.positions.push_back(0.);
          if (feedback.position[j] > 0) {
            walking_pos.efforts.push_back(-5.);
          } else { 
            walking_pos.efforts.push_back(5.);
          }
        }
        else
        {
          walking_pos.positions.push_back(feedback.position[j]);
          if (j % 3 != 0) { 
            if (feedback.effort[j] > 0) { 
              if (j == 4) { 
                walking_pos.efforts.push_back(feedback.effort[j]+6);
              } else if (j == 16) {
                walking_pos.efforts.push_back(feedback.effort[j]+6);
              } else { 
                walking_pos.efforts.push_back(feedback.effort[j]+3);
              }
            } else { 
              walking_pos.efforts.push_back(feedback.effort[j]-3);
            }
          } else if (j == 5) { 
             if (feedback.effort[j] > 0) {
              walking_pos.efforts.push_back(feedback.effort[j]-2);
             } else { 
              walking_pos.efforts.push_back(feedback.effort[j]+2);
             }
          } else if (j == 8) { 
             if (feedback.effort[j] > 0) {
              walking_pos.efforts.push_back(feedback.effort[j]-2);
             } else { 
              walking_pos.efforts.push_back(feedback.effort[j]+2);
             }
          } else if (j == 17) { 
             if (feedback.effort[j] > 0) {
              walking_pos.efforts.push_back(feedback.effort[j]-2);
             } else { 
              walking_pos.efforts.push_back(feedback.effort[j]+2);
             }
          } else if (j == 3)  { 
            walking_pos.efforts.push_back(-1);
          }  else if (j == 6) { 
            walking_pos.efforts.push_back(1);
          } else { 
            walking_pos.efforts.push_back(1);
          }
        }
      }
    }
  }
  //*/
} 


void stand()
{
  double effortFemurDown, effortFemurDownFinal, effortTibiaDown;
  effortFemurDown = 8;
  effortFemurDownFinal = 6;
  effortTibiaDown = 3;
  for (int i = 0; i < 3; i++)
  {
    standing_pos.efforts.clear();
    standing_pos.positions.clear();

    // Create a queue of standing positions.
    if (i == 0)
    {
      for (int j = 0; j < leg_components; j++)
      {
        switch (j) { 
          case 0: 
            standing_pos.positions.push_back(.5); 
            break;
          case 15: 
            standing_pos.positions.push_back(.5); 
            break;
          case 6:
            standing_pos.positions.push_back(0.25);
            break;
          case 9:
            standing_pos.positions.push_back(-0.25);
            break;
          case 3: 
            standing_pos.positions.push_back(-.5);
            break;
          case 12: 
            standing_pos.positions.push_back(-.5);
            break;
          default:
            standing_pos.positions.push_back(0);
        } 
      }
      standing_pos.efforts.push_back(0.);
      standing_pos.velocities.push_back(NULL);
      standing_pos.state = 0;
    }
    else if (i == 1)
    {
      for (int j = 0; j < leg_components; j++)
      {
        standing_pos.velocities.push_back(NULL);
        if (j % 3 == 1)
        {
          if (j == 1 || j == 13)
          {
            standing_pos.positions.push_back(pos1);
            standing_pos.efforts.push_back(effortFemurDown);
          }
          else if (j == 4 || j == 16)
          {
            standing_pos.positions.push_back(-pos1);
            standing_pos.efforts.push_back(-effortFemurDown);
          }
          else if (j == 7)
          {
            standing_pos.positions.push_back(pos1);
            standing_pos.efforts.push_back(effortFemurDown - 2);
          }
          else if (j == 10) 
          {
            standing_pos.positions.push_back(-pos1);
            standing_pos.efforts.push_back(-effortFemurDown + 2);
          }
        }
        else if (j % 3 == 2)
        {
          if (j % 2 == 0)
          {
            standing_pos.efforts.push_back(-effortTibiaDown);
            standing_pos.positions.push_back(0.);
          }
          else
          {
            standing_pos.efforts.push_back(effortTibiaDown);
            standing_pos.positions.push_back(0.);
          }
        }
        else
        {
          switch (j) { 
            case 0: 
              standing_pos.positions.push_back(.5); 
              break;
            case 15: 
              standing_pos.positions.push_back(.5); 
              break;
            case 6:
              standing_pos.positions.push_back(0.25);
              break;
            case 9:
              standing_pos.positions.push_back(-0.25);
              break;
            case 3: 
              standing_pos.positions.push_back(-.5);
              break;
            case 12: 
              standing_pos.positions.push_back(-.5);
              break;
            default: 
              standing_pos.positions.push_back(0);
          }
          standing_pos.efforts.push_back(0.);
        }

        standing_pos.state = 1;
      }
    }
    else
    {
      for (int j = 0; j < leg_components; j++)
      {
        standing_pos.velocities.push_back(NULL);
        if (j % 3 == 1)
        {
          if (j == 1)
          {
            standing_pos.positions.push_back(pos2 - .2);
            standing_pos.efforts.push_back(effortFemurDownFinal-2);
          }
          else if (j == 4)
          {
            standing_pos.positions.push_back(-pos2 + .2);
            standing_pos.efforts.push_back(-effortFemurDownFinal+2);
          }
          else if (j == 7)
          {
            standing_pos.positions.push_back(pos1 - .1);
            standing_pos.efforts.push_back(effortFemurDownFinal-1);
          }
          else if (j == 13)
          {
            standing_pos.positions.push_back(pos2);
            standing_pos.efforts.push_back(effortFemurDownFinal);
          } 
          else if (j == 16)
          {
            standing_pos.positions.push_back(-pos2);
            standing_pos.efforts.push_back(-effortFemurDownFinal);
          }
          else
          {
            // j == 10
            standing_pos.positions.push_back(-pos1 + .1);
            standing_pos.efforts.push_back(-effortFemurDownFinal+1);
          }
        }
        else if (j % 3 == 2)
        {
          if (j == 2)
          {
            standing_pos.positions.push_back(.1);
            standing_pos.efforts.push_back(-effortTibiaDown+3);
          } 
          else if (j == 5) { 
            standing_pos.positions.push_back(-.1);
            standing_pos.efforts.push_back(effortTibiaDown-3);
          } else if (j == 8) { 
            standing_pos.positions.push_back(.1);
            standing_pos.efforts.push_back(-effortTibiaDown+2);
          } else if (j == 11) { 
            standing_pos.positions.push_back(-.1);
            standing_pos.efforts.push_back(effortTibiaDown-1);
          } else if (j == 14) { 
            standing_pos.positions.push_back(.1);
            standing_pos.efforts.push_back(-effortTibiaDown);
          } else if (j == 17) { 
            standing_pos.positions.push_back(-.1);
            standing_pos.efforts.push_back(effortTibiaDown-1);
          }
        }
        else
        {
          switch (j) { 
            case 0: 
              standing_pos.positions.push_back(.5); 
              break;
            case 15: 
              standing_pos.positions.push_back(.5); 
              break;
            case 6:
              standing_pos.positions.push_back(0.25);
              break;
            case 9:
              standing_pos.positions.push_back(-0.25);
              break;
            case 3: 
              standing_pos.positions.push_back(-.5);
              break;
            case 12: 
              standing_pos.positions.push_back(-.5);
              break;
            default: 
              standing_pos.positions.push_back(0);
          }
          standing_pos.efforts.push_back(0.);
        }
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

    for (int w = 0 ; w < leg_components; w ++) { 
      ROS_INFO("testing %d: %f", w, standing_pos.positions[w]);
    } 
    traj_queue.data.push_back(standing_pos);
  }

  reverse(traj_queue.data.begin(), traj_queue.data.end());

  ROS_INFO("done");
  ROS_INFO("stand3: %zd", traj_queue.data.size());
}



int main(int argc, char **argv)
{ 
  // Initialize the basic ROS node, run at 200Hz.
  ros::init(argc, argv, "tripod_brain");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);

  traj_queue.data.reserve(leg_components);

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
        switch (state) { 
          case 0: 
            stand();
            break; 
          case 1: 
            walk();
            ROS_INFO("Queue Size: %zd", traj_queue.data.size());
            break;
        }
        go = true;
      }
  
      if (!published)
      {
        /*
        ROS_INFO("POS0: %f", traj_queue.data[0].positions[1]);
        ROS_INFO("POS1: %f", traj_queue.data[1].positions[1]);
        ROS_INFO("POS2: %f", traj_queue.data[2].positions[1]);
        ROS_INFO("size before: %zd", traj_queue.data.size());
        */
        current_pos = traj_queue.data.back();
        traj_queue.data.pop_back(); 
        published = true;
        ROS_INFO("published: %f", current_pos.positions[1]);
        ROS_INFO("size after: %zd", traj_queue.data.size());
        //goal_publisher.publish(current_pos);
      }
      else if (getGoal)
      {
        startTime = ros::Time::now().toSec();

        ROS_INFO("size get before: %zd", traj_queue.data.size());
        current_pos = traj_queue.data.back();
        traj_queue.data.pop_back();
        getGoal = false;
        //goal_publisher.publish(current_pos);
        ROS_INFO("published get: %f", current_pos.positions[10]);

        ROS_INFO("size get after: %zd", traj_queue.data.size());
        if (traj_queue.data.size() == 0)
        {
          state = 1;
          go = false;
        }
      }
      ///*
      if (current_pos.header != "NULL")
        goal_publisher.publish(current_pos);
      //*/
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
