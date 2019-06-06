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

using namespace std;
using namespace hebiros;

int leg_components = 18;
//int leg_components = 6;
vector<vector<float>> queue;
vector<float> next_set;
vector<double> goalpos(leg_components);        // The goal position
vector<double> goaleffort(leg_components);     // The efforts
volatile int            feedbackvalid = 0;
sensor_msgs::JointState feedback;     // The actuator feedback structure
sensor_msgs::JointState initial_pos;

int currState = -1;
int prevState = -1;

// max seconds to move between points
double moving_time = 3;
double current_time, timeSince;
double d = 10;

std_msgs::Bool moving;
std_msgs::Bool haveNewGoal;
std_msgs::Bool getNewGoal;
bool haveGoal;
bool getGoal;
bool standing = false;
bool alive = false;

vector<double> effortDiffs;

string status;
//hexapod::InstructionQueue positionQueue;

/*
**   Feedback Subscriber Callback
*/
void feedbackCallback(sensor_msgs::JointState data)
{
  feedback = data;
  //ROS_INFO("FBK pos [%f]", feedback.position[1]);
  //ROS_INFO("data pos [%f]", data.position[1]);
  feedbackvalid = 1;
}

/*
**   Goal Subscriber Callback
*/
void goalCallback(const hexapod::Instruction data)
{
  //ROS_INFO("goal_talking");
  goalpos = data.positions;
  //ROS_INFO("Goal Position Callback Empty: %d", goalpos.empty());
  goaleffort = data.efforts;
  if (currState == prevState || currState == -1)
  {
    haveGoal = true;
    currState = data.state;
  }
  status = data.header;
}

void brainAliveCallback(std_msgs::Bool data)
{
  alive = data.data;
  //ROS_INFO("alive callback: %d", alive);
}

int main(int argc, char **argv)
{
  // Initialize the basic ROS node, run at 200Hz.
  ros::init(argc, argv, "tripod_control");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);
  goalpos.clear();
  
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
  ros::Subscriber goalSubscriber = n.subscribe("/goal", 100, goalCallback);

  ros::Subscriber aliveSubscriber = n.subscribe("/alive", 100, brainAliveCallback);

  // Create a subscriber to receive feedback from the actuator group.
  ros::Subscriber feedback_subscriber = n.subscribe("/hebiros/"+group_name+"/feedback/joint_state", 200, feedbackCallback);

  feedback.position.reserve(leg_components);
  feedback.velocity.reserve(leg_components);
  feedback.effort.reserve(leg_components);

  initial_pos.position.reserve(leg_components);
  initial_pos.velocity.reserve(leg_components);
  initial_pos.effort.reserve(leg_components);

  // Create publisher to send moving commands. 
  ros::Publisher moving_publisher = n.advertise<std_msgs::Bool>("/moving", 100);

  // Create a publisher to send commands to the actuator group.
  /*
  ros::Publisher command_publisher = n.advertise<sensor_msgs::JointState>("/hebiros/"+group_name+"/command/joint_state", 100);
  */

  // Publish whether we have the goal position or not
  ros::Publisher getGoalPublisher = n.advertise<std_msgs::Bool>("/getGoal", 100);

  ros::Publisher goalPublisher = n.advertise<sensor_msgs::JointState>("go_to_goal", 100);

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

  haveGoal = false;
  getGoal = false;

  // Wait until we have some feedback from the actuator.
  ROS_INFO("Waiting for initial feedback");
  while (!feedbackvalid)  {
      moving.data = false;
      moving_publisher.publish(moving);
      ros::spinOnce();
      loop_rate.sleep();
  }
  ROS_INFO("Control initial feedback received");


  ROS_INFO("Waiting for brain birth");
  while (!alive && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Brain alive");


  // Prep the servo loop.
  double  dt = loop_rate.expectedCycleTime().toSec();
  double  speed = 1.0;          // Speed to reach goal.
  bool    nextStance = false;
  current_time = ros::Time::now().toSec(); 

  for (int i = 0; i < leg_components; i++)
    effortDiffs.push_back(0.0);

  while(ros::ok())
  {
    moving.data = true;
    moving_publisher.publish(moving);
    timeSince = ros::Time::now().toSec() - current_time;

    if (haveGoal)
    {
      // Got a new target
      initial_pos = feedback;
      current_time = ros::Time::now().toSec();
      timeSince = 0;
      nextStance = false;
      haveGoal = false;
      for (int i = 0; i < leg_components; i++)
        effortDiffs[i] = 0.0;
    }

    if (timeSince < moving_time && !goalpos.empty())
    {
      //ROS_INFO("%f", ros::Time::now().toSec());
      int motor = 0;

      // Keep moving towards the new point using min-jerk trajectory
      for (int i = 0; i < leg_components; i++)
      {
        // Test the first leg first.

        if (true)
        //if (i < 3)
        {
          double givenPos, givenEffort;

          givenPos = initial_pos.position[i] + 
                     (goalpos[i] - initial_pos.position[i]) * 
                     (10 * (timeSince/moving_time) *
                     (timeSince/moving_time) *
                     (timeSince/moving_time) - 
                     15 * (timeSince/moving_time) *
                     (timeSince/moving_time) *
                     (timeSince/moving_time) *
                     (timeSince/moving_time) + 
                     6 * (timeSince/moving_time) *
                     (timeSince/moving_time) *
                     (timeSince/moving_time) *
                     (timeSince/moving_time) *
                     (timeSince/moving_time)); 

          givenEffort = initial_pos.effort[i] + 
                        (goaleffort[i] - initial_pos.effort[i]) * 
                        (10 * (timeSince/moving_time) *
                        (timeSince/moving_time) *
                        (timeSince/moving_time) - 
                        15 * (timeSince/moving_time) *
                        (timeSince/moving_time) *
                        (timeSince/moving_time) *
                        (timeSince/moving_time) + 
                        6 * (timeSince/moving_time) *
                        (timeSince/moving_time) *
                        (timeSince/moving_time) *
                        (timeSince/moving_time) *
                        (timeSince/moving_time));

          double p_front = .5;
          double p_middle = .3;
          double p_back = .2;

          if (i < 6)
          {
            if (abs(feedback.position[i] - givenPos) > 0.1)
              effortDiffs[i] = -p_front * (feedback.position[i] - givenPos);
            else
              effortDiffs[i] = 0;
          }
          else if (i > 5 && i < 12)
          {
            if (abs(feedback.position[i] - givenPos) > 0.1)
              effortDiffs[i] = -p_middle * (feedback.position[i] - givenPos);
            else
              effortDiffs[i] = 0;
          }
          else if (i > 11)
          {
            if (abs(feedback.position[i] - givenPos) > 0.1)
              effortDiffs[i] = -p_back * (feedback.position[i] - givenPos);
            else
              effortDiffs[i] = 0;
          }

          givenEffort += effortDiffs[i];
          command_msg.position[i] = givenPos;
          command_msg.effort[i] = givenEffort;

          int x = 7;
          if (i==x)// || i == 2)
          {
            ROS_INFO("GOAL Effort %d: %f", x, goaleffort[x]);
            ROS_INFO("CMD Effort %d: %f", x, command_msg.effort[x]);
            ROS_INFO("FBK Effort %d: %f", i, feedback.effort[i]);
            ROS_INFO("GOAL Position %d: %f", x, goalpos[x]);
            ROS_INFO("CMD Position %d: %f", x, command_msg.position[x]);
            ROS_INFO("FBK Position %d: %f", i, feedback.position[i]);
          }

          //command_msg.effort[2] = 10;
          /*if (i == 0) { 
            command_msg.effort[i] = 0;
          } else if (i == 1) { 
            command_msg.effort[i] = 15;
          } else if (i == 2) { 
            command_msg.effort[i] = 0; 
          }*/
        }
        else
        {
          command_msg.position[i] = feedback.position[i];
          command_msg.effort[i] = feedback.effort[i];
        }


        double error1 = 0.3,
               error2 = 0.3;
        if (abs(feedback.position[1] - goalpos[1]) < error1 && abs(feedback.position[4] - goalpos[4]) < error1 && abs(feedback.position[2] - goalpos[2]) < error2 && abs(feedback.position[5] - goalpos[5]) < error2)
        //if (abs(feedback.position[i] - goalpos[i]) < error1)
          motor = 2;
          //motor++;
      }

      //if (motor >= 18)
      if (motor >= 2)
      {
        //nextStance = true;
        getGoal = true;
        prevState = currState;
        motor = 0;
      }
      else
      {
        getGoal = false;
      }
      getNewGoal.data = getGoal;
      getGoalPublisher.publish(getNewGoal);
      motor = 0;

      timeSince = ros::Time::now().toSec() - current_time;
      goalPublisher.publish(command_msg);

    }
    else //if (status == "final" || currState == 2)
    {
      // Maintain previous position
      /*if (!goalpos.empty() && !goaleffort.empty())
      {
        //ROS_INFO("final in");
        
        command_msg.position[3] = feedback.position[3];
        command_msg.position[4] = feedback.position[4];
        command_msg.position[5] = feedback.position[5];
        command_msg.position[6] = feedback.position[6];
        command_msg.position[7] = feedback.position[7];
        command_msg.position[8] = feedback.position[8];
        command_msg.position[9] = feedback.position[9];
        command_msg.position[10] = feedback.position[10];
        command_msg.position[11] = feedback.position[11];
        command_msg.position[12] = feedback.position[12];
        command_msg.position[13] = feedback.position[13];
        command_msg.position[14] = feedback.position[14];
        command_msg.position[15] = feedback.position[15];
        command_msg.position[16] = feedback.position[16];
        command_msg.position[17] = feedback.position[17];
        
        //if (status == "final") {
        //  getGoal = false; 
        //} else {
        getGoal = true;
        //}
        getNewGoal.data = getGoal;
        getGoalPublisher.publish(getNewGoal);
      }*/
      //else
      //{
        ROS_INFO("final out");
        command_msg.position = feedback.position;
        command_msg.effort = feedback.effort;
        /*for (int i = 0 ; i < leg_components; i ++) { 
          ROS_INFO("feedback effort %d : %f", i, feedback.effort[i]);
        }*/ 
        goalPublisher.publish(command_msg);
        getGoal = true;
        getNewGoal.data = getGoal;
        getGoalPublisher.publish(getNewGoal);
      //} 

      /*for (int i = 0; i < leg_components; i++)
      {
        command_msg.position[i] = goalpos[i];
        //command_msg.effort[i] = goaleffort[i];
      }

      goalPublisher.publish(command_msg);*/

    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
