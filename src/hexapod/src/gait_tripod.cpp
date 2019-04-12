#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "stdint.h"
#include "math.h"
#include <visualization_msgs/Marker.h>
#include <vector>
#include <algorithm>

sensor_msgs::JointState rviz_command_msg;

std_msgs::Bool done; 

using namespace std;

int motor1;
int motor2;
int motor3;
int i = 0;

vector<vector<float>> queue; 
vector<float> next_set;

void move_cb(geometry_msgs::Vector3 msg) {

  motor1 = msg.x;
  motor2 = msg.y;
  motor3 = msg.z;
  
  vector<float> array1, array2, array3, array4; 
  for (int j = 0 ; i < 3; i ++) {
    array1.push_back(5 * M_PI / 180);
    array2.push_back(14 * M_PI / 180);
    array3.push_back(-5 * M_PI/180);
    array4.push_back(-14 * M_PI/180);
  }

  queue.push_back(array1); 
  queue.push_back(array2); 
  queue.push_back(array3);
  queue.push_back(array4);

  reverse(queue.begin(), queue.end());

  done.data = false;

}


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "gait");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  done.data = true; 
  ros::Subscriber move_subscriber = n.subscribe("move", 100, move_cb);

  ros::Publisher done_publisher = n.advertise<std_msgs::Bool>("move_done", 100);
  ros::Publisher rviz_command_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 100);


  rviz_command_msg.position.resize(18);
  rviz_command_msg.velocity.resize(18);
  rviz_command_msg.effort.resize(18);
  rviz_command_msg.name = {"Motor1_to_JointPlate21", "FemurMotor_to_LegUpper1", 
                           "TibiaMotor1_to_LegLower1", "Motor1_to_JointPlate22", 
                           "FemurMotor_to_LegUpper2", "TibiaMotor1_to_LegLower2", 
                           "Motor1_to_JointPlate23", "FemurMotor_to_LegUpper3", 
                           "TibiaMotor1_to_LegLower3", "Motor1_to_JointPlate24", 
                           "FemurMotor_to_LegUpper4", "TibiaMotor1_to_LegLower4", 
                           "Motor1_to_JointPlate25", "FemurMotor_to_LegUpper5", 
                           "TibiaMotor1_to_LegLower5", "Motor1_to_JointPlate26", 
                           "FemurMotor_to_LegUpper6", "TibiaMotor1_to_LegLower6"};

  while(ros::ok())
  { 
    // Execute tripod walking 
    if (queue.empty()) {
      i = 0;
      done.data = true;
      done_publisher.publish(done);
    } else {
      next_set = queue.back();

      rviz_command_msg.position[motor1 * 3 + (i % 3)] = next_set[0]; 
      rviz_command_msg.position[motor2 * 3 + (i % 3)] = next_set[1]; 
      rviz_command_msg.position[motor3 * 3 + (i % 3)] = next_set[2]; 
      
      queue.pop_back();
      i++;
      rviz_command_publisher.publish(rviz_command_msg);
      done.data = false;
      done_publisher.publish(done);

    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
