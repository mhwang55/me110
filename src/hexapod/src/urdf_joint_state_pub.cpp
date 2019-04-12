#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // message declarations
    sensor_msgs::JointState joint_state;

    double tilt = 0;

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(16);
        joint_state.position.resize(16);
        joint_state.name[0] ="world_to_Base";
        joint_state.position[0] = tilt;
        joint_state.name[1] ="a";
        joint_state.position[1] = tilt;
        joint_state.name[2] ="a";
        joint_state.position[2] = tilt;
        joint_state.name[3] ="a";
        joint_state.position[3] = tilt;
        joint_state.name[4] ="a";
        joint_state.position[4] = tilt;
        joint_state.name[5] ="a";
        joint_state.position[5] = tilt;
        joint_state.name[6] ="a";
        joint_state.position[6] = tilt;
        joint_state.name[7] ="a";
        joint_state.position[7] = tilt;
        joint_state.name[8] ="a";
        joint_state.position[8] = tilt;
        joint_state.name[9] ="a";
        joint_state.position[9] = tilt;
        joint_state.name[10] ="a";
        joint_state.position[10] = tilt;
        joint_state.name[11] ="a";
        joint_state.position[11] = tilt;
        joint_state.name[12] ="a";
        joint_state.position[12] = tilt;
        joint_state.name[13] ="a";
        joint_state.position[13] = tilt;
        joint_state.name[14] ="a";
        joint_state.position[14] = tilt;
        joint_state.name[15] ="a";
        joint_state.position[15] = tilt;

        //send the joint state and transform
        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
