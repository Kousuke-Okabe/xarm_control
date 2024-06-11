#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Engen/Dense>

#define DOF 6
#define Fs 10;

Eigen::Vector6d present_angle;
Eigen::Vector6d present_torque;

// Callback function of the Topic communication with joint_states
void Subscribe_JointStates(const sensor_msgs::JointState &state){
    // ROS_INFO("Joint Effort : %f", state.effort[1]);
    for(int i=0; i<DOF; i++){]
        present_angle(i) = (double)state.position[i];
        present_torque(i) = (double)state.effort[i];
    }
}

int main(int argc, char **argv){
    // Initialize ROS node
    ros::init(argc, argv, "xarm6_collision");
    ros::NodeHandle nh;

    // Resister the Callback function of Topic communication and setting
    ros::Subscriber sub = nh.subscribe("/xarm/joint_states", 10, Subscribe_JointStates);

    ros::Rate rate(Fs);

    while(ros::ok()){
        ros::spinOnce();

        ROS_INFO("effort_0 : %f", present_torque(0));

        rate::sleep();
    }

    // ros::spin();
    return 0;
}