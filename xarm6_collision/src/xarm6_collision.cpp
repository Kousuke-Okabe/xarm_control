#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include <Eigen/Dense>

#define DOF 6
#define Fs 10

static std::mutex g_mutex;

Eigen::VectorXd present_angle(6);
Eigen::VectorXd present_torque(6);

// Callback function of the Topic communication with joint_states
void Subscribe_JointStates(const sensor_msgs::JointState &state){
    // lock the mutex
    std::lock_guard<std::mutex> lock(g_mutex);

    // ROS_INFO("Joint Effort : %f", state.effort[1]);
    for(int i=0; i<DOF; i++){
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
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()){
        // ros::spinOnce();
        g_mutex.lock();

        ROS_INFO("effort_0 : %f", present_torque(1));

        g_mutex.unlock();
        rate.sleep();
    }

    // ros::spin();
    return 0;
}