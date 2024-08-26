#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

double input_linear;

int main(int argc, char** argv) {
    std::cout << "velocity : ";
    std::cin >> input_linear;
    ros::init(argc, argv, "goal_velocity_publisher");
    ros::NodeHandle nh;

    // Publisher 정의
    ros::Publisher goal_angle_pub = nh.advertise<std_msgs::Float64>("/goal_angle", 10);
    ros::Publisher goal_velocity_pub = nh.advertise<std_msgs::Float64>("/goal_velocity", 10);

    ros::Rate rate(1); // 1 Hz

    while (ros::ok()) {
        std_msgs::Float64 angle_msg;
        std_msgs::Float64 velocity_msg;

       
        velocity_msg.data = input_linear;
        goal_angle_pub.publish(angle_msg);
        goal_velocity_pub.publish(velocity_msg);
        
        ROS_INFO("Published goal angle: %f", input_linear);

        ros::spinOnce(); // 콜백 함수 호출을 위해 spinOnce() 사용
        rate.sleep();
    }

    // 노드가 종료되기 전에 속도와 각도를 0으로 설정
    std_msgs::Float64 zero_msg;
    zero_msg.data = 0.0;
    goal_angle_pub.publish(zero_msg);
    goal_velocity_pub.publish(zero_msg);
    
    ROS_INFO("Published goal angle: 0.0");
    ROS_INFO("Published goal velocity: 0.0");

    return 0;
}
