#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

double input_linear;
double current_velocity_ = 0.0;

void velocityCallback(const std_msgs::Float64::ConstPtr& msg) {
    current_velocity_ = msg->data;
    ROS_INFO("Current velocity: %f", current_velocity_);
}

int main(int argc, char** argv) {
    std::cout << "cmd_vel : ";
    std::cin >> input_linear;

    ros::init(argc, argv, "goal_velocity_publisher");
    ros::NodeHandle nh;

    // Subscriber 정의
    ros::Subscriber velocity_sub = nh.subscribe("/current_velocity", 10, velocityCallback);
    
    // Publisher 정의
    ros::Publisher goal_angle_pub = nh.advertise<std_msgs::Float64>("/goal_angle", 10);
    ros::Publisher goal_velocity_pub = nh.advertise<std_msgs::Float64>("/goal_velocity", 10);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate rate(1); // 1 Hz

    while (ros::ok()) {
        geometry_msgs::Twist twist_msg;

        // Twist 메시지에 입력된 속도를 설정
        twist_msg.linear.x = input_linear;
        velocity_pub.publish(twist_msg);

        ROS_INFO("Input cmd_vel: %f", input_linear);

        ros::spinOnce(); // 콜백 함수 호출을 위해 spinOnce() 사용
        rate.sleep();
    }

    // 노드가 종료되기 전에 속도와 각도를 0으로 설정
    std_msgs::Float64 zero_msg;
    zero_msg.data = 0.0;
    velocity_pub.publish(zero_msg);

    return 0;
}
