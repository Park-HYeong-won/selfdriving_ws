#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

double input_angle;

// 콜백 함수 정의
void pushAngleCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    input_angle = msg->angular.z;
    // 여기서 메시지를 처리하는 코드를 추가할 수 있습니다.
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_velocity_publisher");
    ros::NodeHandle nh;

    // Publisher 정의
    ros::Publisher goal_angle_pub = nh.advertise<std_msgs::Float64>("/goal_angle", 10);
    ros::Publisher goal_velocity_pub = nh.advertise<std_msgs::Float64>("/goal_velocity", 10);
    
    // Subscriber 정의
    ros::Subscriber push_angle_sub = nh.subscribe("/push_angle", 10, pushAngleCallback);

    ros::Rate rate(1); // 1 Hz

    while (ros::ok()) {
        std_msgs::Float64 angle_msg;
        std_msgs::Float64 velocity_msg;

        // 입력값을 -15도에서 15도 범위로 제한
        if (input_angle < -15.0) {
            input_angle = -15.0;
        } else if (input_angle > 15.0) {
            input_angle = 15.0;
        }

        angle_msg.data = input_angle;
        velocity_msg.data = 1.0;
        goal_angle_pub.publish(angle_msg);
        goal_velocity_pub.publish(velocity_msg);
        
        ROS_INFO("Published goal angle: %f", input_angle);

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
