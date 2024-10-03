#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

class CMDController {
public:
    CMDController(){
        // 구독자 설정
        input_velocity_sub_ = nh_.subscribe("/linear_output", 10, &CMDController::InputVelocityCallback, this);
        input_angular_sub_ = nh_.subscribe("/angular_output", 10, &CMDController::InputAngularCallback, this);

        // 퍼블리셔 설정
        robot_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        // 현재 시간 저장
        last_time_ = ros::Time::now();
    }

    // 퍼블리시 함수
    void publish_cmd(){
        ros::Rate rate(15);
        while(ros::ok()){
            // Twist 메시지 퍼블리시
            robot_pub_.publish(twist_msg);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // 선형 속도 콜백 함수
    void InputVelocityCallback(const std_msgs::Float64::ConstPtr& msg) {
        input_velocity = msg->data;
        twist_msg.linear.x = input_velocity;   
    }

    // 각속도 콜백 함수
    void InputAngularCallback(const std_msgs::Float64::ConstPtr& msg) {
        input_angular = msg->data;
        twist_msg.angular.z = input_angular;
    }

    ros::NodeHandle nh_; // ROS 노드 핸들
    ros::Subscriber input_velocity_sub_; // 선형 속도 구독자
    ros::Subscriber input_angular_sub_; // 각속도 구독자
    ros::Publisher robot_pub_; // 퍼블리셔

    double input_velocity, input_angular; // 입력된 속도 저장 변수
    geometry_msgs::Twist twist_msg; // Twist 메시지 객체
    ros::Time last_time_; // 마지막 시간 저장
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_vel_output");
    CMDController cmd_controller;
    cmd_controller.publish_cmd();
    return 0;
}
