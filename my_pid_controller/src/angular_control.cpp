#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

class AngularController {
public:
    AngularController() 
        : nh_(), goal_angle_(0.0), current_angle_(0.0), 
          Kp_(8.0), Ki_(0.05), Kd_(0.03), prev_error_(0.0), integral_(0.0) {
        potentiometer_sub_ = nh_.subscribe("/potentiometer", 10, &AngularController::potentiometerCallback, this);
        goal_angle_sub_ = nh_.subscribe("/goal_angle", 10, &AngularController::goalAngleCallback, this);
        angular_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    void controlLoop() {
        ros::Rate rate(15); // 15 Hz

        while (ros::ok()) {
            double error = goal_angle_ - current_angle_;
            integral_ += error;
            double derivative = error - prev_error_;
            double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative ;

            geometry_msgs::Twist twist_msg;
            twist_msg.angular.z = output;
            angular_pub_.publish(twist_msg);

            prev_error_ = error;

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void potentiometerCallback(const std_msgs::Int32::ConstPtr& msg) {
        potentiometer_ = msg->data;
        // Potentiometer 값을 20에서 930 범위로 설정하고 -18도에서 18도로 변환
        if (potentiometer_ < 20) potentiometer_ = 20;
        if (potentiometer_ > 930) potentiometer_ = 930;
        current_angle_ = ((potentiometer_ - 20) / 910.0) * 36.0 - 18.0;
        ROS_INFO("Potentiometer: %d, Angle: %f", potentiometer_, current_angle_);
    }

    void goalAngleCallback(const std_msgs::Float64::ConstPtr& msg) {
        double angle = msg->data;
        // Goal angle을 -18도에서 18도 범위로 제한
        if (angle < -15.0) angle = -15.0;
        if (angle > 15.0) angle = 15.0;
        goal_angle_ = angle;
        ROS_INFO("Goal Angle: %f", goal_angle_);
    }

    ros::NodeHandle nh_;
    ros::Subscriber potentiometer_sub_;
    ros::Subscriber goal_angle_sub_;
    ros::Publisher angular_pub_;

    int potentiometer_;
    double goal_angle_, current_angle_;

    // PID control variables
    double Kp_, Ki_, Kd_;
    double prev_error_;
    double integral_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "angular_control");
    AngularController angularController;
    angularController.controlLoop();
    return 0;
}
