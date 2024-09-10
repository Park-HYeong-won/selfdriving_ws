#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

class PIDController {
public:
    PIDController()
        : nh_("~"), kp_(7.0), ki_(0.05), kd_(0.05), prev_error_(0.0), integral_(0.0), goal_velocity_(0.0), current_velocity_(0.0), force_(0.0) {

        nh_.param("kp", kp_, kp_);
        nh_.param("ki", ki_, ki_);
        nh_.param("kd", kd_, kd_);

        goal_velocity_sub_ = nh_.subscribe("/goal_velocity", 10, &PIDController::goalVelocityCallback, this);
        velocity_sub_ = nh_.subscribe("/current_velocity", 10, &PIDController::velocityCallback, this);

        velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        last_time_ = ros::Time::now();
    }

    void controlLoop() {
        ros::Rate rate(15); // 15 Hz
        while (ros::ok()) {
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_time_).toSec();

            double error = goal_velocity_ - current_velocity_;
            integral_ += error * dt;
            double derivative = (error - prev_error_) / dt;
      
            double output = kp_ * error + ki_ * integral_ + kd_ * derivative + force_;
           
            geometry_msgs::Twist twist_msg;
            twist_msg.linear.x = output;
            velocity_pub_.publish(twist_msg);

            prev_error_ = error;
            last_time_ = current_time;

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void goalVelocityCallback(const std_msgs::Float64::ConstPtr& msg) {
        goal_velocity_ = msg->data;
        switch (goal_velocity_) {
        case 1.0:
            force_ = 30.0;
            break;
        case 2.0:
            force_ = 36.0;
            break;
        case 3.0:
            force_ = 40.0;
            break;
        case 4.0:
            force_ = 44.0;
            break;
        case 5.0:
            force_ = 48.0;
            break;
        default:
            force_ = 0.0;
            break;
        }
    }

    void velocityCallback(const std_msgs::Float64::ConstPtr& msg) {
        current_velocity_ = msg->data;
        ROS_INFO("current velocity: %f", current_velocity_);
    }

    ros::NodeHandle nh_;
    ros::Subscriber goal_velocity_sub_;
    ros::Subscriber velocity_sub_;
    ros::Publisher velocity_pub_;

    double kp_, ki_, kd_;
    double goal_velocity_, current_velocity_;
    double prev_error_, integral_;
    double force_;
    ros::Time last_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "linear_controller");
    PIDController pid_controller;
    pid_controller.controlLoop();
    return 0;
}
