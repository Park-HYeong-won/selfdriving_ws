#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class JoyPush
{
public:
    JoyPush();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

    double linear_;
    double angular_;
    int auto_move_active;
};

JoyPush::JoyPush() :
    linear_(50),
    angular_(50)
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyPush::joyCallback, this);
}

void JoyPush::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;

    // 왼쪽 조이스틱 (축 1: 위/아래)으로 선형 속도를 조절
    twist.linear.x = linear_ * joy->axes[1];

    // 오른쪽 조이스틱 (축 3: 좌/우)으로 각속도를 조절
    twist.angular.z = angular_ * joy->axes[3];
    
    if(joy->buttons[6] != 0) {
        auto_move_active = 0;
    }
    if(joy->buttons[7] != 0){
        auto_move_active = 1;
    }
   

    if(auto_move_active == 1){
    vel_pub_.publish(twist);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_push");
    JoyPush joy_push;

    ros::spin();
}

