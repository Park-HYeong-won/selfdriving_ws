#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include <math.h> // 추가

// 바퀴의 지름 (미터 단위)
const double wheel_diameter = 0.276; // 27.6cm = 0.276m
const double wheel_circumference = wheel_diameter * M_PI; // 바퀴의 둘레

// 엔코더 카운트
const int encoder_counts_per_revolution = 300;

// 속도 계산을 위한 변수
int last_encoder_count = 0;
ros::Time last_time;

ros::Publisher speed_pub;
int current_encoder_count = 0;

void encoderCallback(const std_msgs::Int32::ConstPtr& msg) {
    // 현재 엔코더 카운트를 업데이트
    current_encoder_count = msg->data;
}

int main(int argc, char **argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "speed_calculator");

    // 노드 핸들러 생성
    ros::NodeHandle n;

    // 속도 퍼블리셔 생성
    speed_pub = n.advertise<std_msgs::Float64>("current_velocity", 10);

    // 엔코더 구독자 생성
    ros::Subscriber sub = n.subscribe("encoder_count", 10, encoderCallback);

    // 초기 시간 설정
    last_time = ros::Time::now();

    ros::Rate rate(15); // 10 Hz, 즉 0.1초마다 루프 실행

    while (ros::ok()) {
        ros::spinOnce(); // 콜백 함수 호출

        // 현재 시간
        ros::Time current_time = ros::Time::now();

        // 시간 차이 계산 (초 단위)
        double delta_time = (current_time - last_time).toSec();

        // 시간 차이가 0일 때 무시
        if (delta_time <= 0.0) {
            rate.sleep();
            continue;
        }

        // 엔코더 카운트 차이 계산
        int delta_encoder_count = current_encoder_count - last_encoder_count;

        // 이동 거리 계산 (미터 단위)
        double distance_traveled = (delta_encoder_count / static_cast<double>(encoder_counts_per_revolution)) * wheel_circumference;

        // 속도 계산 (m/s 단위)
        double speed_mps = distance_traveled / delta_time;

        // 속도를 km/h로 변환
        double speed_kph = speed_mps * 3.6;

        // 속도 메시지 생성 및 퍼블리시
        std_msgs::Float64 speed_msg;
        speed_msg.data = speed_kph;
        speed_pub.publish(speed_msg);

        // 현재 엔코더 카운트와 시간을 저장
        last_encoder_count = current_encoder_count;
        last_time = current_time;

        rate.sleep();
    }

    return 0;
}
