#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <vector>
#include <cmath>

#define RAD2DEG(x) ((x) * 180.0 / M_PI)  // 라디안을 도로 변환하는 매크로
#define DEG2RAD(x) ((x) * M_PI / 180.0)  // 도를 라디안으로 변환하는 매크로

// UTM 변환에 필요한 상수들
const double a = 6378137.0;  // 반지름
const double f = 1 / 298.257223563;  // 납작함
const double k0 = 0.9996;  // 축척 계수
const double e = sqrt(f * (2 - f));  // 제1편심률
const double e_sq = e * e;
const double lon0 = 126.0;  // 중앙 경도 (서울 근처 126도)

// Waypoint 구조체
struct Waypoint {
    double x;
    double y;
};

// 전역 변수
std::vector<Waypoint> waypoints;
int current_waypoint_idx = 0;
Waypoint current_position;
Waypoint previous_position;
bool first_gps_received = false;

// Waypoints 파일 읽기 함수
void loadWaypoints(const std::string &filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open waypoints file.");
        return;
    }

    double x, y;
    while (file >> x >> y) {
        waypoints.push_back({x, y});
    }
    file.close();
    ROS_INFO("Waypoints loaded. Total waypoints: %lu", waypoints.size());
}

// 위도, 경도를 UTM으로 변환하는 함수
Waypoint convertGPStoUTM(double lat, double lon) {
    lat = lat * M_PI / 180.0;
    lon = lon * M_PI / 180.0;

    double delta_lon = lon - lon0 * M_PI / 180.0;
    double N = a / sqrt(1 - e_sq * sin(lat) * sin(lat));
    double T = tan(lat) * tan(lat);
    double C = e_sq / (1 - e_sq) * cos(lat) * cos(lat);
    double A = cos(lat) * delta_lon;
    double M = a * ((1 - e_sq / 4 - 3 * e_sq * e_sq / 64 - 5 * e_sq * e_sq * e_sq / 256) * lat 
                    - (3 * e_sq / 8 + 3 * e_sq * e_sq / 32 + 45 * e_sq * e_sq * e_sq / 1024) * sin(2 * lat)
                    + (15 * e_sq * e_sq / 256 + 45 * e_sq * e_sq * e_sq / 1024) * sin(4 * lat)
                    - (35 * e_sq * e_sq * e_sq / 3072) * sin(6 * lat));

    double x = k0 * N * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * e_sq) * A * A * A * A * A / 120);
    double y = k0 * (M + N * tan(lat) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
                                          + (61 - 58 * T + T * T + 600 * C - 330 * e_sq) * A * A * A * A * A * A / 720));

    if (lat < 0) y += 10000000.0;  // 남반구 처리

    return {x, y};
}

// 헤딩 계산 함수 (현재 위치와 이전 위치의 각도 계산)
double calculateCurrentHeading(const Waypoint &prev_pos, const Waypoint &current_pos) {
    double delta_x = current_pos.x - prev_pos.x;
    double delta_y = current_pos.y - prev_pos.y;
    return std::atan2(delta_y, delta_x);  // 라디안 단위
}

// 목표 헤딩 계산 함수 (다음 waypoint 방향)
double calculateTargetHeading(const Waypoint &current_pos, const Waypoint &target_pos) {
    double delta_x = target_pos.x - current_pos.x;
    double delta_y = target_pos.y - current_pos.y;
    return std::atan2(delta_y, delta_x);  // 라디안 단위
}

// 거리 계산 함수
double calculateDistance(const Waypoint &wp1, const Waypoint &wp2) {
    return std::hypot(wp1.x - wp2.x, wp1.y - wp2.y);
}

// GPS 데이터 콜백 함수
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
   if (!first_gps_received) {
        first_gps_received = true;
        previous_position = {msg->latitude, msg->longitude};  // UTM 좌표 사용
    }

    // 현재 위치를 UTM 좌표로 사용
    current_position = {msg->latitude, msg->longitude};
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoint_follower");
    ros::NodeHandle nh;

    std::string waypoint_file;
    nh.param<std::string>("waypoint_file", waypoint_file, "/home/pomong/catkin_ws/src/GPS_GUI/scripts/first_test.txt");

    loadWaypoints(waypoint_file);

    ros::Publisher heading_pub = nh.advertise<std_msgs::Float64>("goal_angle", 10);
    ros::Publisher speed_pub = nh.advertise<std_msgs::Float64>("goal_velocity", 10);

    ros::Subscriber gps_sub = nh.subscribe("ublox_fix", 10, gpsCallback);

    ros::Rate rate(10);

    while (ros::ok()) {
        if (first_gps_received && current_waypoint_idx < waypoints.size()) {
            double distance_to_waypoint = calculateDistance(current_position, waypoints[current_waypoint_idx]);

            if (distance_to_waypoint < 1.0) {
                ROS_INFO("Waypoint %d reached.", current_waypoint_idx + 1);
                current_waypoint_idx++;
                if (current_waypoint_idx >= waypoints.size()) {
                    ROS_INFO("All waypoints reached.");
                    break;
                }
            }

            double current_heading = calculateCurrentHeading(previous_position, current_position);
            double target_heading = calculateTargetHeading(current_position, waypoints[current_waypoint_idx]);

            double heading_error = target_heading - current_heading;

            if (heading_error > M_PI) {
                heading_error -= 2 * M_PI;
            } else if (heading_error < -M_PI) {
                heading_error += 2 * M_PI;
            }

            // 라디안 값을 도(degree)로 변환하고 각도 제한 설정
            double heading_error_deg = RAD2DEG(heading_error);

            // 각도를 -15도 ~ +15도 사이로 제한
            if (heading_error_deg > 15.0) {
                heading_error_deg = 15.0;
            } else if (heading_error_deg < -15.0) {
                heading_error_deg = -15.0;
            }

            // 목표 각도 퍼블리시
            std_msgs::Float64 heading_msg;
            heading_msg.data = heading_error_deg;
            heading_pub.publish(heading_msg);

            // 목표 속도 퍼블리시
            std_msgs::Float64 speed_msg;
            speed_msg.data = 1.0;  // 임의의 목표 속도
            speed_pub.publish(speed_msg);

            previous_position = current_position;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
