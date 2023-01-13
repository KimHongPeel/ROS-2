#include <functional>
#include <memory>
#include <cstdio>
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "linetracer/linetracer_ros2.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <conio.h>

using std::placeholders::_1;
using namespace std::chrono_literals;
//placeholders 클래스 사용
class PubSub : public rclcpp::Node //클래스 정의 Node 클래스 상속
{
public: //접근 지정자
    PubSub() : Node("Camera_subscriber")
        //생성자 정의 Node 클래스의 생성자에 인수 전달
    {
        size_t depth = rmw_qos_profile_default.depth; //qos 프로파일 default의 depth(10) 저장
        rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
        //qos 프로파일 default의 reliability(RELIABLE) 저장
        rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
        //qos 프로파일 default의 history(KEEP_LAST) 저장
        auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
        //history_policy(KEEP_LAST)와 depth(10)를 생성자 인수로 전달 후 QoS 구성
        reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        //BEST_EFFORT로 옵션 지정
        qos_profile.reliability(reliability_policy);
        //위에서 설정한 qos의 reliability를 best_effort로 지정
        pi_camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("pi_image", qos_profile,
            std::bind(&PubSub::image, this, _1));
        //서브스크라이버 설정 Image 타입 이름 image 설정한 qos사용 콜백함수 show_image사용
        auto dynamix_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        dynamixel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", dynamix_qos_profile);
        timer_ = this->create_wall_timer(10ms, std::bind(&PubSub::publish_velcmd_msg, this));
        namedWindow("showimage_pi", cv::WINDOW_AUTOSIZE); //윈도우 생성
        //cv::namedWindow("showimage_usb", cv::WINDOW_AUTOSIZE); //윈도우 생성
    }

private: //접근 지정자
    void image(const sensor_msgs::msg::Image::SharedPtr msg)
        //함수 정의
    {
        TickMeter tm;
        tm.start();
        RCLCPP_INFO(this->get_logger(), "Received image #%s", msg->header.frame_id.c_str());
        //터미널에 메세지 출력
        // Convert to an OpenCV matrix by assigning the data.
        Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding),
            const_cast<unsigned char*>(msg->data.data()), msg->step);
        //frame에 서브스크라이브 한 메세지를 Opencv에서 사용 가능한 Mat객체로 생성
        if (msg->encoding == "rgb8") { //encoding 인터페이스가 rgb8이면
            cvtColor(frame, frame, cv::COLOR_RGB2BGR);
            //RGB에서 BGR로 변경
        }
        Mat cvframe = frame; //위에서 생성한 Mat 객체 대입


        cvtColor(cvframe, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, gray, Size(), 1);
        threshold(gray, gray, 170, 255, THRESH_BINARY);

        dst = gray(Rect(0, gray.rows * 2 / 3, gray.cols, gray.rows / 3));

        Mat labels, stats, centroids; //변수 선언
        int cnt = connectedComponentsWithStats(dst, labels, stats, centroids); //레이블링
        if (cnt > 1) //레이블링 개수에 따라
        {
            for (int i = 1; i < cnt; i++) //반복문
            {
                p = centroids.ptr<double>(i); //무게중심이 저장된 주소 추출
                distance = abs(p[0] - prev_pt.x); //x좌표와 이전 무게중심 차 저장
                vec_double_distance.push_back(distance); //vector에 저장
            }
            min_index = min_element(vec_double_distance.begin(), vec_double_distance.end()) - vec_double_distance.begin(); //그 중 차가 최소인 인덱스 저장
            pt = Point2d(centroids.at<double>(min_index + 1, 0), centroids.at<double>(min_index + 1, 1)); //해당 인덱스의 무게중심 저장
            //if (abs(prev_pt.x - pt.x) > 50) pt = prev_pt; //이전 무게 중심과 현재 무게중심의 차가 15보다 크고 현재 무게중심의 좌표가 0이면
            vec_double_distance.clear(); //초기화
        }
        else pt = prev_pt;
        prev_pt = pt;

        frame_pt.x = pt.x;
        frame_pt.y = pt.y + gray.rows * 2 / 3;

        circle(cvframe, frame_pt, 2, Scalar(0, 0, 255), 2, -1);
        circle(dst, pt, 2, Scalar(0, 0, 255), 2, -1);
        error = dst.cols / 2 - pt.x;
        //std::cout << "error : " << error << std::endl;
        tm.stop();
        cout << "time : " << tm.getTimeMilli() << "ms" << endl;
        //cv::threshold(cvframe, cvframe, 128, 255, cv::THRESH_BINARY);
        //imshow("dst", dst);
        imshow("showimage_pi", cvframe); //함수 호출
        waitKey(1); //함수 호출
    }
    void publish_velcmd_msg()
    {
        msg.linear.x = 110 - error / 2.7; //left
        msg.linear.y = 110 + error / 2.7; //right
        //left_wheel= 50 - error / 10; //left wheel
        //right_wheel= 50 + error / 10; //right wheel
        //msg.linear.x = (left_wheel + right_wheel) / 2.; //linear of robot
        msg.angular.z = error;
        //msg.angular.z = (right_wheel - left_wheel) / 0.162; //angular of robot
        RCLCPP_INFO(this->get_logger(), "Published message: %lf,%lf, error : %lf", msg.linear.x, msg.linear.y, msg.angular.z);
        dynamixel_publisher_->publish(msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pi_camera_subscriber_;
    //멤버 변수 선언
    Mat gray, dst;
    Point pt,frame_pt;
    Point prev_pt = Point(160, 120);
    double error;//,left_wheel,right_wheel;
    vector<double> vec_double_distance; //변수선언
    double distance; //변수선언
    double* p; //변수선언
    int min_index; //변수선언
    geometry_msgs::msg::Twist msg;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dynamixel_publisher_;
};

int main(int argc, char* argv[]) //main 함수
{
    rclcpp::init(argc, argv); //ros2 init
    auto node = std::make_shared<PubSub>(); //노드 생성
    rclcpp::spin(node); //노드 실행
    rclcpp::shutdown(); //프로세스 종료
    return 0; //종료
}