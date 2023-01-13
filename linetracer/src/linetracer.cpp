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
//placeholders Ŭ���� ���
class PubSub : public rclcpp::Node //Ŭ���� ���� Node Ŭ���� ���
{
public: //���� ������
    PubSub() : Node("Camera_subscriber")
        //������ ���� Node Ŭ������ �����ڿ� �μ� ����
    {
        size_t depth = rmw_qos_profile_default.depth; //qos �������� default�� depth(10) ����
        rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
        //qos �������� default�� reliability(RELIABLE) ����
        rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
        //qos �������� default�� history(KEEP_LAST) ����
        auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
        //history_policy(KEEP_LAST)�� depth(10)�� ������ �μ��� ���� �� QoS ����
        reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        //BEST_EFFORT�� �ɼ� ����
        qos_profile.reliability(reliability_policy);
        //������ ������ qos�� reliability�� best_effort�� ����
        pi_camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("pi_image", qos_profile,
            std::bind(&PubSub::image, this, _1));
        //���꽺ũ���̹� ���� Image Ÿ�� �̸� image ������ qos��� �ݹ��Լ� show_image���
        auto dynamix_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        dynamixel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", dynamix_qos_profile);
        timer_ = this->create_wall_timer(10ms, std::bind(&PubSub::publish_velcmd_msg, this));
        namedWindow("showimage_pi", cv::WINDOW_AUTOSIZE); //������ ����
        //cv::namedWindow("showimage_usb", cv::WINDOW_AUTOSIZE); //������ ����
    }

private: //���� ������
    void image(const sensor_msgs::msg::Image::SharedPtr msg)
        //�Լ� ����
    {
        TickMeter tm;
        tm.start();
        RCLCPP_INFO(this->get_logger(), "Received image #%s", msg->header.frame_id.c_str());
        //�͹̳ο� �޼��� ���
        // Convert to an OpenCV matrix by assigning the data.
        Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding),
            const_cast<unsigned char*>(msg->data.data()), msg->step);
        //frame�� ���꽺ũ���̺� �� �޼����� Opencv���� ��� ������ Mat��ü�� ����
        if (msg->encoding == "rgb8") { //encoding �������̽��� rgb8�̸�
            cvtColor(frame, frame, cv::COLOR_RGB2BGR);
            //RGB���� BGR�� ����
        }
        Mat cvframe = frame; //������ ������ Mat ��ü ����


        cvtColor(cvframe, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, gray, Size(), 1);
        threshold(gray, gray, 170, 255, THRESH_BINARY);

        dst = gray(Rect(0, gray.rows * 2 / 3, gray.cols, gray.rows / 3));

        Mat labels, stats, centroids; //���� ����
        int cnt = connectedComponentsWithStats(dst, labels, stats, centroids); //���̺�
        if (cnt > 1) //���̺� ������ ����
        {
            for (int i = 1; i < cnt; i++) //�ݺ���
            {
                p = centroids.ptr<double>(i); //�����߽��� ����� �ּ� ����
                distance = abs(p[0] - prev_pt.x); //x��ǥ�� ���� �����߽� �� ����
                vec_double_distance.push_back(distance); //vector�� ����
            }
            min_index = min_element(vec_double_distance.begin(), vec_double_distance.end()) - vec_double_distance.begin(); //�� �� ���� �ּ��� �ε��� ����
            pt = Point2d(centroids.at<double>(min_index + 1, 0), centroids.at<double>(min_index + 1, 1)); //�ش� �ε����� �����߽� ����
            //if (abs(prev_pt.x - pt.x) > 50) pt = prev_pt; //���� ���� �߽ɰ� ���� �����߽��� ���� 15���� ũ�� ���� �����߽��� ��ǥ�� 0�̸�
            vec_double_distance.clear(); //�ʱ�ȭ
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
        imshow("showimage_pi", cvframe); //�Լ� ȣ��
        waitKey(1); //�Լ� ȣ��
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
    //��� ���� ����
    Mat gray, dst;
    Point pt,frame_pt;
    Point prev_pt = Point(160, 120);
    double error;//,left_wheel,right_wheel;
    vector<double> vec_double_distance; //��������
    double distance; //��������
    double* p; //��������
    int min_index; //��������
    geometry_msgs::msg::Twist msg;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dynamixel_publisher_;
};

int main(int argc, char* argv[]) //main �Լ�
{
    rclcpp::init(argc, argv); //ros2 init
    auto node = std::make_shared<PubSub>(); //��� ����
    rclcpp::spin(node); //��� ����
    rclcpp::shutdown(); //���μ��� ����
    return 0; //����
}