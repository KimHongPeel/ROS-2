// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//cam2image.cpp
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <sstream>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "linetracer/linetracer_ros2.hpp"


/// Convert a sensor_msgs::Image encoding type (stored as a string) to an OpenCV encoding type.
/**
 * \param[in] encoding A string representing the encoding type.
 * \return The OpenCV encoding type.
 */
int encoding2mat_type(const std::string& encoding)
{
    if (encoding == "mono8") {
        return CV_8UC1;
    }
    else if (encoding == "bgr8") {
        return CV_8UC3;
    }
    else if (encoding == "mono16") {
        return CV_16SC1;
    }
    else if (encoding == "rgba8") {
        return CV_8UC4;
    }
    else if (encoding == "bgra8") {
        return CV_8UC4;
    }
    else if (encoding == "32FC1") {
        return CV_32FC1;
    }
    else if (encoding == "rgb8") {
        return CV_8UC3;
    }
    else {
        throw std::runtime_error("Unsupported encoding type");
    }
}

