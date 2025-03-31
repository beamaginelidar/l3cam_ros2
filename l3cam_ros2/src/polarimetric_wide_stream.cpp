/*  Copyright (c) 2023, Beamagine

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:

        - Redistributions of source code must retain the above copyright notice,
          this list of conditions and the following disclaimer.
        - Redistributions in binary form must reproduce the above copyright notice,
          this list of conditions and the following disclaimer in the documentation and/or
          other materials provided with the distribution.
        - Neither the name of copyright holders nor the names of its contributors may be
          used to endorse or promote products derived from this software without specific
          prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
    MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
    TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
    EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "sensor_stream.hpp"

#include <chrono>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <pthread.h>
#include <thread>

#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_interfaces/srv/enable_polarimetric_camera_stream_processed_image.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_process_type.hpp"

using namespace std::chrono_literals;

pthread_t stream_thread;

int g_angle = no_angle;

bool g_listening = false;

bool g_pol = false; // true if polarimetric available, false if wide available
bool g_stream_processed = true;

cv::Mat rgbpol2rgb(cv::Mat img, polAngle angle = no_angle)
{
    /*
    0       1       2       3
    +-------+-------+-------+-------+ 0
    |  90°  |  45°  |  90°  |  45°  |
    |   B   |   B   |   Gb  |   Gb  |
    +-------+-------+-------+-------+ 1
    | 135°  |  0°   | 135°  |  0°   |
    |   B   |   B   |   Gb  |   Gb  |
    +-------+-------+-------+-------+ 2
    |  90°  |  45°  |  90°  |  45°  |
    |   Gr  |   Gr  |   R   |   R   |
    +-------+-------+-------+-------+ 3
    | 135°  |  0°   | 135°  |  0°   |
    |   Gr  |   Gr  |   R   |   R   |
    +-------+-------+-------+-------+
    */

    cv::Mat bayer(img.rows / 2, img.cols / 2, CV_8UC1, cv::Scalar(0));
    cv::Mat rgb;

    auto is_valid_pixel = [](int px_y, int px_x, polAngle angle) -> bool {
        switch (angle)
        {
        case angle_0:   return (px_y == 1 || px_y == 3) && (px_x == 1 || px_x == 3);
        case angle_45:  return (px_y == 0 || px_y == 2) && (px_x == 1 || px_x == 3);
        case angle_90:  return (px_y == 0 || px_y == 2) && (px_x == 0 || px_x == 2);
        case angle_135: return (px_y == 1 || px_y == 3) && (px_x == 0 || px_x == 2);
        default: return false;
        }
    };

    if (angle != no_angle)
    {
        for (int y = 0; y < img.rows; ++y)
        {
            for (int x = 0; x < img.cols; ++x)
            {
                const int px_y = y % 4;
                const int px_x = x % 4;

                if (is_valid_pixel(px_y, px_x, angle))
                {
                    bayer.at<uint8_t>(y / 2, x / 2) = img.at<uint8_t>(y, x);
                }
            }
        }
        cv::cvtColor(bayer, rgb, cv::COLOR_BayerRG2BGR);
    }
    else
    {
        cv::Mat bayer0 = cv::Mat::zeros(img.rows / 2, img.cols / 2, CV_8UC1);
        cv::Mat bayer90 = cv::Mat::zeros(img.rows / 2, img.cols / 2, CV_8UC1);

        for (int y = 0; y < img.rows; ++y)
        {
            for (int x = 0; x < img.cols; ++x)
            {
                const int px_y = y % 4;
                const int px_x = x % 4;

                if ((px_y == 1 || px_y == 3) && (px_x == 1 || px_x == 3))
                {
                    bayer0.at<uint8_t>(y / 2, x / 2) = img.at<uint8_t>(y, x);
                }
                if ((px_y == 0 || px_y == 2) && (px_x == 0 || px_x == 2))
                {
                    bayer90.at<uint8_t>(y / 2, x / 2) = img.at<uint8_t>(y, x);
                }
            }
        }

        cv::Mat rgb0, rgb90;
        cv::cvtColor(bayer0, rgb0, cv::COLOR_BayerRG2BGR);
        cv::cvtColor(bayer90, rgb90, cv::COLOR_BayerRG2BGR);
        rgb = rgb0 / 2 + rgb90 / 2;
    }

    return rgb;
}

void ImageThread(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr extra_publisher)
{
    struct sockaddr_in m_socket;
    int m_socket_descriptor;           // Socket descriptor
    std::string m_address = "0.0.0.0"; // Local address of the network interface port connected to the L3CAM
    int m_udp_port = 6060;             // For Polarimetric and Allied Wide it's 6060

    socklen_t socket_len = sizeof(m_socket);
    char *buffer;
    buffer = (char *)malloc(64000);

    uint16_t m_image_height;
    uint16_t m_image_width;
    uint8_t m_image_channels;
    uint32_t m_timestamp;
    int m_image_data_size;
    bool m_is_reading_image = false;
    char *m_image_buffer = NULL;
    int bytes_count = 0;

    if ((m_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        perror("Opening socket");
        return;
    }
    // else RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Socket Polarimetric created");

    memset((char *)&m_socket, 0, sizeof(struct sockaddr_in));
    m_socket.sin_addr.s_addr = inet_addr((char *)m_address.c_str());
    m_socket.sin_family = AF_INET;
    m_socket.sin_port = htons(m_udp_port);

    if (inet_aton((char *)m_address.c_str(), &m_socket.sin_addr) == 0)
    {
        perror("inet_aton() failed");
        return;
    }

    if (bind(m_socket_descriptor, (struct sockaddr *)&m_socket, sizeof(struct sockaddr_in)) == -1)
    {
        perror("Could not bind name to socket");
        close(m_socket_descriptor);
        return;
    }

    int rcvbufsize = 134217728;
    if (0 != setsockopt(m_socket_descriptor, SOL_SOCKET, SO_RCVBUF, (char *)&rcvbufsize, sizeof(rcvbufsize)))
    {
        perror("Error setting size to socket");
        return;
    }

    // 1 second timeout for socket
    struct timeval read_timeout;
    read_timeout.tv_sec = 1;
    setsockopt(m_socket_descriptor, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);

    g_listening = true;
    if (g_pol)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Polarimetric streaming.");
    else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Allied Wide streaming.");

    uint8_t *image_pointer = NULL;

    while (g_listening)
    {
        int size_read = recvfrom(m_socket_descriptor, buffer, 64000, 0, (struct sockaddr *)&m_socket, &socket_len);
        if (size_read == 11) // Header
        {
            memcpy(&m_image_height, &buffer[1], 2);
            memcpy(&m_image_width, &buffer[3], 2);
            memcpy(&m_image_channels, &buffer[5], 1);

            if (image_pointer != NULL)
            {
                free(image_pointer);
                image_pointer = NULL;
            }
            if (m_image_buffer != NULL)
            {
                free(m_image_buffer);
                m_image_buffer = NULL;
            }

            m_image_buffer = (char *)malloc(m_image_height * m_image_width * m_image_channels);
            image_pointer = (uint8_t *)malloc(m_image_height * m_image_width * m_image_channels);

            memcpy(&m_timestamp, &buffer[6], sizeof(uint32_t));
            m_image_data_size = m_image_height * m_image_width * m_image_channels;
            m_is_reading_image = true;
            bytes_count = 0;
        }
        else if (size_read == 1 && bytes_count == m_image_data_size) // End, send image
        {
            m_is_reading_image = false;
            bytes_count = 0;
            memcpy(image_pointer, m_image_buffer, m_image_data_size);

            cv::Mat img_data;
            if (m_image_channels == 1)
            {
                img_data = cv::Mat(m_image_height, m_image_width, CV_8UC1, image_pointer);
            }
            else if (m_image_channels == 2)
            {
                img_data = cv::Mat(m_image_height, m_image_width, CV_8UC2, image_pointer);
                cv::cvtColor(img_data, img_data, cv::COLOR_YUV2BGR_Y422);
            }
            else if (m_image_channels == 3)
            {
                img_data = cv::Mat(m_image_height, m_image_width, CV_8UC3, image_pointer);
            }

            std_msgs::msg::Header header;
            header.frame_id = g_pol ? "polarimetric" : "allied_wide";
            // m_timestamp format: hhmmsszzz
            time_t t = time(NULL);
            std::tm *time_info = std::localtime(&t);
            time_info->tm_sec = 0;
            time_info->tm_min = 0;
            time_info->tm_hour = 0;
            header.stamp.sec = std::mktime(time_info) +
                               (uint32_t)(m_timestamp / 10000000) * 3600 +     // hh
                               (uint32_t)((m_timestamp / 100000) % 100) * 60 + // mm
                               (uint32_t)((m_timestamp / 1000) % 100);         // ss
            header.stamp.nanosec = (m_timestamp % 1000) * 1e6;                 // zzz

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "frame_id: " << header.frame_id);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "stamp: " << header.stamp.sec << header.stamp.nanosec);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Image Type: " << img_data.type());
            if (img_data.empty()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: img_data is empty!");
                //return;
            }
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Image size: " << img_data.cols << "x" << img_data.rows);
            
            const std::string encoding = m_image_channels == 1 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;
            std::shared_ptr<sensor_msgs::msg::Image> img_msg = cv_bridge::CvImage(header, encoding, img_data).toImageMsg();
            
            publisher->publish(*img_msg);

            if(extra_publisher && g_stream_processed)
            {
                cv::Mat img_processed = rgbpol2rgb(img_data, (polAngle)g_angle);

                std_msgs::msg::Header header_processed = header;
                header.frame_id = "polarimetric_processed";

                std::shared_ptr<sensor_msgs::msg::Image> img_processed_msg = cv_bridge::CvImage(header_processed, sensor_msgs::image_encodings::BGR8, img_processed).toImageMsg();
                extra_publisher->publish(*img_processed_msg);
            }
        }
        else if (size_read > 0 && m_is_reading_image) // Data
        {
            memcpy(&m_image_buffer[bytes_count], buffer, size_read);
            bytes_count += size_read;

            // check if under size
            if (bytes_count >= m_image_data_size)
                m_is_reading_image = false;
        }
        // size_read == -1 --> timeout
    }

    publisher = NULL; //! Without this, the node becomes zombie
    extra_publisher = NULL; //! Without this, the node becomes zombie
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Exiting " << (g_pol ? "polarimetric" : "allied wide") << " streaming thread");
    free(buffer);
    free(m_image_buffer);

    shutdown(m_socket_descriptor, SHUT_RDWR);
    close(m_socket_descriptor);

    pthread_exit(0);
}

namespace l3cam_ros2
{
    class PolarimetricWideStream : public SensorStream
    {
    public:
        explicit PolarimetricWideStream() : SensorStream("polarimetric_wide_stream")
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            rcl_interfaces::msg::IntegerRange intRange;
            this->declare_parameter("polarimetric_camera_stream_processed_image", true);
            intRange.set__from_value(0).set__to_value(4);
            descriptor.integer_range = {intRange};
            this->declare_parameter("polarimetric_camera_process_type", 4, descriptor); // see polAngle

            g_stream_processed = this->get_parameter("polarimetric_camera_stream_processed_image").as_bool();
            g_angle = this->get_parameter("polarimetric_camera_process_type").as_int();
        }

        void declareExtraService()
        {

            srv_enable_polarimetric_camera_stream_processed_ = this->create_service<l3cam_interfaces::srv::EnablePolarimetricCameraStreamProcessedImage>(
                "enable_polarimetric_camera_stream_processed_image",
                std::bind(&PolarimetricWideStream::enableStreamProcessed, this, std::placeholders::_1, std::placeholders::_2));
            srv_change_polarimetric_camera_process_type_ = this->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraProcessType>(
                "change_polarimetric_camera_process_type",
                std::bind(&PolarimetricWideStream::changeProcessType, this, std::placeholders::_1, std::placeholders::_2));
        }

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_, extra_publisher_;

    private:
        void stopListening()
        {
            g_listening = false;
        }

        void enableStreamProcessed(const std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraStreamProcessedImage::Request> req,
                                   std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraStreamProcessedImage::Response> res)
        {
            g_stream_processed = req->enabled;
            res->error = 0;
        }

        void changeProcessType(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraProcessType::Request> req,
                               std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraProcessType::Response> res)
        {
            if(req->type < 0 || req->type > no_angle)
            {
                res->error = L3CAM_ROS2_INVALID_POLARIMETRIC_PROCESS_TYPE;
                return;
            }
            
            g_angle = req->type;
            res->error = 0;
        }

        rclcpp::Service<l3cam_interfaces::srv::EnablePolarimetricCameraStreamProcessedImage>::SharedPtr srv_enable_polarimetric_camera_stream_processed_;
        rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraProcessType>::SharedPtr srv_change_polarimetric_camera_process_type_;

    }; // class PolarimetricWideStream

} // namespace l3cam_ros2

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::PolarimetricWideStream> node = std::make_shared<l3cam_ros2::PolarimetricWideStream>();

    // Check if Polarimetric or Allied Wide is available
    int i = 0;
    while (!node->client_get_sensors_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
            return 0;
        }

        if (i >= node->get_parameter("timeout_secs").as_int())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error: " << getErrorDescription(L3CAM_ROS2_SERVICE_AVAILABILITY_TIMEOUT_ERROR));
            return L3CAM_ROS2_SERVICE_AVAILABILITY_TIMEOUT_ERROR;
        }
        ++i;
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }
    node->undeclare_parameter("timeout_secs");

    auto requestGetSensors = std::make_shared<l3cam_interfaces::srv::GetSensorsAvailable::Request>();
    auto resultGetSensors = node->client_get_sensors_->async_send_request(requestGetSensors);

    int error = L3CAM_OK;
    bool sensor_is_available = false;
    // Shutdown if sensor is not available or if error returned
    if (rclcpp::spin_until_future_complete(node, resultGetSensors) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = resultGetSensors.get();
        error = response->error;

        if (!error)
        {
            for (int i = 0; i < response->num_sensors; ++i)
            {
                if (response->sensors[i].sensor_type == sensor_pol && response->sensors[i].sensor_available)
                {
                    sensor_is_available = true;
                    g_pol = true;
                }
                else if (response->sensors[i].sensor_type == sensor_allied_wide && response->sensors[i].sensor_available)
                {
                    sensor_is_available = true;
                    g_pol = false;
                }
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR " << error << " while checking sensor availability in " << __func__ << ": " << getErrorDescription(error));
            return error;
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service get_sensors_available");
        return L3CAM_ROS2_FAILED_TO_CALL_SERVICE;
    }

    if (sensor_is_available)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), (g_pol ? "Polarimetric" : "Allied Wide") << " camera available for streaming");
        node->declareServiceServers((g_pol ? "polarimetric" : "allied_wide"));
        if(g_pol) node->declareExtraService();
    }
    else
    {
        return 0;
    }

    node->publisher_ = node->create_publisher<sensor_msgs::msg::Image>(g_pol ? "img_pol" : "img_wide", 10);
    if(g_pol)
    {
        node->extra_publisher_ = node->create_publisher<sensor_msgs::msg::Image>("/img_polarimetric_processed", 10);
    }
    std::thread thread(ImageThread, node->publisher_, node->extra_publisher_);
    thread.detach();

    rclcpp::spin(node);

    g_listening = false;
    usleep(2000000);

    rclcpp::shutdown();
    return 0;
}
