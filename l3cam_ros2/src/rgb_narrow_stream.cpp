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

#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

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

#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_interfaces/msg/sensor.hpp"
#include "l3cam_interfaces/srv/get_sensors_available.hpp"

#include "l3cam_interfaces/srv/sensor_disconnected.hpp"

#include "l3cam_ros2_node.hpp" // for ROS2_BMG_UNUSED

using namespace std::chrono_literals;

pthread_t stream_thread;

bool g_listening = false;

bool m_rgb; // true if rgb sensor available, false if narrow available

struct threadData
{
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
};

void *ImageThread(void *functionData)
{
    threadData *data = (struct threadData *)functionData;

    struct sockaddr_in m_socket;
    int m_socket_descriptor;           // Socket descriptor
    std::string m_address = "0.0.0.0"; // Local address of the network interface port connected to the L3CAM
    int m_udp_port = 6020;             // For RGB and Allied Narrow it's 6020

    socklen_t socket_len = sizeof(m_socket);
    char *buffer;
    buffer = (char *)malloc(64000);

    uint16_t m_image_height;
    uint16_t m_image_width;
    uint8_t m_image_channels;
    uint32_t m_timestamp;
    int m_image_data_size;
    bool m_is_reading_image;
    char *m_image_buffer = NULL;
    int bytes_count = 0;

    if ((m_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        perror("Opening socket");
        return 0;
    }
    // else RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Socket RGB created");

    memset((char *)&m_socket, 0, sizeof(struct sockaddr_in));
    m_socket.sin_addr.s_addr = inet_addr((char *)m_address.c_str());
    m_socket.sin_family = AF_INET;
    m_socket.sin_port = htons(m_udp_port);

    if (inet_aton((char *)m_address.c_str(), &m_socket.sin_addr) == 0)
    {
        perror("inet_aton() failed");
        return 0;
    }

    if (bind(m_socket_descriptor, (struct sockaddr *)&m_socket, sizeof(struct sockaddr_in)) == -1)
    {
        perror("Could not bind name to socket");
        close(m_socket_descriptor);
        return 0;
    }

    int rcvbufsize = 134217728;
    if (0 != setsockopt(m_socket_descriptor, SOL_SOCKET, SO_RCVBUF, (char *)&rcvbufsize, sizeof(rcvbufsize)))
    {
        perror("Error setting size to socket");
        return 0;
    }

    g_listening = true;
    if (m_rgb)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RGB streaming.");
    else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Allied Narrow streaming.");

    uint8_t *image_pointer = NULL;

    while (g_listening)
    {
        int size_read = recvfrom(m_socket_descriptor, buffer, 64004, 0, (struct sockaddr *)&m_socket, &socket_len);
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
        else if (size_read == 1) // End, send image
        {
            m_is_reading_image = false;
            bytes_count = 0;
            memcpy(image_pointer, m_image_buffer, m_image_data_size);

            cv::Mat img_data(m_image_height, m_image_width, CV_8UC3, image_pointer);

            cv_bridge::CvImage img_bridge;
            sensor_msgs::msg::Image img_msg; // message to be sent

            std_msgs::msg::Header header;
            header.frame_id = m_rgb ? "rgb" : "allied_narrow";
            // m_timestamp format: hhmmsszzz
            header.stamp.sec = (uint32_t)(m_timestamp / 10000000) * 3600 +     // hh
                               (uint32_t)((m_timestamp / 100000) % 100) * 60 + // mm
                               (uint32_t)((m_timestamp / 1000) % 100);         // ss
            header.stamp.nanosec = m_timestamp % 1000;                         // zzz

            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img_data);
            img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image

            data->publisher->publish(img_msg);
        }
        else if (size_read > 0) // Data
        {
            if (m_is_reading_image)
            {
                memcpy(&m_image_buffer[bytes_count], buffer, size_read);
                bytes_count += size_read;

                // check if under size
                if (bytes_count >= m_image_data_size)
                    m_is_reading_image = false;
            }
        }
    }

    free(buffer);
    free(m_image_buffer);

    shutdown(m_socket_descriptor, SHUT_RDWR);
    close(m_socket_descriptor);

    pthread_exit(0);
}

namespace l3cam_ros2
{
    class RgbNarrowStream : public rclcpp::Node
    {
    public:
        RgbNarrowStream() : Node("rgb_narrow_stream")
        {
            clientGetSensors = this->create_client<l3cam_interfaces::srv::GetSensorsAvailable>("get_sensors_available");
        }

        void declareServiceServers()
        {
            std::string sensor = m_rgb ? "rgb" : "narrow";
            srvSensorDisconnected = this->create_service<l3cam_interfaces::srv::SensorDisconnected>(
                sensor + "_stream_disconnected", std::bind(&RgbNarrowStream::sensorDisconnectedCallback, this, std::placeholders::_1, std::placeholders::_2));
        }

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
        rclcpp::Client<l3cam_interfaces::srv::GetSensorsAvailable>::SharedPtr clientGetSensors;

    private:
        void sensorDisconnectedCallback(const std::shared_ptr<l3cam_interfaces::srv::SensorDisconnected::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::SensorDisconnected::Response> res)
        {
            ROS2_BMG_UNUSED(res);
            g_listening = false;

            if (req->code == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Exiting cleanly.");
            }
            else
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Exiting. Sensor got disconnected with error " << req->code << ": " << getBeamErrorDescription(req->code));
            }

            rclcpp::shutdown();
        }

        rclcpp::Service<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr srvSensorDisconnected;

    }; // class RgbNarrowStream

} // namespace l3cam_ros2

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::RgbNarrowStream> node = std::make_shared<l3cam_ros2::RgbNarrowStream>();

    // Check if RGB or Allied Narrow is available
    while (!node->clientGetSensors->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
            return 0;
        }
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto requestGetSensors = std::make_shared<l3cam_interfaces::srv::GetSensorsAvailable::Request>();
    auto resultGetSensors = node->clientGetSensors->async_send_request(requestGetSensors);

    int error = L3CAM_OK;
    bool sensor_is_available = false;
    // Shutdown if sensor is not available or if error returned
    if (rclcpp::spin_until_future_complete(node, resultGetSensors) == rclcpp::FutureReturnCode::SUCCESS)
    {
        error = resultGetSensors.get()->error;

        if (!error)
        {
            for (int i = 0; i < resultGetSensors.get()->num_sensors; ++i)
            {
                if (resultGetSensors.get()->sensors[i].sensor_type == sensor_econ_rgb && resultGetSensors.get()->sensors[i].sensor_available)
                {
                    sensor_is_available = true;
                    m_rgb = true;
                }
                else if (resultGetSensors.get()->sensors[i].sensor_type == sensor_allied_narrow && resultGetSensors.get()->sensors[i].sensor_available)
                {
                    sensor_is_available = true;
                    m_rgb = false;
                }
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR " << error << " while checking sensor availability in " << __func__ << ": " << getBeamErrorDescription(error));
            return 1;
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service get_sensors_available");
        return 1;
    }

    if (sensor_is_available)
    {
        std::string sensor = (m_rgb ? "RGB" : "Allied Narrow");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), sensor << " camera available for streaming");
        node->declareServiceServers();
    }
    else
    {
        return 0;
    }

    node->publisher_ = node->create_publisher<sensor_msgs::msg::Image>(m_rgb ? "img_rgb" : "img_narrow", 10);

    threadData *data = (struct threadData *)malloc(sizeof(struct threadData));
    data->publisher = node->publisher_;
    pthread_create(&stream_thread, NULL, &ImageThread, (void *)data);

    rclcpp::spin(node);

    g_listening = false;
    usleep(500000);

    rclcpp::shutdown();
    return 0;
}
