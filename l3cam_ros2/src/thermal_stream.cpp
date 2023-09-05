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

using namespace std::chrono_literals;

pthread_t stream_thread;

bool g_listening = false;

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

void *ImageThread(void *functionData)
{
    struct sockaddr_in m_socket;
    int m_socket_descriptor;           // Socket descriptor
    std::string m_address = "0.0.0.0"; // Local address of the network interface port connected to the L3CAM
    int m_udp_port = 6030;             // For Thermal it's 6030

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
    // else RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Socket Thermal created");
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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Thermal streaming");

    uint8_t *image_pointer = NULL;

    while (g_listening)
    {
        int size_read = recvfrom(m_socket_descriptor, buffer, 64004, 0, (struct sockaddr *)&m_socket, &socket_len);
        if (size_read == 11)
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
        else if (size_read == 1)
        {
            m_is_reading_image = false;
            bytes_count = 0;
            memcpy(image_pointer, m_image_buffer, m_image_data_size);

            cv::Mat img_data;
            if(m_image_channels == 1)
            {
                img_data = cv::Mat(m_image_height, m_image_width, CV_8UC1, image_pointer);
                cv::cvtColor(img_data, img_data, cv::COLOR_GRAY2BGR);
            }
            else if(m_image_channels == 3)
            {
                img_data = cv::Mat(m_image_height, m_image_width, CV_8UC3, image_pointer);
            }

            cv_bridge::CvImage img_bridge;
            sensor_msgs::msg::Image img_msg; // message to be sent

            std_msgs::msg::Header header;
            header.frame_id = "thermal";
            // m_timestamp format: hhmmsszzz
            header.stamp.sec = (uint32_t)(m_timestamp / 10000000) * 3600 + // hh
                               (uint32_t)((m_timestamp / 100000) % 100) * 60 + // mm
                               (uint32_t)((m_timestamp / 1000) % 100); // ss
            header.stamp.nanosec = m_timestamp % 1000; // zzz

            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img_data);
            img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
            publisher_->publish(img_msg);
        }
        else if (size_read > 0)
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

    shutdown(m_socket_descriptor, SHUT_RDWR);
    close(m_socket_descriptor);

    pthread_exit(0);
}

bool isSensorAvailable(rclcpp::Client<l3cam_interfaces::srv::GetSensorsAvailable>::SharedPtr clientGetSensors,
                       std::shared_ptr<rclcpp::Node> node, sensorTypes sensor_type)
{
    int error = L3CAM_OK;

    auto requestGetSensors = std::make_shared<l3cam_interfaces::srv::GetSensorsAvailable::Request>();
    auto resultGetSensors = clientGetSensors->async_send_request(requestGetSensors);

    if (rclcpp::spin_until_future_complete(node, resultGetSensors) == rclcpp::FutureReturnCode::SUCCESS)
    {
        error = resultGetSensors.get()->error;

        if (!error)
            for (int i = 0; i < resultGetSensors.get()->num_sensors; ++i)
            {
                if (resultGetSensors.get()->sensors[i].sensor_type == sensor_type)
                    return true;
            }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_sensors_available");
        return false;
    }

    return false;
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("thermal_stream");

    // Check if Thermal is available
    rclcpp::Client<l3cam_interfaces::srv::GetSensorsAvailable>::SharedPtr clientGetSensors =
        node->create_client<l3cam_interfaces::srv::GetSensorsAvailable>("get_sensors_available");

    while (!clientGetSensors->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
            return 0;
        }
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    if (!isSensorAvailable(clientGetSensors, node, sensor_thermal))
        return 0;

    publisher_ = node->create_publisher<sensor_msgs::msg::Image>("img_thermal", 10);

    pthread_create(&stream_thread, NULL, &ImageThread, NULL);

    while (rclcpp::ok() && isSensorAvailable(clientGetSensors, node, sensor_thermal))
    {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
