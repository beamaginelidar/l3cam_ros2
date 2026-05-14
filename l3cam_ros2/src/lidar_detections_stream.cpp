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

#include <thread>

#include <vision_msgs/msg/detection3_d_array.hpp>

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

using namespace std::chrono_literals;

bool g_listening = false;

void DetectionsThread(rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher)
{
    struct sockaddr_in m_socket;
    int m_socket_descriptor;           // Socket descriptor
    std::string m_address = "0.0.0.0"; // Local address of the network interface port connected to the L3CAM
    int m_udp_port = 6049;             // For the lidar detections it's 6049

    socklen_t socket_len = sizeof(m_socket);
    char *buffer;
    buffer = (char *)malloc(64000);

    uint32_t num_detections = 0;
    uint32_t num_detections_pack = 0;
    uint32_t detections_recv = 0;
    bool m_is_reading_detections;
    vision_msgs::msg::Detection3DArray m_3d_detections;
    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);

    std_msgs::msg::Header header;
    header.frame_id = "lidar";

    if ((m_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        perror("Opening socket");
        return;
    }
    // else RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Socket Lidar created");

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

    // VERIFY what the kernel actually gave you
    int actual_buf_size = 0;
    socklen_t optlen = sizeof(actual_buf_size);
    if (getsockopt(m_socket_descriptor, SOL_SOCKET, SO_RCVBUF, &actual_buf_size, &optlen) == 0)
    {
        // Note: Kernel doubles the requested value for internal bookkeeping, so actual might be 2x rcvbufsize
        if (actual_buf_size < rcvbufsize)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Socket receive buffer is set to " << actual_buf_size << " bytes instead of " << rcvbufsize);
        }
    }

    // 1 second timeout for socket
    struct timeval read_timeout;
    read_timeout.tv_sec = 1;
    setsockopt(m_socket_descriptor, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);

    g_listening = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LiDAR detections streaming.");

    while (g_listening)
    {
        int size_read = recvfrom(m_socket_descriptor, buffer, 64000, 0, (struct sockaddr *)&m_socket, &socket_len);

        if (size_read == 6) // Header
        {
            m_is_reading_detections = true;
            memcpy(&num_detections, &buffer[2], sizeof(uint32_t));
            detections_recv = 0;

            m_3d_detections.detections.clear();
            header.stamp = system_clock.now();
            m_3d_detections.header = header;
        }
        else if (size_read == 1 || detections_recv == num_detections) // End, send point cloud
        {
            m_is_reading_detections = false;
            num_detections = 0;
            detections_recv = 0;
            publisher->publish(m_3d_detections);
        }
        else if (size_read > 0 && m_is_reading_detections) // Data
        {
            num_detections_pack = buffer[0];
            detections_recv += num_detections_pack;
            int offset = 1;
            if (num_detections > 0)
            {
                for (uint32_t n = 0; n < num_detections_pack; ++n)
                {
                    //! read detections packages
                    uint16_t confidence, label;
                    uint8_t sensor_ori, red, green, blue;
                    memcpy(&confidence, &buffer[offset], sizeof(uint16_t));
                    memcpy(&label, &buffer[offset + 2], sizeof(uint16_t));
                    memcpy(&sensor_ori, &buffer[offset + 4], sizeof(uint8_t));
                    memcpy(&red, &buffer[offset + 5], sizeof(uint8_t));
                    memcpy(&green, &buffer[offset + 6], sizeof(uint8_t));
                    memcpy(&blue, &buffer[offset + 7], sizeof(uint8_t));
                    float cx, cy, cz, sx, sy, sz, rw, rx, ry, rz;
                    memcpy(&cx, &buffer[offset + 8], sizeof(float));
                    memcpy(&cy, &buffer[offset + 12], sizeof(float));
                    memcpy(&cz, &buffer[offset + 16], sizeof(float));
                    memcpy(&sx, &buffer[offset + 20], sizeof(float));
                    memcpy(&sy, &buffer[offset + 24], sizeof(float));
                    memcpy(&sz, &buffer[offset + 28], sizeof(float));
                    memcpy(&rw, &buffer[offset + 32], sizeof(float));
                    memcpy(&rx, &buffer[offset + 36], sizeof(float));
                    memcpy(&ry, &buffer[offset + 40], sizeof(float));
                    memcpy(&rz, &buffer[offset + 44], sizeof(float));
                    offset += 48;

                    vision_msgs::msg::Detection3D det;
                    det.header = header;
                    det.bbox.center.position.x = cz / 1e3;
                    det.bbox.center.position.y = -cx / 1e3;
                    det.bbox.center.position.z = -cy / 1e3;
                    det.bbox.center.orientation.w = 0.5 * ( rw + rx - ry + rz);
                    det.bbox.center.orientation.x = 0.5 * (-rw + rx + ry + rz);
                    det.bbox.center.orientation.y = 0.5 * ( rw - rx + ry + rz);
                    det.bbox.center.orientation.z = 0.5 * (-rw - rx - ry + rz);
                    det.bbox.size.x = sx / 1e3;
                    det.bbox.size.y = sy / 1e3;
                    det.bbox.size.z = sz / 1e3;
                    vision_msgs::msg::ObjectHypothesisWithPose hyp_3d;
                    hyp_3d.hypothesis.class_id = std::to_string(label);
                    hyp_3d.hypothesis.score = (double)confidence;
                    det.results.push_back(hyp_3d);

                    m_3d_detections.detections.push_back(det);
                }
            }

            // check if under size
            if (detections_recv > num_detections)
                m_is_reading_detections = false;
        }
    }

    publisher = NULL; //! Without this, the node becomes zombie
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Exiting lidar detections streaming thread");
    free(buffer);

    shutdown(m_socket_descriptor, SHUT_RDWR);
    close(m_socket_descriptor);
}

namespace l3cam_ros2
{
    class LidarDetectionsStream : public SensorStream
    {
    public:
        explicit LidarDetectionsStream() : SensorStream("lidar_stream")
        {
            declareServiceServers("lidar");
        }

        rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_;

    private:
        void stopListening()
        {
            g_listening = false;
        }

    }; // class LidarDetectionsStream

} // namespace l3cam_ros2

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::LidarDetectionsStream> node = std::make_shared<l3cam_ros2::LidarDetectionsStream>();

    if (!node->get_parameter("simulator").as_bool())
    {
        // Check if LiDAR is available
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
                    if (response->sensors[i].sensor_type == sensor_lidar && response->sensors[i].sensor_available)
                    {
                        sensor_is_available = true;
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
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LiDAR detections available for streaming");
        }
        else
        {
            return 0;
        }
    }
    node->undeclare_parameter("simulator");

    node->publisher_ = node->create_publisher<vision_msgs::msg::Detection3DArray>("lidar_detections", 10);
    std::thread thread(DetectionsThread, node->publisher_);
    thread.detach();

    rclcpp::spin(node);

    g_listening = false;
    usleep(2000000);

    rclcpp::shutdown();
    return 0;
}
