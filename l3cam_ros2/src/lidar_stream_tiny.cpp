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
#include <unordered_map>
#include <mutex>

#include <thread>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "l3cam_interfaces/msg/tiny_point_cloud.hpp"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

using namespace std::chrono_literals;

bool g_listening = false;

std::mutex g_mutex;
int g_thread_counter = 0;
const int g_max_thread = 100;

int16_t clamp_to_int16(int32_t value)
{
    if (value > INT16_MAX)
        return INT16_MAX;
    if (value < INT16_MIN)
        return INT16_MIN;
    return (int16_t)value;
}

uint8_t clamp_to_uint8(float value)
{
    if ((int)value > UINT8_MAX)
        return UINT8_MAX;
    if ((int)value < 0)
        return 0;
    return (uint8_t)value;
}

void CompressSendPointCloudThread(std::vector<int32_t> point_cloud_data,
                                  rclcpp::Publisher<l3cam_interfaces::msg::TinyPointCloud>::SharedPtr publisher,
                                  uint8_t precision,
                                  int divisor,
                                  bool deduplicate,
                                  int intensity_th,
                                  std_msgs::msg::Header header)
{
    if (g_thread_counter >= g_max_thread)
        return;

    g_mutex.lock();
    ++g_thread_counter;
    g_mutex.unlock();

    l3cam_interfaces::msg::TinyPointCloud tpc_msg;
    tpc_msg.header = header;
    tpc_msg.precision = precision;

    std::unordered_map<uint64_t, std::vector<int16_t>> spatial_grid;
    if (deduplicate)
    {
        spatial_grid.reserve(point_cloud_data.size());
    }

    int count_removed = 0;
    int32_t x, y, z;
    float intensity;
    int16_t x_16, y_16, z_16;
    uint8_t i_8;
    for (uint i = 0; i < point_cloud_data.size() / 5; ++i)
    {
        y = -point_cloud_data[5 * i + 1];
        z = -point_cloud_data[5 * i + 2];
        x = point_cloud_data[5 * i + 3];
        intensity = (float)point_cloud_data[5 * i + 4];

        // Scale, Clamp
        x_16 = clamp_to_int16(x / divisor);
        y_16 = clamp_to_int16(y / divisor);
        z_16 = clamp_to_int16(z / divisor);
        i_8 = clamp_to_uint8((intensity - 500) / 4500 * 256);

        if (deduplicate)
        {
            // Pack 3x int16 into one uint64 for O(1) lookup
            // Casting to uint16_t first ensures bits are preserved correctly for negative numbers
            uint64_t key = ((uint64_t)(uint16_t)x_16 << 32) |
                           ((uint64_t)(uint16_t)y_16 << 16) |
                           (uint64_t)(uint16_t)z_16;

            // Check if this coordinate exists
            auto &existing_intensities = spatial_grid[key];

            bool is_duplicate = false;
            // Only iterate through points at THIS EXACT coordinate (usually 0 or 1)
            for (int16_t existing_i : existing_intensities)
            {
                if (abs(i_8 - existing_i) < intensity_th)
                {
                    is_duplicate = true;
                    break;
                }
            }

            if (is_duplicate)
            {
                count_removed++;
                continue; // Skip writing
            }

            // Not a duplicate, store intensity for future checks at this coord
            existing_intensities.push_back(i_8);
        }

        tpc_msg.points.push_back(x_16);
        tpc_msg.points.push_back(y_16);
        tpc_msg.points.push_back(z_16);
        tpc_msg.intensities.push_back(i_8);
    }
    /*
    if (count_removed > 0)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Removed " << count_removed << " duplicated points");
    }
    */
    publisher->publish(tpc_msg);

    g_mutex.lock();
    --g_thread_counter;
    g_mutex.unlock();
}

void PointCloudThread(rclcpp::Publisher<l3cam_interfaces::msg::TinyPointCloud>::SharedPtr publisher,
                      uint8_t precision,
                      bool deduplicate,
                      int intensity_th)
{
    struct sockaddr_in m_socket;
    int m_socket_descriptor;           // Socket descriptor
    std::string m_address = "0.0.0.0"; // Local address of the network interface port connected to the L3CAM
    int m_udp_port = 6050;             // For the lidar it's 6050

    socklen_t socket_len = sizeof(m_socket);
    char *buffer;
    buffer = (char *)malloc(64000);

    int32_t m_pointcloud_size;
    int32_t *m_pointcloud_data;
    uint32_t m_timestamp;
    bool m_is_reading_pointcloud = false;
    int points_received = 1;
    int pointcloud_index = 1;

    std_msgs::msg::Header header;
    header.frame_id = "lidar";

    int32_t divisor = 0; // mm
    switch (precision)
    {
    case 1: // cm
        divisor = 10;
        break;
    case 2: // dm
        divisor = 100;
        break;
    case 3: // m
        divisor = 1000;
        break;
    }

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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LiDAR streaming.");

    while (g_listening)
    {
        int size_read = recvfrom(m_socket_descriptor, buffer, 64000, 0, (struct sockaddr *)&m_socket, &socket_len);

        if (size_read == 17) // Header
        {
            memcpy(&m_pointcloud_size, &buffer[1], 4);
            m_pointcloud_data = (int32_t *)malloc(sizeof(int32_t) * (((m_pointcloud_size) * 5) + 1));
            memcpy(&m_pointcloud_data[0], &m_pointcloud_size, sizeof(int32_t));
            int32_t suma_1, suma_2;
            memcpy(&suma_1, &buffer[5], sizeof(int32_t));
            memcpy(&suma_2, &buffer[9], sizeof(int32_t));
            memcpy(&m_timestamp, &buffer[13], sizeof(uint32_t));
            m_is_reading_pointcloud = true;
            points_received = 0;
            pointcloud_index = 1;
        }
        else if (size_read == 1) // End, send point cloud
        {
            if (points_received != m_pointcloud_size)
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "lidar NET PROBLEM: points_received != m_pointcloud_size: " << points_received << " != " << m_pointcloud_size);
                continue;
            }

            m_is_reading_pointcloud = false;

            int size_pc = m_pointcloud_data[0];

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

            std::vector<int32_t> point_cloud_data(size_pc * 5);
            for (int i = 0; i < size_pc * 5; ++i)
            {
                point_cloud_data[i] = m_pointcloud_data[i];
            }

            std::thread comp_thread(CompressSendPointCloudThread, point_cloud_data, publisher, precision, divisor, deduplicate, intensity_th, header);
            comp_thread.detach();

            free(m_pointcloud_data);
            m_pointcloud_data = nullptr;
            points_received = 0;
            pointcloud_index = 1;
        }
        else if (size_read > 0 && m_is_reading_pointcloud) // Data
        {
            int32_t points = 0;
            memcpy(&points, &buffer[0], 4);
            memcpy(&m_pointcloud_data[pointcloud_index], &buffer[4], (sizeof(int32_t) * (points * 5)));

            pointcloud_index += (points * 5);

            points_received += points;

            // check if under size
            if (points_received > m_pointcloud_size)
                m_is_reading_pointcloud = false;
        }
        // size_read == -1 --> timeout
    }

    publisher = NULL; //! Without this, the node becomes zombie
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Exiting lidar streaming thread");
    free(buffer);
    if (m_pointcloud_data)
    {
        free(m_pointcloud_data);
    }

    shutdown(m_socket_descriptor, SHUT_RDWR);
    close(m_socket_descriptor);
}

namespace l3cam_ros2
{
    class LidarStream : public SensorStream
    {
    public:
        explicit LidarStream() : SensorStream("lidar_stream")
        {
            declareServiceServers("lidar");
            this->declare_parameter("lidar_precision", rclcpp::ParameterValue(1));
            this->declare_parameter("lidar_deduplicate", rclcpp::ParameterValue(true));
            this->declare_parameter("lidar_deduplicate_intensity_threshold", rclcpp::ParameterValue(12));
        }

        rclcpp::Publisher<l3cam_interfaces::msg::TinyPointCloud>::SharedPtr publisher_;

    private:
        void stopListening()
        {
            g_listening = false;
        }

    }; // class LidarStream

} // namespace l3cam_ros2

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::LidarStream> node = std::make_shared<l3cam_ros2::LidarStream>();

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
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LiDAR available for streaming");
        }
        else
        {
            return 0;
        }
    }
    node->undeclare_parameter("simulator");

    node->publisher_ = node->create_publisher<l3cam_interfaces::msg::TinyPointCloud>("PC2_lidar/tiny", 10);
    std::thread thread(PointCloudThread, node->publisher_, (uint8_t)node->get_parameter("lidar_precision").as_int(), node->get_parameter("lidar_deduplicate").as_bool(), node->get_parameter("lidar_deduplicate_intensity_threshold").as_int());
    thread.detach();

    rclcpp::spin(node);

    g_listening = false;
    usleep(2000000);

    rclcpp::shutdown();
    return 0;
}
