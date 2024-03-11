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
#include <rclcpp/rclcpp.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <libL3Cam.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_interfaces/msg/sensor.hpp"
#include "l3cam_interfaces/srv/get_sensors_available.hpp"

#include "l3cam_interfaces/srv/change_pointcloud_color.hpp"
#include "l3cam_interfaces/srv/change_pointcloud_color_range.hpp"
#include "l3cam_interfaces/srv/change_distance_range.hpp"
#include "l3cam_interfaces/srv/enable_auto_bias.hpp"
#include "l3cam_interfaces/srv/change_bias_value.hpp"
#include "l3cam_interfaces/srv/change_streaming_protocol.hpp"
#include "l3cam_interfaces/srv/get_rtsp_pipeline.hpp"

#include "l3cam_interfaces/srv/sensor_disconnected.hpp"

#include "l3cam_ros2_utils.hpp"

using namespace std::chrono_literals;

namespace l3cam_ros2
{
    class LidarConfiguration : public rclcpp::Node
    {
    public:
        explicit LidarConfiguration() : Node("lidar_configuration")
        {
            // Create service clients
            client_get_sensors_ = this->create_client<l3cam_interfaces::srv::GetSensorsAvailable>("get_sensors_available");
            client_color_ = this->create_client<l3cam_interfaces::srv::ChangePointcloudColor>("change_pointcloud_color");
            client_color_range_ = this->create_client<l3cam_interfaces::srv::ChangePointcloudColorRange>("change_pointcloud_color_range");
            client_distance_range_ = this->create_client<l3cam_interfaces::srv::ChangeDistanceRange>("change_distance_range");
            client_auto_bias_ = this->create_client<l3cam_interfaces::srv::EnableAutoBias>("enable_auto_bias");
            client_bias_value_ = this->create_client<l3cam_interfaces::srv::ChangeBiasValue>("change_bias_value");
            client_streaming_protocol_ = this->create_client<l3cam_interfaces::srv::ChangeStreamingProtocol>("change_streaming_protocol");
            client_get_rtsp_pipeline_ = this->create_client<l3cam_interfaces::srv::GetRtspPipeline>("get_rtsp_pipeline");

            declareParams();
            loadDefaultParams();

            // Create service server
            srv_sensor_disconnected_ = this->create_service<l3cam_interfaces::srv::SensorDisconnected>(
                "lidar_configuration_disconnected", std::bind(&LidarConfiguration::sensorDisconnectedCallback, this, std::placeholders::_1, std::placeholders::_2));

            // Callback on parameters changed
            callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&LidarConfiguration::parametersCallback, this, std::placeholders::_1));
        }

        rclcpp::Client<l3cam_interfaces::srv::GetSensorsAvailable>::SharedPtr client_get_sensors_;
        rclcpp::Client<l3cam_interfaces::srv::GetRtspPipeline>::SharedPtr client_get_rtsp_pipeline_;

    private:
        void declareParams()
        {
            // Declare parameters with range
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            rcl_interfaces::msg::IntegerRange range;
            this->declare_parameter("timeout_secs", 60);
            range.set__from_value(0).set__to_value(13); // TBD: dynamic reconfigure dropdown menu pointCloudColor
            descriptor.integer_range = {range};
            descriptor.description =
                "Value must be: (pointCloudColor)\n"
                "\tRAINBOW = 0\n"
                "\tRAINBOW_Z = 1\n"
                "\tINTENSITY = 2\n"
                "\tRGB_FUSION = 3\n"
                "\tPOLARIMETRIC_FUSION = 4\n"
                "\tPOL_PROCESSED_FUSION = 5\n"
                "\tTHERMAL_FUSION = 6\n"
                "\tRGBT_FUSION = 7\n"
                "\tALLIED_NARROW_FUSION = 12\n"
                "\tALLIED_WIDE_FUSION = 13";
            this->declare_parameter("pointcloud_color", 0, descriptor); // see pointCloudColor
            descriptor.description = "";
            range.set__from_value(0).set__to_value(400000);
            descriptor.integer_range = {range};
            this->declare_parameter("pointcloud_color_range_minimum", 0, descriptor); // 0 - 400000
            range.set__from_value(0).set__to_value(400000);
            descriptor.integer_range = {range};
            this->declare_parameter("pointcloud_color_range_maximum", 400000, descriptor); // 0 - 400000
            range.set__from_value(0).set__to_value(400000);
            descriptor.integer_range = {range};
            this->declare_parameter("distance_range_minimum", 0, descriptor); // 0 - 400000
            range.set__from_value(0).set__to_value(400000);
            descriptor.integer_range = {range};
            this->declare_parameter("distance_range_maximum", 400000, descriptor); // 0 - 400000
            this->declare_parameter("auto_bias", true);
            range.set__from_value(700).set__to_value(3500);
            descriptor.integer_range = {range};
            this->declare_parameter("bias_value_right", 1580, descriptor); // 700 - 3500
            this->declare_parameter("bias_value_left", 1380, descriptor);  // 700 - 3500
            range.set__from_value(0).set__to_value(1);
            descriptor.integer_range = {range};
            descriptor.description = 
                "Value must be:\n"
                "\tprotocol_raw_udp = 0\n"
                "\tprotocol_gstreamer = 1";
            this->declare_parameter("lidar_streaming_protocol", 0, descriptor); // 0(protocol_raw_udp), 1(protocol_gstreamer)
        }

        void loadDefaultParams()
        {
            // Get and save parameters
            pointcloud_color_ = this->get_parameter("pointcloud_color").as_int();
            pointcloud_color_range_minimum_ = this->get_parameter("pointcloud_color_range_minimum").as_int();
            pointcloud_color_range_maximum_ = this->get_parameter("pointcloud_color_range_maximum").as_int();
            distance_range_minimum_ = this->get_parameter("distance_range_minimum").as_int();
            distance_range_maximum_ = this->get_parameter("distance_range_maximum").as_int();
            auto_bias_ = this->get_parameter("auto_bias").as_bool();
            bias_value_right_ = this->get_parameter("bias_value_right").as_int();
            bias_value_left_ = this->get_parameter("bias_value_left").as_int();
            streaming_protocol_ = this->get_parameter("lidar_streaming_protocol").as_int();
        }

        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters)
        {
            // parameters contains the values of the changed parameters
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "success";

            // Filter by parameter and call service
            for (const auto &param : parameters)
            {
                std::string param_name = param.get_name();
                if (param_name == "pointcloud_color" && param.as_int() != pointcloud_color_)
                {
                    callColor(param.as_int());
                }
                if (param_name == "pointcloud_color_range_minimum" && param.as_int() != pointcloud_color_range_minimum_)
                {
                    callColorRange(param.as_int(), pointcloud_color_range_maximum_);
                }
                if (param_name == "pointcloud_color_range_maximum" && param.as_int() != pointcloud_color_range_maximum_)
                {
                    callColorRange(pointcloud_color_range_minimum_, param.as_int());
                }
                if (param_name == "distance_range_minimum" && param.as_int() != distance_range_minimum_)
                {
                    callDistanceRange(param.as_int(), distance_range_maximum_);
                }
                if (param_name == "distance_range_maximum" && param.as_int() != distance_range_maximum_)
                {
                    callDistanceRange(distance_range_minimum_, param.as_int());
                }
                if (param_name == "auto_bias" && param.as_bool() != auto_bias_)
                {
                    callAutoBias(param.as_bool());
                }
                if (param_name == "bias_value_right" && param.as_int() != bias_value_right_)
                {
                    callBiasValue(1, param.as_int());
                }
                if (param_name == "bias_value_left" && param.as_int() != bias_value_left_)
                {
                    callBiasValue(2, param.as_int());
                }
                if (param_name == "lidar_streaming_protocol" && param.as_int() != streaming_protocol_)
                {
                    callStreamingProtocol(param.as_int());
                }
            }

            return result;
        }

        // Service calls
        void callColor(int visualization_color)
        {
            while (!client_color_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestColor = std::make_shared<l3cam_interfaces::srv::ChangePointcloudColor::Request>();
            requestColor->visualization_color = visualization_color;

            auto resultColor = client_color_->async_send_request(
                requestColor, std::bind(&LidarConfiguration::colorResponseCallback, this, std::placeholders::_1));
        }

        void callColorRange(int range_min, int range_max)
        {
            while (!client_color_range_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestColorRange = std::make_shared<l3cam_interfaces::srv::ChangePointcloudColorRange::Request>();
            requestColorRange->min_value = range_min;
            requestColorRange->max_value = range_max;

            auto resultColorRange = client_color_range_->async_send_request(
                requestColorRange, std::bind(&LidarConfiguration::colorRangeResponseCallback, this, std::placeholders::_1));
        }

        void callDistanceRange(int range_min, int range_max)
        {
            while (!client_distance_range_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestDistanceRange = std::make_shared<l3cam_interfaces::srv::ChangeDistanceRange::Request>();
            requestDistanceRange->min_value = range_min;
            requestDistanceRange->max_value = range_max;

            auto resultDistanceRange = client_distance_range_->async_send_request(
                requestDistanceRange, std::bind(&LidarConfiguration::distanceRangeResponseCallback, this, std::placeholders::_1));
        }

        void callAutoBias(bool enabled)
        {
            while (!client_auto_bias_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestAutoBias = std::make_shared<l3cam_interfaces::srv::EnableAutoBias::Request>();
            requestAutoBias->enabled = enabled;

            auto resultAutoBias = client_auto_bias_->async_send_request(
                requestAutoBias, std::bind(&LidarConfiguration::autoBiasResponseCallback, this, std::placeholders::_1));
        }

        void callBiasValue(int index, int bias)
        {
            while (!client_bias_value_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestBiasValueRight = std::make_shared<l3cam_interfaces::srv::ChangeBiasValue::Request>();
            requestBiasValueRight->index = index;
            requestBiasValueRight->bias = bias;

            auto resultBiasValueRight = client_bias_value_->async_send_request(
                requestBiasValueRight, std::bind(&LidarConfiguration::biasValueRightResponseCallback, this, std::placeholders::_1));
        }

        void callStreamingProtocol(int protocol)
        {
            while (!client_streaming_protocol_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestStreamingProtocol = std::make_shared<l3cam_interfaces::srv::ChangeStreamingProtocol::Request>();
            requestStreamingProtocol->sensor_type = (int)sensorTypes::sensor_lidar;
            requestStreamingProtocol->protocol = protocol;

            auto resultStreamingProtocol = client_streaming_protocol_->async_send_request(
                requestStreamingProtocol, std::bind(&LidarConfiguration::streamingProtocolResponseCallback, this, std::placeholders::_1));
        }

        // Service callbacks
        void colorResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangePointcloudColor>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    pointcloud_color_ = this->get_parameter("pointcloud_color").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("pointcloud_color", pointcloud_color_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_pointcloud_color");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("pointcloud_color", pointcloud_color_));
            }
        }

        void colorRangeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangePointcloudColorRange>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameters changed successfully, save value
                    pointcloud_color_range_minimum_ = this->get_parameter("pointcloud_color_range_minimum").as_int();
                    pointcloud_color_range_maximum_ = this->get_parameter("pointcloud_color_range_maximum").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    this->set_parameter(rclcpp::Parameter("pointcloud_color_range_minimum", pointcloud_color_range_minimum_));
                    this->set_parameter(rclcpp::Parameter("pointcloud_color_range_maximum", pointcloud_color_range_maximum_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_pointcloud_color_range");
                // Service could not be called, reset parameters to value before change
                this->set_parameter(rclcpp::Parameter("pointcloud_color_range_minimum", pointcloud_color_range_minimum_));
                this->set_parameter(rclcpp::Parameter("pointcloud_color_range_maximum", pointcloud_color_range_maximum_));
            }
        }

        void distanceRangeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeDistanceRange>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameters changed successfully, save value
                    distance_range_minimum_ = this->get_parameter("distance_range_minimum").as_int();
                    distance_range_maximum_ = this->get_parameter("distance_range_maximum").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    this->set_parameter(rclcpp::Parameter("distance_range_minimum", distance_range_minimum_));
                    this->set_parameter(rclcpp::Parameter("distance_range_maximum", distance_range_maximum_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_distance_range");
                // Service could not be called, reset parameters to value before change
                this->set_parameter(rclcpp::Parameter("distance_range_minimum", distance_range_minimum_));
                this->set_parameter(rclcpp::Parameter("distance_range_maximum", distance_range_maximum_));
            }
        }

        void autoBiasResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::EnableAutoBias>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                // Parameter changed successfully, save value
                auto_bias_ = this->get_parameter("auto_bias").as_bool();
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service enable_auto_bias");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("auto_bias", auto_bias_));
            }
        }

        void biasValueRightResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeBiasValue>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                // Parameter changed successfully, save value
                bias_value_right_ = this->get_parameter("bias_value_right").as_int();
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service bias_value");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("bias_value_right", bias_value_right_));
            }
        }

        void biasValueLeftResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeBiasValue>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                // Parameter changed successfully, save value
                bias_value_left_ = this->get_parameter("bias_value_left").as_int();
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service bias_value");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("bias_value_left", bias_value_left_));
            }
        }

        void streamingProtocolResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeStreamingProtocol>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    streaming_protocol_ = this->get_parameter("lidar_streaming_protocol").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("lidar_streaming_protocol", streaming_protocol_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_streaming_protocol");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("lidar_streaming_protocol", streaming_protocol_));
            }
        }

        void sensorDisconnectedCallback(const std::shared_ptr<l3cam_interfaces::srv::SensorDisconnected::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::SensorDisconnected::Response> res)
        {
            ROS2_BMG_UNUSED(res);
            if (req->code == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Exiting cleanly.");
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Exiting. Sensor got disconnected with error " << req->code << ": " << getErrorDescription(req->code));
            }

            rclcpp::shutdown();
        }

        rclcpp::Client<l3cam_interfaces::srv::ChangePointcloudColor>::SharedPtr client_color_;
        rclcpp::Client<l3cam_interfaces::srv::ChangePointcloudColorRange>::SharedPtr client_color_range_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeDistanceRange>::SharedPtr client_distance_range_;
        rclcpp::Client<l3cam_interfaces::srv::EnableAutoBias>::SharedPtr client_auto_bias_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeBiasValue>::SharedPtr client_bias_value_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeStreamingProtocol>::SharedPtr client_streaming_protocol_;

        rclcpp::Service<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr srv_sensor_disconnected_;

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

        int pointcloud_color_;
        int pointcloud_color_range_minimum_;
        int pointcloud_color_range_maximum_;
        int distance_range_minimum_;
        int distance_range_maximum_;
        bool auto_bias_;
        int bias_value_right_;
        int bias_value_left_;
        int streaming_protocol_;

    }; // class LidarConfiguration

} // namespace l3cam_ros2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::LidarConfiguration> node = std::make_shared<l3cam_ros2::LidarConfiguration>();

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
        error = resultGetSensors.get()->error;

        if (!error)
        {
            for (int i = 0; i < resultGetSensors.get()->num_sensors; ++i)
            {
                if (resultGetSensors.get()->sensors[i].sensor_type == sensor_lidar && resultGetSensors.get()->sensors[i].sensor_available)
                    sensor_is_available = true;
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LiDAR configuration is available");
    }
    else
    {
        return 0;
    }

    // Get pipeline
    while (!node->client_get_rtsp_pipeline_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
            return 0;
        }
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto requestGetRtspPipeline = std::make_shared<l3cam_interfaces::srv::GetRtspPipeline::Request>();
    requestGetRtspPipeline.get()->sensor_type = (int)sensorTypes::sensor_lidar;
    auto resultGetRtspPipeline = node->client_get_rtsp_pipeline_->async_send_request(requestGetRtspPipeline);

    if (rclcpp::spin_until_future_complete(node, resultGetRtspPipeline) == rclcpp::FutureReturnCode::SUCCESS)
    {
        error = resultGetRtspPipeline.get()->error;

        if (!error)
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.read_only = true;
            node->declare_parameter("lidar_rtsp_pipeline", resultGetRtspPipeline.get()->pipeline, descriptor);
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR " << error << " while getting pipeline in " << __func__ << ": " << getErrorDescription(error));
            return error;
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service get_rtsp_pipeline");
        return L3CAM_ROS2_FAILED_TO_CALL_SERVICE;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
