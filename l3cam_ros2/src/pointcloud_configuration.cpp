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

#include "l3cam_interfaces/srv/sensor_disconnected.hpp"

#include "l3cam_ros2_node.hpp" // for ROS2_BMG_UNUSED

using namespace std::chrono_literals;

namespace l3cam_ros2
{
    class PointCloudConfiguration : public rclcpp::Node
    {
    public:
        PointCloudConfiguration() : Node("pointcloud_configuration")
        {
            declareGetParameters();

            // Create service clients
            clientColor = this->create_client<l3cam_interfaces::srv::ChangePointcloudColor>("change_pointcloud_color");
            clientColorRange = this->create_client<l3cam_interfaces::srv::ChangePointcloudColorRange>("change_pointcloud_color_range");
            clientDistanceRange = this->create_client<l3cam_interfaces::srv::ChangeDistanceRange>("change_distance_range");
            clientAutoBias = this->create_client<l3cam_interfaces::srv::EnableAutoBias>("enable_auto_bias");
            clientBiasValue = this->create_client<l3cam_interfaces::srv::ChangeBiasValue>("change_bias_value");

            clientGetSensors = this->create_client<l3cam_interfaces::srv::GetSensorsAvailable>("get_sensors_available");

            // Create service server
            srvSensorDisconnected = this->create_service<l3cam_interfaces::srv::SensorDisconnected>(
                "pointcloud_configuration_disconnected", std::bind(&PointCloudConfiguration::sensorDisconnectedCallback, this, std::placeholders::_1, std::placeholders::_2));

            // Callback on parameters changed
            callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&PointCloudConfiguration::parametersCallback, this, std::placeholders::_1));
        }

        rclcpp::Client<l3cam_interfaces::srv::GetSensorsAvailable>::SharedPtr clientGetSensors;

    private:
        void declareGetParameters()
        {
            // Declare parameters with range
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            rcl_interfaces::msg::IntegerRange range;
            range.set__from_value(0).set__to_value(13); // TODO: dynamic reconfigure dropdown menu pointCloudColor
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

            // Get and save parameters
            pointcloud_color = this->get_parameter("pointcloud_color").as_int();
            pointcloud_color_range_minimum = this->get_parameter("pointcloud_color_range_minimum").as_int();
            pointcloud_color_range_maximum = this->get_parameter("pointcloud_color_range_maximum").as_int();
            distance_range_minimum = this->get_parameter("distance_range_minimum").as_int();
            distance_range_maximum = this->get_parameter("distance_range_maximum").as_int();
            auto_bias = this->get_parameter("auto_bias").as_bool();
            bias_value_right = this->get_parameter("bias_value_right").as_int();
            bias_value_left = this->get_parameter("bias_value_left").as_int();
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
                if (param_name == "pointcloud_color" && param.as_int() != pointcloud_color)
                {
                    while (!clientColor->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestColor = std::make_shared<l3cam_interfaces::srv::ChangePointcloudColor::Request>();
                    requestColor->visualization_color = param.as_int();

                    auto resultColor = clientColor->async_send_request(
                        requestColor, std::bind(&PointCloudConfiguration::colorResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "pointcloud_color_range_minimum" && param.as_int() != pointcloud_color_range_minimum)
                {
                    while (!clientColorRange->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestColorRange = std::make_shared<l3cam_interfaces::srv::ChangePointcloudColorRange::Request>();
                    requestColorRange->min_value = param.as_int();
                    requestColorRange->max_value = pointcloud_color_range_maximum;

                    auto resultColorRange = clientColorRange->async_send_request(
                        requestColorRange, std::bind(&PointCloudConfiguration::colorRangeResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "pointcloud_color_range_maximum" && param.as_int() != pointcloud_color_range_maximum)
                {
                    while (!clientColorRange->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestColorRange = std::make_shared<l3cam_interfaces::srv::ChangePointcloudColorRange::Request>();
                    requestColorRange->min_value = pointcloud_color_range_minimum;
                    requestColorRange->max_value = param.as_int();

                    auto resultColorRange = clientColorRange->async_send_request(
                        requestColorRange, std::bind(&PointCloudConfiguration::colorRangeResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "distance_range_minimum" && param.as_int() != distance_range_minimum)
                {
                    while (!clientDistanceRange->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestDistanceRange = std::make_shared<l3cam_interfaces::srv::ChangeDistanceRange::Request>();
                    requestDistanceRange->min_value = param.as_int();
                    requestDistanceRange->max_value = distance_range_maximum;

                    auto resultDistanceRange = clientDistanceRange->async_send_request(
                        requestDistanceRange, std::bind(&PointCloudConfiguration::distanceRangeResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "distance_range_maximum" && param.as_int() != distance_range_maximum)
                {
                    while (!clientDistanceRange->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestDistanceRange = std::make_shared<l3cam_interfaces::srv::ChangeDistanceRange::Request>();
                    requestDistanceRange->min_value = distance_range_minimum;
                    requestDistanceRange->max_value = param.as_int();

                    auto resultDistanceRange = clientDistanceRange->async_send_request(
                        requestDistanceRange, std::bind(&PointCloudConfiguration::distanceRangeResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "auto_bias" && param.as_bool() != auto_bias)
                {
                    while (!clientAutoBias->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestAutoBias = std::make_shared<l3cam_interfaces::srv::EnableAutoBias::Request>();
                    requestAutoBias->enabled = param.as_bool();

                    auto resultAutoBias = clientAutoBias->async_send_request(
                        requestAutoBias, std::bind(&PointCloudConfiguration::autoBiasResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "bias_value_right" && param.as_int() != bias_value_right)
                {
                    while (!clientBiasValue->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestBiasValueRight = std::make_shared<l3cam_interfaces::srv::ChangeBiasValue::Request>();
                    requestBiasValueRight->index = 1;
                    requestBiasValueRight->bias = param.as_int();

                    auto resultBiasValueRight = clientBiasValue->async_send_request(
                        requestBiasValueRight, std::bind(&PointCloudConfiguration::biasValueRightResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "bias_value_left" && param.as_int() != bias_value_left)
                {
                    while (!clientBiasValue->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestBiasValue = std::make_shared<l3cam_interfaces::srv::ChangeBiasValue::Request>();
                    requestBiasValue->index = 2;
                    requestBiasValue->bias = param.as_int();

                    auto resultBiasValue = clientBiasValue->async_send_request(
                        requestBiasValue, std::bind(&PointCloudConfiguration::biasValueLeftResponseCallback, this, std::placeholders::_1));
                }
            }

            return result;
        }

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
                    pointcloud_color = this->get_parameter("pointcloud_color").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("pointcloud_color", pointcloud_color));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service change_pointcloud_color");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("pointcloud_color", pointcloud_color));
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
                    pointcloud_color_range_minimum = this->get_parameter("pointcloud_color_range_minimum").as_int();
                    pointcloud_color_range_maximum = this->get_parameter("pointcloud_color_range_maximum").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getBeamErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    this->set_parameter(rclcpp::Parameter("pointcloud_color_range_minimum", pointcloud_color_range_minimum));
                    this->set_parameter(rclcpp::Parameter("pointcloud_color_range_maximum", pointcloud_color_range_maximum));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service change_pointcloud_color_range");
                // Service could not be called, reset parameters to value before change
                this->set_parameter(rclcpp::Parameter("pointcloud_color_range_minimum", pointcloud_color_range_minimum));
                this->set_parameter(rclcpp::Parameter("pointcloud_color_range_maximum", pointcloud_color_range_maximum));
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
                    distance_range_minimum = this->get_parameter("distance_range_minimum").as_int();
                    distance_range_maximum = this->get_parameter("distance_range_maximum").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getBeamErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    this->set_parameter(rclcpp::Parameter("distance_range_minimum", distance_range_minimum));
                    this->set_parameter(rclcpp::Parameter("distance_range_maximum", distance_range_maximum));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service change_distance_range");
                // Service could not be called, reset parameters to value before change
                this->set_parameter(rclcpp::Parameter("distance_range_minimum", distance_range_minimum));
                this->set_parameter(rclcpp::Parameter("distance_range_maximum", distance_range_maximum));
            }
        }

        void autoBiasResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::EnableAutoBias>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                // Parameter changed successfully, save value
                auto_bias = this->get_parameter("auto_bias").as_bool();
            }
            else
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service enable_auto_bias");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("auto_bias", auto_bias));
            }
        }

        void biasValueRightResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeBiasValue>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                // Parameter changed successfully, save value
                bias_value_right = this->get_parameter("bias_value_right").as_int();
            }
            else
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service bias_value");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("bias_value_right", bias_value_right));
            }
        }

        void biasValueLeftResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeBiasValue>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                // Parameter changed successfully, save value
                bias_value_left = this->get_parameter("bias_value_left").as_int();
            }
            else
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service bias_value");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("bias_value_left", bias_value_left));
            }
        }

        void sensorDisconnectedCallback(const std::shared_ptr<l3cam_interfaces::srv::SensorDisconnected::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::SensorDisconnected::Response> res)
        {
            ROS2_BMG_UNUSED(res);
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

        int pointcloud_color;
        int pointcloud_color_range_minimum;
        int pointcloud_color_range_maximum;
        int distance_range_minimum;
        int distance_range_maximum;
        bool auto_bias;
        int bias_value_right;
        int bias_value_left;

        rclcpp::Client<l3cam_interfaces::srv::ChangePointcloudColor>::SharedPtr clientColor;
        rclcpp::Client<l3cam_interfaces::srv::ChangePointcloudColorRange>::SharedPtr clientColorRange;
        rclcpp::Client<l3cam_interfaces::srv::ChangeDistanceRange>::SharedPtr clientDistanceRange;
        rclcpp::Client<l3cam_interfaces::srv::EnableAutoBias>::SharedPtr clientAutoBias;
        rclcpp::Client<l3cam_interfaces::srv::ChangeBiasValue>::SharedPtr clientBiasValue;

        rclcpp::Service<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr srvSensorDisconnected;

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    }; // class PointCloudConfiguration

} // namespace l3cam_ros2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::PointCloudConfiguration> node = std::make_shared<l3cam_ros2::PointCloudConfiguration>();

    // Check if LiDAR is available
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
                if (resultGetSensors.get()->sensors[i].sensor_type == sensor_lidar && resultGetSensors.get()->sensors[i].sensor_available)
                    sensor_is_available = true;
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LiDAR configuration is available");
    }
    else
    {
        return 0;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
