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

#include "l3cam_interfaces/srv/change_allied_camera_exposure_time.hpp"
#include "l3cam_interfaces/srv/enable_allied_camera_auto_exposure_time.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_auto_exposure_time_range.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_gain.hpp"
#include "l3cam_interfaces/srv/enable_allied_camera_auto_gain.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_auto_gain_range.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_gamma.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_hue.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_intensity_auto_precedence.hpp"
#include "l3cam_interfaces/srv/enable_allied_camera_auto_white_balance.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_balance_ratio_selector.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_balance_ratio.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_balance_white_auto_rate.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_balance_white_auto_tolerance.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_intensity_controller_region.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_intensity_controller_target.hpp"

#include "l3cam_interfaces/srv/sensor_disconnected.hpp"

#include "l3cam_ros2_node.hpp" // for ROS2_BMG_UNUSED

using namespace std::chrono_literals;

namespace l3cam_ros2
{
    class AlliedNarrowConfiguration : public rclcpp::Node
    {
    public:
        AlliedNarrowConfiguration() : Node("allied_narrow_configuration")
        {
            declareGetParameters();

            // Create service clients
            clientGetSensors = this->create_client<l3cam_interfaces::srv::GetSensorsAvailable>("get_sensors_available");
            clientExposureTime = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime>("change_allied_camera_exposure_time");
            clientAutoExposureTime = this->create_client<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime>("enable_allied_camera_auto_exposure_time");
            clientAutoExposureTimeRange = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange>("change_allied_camera_auto_exposure_time_range");
            clientGain = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraGain>("change_allied_camera_gain");
            clientAutoGain = this->create_client<l3cam_interfaces::srv::EnableAlliedCameraAutoGain>("enable_allied_camera_auto_gain");
            clientAutoGainRange = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange>("change_allied_camera_auto_gain_range");
            clientGamma = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraGamma>("change_allied_camera_gamma");
            clientHue = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraHue>("change_allied_camera_hue");
            clientIntensityAutoPrecedence = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence>("change_allied_camera_intensity_auto_precedence");
            clientAutoWhiteBalance = this->create_client<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance>("enable_allied_camera_auto_white_balance");
            clientBalanceRatioSelector = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector>("change_allied_camera_balance_ratio_selector");
            clientBalanceRatio = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio>("change_allied_camera_balance_ratio");
            clientBalanceWhiteAutoRate = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate>("change_allied_camera_balance_white_auto_rate");
            clientBalanceWhiteAutoTolerance = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance>("change_allied_camera_balance_white_auto_tolerance");
            clientIntensityControllerRegion = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion>("change_allied_camera_intensity_controller_region");
            clientIntensityControllerTarget = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget>("change_allied_camera_intensity_controller_target");

            // Create service server
            srvSensorDisconnected = this->create_service<l3cam_interfaces::srv::SensorDisconnected>(
                "allied_narrow_configuration_disconnected", std::bind(&AlliedNarrowConfiguration::sensorDisconnectedCallback, this, std::placeholders::_1, std::placeholders::_2));

            // Callback on parameters changed
            callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&AlliedNarrowConfiguration::parametersCallback, this, std::placeholders::_1));
        }

        rclcpp::Client<l3cam_interfaces::srv::GetSensorsAvailable>::SharedPtr clientGetSensors;

    private:
        void declareGetParameters()
        {
            // Declare parameters with range
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            rcl_interfaces::msg::IntegerRange intRange;
            rcl_interfaces::msg::FloatingPointRange floatRange;
            // this->declare_parameter("allied_narrow_camera_black_level", 0.0); // 0 - 4095
            floatRange.set__from_value(63.0).set__to_value(10000000.0).set__step(0.1);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_narrow_camera_exposure_time", 4992.4, descriptor); // 63 - 10000000
            this->declare_parameter("allied_narrow_camera_auto_exposure_time", false);
            floatRange.set__from_value(63.1).set__to_value(8999990.0).set__step(0.1);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_narrow_camera_auto_exposure_time_range_min", 87.6, descriptor); // 63.1 - 8999990
            floatRange.set__from_value(87.6).set__to_value(10000000.0).set__step(0.1);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_narrow_camera_auto_exposure_time_range_max", 8999990.0, descriptor); // 87.6 - 10000000
            floatRange.set__from_value(0.0).set__to_value(48.0).set__step(0.1);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_narrow_camera_gain", 0.0, descriptor); // 0 - 48
            this->declare_parameter("allied_narrow_camera_auto_gain", false);
            floatRange.set__from_value(0.0).set__to_value(48.0).set__step(0.1);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_narrow_camera_auto_gain_range_min", 0.0, descriptor); // 0 - 48
            floatRange.set__from_value(0.0).set__to_value(48.0).set__step(0.1);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_narrow_camera_auto_gain_range_max", 48.0, descriptor); // 0 - 48
            floatRange.set__from_value(0.4).set__to_value(2.4).set__step(0.1);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_narrow_camera_gamma", 1.0, descriptor); // 0.4 - 2.4
            floatRange.set__from_value(0.0).set__to_value(2.0).set__step(0.1);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_narrow_camera_saturation", 1.0, descriptor); // 0 - 2
            // this->declare_parameter("allied_narrow_camera_sharpness", 0.0); // -12 - 12
            floatRange.set__from_value(-40).set__to_value(40.0).set__step(0.1);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_narrow_camera_hue", 0.0, descriptor); // -40 - 40
            intRange.set__from_value(0).set__to_value(1).set__step(1);            // TODO: dropdown menu
            descriptor.integer_range = {intRange};
            descriptor.description =
                "Value must be:\n"
                "\tMinimizeNoise = 0\n"
                "\tMinimizeBlur = 1";
            this->declare_parameter("allied_narrow_camera_intensity_auto_precedence", 0, descriptor); // 0(MinimizeNoise) or 1(MinimizeBlur)
            descriptor.description = "";
            this->declare_parameter("allied_narrow_camera_auto_white_balance", false);
            intRange.set__from_value(0).set__to_value(1).set__step(1); // TODO: dropdown menu
            descriptor.integer_range = {intRange};
            descriptor.description =
                "Value must be:\n"
                "\tRed = 0\n"
                "\tBlue = 1";
            this->declare_parameter("allied_narrow_camera_balance_ratio_selector", 0, descriptor); // 0(Red), 1(Blue)
            descriptor.description = "";
            floatRange.set__from_value(0.0).set__to_value(8.0).set__step(0.1);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_narrow_camera_balance_ratio", 2.4, descriptor); // 0 - 8
            floatRange.set__from_value(0.0).set__to_value(100.0).set__step(0.1);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_narrow_camera_balance_white_auto_rate", 100.0, descriptor); // 0 - 100
            floatRange.set__from_value(0.0).set__to_value(50.0).set__step(0.1);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_narrow_camera_balance_white_auto_tolerance", 5.0, descriptor); // 0 - 50
            // this->declare_parameter("allied_narrow_camera_auto_mode_region_height", 2056); // 0 - 2056
            // this->declare_parameter("allied_narrow_camera_auto_mode_region_width", 2464); // 0 - 2464
            intRange.set__from_value(0).set__to_value(4).set__step(1); // TODO: dropdown menu
            descriptor.integer_range = {intRange};
            descriptor.description =
                "Value must be:\n"
                "\tAutoMode = 0\n"
                "\tFullImage = 4";
            this->declare_parameter("allied_narrow_camera_intensity_controller_region", 0, descriptor); // 0(AutoMode), 4(FullImage)
            descriptor.description = "";
            floatRange.set__from_value(10).set__to_value(90).set__step(0.1);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_narrow_camera_intensity_controller_target", 50.0, descriptor); // 10 - 90
            // this->declare_parameter("allied_narrow_camera_max_driver_buffers_count", 64); // 1 - 4096

            // Get and save parameters
            allied_narrow_camera_exposure_time = this->get_parameter("allied_narrow_camera_exposure_time").as_double();
            allied_narrow_camera_auto_exposure_time = this->get_parameter("allied_narrow_camera_auto_exposure_time").as_bool();
            allied_narrow_camera_auto_exposure_time_range_min = this->get_parameter("allied_narrow_camera_auto_exposure_time_range_min").as_double();
            allied_narrow_camera_auto_exposure_time_range_max = this->get_parameter("allied_narrow_camera_auto_exposure_time_range_max").as_double();
            allied_narrow_camera_gain = this->get_parameter("allied_narrow_camera_gain").as_double(); //!!!!!
            allied_narrow_camera_auto_gain = this->get_parameter("allied_narrow_camera_auto_gain").as_bool();
            allied_narrow_camera_auto_gain_range_min = this->get_parameter("allied_narrow_camera_auto_gain_range_min").as_double();
            allied_narrow_camera_auto_gain_range_max = this->get_parameter("allied_narrow_camera_auto_gain_range_max").as_double();
            allied_narrow_camera_gamma = this->get_parameter("allied_narrow_camera_gamma").as_double();
            allied_narrow_camera_saturation = this->get_parameter("allied_narrow_camera_saturation").as_double();
            allied_narrow_camera_hue = this->get_parameter("allied_narrow_camera_hue").as_double();
            allied_narrow_camera_intensity_auto_precedence = this->get_parameter("allied_narrow_camera_intensity_auto_precedence").as_int();
            allied_narrow_camera_auto_white_balance = this->get_parameter("allied_narrow_camera_auto_white_balance").as_bool();
            allied_narrow_camera_balance_ratio_selector = this->get_parameter("allied_narrow_camera_balance_ratio_selector").as_int();
            allied_narrow_camera_balance_ratio = this->get_parameter("allied_narrow_camera_balance_ratio").as_double();
            allied_narrow_camera_balance_white_auto_rate = this->get_parameter("allied_narrow_camera_balance_white_auto_rate").as_double();
            allied_narrow_camera_balance_white_auto_tolerance = this->get_parameter("allied_narrow_camera_balance_white_auto_tolerance").as_double();
            allied_narrow_camera_intensity_controller_region = this->get_parameter("allied_narrow_camera_intensity_controller_region").as_int();
            allied_narrow_camera_intensity_controller_target = this->get_parameter("allied_narrow_camera_intensity_controller_target").as_double();
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
                if (param_name == "allied_narrow_camera_exposure_time")
                {
                    while (!clientExposureTime->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestExposureTime = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime::Request>();
                    requestExposureTime->allied_type = 2;
                    requestExposureTime->exposure_time = param.as_double();

                    auto resultExposureTime = clientExposureTime->async_send_request(
                        requestExposureTime, std::bind(&AlliedNarrowConfiguration::exposureTimeResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_auto_exposure_time")
                {
                    while (!clientAutoExposureTime->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestAutoExposureTime = std::make_shared<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime::Request>();
                    requestAutoExposureTime->allied_type = 2;
                    requestAutoExposureTime->enabled = param.as_bool();

                    auto resultAutoExposureTime = clientAutoExposureTime->async_send_request(
                        requestAutoExposureTime, std::bind(&AlliedNarrowConfiguration::autoExposureTimeResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_auto_exposure_time_range_min")
                {
                    while (!clientAutoExposureTimeRange->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestAutoExposureTimeRange = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange::Request>();
                    requestAutoExposureTimeRange->allied_type = 2;
                    requestAutoExposureTimeRange->auto_exposure_time_range_min = param.as_double();
                    requestAutoExposureTimeRange->auto_exposure_time_range_max = allied_narrow_camera_auto_exposure_time_range_max;

                    auto resultAutoExposureTimeRange = clientAutoExposureTimeRange->async_send_request(
                        requestAutoExposureTimeRange, std::bind(&AlliedNarrowConfiguration::autoExposureTimeRangeResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_auto_exposure_time_range_max")
                {
                    while (!clientAutoExposureTimeRange->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestAutoExposureTimeRange = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange::Request>();
                    requestAutoExposureTimeRange->allied_type = 2;
                    requestAutoExposureTimeRange->auto_exposure_time_range_min = allied_narrow_camera_auto_exposure_time_range_min;
                    requestAutoExposureTimeRange->auto_exposure_time_range_max = param.as_double();

                    auto resultAutoExposureTimeRange = clientAutoExposureTimeRange->async_send_request(
                        requestAutoExposureTimeRange, std::bind(&AlliedNarrowConfiguration::autoExposureTimeRangeResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_gain")
                {
                    while (!clientGain->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestGain = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraGain::Request>();
                    requestGain->allied_type = 2;
                    requestGain->gain = param.as_double();

                    auto resultGain = clientGain->async_send_request(
                        requestGain, std::bind(&AlliedNarrowConfiguration::gainResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_auto_gain")
                {
                    while (!clientAutoGain->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestAutoGain = std::make_shared<l3cam_interfaces::srv::EnableAlliedCameraAutoGain::Request>();
                    requestAutoGain->allied_type = 2;
                    requestAutoGain->enabled = param.as_bool();

                    auto resultAutoGain = clientAutoGain->async_send_request(
                        requestAutoGain, std::bind(&AlliedNarrowConfiguration::autoGainResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_auto_gain_range_min")
                {
                    while (!clientAutoGainRange->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestAutoGainRange = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange::Request>();
                    requestAutoGainRange->allied_type = 2;
                    requestAutoGainRange->auto_gain_range_min = param.as_double();
                    requestAutoGainRange->auto_gain_range_max = allied_narrow_camera_auto_gain_range_max;

                    auto resultAutoGainRange = clientAutoGainRange->async_send_request(
                        requestAutoGainRange, std::bind(&AlliedNarrowConfiguration::autoGainRangeResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_auto_gain_range_max")
                {
                    while (!clientAutoGainRange->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestAutoGainRange = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange::Request>();
                    requestAutoGainRange->allied_type = 2;
                    requestAutoGainRange->auto_gain_range_min = allied_narrow_camera_auto_gain_range_min;
                    requestAutoGainRange->auto_gain_range_max = param.as_double();

                    auto resultAutoGainRange = clientAutoGainRange->async_send_request(
                        requestAutoGainRange, std::bind(&AlliedNarrowConfiguration::autoGainRangeResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_gamma")
                {
                    while (!clientGamma->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestGamma = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraGamma::Request>();
                    requestGamma->allied_type = 2;
                    requestGamma->gamma = param.as_double();

                    auto resultGamma = clientGamma->async_send_request(
                        requestGamma, std::bind(&AlliedNarrowConfiguration::gammaResponseCallback, this, std::placeholders::_1));
                }

                if (param_name == "allied_narrow_camera_hue")
                {
                    while (!clientHue->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestHue = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraHue::Request>();
                    requestHue->allied_type = 2;
                    requestHue->hue = param.as_double();

                    auto resultHue = clientHue->async_send_request(
                        requestHue, std::bind(&AlliedNarrowConfiguration::hueResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_intensity_auto_precedence")
                {
                    while (!clientIntensityAutoPrecedence->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestIntensityAutoPrecedence = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence::Request>();
                    requestIntensityAutoPrecedence->allied_type = 2;
                    requestIntensityAutoPrecedence->intensity_auto_precedence = param.as_int();

                    auto resultIntensityAutoPrecedence = clientIntensityAutoPrecedence->async_send_request(
                        requestIntensityAutoPrecedence, std::bind(&AlliedNarrowConfiguration::intensityAutoPrecedenceResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_auto_white_balance")
                {
                    while (!clientAutoWhiteBalance->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting aWhiteBalance...");
                    }

                    auto requestAutoWhiteBalance = std::make_shared<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance::Request>();
                    requestAutoWhiteBalance->allied_type = 2;
                    requestAutoWhiteBalance->enabled = param.as_bool();

                    auto resultAutoWhiteBalance = clientAutoWhiteBalance->async_send_request(
                        requestAutoWhiteBalance, std::bind(&AlliedNarrowConfiguration::autoWhiteBalanceResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_balance_ratio_selector")
                {
                    while (!clientBalanceRatioSelector->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestBalanceRatioSelector = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector::Request>();
                    requestBalanceRatioSelector->allied_type = 2;
                    requestBalanceRatioSelector->white_balance_ratio_selector = param.as_int();

                    auto resultBalanceRatioSelector = clientBalanceRatioSelector->async_send_request(
                        requestBalanceRatioSelector, std::bind(&AlliedNarrowConfiguration::balanceRatioSelectorResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_balance_ratio")
                {
                    while (!clientBalanceRatio->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestBalanceRatio = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio::Request>();
                    requestBalanceRatio->allied_type = 2;
                    requestBalanceRatio->balance_ratio = param.as_double();

                    auto resultBalanceRatio = clientBalanceRatio->async_send_request(
                        requestBalanceRatio, std::bind(&AlliedNarrowConfiguration::balanceRatioResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_balance_white_auto_rate")
                {
                    while (!clientBalanceWhiteAutoRate->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestBalanceWhiteAutoRate = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate::Request>();
                    requestBalanceWhiteAutoRate->allied_type = 2;
                    requestBalanceWhiteAutoRate->white_balance_auto_rate = param.as_double();

                    auto resultBalanceWhiteAutoRate = clientBalanceWhiteAutoRate->async_send_request(
                        requestBalanceWhiteAutoRate, std::bind(&AlliedNarrowConfiguration::balanceWhiteAutoRateResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_balance_white_auto_tolerance")
                {
                    while (!clientBalanceWhiteAutoTolerance->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestBalanceWhiteAutoTolerance = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance::Request>();
                    requestBalanceWhiteAutoTolerance->allied_type = 2;
                    requestBalanceWhiteAutoTolerance->white_balance_auto_tolerance = param.as_double();

                    auto resultBalanceWhiteAutoTolerance = clientBalanceWhiteAutoTolerance->async_send_request(
                        requestBalanceWhiteAutoTolerance, std::bind(&AlliedNarrowConfiguration::balanceWhiteAutoToleranceResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_intensity_controller_region")
                {
                    while (!clientIntensityControllerRegion->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestIntensityControllerRegion = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion::Request>();
                    requestIntensityControllerRegion->allied_type = 2;
                    requestIntensityControllerRegion->intensity_controller_region = param.as_int();

                    auto resultIntensityControllerRegion = clientIntensityControllerRegion->async_send_request(
                        requestIntensityControllerRegion, std::bind(&AlliedNarrowConfiguration::intensityControllerRegionResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "allied_narrow_camera_intensity_controller_target")
                {
                    while (!clientIntensityControllerTarget->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestIntensityControllerTarget = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget::Request>();
                    requestIntensityControllerTarget->allied_type = 2;
                    requestIntensityControllerTarget->intensity_controller_target = param.as_double();

                    auto resultIntensityControllerTarget = clientIntensityControllerTarget->async_send_request(
                        requestIntensityControllerTarget, std::bind(&AlliedNarrowConfiguration::intensityControllerTargetResponseCallback, this, std::placeholders::_1));
                }
            }

            return result;
        }

        void exposureTimeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_exposure_time = this->get_parameter("allied_narrow_camera_exposure_time").as_double();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_exposure_time", allied_narrow_camera_exposure_time));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_allied_camera_exposure_time");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_exposure_time", allied_narrow_camera_exposure_time));
            }
        }

        void autoExposureTimeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_auto_exposure_time = this->get_parameter("allied_narrow_camera_auto_exposure_time").as_bool();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_exposure_time", allied_narrow_camera_auto_exposure_time));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service enable_allied_camera_auto_exposure_time");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_exposure_time", allied_narrow_camera_auto_exposure_time));
            }
        }

        void autoExposureTimeRangeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameters changed successfully, save values
                    allied_narrow_camera_auto_exposure_time_range_min = this->get_parameter("allied_narrow_camera_auto_exposure_time_range_min").as_double();
                    allied_narrow_camera_auto_exposure_time_range_max = this->get_parameter("allied_narrow_camera_auto_exposure_time_range_max").as_double();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_exposure_time_range_min", allied_narrow_camera_auto_exposure_time_range_min));
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_exposure_time_range_max", allied_narrow_camera_auto_exposure_time_range_max));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_allied_camera_auto_exposure_time_range");
                // Service could not be called, reset parameters to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_exposure_time_range_min", allied_narrow_camera_auto_exposure_time_range_min));
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_exposure_time_range_max", allied_narrow_camera_auto_exposure_time_range_min));
            }
        }

        void gainResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraGain>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_gain = this->get_parameter("allied_narrow_camera_gain").as_double();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_gain", allied_narrow_camera_gain));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_allied_camera_gain");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_gain", allied_narrow_camera_gain));
            }
        }

        void autoGainResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::EnableAlliedCameraAutoGain>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_auto_gain = this->get_parameter("allied_narrow_camera_auto_gain").as_bool();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_gain", allied_narrow_camera_auto_gain));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service enable_allied_camera_auto_gain");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_gain", allied_narrow_camera_auto_gain));
            }
        }

        void autoGainRangeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameters changed successfully, save values
                    allied_narrow_camera_auto_gain_range_min = this->get_parameter("allied_narrow_camera_auto_gain_range_min").as_double();
                    allied_narrow_camera_auto_gain_range_max = this->get_parameter("allied_narrow_camera_auto_gain_range_max").as_double();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_gain_range_min", allied_narrow_camera_auto_gain_range_min));
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_gain_range_max", allied_narrow_camera_auto_gain_range_max));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_allied_camera_auto_gain_range");
                // Service could not be called, reset parameters to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_gain_range_min", allied_narrow_camera_auto_gain_range_min));
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_gain_range_max", allied_narrow_camera_auto_gain_range_min));
            }
        }

        void gammaResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraGamma>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_gamma = this->get_parameter("allied_narrow_camera_gamma").as_double();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_gamma", allied_narrow_camera_gamma));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_allied_camera_gamma");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_gamma", allied_narrow_camera_gamma));
            }
        }

        void hueResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraHue>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_hue = this->get_parameter("allied_narrow_camera_hue").as_double();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_hue", allied_narrow_camera_hue));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_allied_camera_hue");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_hue", allied_narrow_camera_hue));
            }
        }

        void intensityAutoPrecedenceResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_intensity_auto_precedence = this->get_parameter("allied_narrow_camera_intensity_auto_precedence").as_int();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_intensity_auto_precedence", allied_narrow_camera_intensity_auto_precedence));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_allied_camera_intensity_auto_precedence");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_intensity_auto_precedence", allied_narrow_camera_intensity_auto_precedence));
            }
        }

        void autoWhiteBalanceResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_auto_white_balance = this->get_parameter("allied_narrow_camera_auto_white_balance").as_bool();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_white_balance", allied_narrow_camera_auto_white_balance));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service enable_allied_camera_auto_white_balance");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_auto_white_balance", allied_narrow_camera_auto_white_balance));
            }
        }

        void balanceRatioSelectorResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_balance_ratio_selector = this->get_parameter("allied_narrow_camera_balance_ratio_selector").as_int();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_balance_ratio_selector", allied_narrow_camera_balance_ratio_selector));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_allied_camera_balance_ratio_selector");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_balance_ratio_selector", allied_narrow_camera_balance_ratio_selector));
            }
        }

        void balanceRatioResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_balance_ratio = this->get_parameter("allied_narrow_camera_balance_ratio").as_double();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_balance_ratio", allied_narrow_camera_balance_ratio));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_allied_camera_balance_ratio");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_balance_ratio", allied_narrow_camera_balance_ratio));
            }
        }

        void balanceWhiteAutoRateResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_balance_white_auto_rate = this->get_parameter("allied_narrow_camera_balance_white_auto_rate").as_double();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_balance_white_auto_rate", allied_narrow_camera_balance_white_auto_rate));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_allied_camera_balance_white_auto_rate");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_balance_white_auto_rate", allied_narrow_camera_balance_white_auto_rate));
            }
        }

        void balanceWhiteAutoToleranceResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_balance_white_auto_tolerance = this->get_parameter("allied_narrow_camera_balance_white_auto_tolerance").as_double();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_balance_white_auto_tolerance", allied_narrow_camera_balance_white_auto_tolerance));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_allied_camera_balance_white_auto_tolerance");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_balance_white_auto_tolerance", allied_narrow_camera_balance_white_auto_tolerance));
            }
        }

        void intensityControllerRegionResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_intensity_controller_region = this->get_parameter("allied_narrow_camera_intensity_controller_region").as_int();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_intensity_controller_region", allied_narrow_camera_intensity_controller_region));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_allied_camera_intensity_controller_region");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_intensity_controller_region", allied_narrow_camera_intensity_controller_region));
            }
        }

        void intensityControllerTargetResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_narrow_camera_intensity_controller_target = this->get_parameter("allied_narrow_camera_intensity_controller_target").as_double();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_intensity_controller_target", allied_narrow_camera_intensity_controller_target));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_allied_camera_intensity_controller_target");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("allied_narrow_camera_intensity_controller_target", allied_narrow_camera_intensity_controller_target));
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

        double allied_narrow_camera_exposure_time;
        bool allied_narrow_camera_auto_exposure_time;
        double allied_narrow_camera_auto_exposure_time_range_min;
        double allied_narrow_camera_auto_exposure_time_range_max;
        double allied_narrow_camera_gain;
        bool allied_narrow_camera_auto_gain;
        double allied_narrow_camera_auto_gain_range_min;
        double allied_narrow_camera_auto_gain_range_max;
        double allied_narrow_camera_gamma;
        double allied_narrow_camera_saturation;
        double allied_narrow_camera_hue;
        int allied_narrow_camera_intensity_auto_precedence;
        bool allied_narrow_camera_auto_white_balance;
        int allied_narrow_camera_balance_ratio_selector;
        double allied_narrow_camera_balance_ratio;
        double allied_narrow_camera_balance_white_auto_rate;
        double allied_narrow_camera_balance_white_auto_tolerance;
        int allied_narrow_camera_intensity_controller_region;
        double allied_narrow_camera_intensity_controller_target;

        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime>::SharedPtr clientExposureTime;
        rclcpp::Client<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime>::SharedPtr clientAutoExposureTime;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange>::SharedPtr clientAutoExposureTimeRange;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraGain>::SharedPtr clientGain;
        rclcpp::Client<l3cam_interfaces::srv::EnableAlliedCameraAutoGain>::SharedPtr clientAutoGain;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange>::SharedPtr clientAutoGainRange;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraGamma>::SharedPtr clientGamma;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraHue>::SharedPtr clientHue;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence>::SharedPtr clientIntensityAutoPrecedence;
        rclcpp::Client<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance>::SharedPtr clientAutoWhiteBalance;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector>::SharedPtr clientBalanceRatioSelector;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio>::SharedPtr clientBalanceRatio;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate>::SharedPtr clientBalanceWhiteAutoRate;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance>::SharedPtr clientBalanceWhiteAutoTolerance;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion>::SharedPtr clientIntensityControllerRegion;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget>::SharedPtr clientIntensityControllerTarget;

        rclcpp::Service<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr srvSensorDisconnected;

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    }; // class AlliedNarrowConfiguration

} // namespace l3cam_ros2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::AlliedNarrowConfiguration> node = std::make_shared<l3cam_ros2::AlliedNarrowConfiguration>();

    // Check if Allied Narrow is available
    while (!node->clientGetSensors->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
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
                if (resultGetSensors.get()->sensors[i].sensor_type == sensor_allied_narrow)
                    sensor_is_available = true;
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while checking sensor availability: " << getBeamErrorDescription(error));
            return 1;
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_sensors_available");
        return 1;
    }

    if (sensor_is_available)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Allied Narrow camera configuration is available");
    }
    else
    {
        return 0;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
