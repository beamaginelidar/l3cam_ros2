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
#include "l3cam_interfaces/srv/change_allied_camera_saturation.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_hue.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_intensity_auto_precedence.hpp"
#include "l3cam_interfaces/srv/enable_allied_camera_auto_white_balance.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_balance_ratio_selector.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_balance_ratio.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_balance_white_auto_rate.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_balance_white_auto_tolerance.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_intensity_controller_region.hpp"
#include "l3cam_interfaces/srv/change_allied_camera_intensity_controller_target.hpp"
#include "l3cam_interfaces/srv/change_streaming_protocol.hpp"
#include "l3cam_interfaces/srv/get_rtsp_pipeline.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_exposure_time.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_gain.hpp"

#include "l3cam_interfaces/srv/sensor_disconnected.hpp"

#include "l3cam_ros2_utils.hpp"

using namespace std::chrono_literals;

namespace l3cam_ros2
{
    class AlliedWideConfiguration : public rclcpp::Node
    {
    public:
        explicit AlliedWideConfiguration() : Node("allied_wide_configuration")
        {
            // Create service clients
            client_get_sensors_ = this->create_client<l3cam_interfaces::srv::GetSensorsAvailable>("get_sensors_available");
            client_exposure_time_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime>("change_allied_camera_exposure_time");
            client_auto_exposure_time_ = this->create_client<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime>("enable_allied_camera_auto_exposure_time");
            client_auto_exposure_time_range_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange>("change_allied_camera_auto_exposure_time_range");
            client_gain_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraGain>("change_allied_camera_gain");
            client_auto_gain_ = this->create_client<l3cam_interfaces::srv::EnableAlliedCameraAutoGain>("enable_allied_camera_auto_gain");
            client_auto_gain_range_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange>("change_allied_camera_auto_gain_range");
            client_gamma_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraGamma>("change_allied_camera_gamma");
            client_saturation_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraSaturation>("change_allied_camera_saturation");
            client_hue_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraHue>("change_allied_camera_hue");
            client_intensity_auto_precedence_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence>("change_allied_camera_intensity_auto_precedence");
            client_auto_white_balance_ = this->create_client<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance>("enable_allied_camera_auto_white_balance");
            client_balance_ratio_selector_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector>("change_allied_camera_balance_ratio_selector");
            client_balance_ratio_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio>("change_allied_camera_balance_ratio");
            client_balance_white_auto_rate_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate>("change_allied_camera_balance_white_auto_rate");
            client_balance_white_auto_tolerance_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance>("change_allied_camera_balance_white_auto_tolerance");
            client_intensity_controller_region_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion>("change_allied_camera_intensity_controller_region");
            client_intensity_controller_target_ = this->create_client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget>("change_allied_camera_intensity_controller_target");
            client_streaming_protocol_ = this->create_client<l3cam_interfaces::srv::ChangeStreamingProtocol>("change_streaming_protocol");
            client_get_rtsp_pipeline_ = this->create_client<l3cam_interfaces::srv::GetRtspPipeline>("get_rtsp_pipeline");
            client_get_exposure_time_ = this->create_client<l3cam_interfaces::srv::GetAlliedCameraExposureTime>("get_allied_camera_exposure_time");
            client_get_gain_ = this->create_client<l3cam_interfaces::srv::GetAlliedCameraGain>("get_allied_camera_gain");

            declareParams();
            loadDefaultParams();

            // Create service server
            srv_sensor_disconnected_ = this->create_service<l3cam_interfaces::srv::SensorDisconnected>(
                "allied_wide_configuration_disconnected", std::bind(&AlliedWideConfiguration::sensorDisconnectedCallback, this, std::placeholders::_1, std::placeholders::_2));

            // Callback on parameters changed
            callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&AlliedWideConfiguration::parametersCallback, this, std::placeholders::_1));
        }

        rclcpp::Client<l3cam_interfaces::srv::GetSensorsAvailable>::SharedPtr client_get_sensors_;
        rclcpp::Client<l3cam_interfaces::srv::GetRtspPipeline>::SharedPtr client_get_rtsp_pipeline_;

    private:
        void declareParams()
        {
            // Declare parameters with range
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            rcl_interfaces::msg::IntegerRange intRange;
            rcl_interfaces::msg::FloatingPointRange floatRange;
            this->declare_parameter("timeout_secs", 60);
            floatRange.set__from_value(63.0).set__to_value(10000000.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_wide_camera_exposure_time", 4992.4, descriptor); // 63 - 10000000
            this->declare_parameter("allied_wide_camera_auto_exposure_time", false);
            floatRange.set__from_value(63.1).set__to_value(8999990.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_wide_camera_auto_exposure_time_range_min", 87.6, descriptor); // 63.1 - 8999990
            floatRange.set__from_value(87.6).set__to_value(10000000.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_wide_camera_auto_exposure_time_range_max", 8999990.0, descriptor); // 87.6 - 10000000
            floatRange.set__from_value(0.0).set__to_value(48.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_wide_camera_gain", 0.0, descriptor); // 0 - 48
            this->declare_parameter("allied_wide_camera_auto_gain", false);
            floatRange.set__from_value(0.0).set__to_value(48.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_wide_camera_auto_gain_range_min", 0.0, descriptor); // 0 - 48
            floatRange.set__from_value(0.0).set__to_value(48.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_wide_camera_auto_gain_range_max", 48.0, descriptor); // 0 - 48
            floatRange.set__from_value(0.4).set__to_value(2.4);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_wide_camera_gamma", 1.0, descriptor); // 0.4 - 2.4
            floatRange.set__from_value(0.0).set__to_value(2.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_wide_camera_saturation", 1.0, descriptor); // 0 - 2
            floatRange.set__from_value(-40).set__to_value(40.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_wide_camera_hue", 0.0, descriptor); // -40 - 40
            intRange.set__from_value(0).set__to_value(1);                       // TBD: dynamic reconfigure dropdown menu
            descriptor.integer_range = {intRange};
            descriptor.description =
                "Value must be:\n"
                "\tMinimizeNoise = 0\n"
                "\tMinimizeBlur = 1";
            this->declare_parameter("allied_wide_camera_intensity_auto_precedence", 0, descriptor); // 0(MinimizeNoise) or 1(MinimizeBlur)
            descriptor.description = "";
            this->declare_parameter("allied_wide_camera_auto_white_balance", false);
            intRange.set__from_value(0).set__to_value(1); // TBD: dynamic reconfigure dropdown menu
            descriptor.integer_range = {intRange};
            descriptor.description =
                "Value must be:\n"
                "\tRed = 0\n"
                "\tBlue = 1";
            this->declare_parameter("allied_wide_camera_balance_ratio_selector", 0, descriptor); // 0(Red), 1(Blue)
            descriptor.description = "";
            floatRange.set__from_value(0.0).set__to_value(8.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_wide_camera_balance_ratio", 2.4, descriptor); // 0 - 8
            floatRange.set__from_value(0.0).set__to_value(100.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_wide_camera_balance_white_auto_rate", 100.0, descriptor); // 0 - 100
            floatRange.set__from_value(0.0).set__to_value(50.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_wide_camera_balance_white_auto_tolerance", 5.0, descriptor); // 0 - 50
            intRange.set__from_value(0).set__to_value(4);                                                // TBD: dynamic reconfigure dropdown menu
            descriptor.integer_range = {intRange};
            descriptor.description =
                "Value must be:\n"
                "\tAutoMode = 0\n"
                "\tFullImage = 4";
            this->declare_parameter("allied_wide_camera_intensity_controller_region", 0, descriptor); // 0(AutoMode), 4(FullImage)
            descriptor.description = "";
            floatRange.set__from_value(10).set__to_value(90);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("allied_wide_camera_intensity_controller_target", 50.0, descriptor); // 10 - 90
            intRange.set__from_value(0).set__to_value(1);
            descriptor.integer_range = {intRange};
            this->declare_parameter("allied_wide_streaming_protocol", 0, descriptor); // 0(protocol_raw_udp), 1(protocol_gstreamer)
        }

        void loadDefaultParams()
        {
            // Get and save parameters
            allied_wide_camera_exposure_time_ = this->get_parameter("allied_wide_camera_exposure_time").as_double();
            allied_wide_camera_auto_exposure_time_ = this->get_parameter("allied_wide_camera_auto_exposure_time").as_bool();
            allied_wide_camera_auto_exposure_time_range_min_ = this->get_parameter("allied_wide_camera_auto_exposure_time_range_min").as_double();
            allied_wide_camera_auto_exposure_time_range_max_ = this->get_parameter("allied_wide_camera_auto_exposure_time_range_max").as_double();
            allied_wide_camera_gain_ = this->get_parameter("allied_wide_camera_gain").as_double();
            allied_wide_camera_auto_gain_ = this->get_parameter("allied_wide_camera_auto_gain").as_bool();
            allied_wide_camera_auto_gain_range_min_ = this->get_parameter("allied_wide_camera_auto_gain_range_min").as_double();
            allied_wide_camera_auto_gain_range_max_ = this->get_parameter("allied_wide_camera_auto_gain_range_max").as_double();
            allied_wide_camera_gamma_ = this->get_parameter("allied_wide_camera_gamma").as_double();
            allied_wide_camera_saturation_ = this->get_parameter("allied_wide_camera_saturation").as_double();
            allied_wide_camera_hue_ = this->get_parameter("allied_wide_camera_hue").as_double();
            allied_wide_camera_intensity_auto_precedence_ = this->get_parameter("allied_wide_camera_intensity_auto_precedence").as_int();
            allied_wide_camera_auto_white_balance_ = this->get_parameter("allied_wide_camera_auto_white_balance").as_bool();
            allied_wide_camera_balance_ratio_selector_ = this->get_parameter("allied_wide_camera_balance_ratio_selector").as_int();
            allied_wide_camera_balance_ratio_ = this->get_parameter("allied_wide_camera_balance_ratio").as_double();
            allied_wide_camera_balance_white_auto_rate_ = this->get_parameter("allied_wide_camera_balance_white_auto_rate").as_double();
            allied_wide_camera_balance_white_auto_tolerance_ = this->get_parameter("allied_wide_camera_balance_white_auto_tolerance").as_double();
            allied_wide_camera_intensity_controller_region_ = this->get_parameter("allied_wide_camera_intensity_controller_region").as_int();
            allied_wide_camera_intensity_controller_target_ = this->get_parameter("allied_wide_camera_intensity_controller_target").as_double();
            streaming_protocol_ = this->get_parameter("allied_wide_streaming_protocol").as_int();
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

                if (param_name == "allied_wide_camera_exposure_time" && param.as_double() != allied_wide_camera_exposure_time_)
                {
                    callExposureTime(param.as_double());
                }
                if (param_name == "allied_wide_camera_auto_exposure_time" && param.as_bool() != allied_wide_camera_auto_exposure_time_)
                {
                    callAutoExposureTime(param.as_bool());
                }
                if (param_name == "allied_wide_camera_auto_exposure_time_range_min" && param.as_double() != allied_wide_camera_auto_exposure_time_range_min_)
                {
                    callAutoExposureTimeRange(param.as_double(), allied_wide_camera_auto_exposure_time_range_max_);
                }
                if (param_name == "allied_wide_camera_auto_exposure_time_range_max" && param.as_double() != allied_wide_camera_auto_exposure_time_range_max_)
                {
                    callAutoExposureTimeRange(allied_wide_camera_auto_exposure_time_range_min_, param.as_double());
                }
                if (param_name == "allied_wide_camera_gain" && param.as_double() != allied_wide_camera_gain_)
                {
                    callGain(param.as_double());
                }
                if (param_name == "allied_wide_camera_auto_gain" && param.as_bool() != allied_wide_camera_auto_gain_)
                {
                    callAutoGain(param.as_bool());
                }
                if (param_name == "allied_wide_camera_auto_gain_range_min" && param.as_double() != allied_wide_camera_auto_gain_range_min_)
                {
                    callAutoGainRange(param.as_double(), allied_wide_camera_auto_gain_range_max_);
                }
                if (param_name == "allied_wide_camera_auto_gain_range_max" && param.as_double() != allied_wide_camera_auto_gain_range_max_)
                {
                    callAutoGainRange(allied_wide_camera_auto_gain_range_min_, param.as_double());
                }
                if (param_name == "allied_wide_camera_gamma" && param.as_double() != allied_wide_camera_gamma_)
                {
                    callGamma(param.as_double());
                }
                if (param_name == "allied_wide_camera_saturation" && param.as_double() != allied_wide_camera_saturation_)
                {
                    callSaturation(param.as_double());
                }
                if (param_name == "allied_wide_camera_hue" && param.as_double() != allied_wide_camera_hue_)
                {
                    callHue(param.as_double());
                }
                if (param_name == "allied_wide_camera_intensity_auto_precedence" && param.as_int() != allied_wide_camera_intensity_auto_precedence_)
                {
                    callIntensityAutoPrecedence(param.as_int());
                }
                if (param_name == "allied_wide_camera_auto_white_balance" && param.as_bool() != allied_wide_camera_auto_white_balance_)
                {
                    callAutoWhiteBalance(param.as_bool());
                }
                if (param_name == "allied_wide_camera_balance_ratio_selector" && param.as_int() != allied_wide_camera_balance_ratio_selector_)
                {
                    callBalanceRatioSelector(param.as_int());
                }
                if (param_name == "allied_wide_camera_balance_ratio" && param.as_double() != allied_wide_camera_balance_ratio_)
                {
                    callBalanceRatio(param.as_double());
                }
                if (param_name == "allied_wide_camera_balance_white_auto_rate" && param.as_double() != allied_wide_camera_balance_white_auto_rate_)
                {
                    callBalanceWhiteAutoRate(param.as_double());
                }
                if (param_name == "allied_wide_camera_balance_white_auto_tolerance" && param.as_double() != allied_wide_camera_balance_white_auto_tolerance_)
                {
                    callBalanceWhiteAutoTolerance(param.as_double());
                }
                if (param_name == "allied_wide_camera_intensity_controller_region" && param.as_int() != allied_wide_camera_intensity_controller_region_)
                {
                    callIntensityControllerRegion(param.as_int());
                }
                if (param_name == "allied_wide_camera_intensity_controller_target" && param.as_double() != allied_wide_camera_intensity_controller_target_)
                {
                    callIntensityControllerTarget(param.as_double());
                }
                if (param_name == "allied_wide_streaming_protocol" && param.as_int() != streaming_protocol_)
                {
                    callStreamingProtocol(param.as_int());
                }
            }

            return result;
        }

        // Service calls
        void callExposureTime(double exposure_time)
        {
            while (!client_exposure_time_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestExposureTime = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime::Request>();
            requestExposureTime->allied_type = alliedCamerasIds::wide_camera;
            requestExposureTime->exposure_time = exposure_time;

            auto resultExposureTime = client_exposure_time_->async_send_request(
                requestExposureTime, std::bind(&AlliedWideConfiguration::exposureTimeResponseCallback, this, std::placeholders::_1));
        }

        void callAutoExposureTime(bool enabled)
        {
            while (!client_auto_exposure_time_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestAutoExposureTime = std::make_shared<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime::Request>();
            requestAutoExposureTime->allied_type = alliedCamerasIds::wide_camera;
            requestAutoExposureTime->enabled = enabled;

            auto resultAutoExposureTime = client_auto_exposure_time_->async_send_request(
                requestAutoExposureTime, std::bind(&AlliedWideConfiguration::autoExposureTimeResponseCallback, this, std::placeholders::_1));
        }

        void callAutoExposureTimeRange(double range_min, double range_max)
        {
            while (!client_auto_exposure_time_range_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestAutoExposureTimeRange = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange::Request>();
            requestAutoExposureTimeRange->allied_type = alliedCamerasIds::wide_camera;
            requestAutoExposureTimeRange->auto_exposure_time_range_min = range_min;
            requestAutoExposureTimeRange->auto_exposure_time_range_max = range_max;

            auto resultAutoExposureTimeRange = client_auto_exposure_time_range_->async_send_request(
                requestAutoExposureTimeRange, std::bind(&AlliedWideConfiguration::autoExposureTimeRangeResponseCallback, this, std::placeholders::_1));
        }

        void callGain(double gain)
        {
            while (!client_gain_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestGain = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraGain::Request>();
            requestGain->allied_type = alliedCamerasIds::wide_camera;
            requestGain->gain = gain;

            auto resultGain = client_gain_->async_send_request(
                requestGain, std::bind(&AlliedWideConfiguration::gainResponseCallback, this, std::placeholders::_1));
        }

        void callAutoGain(bool enabled)
        {
            while (!client_auto_gain_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestAutoGain = std::make_shared<l3cam_interfaces::srv::EnableAlliedCameraAutoGain::Request>();
            requestAutoGain->allied_type = alliedCamerasIds::wide_camera;
            requestAutoGain->enabled = enabled;

            auto resultAutoGain = client_auto_gain_->async_send_request(
                requestAutoGain, std::bind(&AlliedWideConfiguration::autoGainResponseCallback, this, std::placeholders::_1));
        }

        void callAutoGainRange(double range_min, double range_max)
        {
            while (!client_auto_gain_range_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestAutoGainRange = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange::Request>();
            requestAutoGainRange->allied_type = alliedCamerasIds::wide_camera;
            requestAutoGainRange->auto_gain_range_min = range_min;
            requestAutoGainRange->auto_gain_range_max = range_max;

            auto resultAutoGainRange = client_auto_gain_range_->async_send_request(
                requestAutoGainRange, std::bind(&AlliedWideConfiguration::autoGainRangeResponseCallback, this, std::placeholders::_1));
        }

        void callGamma(double gamma)
        {
            while (!client_gamma_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestGamma = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraGamma::Request>();
            requestGamma->allied_type = alliedCamerasIds::wide_camera;
            requestGamma->gamma = gamma;

            auto resultGamma = client_gamma_->async_send_request(
                requestGamma, std::bind(&AlliedWideConfiguration::gammaResponseCallback, this, std::placeholders::_1));
        }

        void callSaturation(double saturation)
        {
            while (!client_saturation_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestSaturation = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraSaturation::Request>();
            requestSaturation->allied_type = alliedCamerasIds::wide_camera;
            requestSaturation->saturation = saturation;

            auto resultSaturation = client_saturation_->async_send_request(
                requestSaturation, std::bind(&AlliedWideConfiguration::saturationResponseCallback, this, std::placeholders::_1));
        }

        void callHue(double hue)
        {
            while (!client_hue_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestHue = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraHue::Request>();
            requestHue->allied_type = alliedCamerasIds::wide_camera;
            requestHue->hue = hue;

            auto resultHue = client_hue_->async_send_request(
                requestHue, std::bind(&AlliedWideConfiguration::hueResponseCallback, this, std::placeholders::_1));
        }

        void callIntensityAutoPrecedence(int intensity_auto_precedence)
        {
            while (!client_intensity_auto_precedence_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestIntensityAutoPrecedence = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence::Request>();
            requestIntensityAutoPrecedence->allied_type = alliedCamerasIds::wide_camera;
            requestIntensityAutoPrecedence->intensity_auto_precedence = intensity_auto_precedence;

            auto resultIntensityAutoPrecedence = client_intensity_auto_precedence_->async_send_request(
                requestIntensityAutoPrecedence, std::bind(&AlliedWideConfiguration::intensityAutoPrecedenceResponseCallback, this, std::placeholders::_1));
        }

        void callAutoWhiteBalance(bool enabled)
        {
            while (!client_auto_white_balance_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting aWhiteBalance...");
            }

            auto requestAutoWhiteBalance = std::make_shared<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance::Request>();
            requestAutoWhiteBalance->allied_type = alliedCamerasIds::wide_camera;
            requestAutoWhiteBalance->enabled = enabled;

            auto resultAutoWhiteBalance = client_auto_white_balance_->async_send_request(
                requestAutoWhiteBalance, std::bind(&AlliedWideConfiguration::autoWhiteBalanceResponseCallback, this, std::placeholders::_1));
        }

        void callBalanceRatioSelector(int white_balance_ratio_selector)
        {
            while (!client_balance_ratio_selector_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestBalanceRatioSelector = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector::Request>();
            requestBalanceRatioSelector->allied_type = alliedCamerasIds::wide_camera;
            requestBalanceRatioSelector->white_balance_ratio_selector = white_balance_ratio_selector;

            auto resultBalanceRatioSelector = client_balance_ratio_selector_->async_send_request(
                requestBalanceRatioSelector, std::bind(&AlliedWideConfiguration::balanceRatioSelectorResponseCallback, this, std::placeholders::_1));
        }

        void callBalanceRatio(double balance_ratio)
        {
            while (!client_balance_ratio_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestBalanceRatio = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio::Request>();
            requestBalanceRatio->allied_type = alliedCamerasIds::wide_camera;
            requestBalanceRatio->balance_ratio = balance_ratio;

            auto resultBalanceRatio = client_balance_ratio_->async_send_request(
                requestBalanceRatio, std::bind(&AlliedWideConfiguration::balanceRatioResponseCallback, this, std::placeholders::_1));
        }

        void callBalanceWhiteAutoRate(double white_balance_auto_rate)
        {
            while (!client_balance_white_auto_rate_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestBalanceWhiteAutoRate = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate::Request>();
            requestBalanceWhiteAutoRate->allied_type = alliedCamerasIds::wide_camera;
            requestBalanceWhiteAutoRate->white_balance_auto_rate = white_balance_auto_rate;

            auto resultBalanceWhiteAutoRate = client_balance_white_auto_rate_->async_send_request(
                requestBalanceWhiteAutoRate, std::bind(&AlliedWideConfiguration::balanceWhiteAutoRateResponseCallback, this, std::placeholders::_1));
        }

        void callBalanceWhiteAutoTolerance(double white_balance_auto_tolerance)
        {
            while (!client_balance_white_auto_tolerance_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestBalanceWhiteAutoTolerance = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance::Request>();
            requestBalanceWhiteAutoTolerance->allied_type = alliedCamerasIds::wide_camera;
            requestBalanceWhiteAutoTolerance->white_balance_auto_tolerance = white_balance_auto_tolerance;

            auto resultBalanceWhiteAutoTolerance = client_balance_white_auto_tolerance_->async_send_request(
                requestBalanceWhiteAutoTolerance, std::bind(&AlliedWideConfiguration::balanceWhiteAutoToleranceResponseCallback, this, std::placeholders::_1));
        }

        void callIntensityControllerRegion(int intensity_controller_region)
        {
            while (!client_intensity_controller_region_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestIntensityControllerRegion = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion::Request>();
            requestIntensityControllerRegion->allied_type = alliedCamerasIds::wide_camera;
            requestIntensityControllerRegion->intensity_controller_region = intensity_controller_region;

            auto resultIntensityControllerRegion = client_intensity_controller_region_->async_send_request(
                requestIntensityControllerRegion, std::bind(&AlliedWideConfiguration::intensityControllerRegionResponseCallback, this, std::placeholders::_1));
        }

        void callIntensityControllerTarget(double intensity_controller_target)
        {
            while (!client_intensity_controller_target_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestIntensityControllerTarget = std::make_shared<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget::Request>();
            requestIntensityControllerTarget->allied_type = alliedCamerasIds::wide_camera;
            requestIntensityControllerTarget->intensity_controller_target = intensity_controller_target;

            auto resultIntensityControllerTarget = client_intensity_controller_target_->async_send_request(
                requestIntensityControllerTarget, std::bind(&AlliedWideConfiguration::intensityControllerTargetResponseCallback, this, std::placeholders::_1));
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
            requestStreamingProtocol->sensor_type = (int)sensorTypes::sensor_allied_wide;
            requestStreamingProtocol->protocol = protocol;

            auto resultStreamingProtocol = client_streaming_protocol_->async_send_request(
                requestStreamingProtocol, std::bind(&AlliedWideConfiguration::streamingProtocolResponseCallback, this, std::placeholders::_1));
        }

        // Service callbacks
        void exposureTimeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully
                    allied_wide_camera_exposure_time_ = this->get_parameter("allied_wide_camera_exposure_time").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_exposure_time", allied_wide_camera_exposure_time_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_exposure_time");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_exposure_time", allied_wide_camera_exposure_time_));
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
                    // Parameter changed successfully
                    allied_wide_camera_auto_exposure_time_ = this->get_parameter("allied_wide_camera_auto_exposure_time").as_bool();

                    // If auto exposure time deactivated we have to get the actual exposure time to know its value
                    if (!allied_wide_camera_auto_exposure_time_)
                    {
                        while (!client_get_exposure_time_->wait_for_service(1s))
                        {
                            if (!rclcpp::ok())
                            {
                                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                                break;
                            }
                            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
                        }

                        auto requestGetExposureTime = std::make_shared<l3cam_interfaces::srv::GetAlliedCameraExposureTime::Request>();
                        requestGetExposureTime->allied_type = alliedCamerasIds::wide_camera;

                        auto resultGetExposureTime = client_get_exposure_time_->async_send_request(
                            requestGetExposureTime, std::bind(&AlliedWideConfiguration::getExposureTimeResponseCallback, this, std::placeholders::_1));
                    }
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_exposure_time", allied_wide_camera_auto_exposure_time_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service enable_allied_camera_auto_exposure_time");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_exposure_time", allied_wide_camera_auto_exposure_time_));
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
                    // Parameters changed successfully
                    allied_wide_camera_auto_exposure_time_range_min_ = this->get_parameter("allied_wide_camera_auto_exposure_time_range_min").as_double();
                    allied_wide_camera_auto_exposure_time_range_max_ = this->get_parameter("allied_wide_camera_auto_exposure_time_range_max").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_exposure_time_range_min", allied_wide_camera_auto_exposure_time_range_min_));
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_exposure_time_range_max", allied_wide_camera_auto_exposure_time_range_max_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_auto_exposure_time_range");
                // Service could not be called, reset parameters to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_exposure_time_range_min", allied_wide_camera_auto_exposure_time_range_min_));
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_exposure_time_range_max", allied_wide_camera_auto_exposure_time_range_min_));
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
                    // Parameter changed successfully
                    allied_wide_camera_gain_ = this->get_parameter("allied_wide_camera_gain").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_gain", allied_wide_camera_gain_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_gain");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_gain", allied_wide_camera_gain_));
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
                    // Parameter changed successfully
                    allied_wide_camera_auto_gain_ = this->get_parameter("allied_wide_camera_auto_gain").as_bool();

                    // If auto gain deactivated we have to get the actual gain to know its value
                    if (!allied_wide_camera_auto_gain_)
                    {
                        while (!client_get_gain_->wait_for_service(1s))
                        {
                            if (!rclcpp::ok())
                            {
                                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                                break;
                            }
                            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
                        }

                        auto requestGetGain = std::make_shared<l3cam_interfaces::srv::GetAlliedCameraGain::Request>();
                        requestGetGain->allied_type = alliedCamerasIds::wide_camera;

                        auto resultGetGain = client_get_gain_->async_send_request(
                            requestGetGain, std::bind(&AlliedWideConfiguration::getGainResponseCallback, this, std::placeholders::_1));
                    }
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_gain", allied_wide_camera_auto_gain_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service enable_allied_camera_auto_gain");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_gain", allied_wide_camera_auto_gain_));
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
                    allied_wide_camera_auto_gain_range_min_ = this->get_parameter("allied_wide_camera_auto_gain_range_min").as_double();
                    allied_wide_camera_auto_gain_range_max_ = this->get_parameter("allied_wide_camera_auto_gain_range_max").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_gain_range_min", allied_wide_camera_auto_gain_range_min_));
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_gain_range_max", allied_wide_camera_auto_gain_range_max_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_auto_gain_range");
                // Service could not be called, reset parameters to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_gain_range_min", allied_wide_camera_auto_gain_range_min_));
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_gain_range_max", allied_wide_camera_auto_gain_range_min_));
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
                    allied_wide_camera_gamma_ = this->get_parameter("allied_wide_camera_gamma").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_gamma", allied_wide_camera_gamma_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_gamma");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_gamma", allied_wide_camera_gamma_));
            }
        }

        void saturationResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraSaturation>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    allied_wide_camera_saturation_ = this->get_parameter("allied_wide_camera_saturation").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_saturation", allied_wide_camera_saturation_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_saturation");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_saturation", allied_wide_camera_saturation_));
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
                    allied_wide_camera_hue_ = this->get_parameter("allied_wide_camera_hue").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_hue", allied_wide_camera_hue_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_hue");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_hue", allied_wide_camera_hue_));
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
                    allied_wide_camera_intensity_auto_precedence_ = this->get_parameter("allied_wide_camera_intensity_auto_precedence").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_intensity_auto_precedence", allied_wide_camera_intensity_auto_precedence_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_intensity_auto_precedence");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_intensity_auto_precedence", allied_wide_camera_intensity_auto_precedence_));
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
                    allied_wide_camera_auto_white_balance_ = this->get_parameter("allied_wide_camera_auto_white_balance").as_bool();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_white_balance", allied_wide_camera_auto_white_balance_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service enable_allied_camera_auto_white_balance");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_auto_white_balance", allied_wide_camera_auto_white_balance_));
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
                    allied_wide_camera_balance_ratio_selector_ = this->get_parameter("allied_wide_camera_balance_ratio_selector").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_balance_ratio_selector", allied_wide_camera_balance_ratio_selector_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_balance_ratio_selector");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_balance_ratio_selector", allied_wide_camera_balance_ratio_selector_));
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
                    allied_wide_camera_balance_ratio_ = this->get_parameter("allied_wide_camera_balance_ratio").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_balance_ratio", allied_wide_camera_balance_ratio_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_balance_ratio");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_balance_ratio", allied_wide_camera_balance_ratio_));
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
                    allied_wide_camera_balance_white_auto_rate_ = this->get_parameter("allied_wide_camera_balance_white_auto_rate").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_balance_white_auto_rate", allied_wide_camera_balance_white_auto_rate_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_balance_white_auto_rate");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_balance_white_auto_rate", allied_wide_camera_balance_white_auto_rate_));
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
                    allied_wide_camera_balance_white_auto_tolerance_ = this->get_parameter("allied_wide_camera_balance_white_auto_tolerance").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_balance_white_auto_tolerance", allied_wide_camera_balance_white_auto_tolerance_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_balance_white_auto_tolerance");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_balance_white_auto_tolerance", allied_wide_camera_balance_white_auto_tolerance_));
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
                    allied_wide_camera_intensity_controller_region_ = this->get_parameter("allied_wide_camera_intensity_controller_region").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_intensity_controller_region", allied_wide_camera_intensity_controller_region_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_intensity_controller_region");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_intensity_controller_region", allied_wide_camera_intensity_controller_region_));
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
                    allied_wide_camera_intensity_controller_target_ = this->get_parameter("allied_wide_camera_intensity_controller_target").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_intensity_controller_target", allied_wide_camera_intensity_controller_target_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_allied_camera_intensity_controller_target");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_camera_intensity_controller_target", allied_wide_camera_intensity_controller_target_));
            }
        }

        void getExposureTimeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::GetAlliedCameraExposureTime>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Got parameter successfully
                    allied_wide_camera_exposure_time_ = future.get()->exposure_time;
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_exposure_time", future.get()->exposure_time));
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while getting parameter: " << getErrorDescription(error));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service get_allied_camera_exposure_time");
            }
        }

        void getGainResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::GetAlliedCameraGain>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Got parameter successfully
                    allied_wide_camera_gain_ = future.get()->gain;
                    this->set_parameter(rclcpp::Parameter("allied_wide_camera_gain", future.get()->gain));
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while getting parameter: " << getErrorDescription(error));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service get_allied_camera_gain");
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
                    streaming_protocol_ = this->get_parameter("allied_wide_streaming_protocol").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("allied_wide_streaming_protocol", streaming_protocol_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_streaming_protocol");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("allied_wide_streaming_protocol", streaming_protocol_));
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

        double allied_wide_camera_exposure_time_;
        bool allied_wide_camera_auto_exposure_time_;
        double allied_wide_camera_auto_exposure_time_range_min_;
        double allied_wide_camera_auto_exposure_time_range_max_;
        double allied_wide_camera_gain_;
        bool allied_wide_camera_auto_gain_;
        double allied_wide_camera_auto_gain_range_min_;
        double allied_wide_camera_auto_gain_range_max_;
        double allied_wide_camera_gamma_;
        double allied_wide_camera_saturation_;
        double allied_wide_camera_hue_;
        int allied_wide_camera_intensity_auto_precedence_;
        bool allied_wide_camera_auto_white_balance_;
        int allied_wide_camera_balance_ratio_selector_;
        double allied_wide_camera_balance_ratio_;
        double allied_wide_camera_balance_white_auto_rate_;
        double allied_wide_camera_balance_white_auto_tolerance_;
        int allied_wide_camera_intensity_controller_region_;
        double allied_wide_camera_intensity_controller_target_;
        int streaming_protocol_;

        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime>::SharedPtr client_exposure_time_;
        rclcpp::Client<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime>::SharedPtr client_auto_exposure_time_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange>::SharedPtr client_auto_exposure_time_range_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraGain>::SharedPtr client_gain_;
        rclcpp::Client<l3cam_interfaces::srv::EnableAlliedCameraAutoGain>::SharedPtr client_auto_gain_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange>::SharedPtr client_auto_gain_range_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraGamma>::SharedPtr client_gamma_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraSaturation>::SharedPtr client_saturation_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraHue>::SharedPtr client_hue_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence>::SharedPtr client_intensity_auto_precedence_;
        rclcpp::Client<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance>::SharedPtr client_auto_white_balance_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector>::SharedPtr client_balance_ratio_selector_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio>::SharedPtr client_balance_ratio_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate>::SharedPtr client_balance_white_auto_rate_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance>::SharedPtr client_balance_white_auto_tolerance_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion>::SharedPtr client_intensity_controller_region_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget>::SharedPtr client_intensity_controller_target_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeStreamingProtocol>::SharedPtr client_streaming_protocol_;
        rclcpp::Client<l3cam_interfaces::srv::GetAlliedCameraExposureTime>::SharedPtr client_get_exposure_time_;
        rclcpp::Client<l3cam_interfaces::srv::GetAlliedCameraGain>::SharedPtr client_get_gain_;

        rclcpp::Service<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr srv_sensor_disconnected_;

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    }; // class AlliedWideConfiguration

} // namespace l3cam_ros2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::AlliedWideConfiguration> node = std::make_shared<l3cam_ros2::AlliedWideConfiguration>();

    // Check if Allied Wide is available
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
                if (resultGetSensors.get()->sensors[i].sensor_type == sensor_allied_wide && resultGetSensors.get()->sensors[i].sensor_available)
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Allied Wide camera configuration is available");
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
    requestGetRtspPipeline.get()->sensor_type = (int)sensorTypes::sensor_allied_wide;
    auto resultGetRtspPipeline = node->client_get_rtsp_pipeline_->async_send_request(requestGetRtspPipeline);

    if (rclcpp::spin_until_future_complete(node, resultGetRtspPipeline) == rclcpp::FutureReturnCode::SUCCESS)
    {
        error = resultGetRtspPipeline.get()->error;

        if (!error)
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.read_only = true;
            node->declare_parameter("allied_wide_rtsp_pipeline", resultGetRtspPipeline.get()->pipeline, descriptor);
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
