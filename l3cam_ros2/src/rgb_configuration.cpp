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

#include "l3cam_interfaces/srv/change_rgb_camera_brightness.hpp"
#include "l3cam_interfaces/srv/change_rgb_camera_contrast.hpp"
#include "l3cam_interfaces/srv/change_rgb_camera_saturation.hpp"
#include "l3cam_interfaces/srv/change_rgb_camera_sharpness.hpp"
#include "l3cam_interfaces/srv/change_rgb_camera_gamma.hpp"
#include "l3cam_interfaces/srv/change_rgb_camera_gain.hpp"
#include "l3cam_interfaces/srv/enable_rgb_camera_auto_white_balance.hpp"
#include "l3cam_interfaces/srv/change_rgb_camera_white_balance.hpp"
#include "l3cam_interfaces/srv/enable_rgb_camera_auto_exposure_time.hpp"
#include "l3cam_interfaces/srv/change_rgb_camera_exposure_time.hpp"
#include "l3cam_interfaces/srv/change_streaming_protocol.hpp"
#include "l3cam_interfaces/srv/get_rtsp_pipeline.hpp"

#include "l3cam_interfaces/srv/sensor_disconnected.hpp"

#include "l3cam_ros2_utils.hpp"

using namespace std::chrono_literals;

namespace l3cam_ros2
{
    class RgbConfiguration : public rclcpp::Node
    {
    public:
        explicit RgbConfiguration() : Node("rgb_configuration")
        {
            // Create service clients
            client_get_sensors_ = this->create_client<l3cam_interfaces::srv::GetSensorsAvailable>("get_sensors_available");
            client_brightness_ = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraBrightness>("change_rgb_camera_brightness");
            client_contrast_ = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraContrast>("change_rgb_camera_contrast");
            client_saturation_ = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraSaturation>("change_rgb_camera_saturation");
            client_sharpness_ = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraSharpness>("change_rgb_camera_sharpness");
            client_gamma_ = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraGamma>("change_rgb_camera_gamma");
            client_gain_ = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraGain>("change_rgb_camera_gain");
            client_auto_white_balance_ = this->create_client<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance>("enable_rgb_camera_auto_white_balance");
            client_white_balance_ = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance>("change_rgb_camera_white_balance");
            client_auto_exposure_time_ = this->create_client<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime>("enable_rgb_camera_auto_exposure_time");
            client_exposure_time_ = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraExposureTime>("change_rgb_camera_exposure_time");
            client_streaming_protocol_ = this->create_client<l3cam_interfaces::srv::ChangeStreamingProtocol>("change_streaming_protocol");
            client_get_rtsp_pipeline_ = this->create_client<l3cam_interfaces::srv::GetRtspPipeline>("get_rtsp_pipeline");

            declareParams();
            loadDefaultParams();

            // Create service server
            srv_sensor_disconnected_ = this->create_service<l3cam_interfaces::srv::SensorDisconnected>(
                "rgb_configuration_disconnected", std::bind(&RgbConfiguration::sensorDisconnectedCallback, this, std::placeholders::_1, std::placeholders::_2));

            // Callback on parameters changed
            callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&RgbConfiguration::parametersCallback, this, std::placeholders::_1));
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
            range.set__from_value(-15).set__to_value(15);
            descriptor.integer_range = {range};
            this->declare_parameter("rgb_camera_brightness", 0, descriptor); // -15 - 15
            this->declare_parameter("rgb_camera_contrast", 10, descriptor);  // 0 - 30
            range.set__from_value(0).set__to_value(60);
            descriptor.integer_range = {range};
            this->declare_parameter("rgb_camera_saturation", 16, descriptor); // 0 - 60
            range.set__from_value(0).set__to_value(127);
            descriptor.integer_range = {range};
            this->declare_parameter("rgb_camera_sharpness", 16, descriptor); // 0 - 127
            range.set__from_value(40).set__to_value(500);
            descriptor.integer_range = {range};
            this->declare_parameter("rgb_camera_gamma", 220, descriptor); // 40 - 500
            range.set__from_value(0).set__to_value(63);
            descriptor.integer_range = {range};
            this->declare_parameter("rgb_camera_gain", 0, descriptor); // 0 - 63
            this->declare_parameter("rgb_camera_auto_white_balance", true);
            range.set__from_value(1000).set__to_value(10000);
            descriptor.integer_range = {range};
            this->declare_parameter("rgb_camera_white_balance", 5000, descriptor); // 1000 - 10000
            this->declare_parameter("rgb_camera_auto_exposure_time", true);
            range.set__from_value(1).set__to_value(10000);
            descriptor.integer_range = {range};
            this->declare_parameter("rgb_camera_exposure_time", 156, descriptor); // 1 - 10000
            range.set__from_value(0).set__to_value(1);
            descriptor.integer_range = {range};
            this->declare_parameter("rgb_streaming_protocol", 0, descriptor); // 0(protocol_raw_udp), 1(protocol_gstreamer)
        }

        void loadDefaultParams()
        {
            // Get and save parameters
            rgb_camera_brightness_ = this->get_parameter("rgb_camera_brightness").as_int();
            rgb_camera_contrast_ = this->get_parameter("rgb_camera_contrast").as_int();
            rgb_camera_saturation_ = this->get_parameter("rgb_camera_saturation").as_int();
            rgb_camera_sharpness_ = this->get_parameter("rgb_camera_sharpness").as_int();
            rgb_camera_gamma_ = this->get_parameter("rgb_camera_gamma").as_int();
            rgb_camera_gain_ = this->get_parameter("rgb_camera_gain").as_int();
            rgb_camera_auto_white_balance_ = this->get_parameter("rgb_camera_auto_white_balance").as_bool();
            rgb_camera_white_balance_ = this->get_parameter("rgb_camera_white_balance").as_int();
            rgb_camera_auto_exposure_time_ = this->get_parameter("rgb_camera_auto_exposure_time").as_bool();
            rgb_camera_exposure_time_ = this->get_parameter("rgb_camera_exposure_time").as_int();
            streaming_protocol_ = this->get_parameter("rgb_streaming_protocol").as_int();
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
                if (param_name == "rgb_camera_brightness" && param.as_int() != rgb_camera_brightness_)
                {
                    callBrightness(param.as_int());
                }
                if (param_name == "rgb_camera_contrast" && param.as_int() != rgb_camera_contrast_)
                {
                    callContrast(param.as_int());
                }
                if (param_name == "rgb_camera_saturation" && param.as_int() != rgb_camera_saturation_)
                {
                    callSaturation(param.as_int());
                }
                if (param_name == "rgb_camera_sharpness" && param.as_int() != rgb_camera_sharpness_)
                {
                    callSharpness(param.as_int());
                }
                if (param_name == "rgb_camera_gamma" && param.as_int() != rgb_camera_gamma_)
                {
                    callGamma(param.as_int());
                }
                if (param_name == "rgb_camera_gain" && param.as_int() != rgb_camera_gain_)
                {
                    callGain(param.as_int());
                }
                if (param_name == "rgb_camera_auto_white_balance" && param.as_bool() != rgb_camera_auto_white_balance_)
                {
                    callAutoWhiteBalance(param.as_bool());
                }
                if (param_name == "rgb_camera_white_balance" && param.as_int() != rgb_camera_white_balance_)
                {
                    callWhiteBalance(param.as_int());
                }
                if (param_name == "rgb_camera_auto_exposure_time" && param.as_bool() != rgb_camera_auto_exposure_time_)
                {
                    callAutoExposureTime(param.as_bool());
                }
                if (param_name == "rgb_camera_exposure_time" && param.as_int() != rgb_camera_exposure_time_)
                {
                    callExposureTime(param.as_int());
                }
                if (param_name == "rgb_streaming_protocol" && param.as_int() != streaming_protocol_)
                {
                    callStreamingProtocol(param.as_int());
                }
            }

            return result;
        }

        // Service calls
        void callBrightness(int brightness)
        {
            while (!client_brightness_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestBrightness = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraBrightness::Request>();
            requestBrightness->brightness = brightness;

            auto resultBrightness = client_brightness_->async_send_request(
                requestBrightness, std::bind(&RgbConfiguration::brightnessResponseCallback, this, std::placeholders::_1));
        }

        void callContrast(int contrast)
        {
            while (!client_contrast_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestContrast = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraContrast::Request>();
            requestContrast->contrast = contrast;

            auto resultContrast = client_contrast_->async_send_request(
                requestContrast, std::bind(&RgbConfiguration::contrastResponseCallback, this, std::placeholders::_1));
        }

        void callSaturation(int saturation)
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

            auto requestSaturation = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraSaturation::Request>();
            requestSaturation->saturation = saturation;

            auto resultSaturation = client_saturation_->async_send_request(
                requestSaturation, std::bind(&RgbConfiguration::saturationResponseCallback, this, std::placeholders::_1));
        }

        void callSharpness(int sharpness)
        {
            while (!client_sharpness_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestSharpness = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraSharpness::Request>();
            requestSharpness->sharpness = sharpness;

            auto resultSharpness = client_sharpness_->async_send_request(
                requestSharpness, std::bind(&RgbConfiguration::sharpnessResponseCallback, this, std::placeholders::_1));
        }

        void callGamma(int gamma)
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

            auto requestGamma = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraGamma::Request>();
            requestGamma->gamma = gamma;

            auto resultGamma = client_gamma_->async_send_request(
                requestGamma, std::bind(&RgbConfiguration::gammaResponseCallback, this, std::placeholders::_1));
        }

        void callGain(int gain)
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

            auto requestGain = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraGain::Request>();
            requestGain->gain = gain;

            auto resultGain = client_gain_->async_send_request(
                requestGain, std::bind(&RgbConfiguration::gainResponseCallback, this, std::placeholders::_1));
        }

        void callAutoWhiteBalance(int gain)
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

            auto requestGain = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraGain::Request>();
            requestGain->gain = gain;

            auto resultGain = client_gain_->async_send_request(
                requestGain, std::bind(&RgbConfiguration::gainResponseCallback, this, std::placeholders::_1));
        }

        void callWhiteBalance(int white_balance)
        {
            while (!client_white_balance_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestWhiteBalance = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance::Request>();
            requestWhiteBalance->white_balance = white_balance;

            auto resultWhiteBalance = client_white_balance_->async_send_request(
                requestWhiteBalance, std::bind(&RgbConfiguration::whiteBalanceResponseCallback, this, std::placeholders::_1));
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

            auto requestAutoExposureTime = std::make_shared<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime::Request>();
            requestAutoExposureTime->enabled = enabled;

            auto resultAutoExposureTime = client_auto_exposure_time_->async_send_request(
                requestAutoExposureTime, std::bind(&RgbConfiguration::autoExposureTimeResponseCallback, this, std::placeholders::_1));
        }

        void callExposureTime(int exposure_time)
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

            auto requestExposureTime = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraExposureTime::Request>();
            requestExposureTime->exposure_time = exposure_time;

            auto resultExposureTime = client_exposure_time_->async_send_request(
                requestExposureTime, std::bind(&RgbConfiguration::exposureTimeResponseCallback, this, std::placeholders::_1));
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
            requestStreamingProtocol->sensor_type = (int)sensorTypes::sensor_econ_rgb;
            requestStreamingProtocol->protocol = protocol;

            auto resultStreamingProtocol = client_streaming_protocol_->async_send_request(
                requestStreamingProtocol, std::bind(&RgbConfiguration::streamingProtocolResponseCallback, this, std::placeholders::_1));
        }

        // Service callbacks
        void brightnessResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraBrightness>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_camera_brightness_ = this->get_parameter("rgb_camera_brightness").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("rgb_camera_brightness", rgb_camera_brightness_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_rgb_camera_brightness");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("rgb_camera_brightness", rgb_camera_brightness_));
            }
        }

        void contrastResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraContrast>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_camera_contrast_ = this->get_parameter("rgb_camera_contrast").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("rgb_camera_contrast", rgb_camera_contrast_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_rgb_camera_contrast");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("rgb_camera_contrast", rgb_camera_contrast_));
            }
        }

        void saturationResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraSaturation>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_camera_saturation_ = this->get_parameter("rgb_camera_saturation").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("rgb_camera_saturation", rgb_camera_saturation_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_rgb_camera_saturation");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("rgb_camera_saturation", rgb_camera_saturation_));
            }
        }

        void sharpnessResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraSharpness>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_camera_sharpness_ = this->get_parameter("rgb_camera_sharpness").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("rgb_camera_sharpness", rgb_camera_sharpness_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_rgb_camera_sharpness");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("rgb_camera_sharpness", rgb_camera_sharpness_));
            }
        }

        void gammaResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraGamma>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_camera_gamma_ = this->get_parameter("rgb_camera_gamma").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("rgb_camera_gamma", rgb_camera_gamma_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_rgb_camera_gamma");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("rgb_camera_gamma", rgb_camera_gamma_));
            }
        }

        void gainResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraGain>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_camera_gain_ = this->get_parameter("rgb_camera_gain").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("rgb_camera_gain", rgb_camera_gain_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_rgb_camera_gain");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("rgb_camera_gain", rgb_camera_gain_));
            }
        }

        void autoWhiteBalanceResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_camera_auto_white_balance_ = this->get_parameter("rgb_camera_auto_white_balance").as_bool();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("rgb_camera_auto_white_balance", rgb_camera_auto_white_balance_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service enable_rgb_camera_auto_white_balance");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("rgb_camera_auto_white_balance", rgb_camera_auto_white_balance_));
            }
        }

        void whiteBalanceResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance>::SharedFuture future)
        {
            if (!rgb_camera_auto_white_balance_)
            {
                auto status = future.wait_for(1s);
                if (status == std::future_status::ready)
                {
                    int error = future.get()->error;
                    if (!error)
                    {
                        // Parameter changed successfully, save value
                        rgb_camera_white_balance_ = this->get_parameter("rgb_camera_white_balance").as_int();
                    }
                    else
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                        // Parameter could not be changed, reset parameter to value before change
                        this->set_parameter(rclcpp::Parameter("rgb_camera_white_balance", rgb_camera_white_balance_));
                    }
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_rgb_camera_white_balance");
                    // Service could not be called, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("rgb_camera_white_balance", rgb_camera_white_balance_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "RGB camera auto white balance must be disabled to change white balance");
                // Reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("rgb_camera_white_balance", rgb_camera_white_balance_));
            }
        }

        void autoExposureTimeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    rgb_camera_auto_exposure_time_ = this->get_parameter("rgb_camera_auto_exposure_time").as_bool();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("rgb_camera_auto_exposure_time", rgb_camera_auto_exposure_time_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service enable_rgb_camera_auto_exposure_time");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("rgb_camera_auto_exposure_time", rgb_camera_auto_exposure_time_));
            }
        }

        void exposureTimeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraExposureTime>::SharedFuture future)
        {
            if (!rgb_camera_auto_exposure_time_)
            {
                auto status = future.wait_for(1s);
                if (status == std::future_status::ready)
                {
                    int error = future.get()->error;
                    if (!error)
                    {
                        // Parameter changed successfully, save value
                        rgb_camera_exposure_time_ = this->get_parameter("rgb_camera_exposure_time").as_int();
                    }
                    else
                    {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                        // Parameter could not be changed, reset parameter to value before change
                        this->set_parameter(rclcpp::Parameter("rgb_camera_exposure_time", rgb_camera_exposure_time_));
                    }
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_rgb_camera_exposure_time");
                    // Service could not be called, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("rgb_camera_exposure_time", rgb_camera_exposure_time_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "RGB camera auto exposure time must be disabled to change exposure time");
                // Reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("rgb_camera_exposure_time", rgb_camera_exposure_time_));
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
                    streaming_protocol_ = this->get_parameter("rgb_streaming_protocol").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("rgb_streaming_protocol", streaming_protocol_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_streaming_protocol");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("rgb_streaming_protocol", streaming_protocol_));
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

        int rgb_camera_brightness_;
        int rgb_camera_contrast_;
        int rgb_camera_saturation_;
        int rgb_camera_sharpness_;
        int rgb_camera_gamma_;
        int rgb_camera_gain_;
        bool rgb_camera_auto_white_balance_;
        int rgb_camera_white_balance_;
        bool rgb_camera_auto_exposure_time_;
        int rgb_camera_exposure_time_;
        int streaming_protocol_;

        rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraBrightness>::SharedPtr client_brightness_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraContrast>::SharedPtr client_contrast_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraSaturation>::SharedPtr client_saturation_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraSharpness>::SharedPtr client_sharpness_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraGamma>::SharedPtr client_gamma_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraGain>::SharedPtr client_gain_;
        rclcpp::Client<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance>::SharedPtr client_auto_white_balance_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance>::SharedPtr client_white_balance_;
        rclcpp::Client<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime>::SharedPtr client_auto_exposure_time_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraExposureTime>::SharedPtr client_exposure_time_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeStreamingProtocol>::SharedPtr client_streaming_protocol_;

        rclcpp::Service<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr srv_sensor_disconnected_;

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    }; // class RgbConfiguration

} // namespace l3cam_ros2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::RgbConfiguration> node = std::make_shared<l3cam_ros2::RgbConfiguration>();

    // Check if RGB is available
    int i = 0;
    while (!node->client_get_sensors_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
            return 0;
        }
        
        if (i >= node->get_parameter("timeout_secs").as_int())
            return 0;
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
                if (resultGetSensors.get()->sensors[i].sensor_type == sensor_econ_rgb && resultGetSensors.get()->sensors[i].sensor_available)
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RGB camera configuration is available");
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
    requestGetRtspPipeline.get()->sensor_type = (int)sensorTypes::sensor_econ_rgb;
    auto resultGetRtspPipeline = node->client_get_rtsp_pipeline_->async_send_request(requestGetRtspPipeline);

    if (rclcpp::spin_until_future_complete(node, resultGetRtspPipeline) == rclcpp::FutureReturnCode::SUCCESS)
    {
        error = resultGetRtspPipeline.get()->error;

        if (!error)
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.read_only = true;
            node->declare_parameter("rgb_rtsp_pipeline", resultGetRtspPipeline.get()->pipeline, descriptor);
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
