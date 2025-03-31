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

#include "l3cam_interfaces/srv/enable_polarimetric_camera_stream_processed_image.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_process_type.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_brightness.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_black_level.hpp"
#include "l3cam_interfaces/srv/enable_polarimetric_camera_auto_gain.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_auto_gain_range.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_gain.hpp"
#include "l3cam_interfaces/srv/enable_polarimetric_camera_auto_exposure_time.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_auto_exposure_time_range.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_exposure_time.hpp"
#include "l3cam_interfaces/srv/change_streaming_protocol.hpp"
#include "l3cam_interfaces/srv/get_rtsp_pipeline.hpp"

#include "l3cam_interfaces/srv/sensor_disconnected.hpp"

#include "l3cam_ros2_utils.hpp"

using namespace std::chrono_literals;

namespace l3cam_ros2
{
    class PolarimetricConfiguration : public rclcpp::Node
    {
    public:
        explicit PolarimetricConfiguration() : Node("polarimetric_configuration")
        {
            // Create service clients
            client_get_sensors_ = this->create_client<l3cam_interfaces::srv::GetSensorsAvailable>("get_sensors_available");
            client_stream_processed_ = this->create_client<l3cam_interfaces::srv::EnablePolarimetricCameraStreamProcessedImage>("enable_polarimetric_camera_stream_processed_image");
            client_process_type_ = this->create_client<l3cam_interfaces::srv::ChangePolarimetricCameraProcessType>("change_polarimetric_camera_process_type");
            client_brightness_ = this->create_client<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness>("change_polarimetric_camera_brightness");
            client_black_level_ = this->create_client<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel>("change_polarimetric_camera_black_level");
            client_auto_gain_ = this->create_client<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain>("enable_polarimetric_camera_auto_gain");
            client_auto_gain_range_ = this->create_client<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange>("change_polarimetric_camera_auto_gain_range");
            client_gain_ = this->create_client<l3cam_interfaces::srv::ChangePolarimetricCameraGain>("change_polarimetric_camera_gain");
            client_auto_exposure_time_ = this->create_client<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime>("enable_polarimetric_camera_auto_exposure_time");
            client_auto_exposure_time_range_ = this->create_client<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange>("change_polarimetric_camera_auto_exposure_time_range");
            client_exposure_time_ = this->create_client<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime>("change_polarimetric_camera_exposure_time");
            client_streaming_protocol_ = this->create_client<l3cam_interfaces::srv::ChangeStreamingProtocol>("change_streaming_protocol");
            client_get_rtsp_pipeline_ = this->create_client<l3cam_interfaces::srv::GetRtspPipeline>("get_rtsp_pipeline");

            declareParams();
            loadDefaultParams();

            // Create service server
            srv_sensor_disconnected_ = this->create_service<l3cam_interfaces::srv::SensorDisconnected>(
                "polarimetric_configuration_disconnected", std::bind(&PolarimetricConfiguration::sensorDisconnectedCallback, this, std::placeholders::_1, std::placeholders::_2));

            // Callback on parameters changed
            callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&PolarimetricConfiguration::parametersCallback, this, std::placeholders::_1));
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
            this->declare_parameter("polarimetric_camera_stream_processed_image", true);
            intRange.set__from_value(0).set__to_value(4);
            descriptor.integer_range = {intRange};
            this->declare_parameter("polarimetric_camera_process_type", 4, descriptor); // see polAngle
            intRange.set__from_value(0).set__to_value(255);
            descriptor.integer_range = {intRange};
            this->declare_parameter("polarimetric_camera_brightness", 127, descriptor); // 0 - 255
            floatRange.set__from_value(0.0).set__to_value(12.5);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("polarimetric_camera_black_level", 6.0, descriptor); // 0 - 12.5
            this->declare_parameter("polarimetric_camera_auto_gain", true);
            floatRange.set__from_value(0.0).set__to_value(48.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("polarimetric_camera_auto_gain_range_minimum", 0.0, descriptor); // 0 - 48
            floatRange.set__from_value(0.0).set__to_value(48.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("polarimetric_camera_auto_gain_range_maximum", 48.0, descriptor); // 0 - 48
            floatRange.set__from_value(0.0).set__to_value(48.0);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("polarimetric_camera_gain", 24.0, descriptor); // 0 - 48
            this->declare_parameter("polarimetric_camera_auto_exposure_time", true);
            floatRange.set__from_value(33.5).set__to_value(66470.6);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("polarimetric_camera_auto_exposure_time_range_minimum", 33.5, descriptor); // 33.5 - 66470.6
            floatRange.set__from_value(33.5).set__to_value(66470.6);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("polarimetric_camera_auto_exposure_time_range_maximum", 66470.6, descriptor); // 33.5 - 66470.6
            floatRange.set__from_value(33.5).set__to_value(66470.6);
            descriptor.floating_point_range = {floatRange};
            this->declare_parameter("polarimetric_camera_exposure_time", 33.5, descriptor); // 33.5 - 66470.6
            intRange.set__from_value(0).set__to_value(1);
            descriptor.integer_range = {intRange};
            descriptor.description = 
                "Value must be:\n"
                "\tprotocol_raw_udp = 0\n"
                "\tprotocol_gstreamer = 1";
            this->declare_parameter("polarimetric_streaming_protocol", 0, descriptor); // 0(protocol_raw_udp), 1(protocol_gstreamer)
        }

        void loadDefaultParams()
        {
            // Get and save parameters
            polarimetric_camera_stream_processed_ = this->get_parameter("polarimetric_camera_stream_processed_image").as_bool();
            polarimetric_camera_process_type_ = this->get_parameter("polarimetric_camera_process_type").as_int();
            polarimetric_camera_brightness_ = this->get_parameter("polarimetric_camera_brightness").as_int();
            polarimetric_camera_black_level_ = this->get_parameter("polarimetric_camera_black_level").as_double();
            polarimetric_camera_auto_gain_ = this->get_parameter("polarimetric_camera_auto_gain").as_bool();
            polarimetric_camera_auto_gain_range_minimum_ = this->get_parameter("polarimetric_camera_auto_gain_range_minimum").as_double();
            polarimetric_camera_auto_gain_range_maximum_ = this->get_parameter("polarimetric_camera_auto_gain_range_maximum").as_double();
            polarimetric_camera_gain_ = this->get_parameter("polarimetric_camera_gain").as_double();
            polarimetric_camera_auto_exposure_time_ = this->get_parameter("polarimetric_camera_auto_exposure_time").as_bool();
            polarimetric_camera_auto_exposure_time_range_minimum_ = this->get_parameter("polarimetric_camera_auto_exposure_time_range_minimum").as_double();
            polarimetric_camera_auto_exposure_time_range_maximum_ = this->get_parameter("polarimetric_camera_auto_exposure_time_range_maximum").as_double();
            polarimetric_camera_exposure_time_ = this->get_parameter("polarimetric_camera_exposure_time").as_double();
            streaming_protocol_ = this->get_parameter("polarimetric_streaming_protocol").as_int();
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
                if (param_name == "polarimetric_camera_stream_processed_image" && param.as_int() != polarimetric_camera_stream_processed_)
                {
                    callStreamProcessed(param.as_int());
                }
                if (param_name == "polarimetric_camera_process_type" && param.as_int() != polarimetric_camera_process_type_)
                {
                    callProcessType(param.as_int());
                }
                if (param_name == "polarimetric_camera_brightness" && param.as_int() != polarimetric_camera_brightness_)
                {
                    callBrightness(param.as_int());
                }
                if (param_name == "polarimetric_camera_black_level" && param.as_double() != polarimetric_camera_black_level_)
                {
                    callBlackLevel(param.as_double());
                }
                if (param_name == "polarimetric_camera_auto_gain" && param.as_bool() != polarimetric_camera_auto_gain_)
                {
                    callAutoGain(param.as_bool());
                }
                if (param_name == "polarimetric_camera_auto_gain_range_minimum" && param.as_double() != polarimetric_camera_auto_gain_range_minimum_)
                {
                    callAutoGainRange(param.as_double(), polarimetric_camera_auto_gain_range_maximum_);
                }
                if (param_name == "polarimetric_camera_auto_gain_range_maximum" && param.as_double() != polarimetric_camera_auto_gain_range_maximum_)
                {
                    callAutoGainRange(polarimetric_camera_auto_gain_range_minimum_, param.as_double());
                }
                if (param_name == "polarimetric_camera_gain" && param.as_double() != polarimetric_camera_gain_)
                {
                    callGain(param.as_double());
                }
                if (param_name == "polarimetric_camera_auto_exposure_time" && param.as_bool() != polarimetric_camera_auto_exposure_time_)
                {
                    callAutoExposureTime(param.as_bool());
                }
                if (param_name == "polarimetric_camera_auto_exposure_time_range_minimum" && param.as_double() != polarimetric_camera_auto_exposure_time_range_minimum_)
                {
                    callAutoExposureTimeRange(param.as_double(), polarimetric_camera_auto_exposure_time_range_maximum_);
                }
                if (param_name == "polarimetric_camera_auto_exposure_time_range_maximum" && param.as_double() != polarimetric_camera_auto_exposure_time_range_maximum_)
                {
                    callAutoExposureTimeRange(polarimetric_camera_auto_exposure_time_range_minimum_, param.as_double());
                }
                if (param_name == "polarimetric_camera_exposure_time" && param.as_double() != polarimetric_camera_exposure_time_)
                {
                    callExposureTime(param.as_double());
                }
                if (param_name == "polarimetric_streaming_protocol" && param.as_int() != streaming_protocol_)
                {
                    callStreamingProtocol(param.as_int());
                }
            }

            return result;
        }

        // Service calls
        void callStreamProcessed(bool enabled)
        {
            while (!client_stream_processed_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestStreamProcessed = std::make_shared<l3cam_interfaces::srv::EnablePolarimetricCameraStreamProcessedImage::Request>();
            requestStreamProcessed->enabled = enabled;

            auto resultStreamProcessed = client_stream_processed_->async_send_request(
                requestStreamProcessed, std::bind(&PolarimetricConfiguration::streamProcessedResponseCallback, this, std::placeholders::_1));
        }

        void callProcessType(int type)
        {
            while (!client_process_type_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestProcessType = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraProcessType::Request>();
            requestProcessType->type = type;

            auto resultProcessType = client_process_type_->async_send_request(
                requestProcessType, std::bind(&PolarimetricConfiguration::processTypeResponseCallback, this, std::placeholders::_1));
        }

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

            auto requestBrightness = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness::Request>();
            requestBrightness->brightness = brightness;

            auto resultBrightness = client_brightness_->async_send_request(
                requestBrightness, std::bind(&PolarimetricConfiguration::brightnessResponseCallback, this, std::placeholders::_1));
        }

        void callBlackLevel(double black_level)
        {
            while (!client_black_level_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestBlackLevel = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel::Request>();
            requestBlackLevel->black_level = black_level;

            auto resultBlackLevel = client_black_level_->async_send_request(
                requestBlackLevel, std::bind(&PolarimetricConfiguration::blackLevelResponseCallback, this, std::placeholders::_1));
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

            auto requestAutoGain = std::make_shared<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain::Request>();
            requestAutoGain->enabled = enabled;

            auto resultAutoGain = client_auto_gain_->async_send_request(
                requestAutoGain, std::bind(&PolarimetricConfiguration::autoGainResponseCallback, this, std::placeholders::_1));
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

            auto requestAutoGainRange = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange::Request>();
            requestAutoGainRange->min_gain = range_min;
            requestAutoGainRange->max_gain = range_max;

            auto resultAutoGainRange = client_auto_gain_range_->async_send_request(
                requestAutoGainRange, std::bind(&PolarimetricConfiguration::autoGainRangeResponseCallback, this, std::placeholders::_1));
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

            auto requestGain = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraGain::Request>();
            requestGain->gain = gain;

            auto resultGain = client_gain_->async_send_request(
                requestGain, std::bind(&PolarimetricConfiguration::gainResponseCallback, this, std::placeholders::_1));
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

            auto requestAutoExposureTime = std::make_shared<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime::Request>();
            requestAutoExposureTime->enabled = enabled;

            auto resultAutoExposureTime = client_auto_exposure_time_->async_send_request(
                requestAutoExposureTime, std::bind(&PolarimetricConfiguration::autoExposureTimeResponseCallback, this, std::placeholders::_1));
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
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting aExposureTime...");
            }

            auto requestAutoExposureTimeRange = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange::Request>();
            requestAutoExposureTimeRange->min_exposure = range_min;
            requestAutoExposureTimeRange->max_exposure = range_max;

            auto resultAutoExposureTimeRange = client_auto_exposure_time_range_->async_send_request(
                requestAutoExposureTimeRange, std::bind(&PolarimetricConfiguration::autoExposureTimeRangeResponseCallback, this, std::placeholders::_1));
        }

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

            auto requestExposureTime = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime::Request>();
            requestExposureTime->exposure_time = exposure_time;

            auto resultExposureTime = client_exposure_time_->async_send_request(
                requestExposureTime, std::bind(&PolarimetricConfiguration::exposureTimeResponseCallback, this, std::placeholders::_1));
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
            requestStreamingProtocol->sensor_type = (int)sensorTypes::sensor_pol;
            requestStreamingProtocol->protocol = protocol;

            auto resultStreamingProtocol = client_streaming_protocol_->async_send_request(
                requestStreamingProtocol, std::bind(&PolarimetricConfiguration::streamingProtocolResponseCallback, this, std::placeholders::_1));
        }

        // Service callbacks
        void streamProcessedResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::EnablePolarimetricCameraStreamProcessedImage>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_camera_stream_processed_ = this->get_parameter("polarimetric_camera_stream_processed_image").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("polarimetric_camera_stream_processed_image", polarimetric_camera_stream_processed_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_polarimetric_camera_stream_processed_image");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("polarimetric_camera_stream_processed_image", polarimetric_camera_stream_processed_));
            }
        }

        void processTypeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraProcessType>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_camera_process_type_ = this->get_parameter("polarimetric_camera_process_type").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("polarimetric_camera_process_type", polarimetric_camera_process_type_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_polarimetric_camera_process_type");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("polarimetric_camera_process_type", polarimetric_camera_process_type_));
            }
        }

        void brightnessResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_camera_brightness_ = this->get_parameter("polarimetric_camera_brightness").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("polarimetric_camera_brightness", polarimetric_camera_brightness_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_polarimetric_camera_brightness");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("polarimetric_camera_brightness", polarimetric_camera_brightness_));
            }
        }

        void blackLevelResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_camera_black_level_ = this->get_parameter("polarimetric_camera_black_level").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("polarimetric_camera_black_level", polarimetric_camera_black_level_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_polarimetric_camera_black_level");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("polarimetric_camera_black_level", polarimetric_camera_black_level_));
            }
        }

        void autoGainResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_camera_auto_gain_ = this->get_parameter("polarimetric_camera_auto_gain").as_bool();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_gain", polarimetric_camera_auto_gain_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_polarimetric_camera_auto_gain");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_gain", polarimetric_camera_auto_gain_));
            }
        }

        void autoGainRangeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameters changed successfully, save value
                    polarimetric_camera_auto_gain_range_minimum_ = this->get_parameter("polarimetric_camera_auto_gain_range_minimum").as_double();
                    polarimetric_camera_auto_gain_range_maximum_ = this->get_parameter("polarimetric_camera_auto_gain_range_maximum").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_gain_range_minimum", polarimetric_camera_auto_gain_range_minimum_));
                    this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_gain_range_maximum", polarimetric_camera_auto_gain_range_maximum_));
                }
            }
            else
            {
                // Service could not be called, reset parameters to value before change
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_polarimetric_camera_auto_gain_range");
                this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_gain_range_minimum", polarimetric_camera_auto_gain_range_minimum_));
                this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_gain_range_maximum", polarimetric_camera_auto_gain_range_minimum_));
            }
        }

        void gainResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraGain>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_camera_gain_ = this->get_parameter("polarimetric_camera_gain").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("polarimetric_camera_gain", polarimetric_camera_gain_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_polarimetric_camera_gain");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("polarimetric_camera_gain", polarimetric_camera_gain_));
            }
        }

        void autoExposureTimeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_camera_auto_exposure_time_ = this->get_parameter("polarimetric_camera_auto_exposure_time").as_bool();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_exposure_time", polarimetric_camera_auto_exposure_time_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_polarimetric_camera_auto_exposure_time");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_exposure_time", polarimetric_camera_auto_exposure_time_));
            }
        }

        void autoExposureTimeRangeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameters changed successfully, save value
                    polarimetric_camera_auto_exposure_time_range_minimum_ = this->get_parameter("polarimetric_camera_auto_exposure_time_range_minimum").as_double();
                    polarimetric_camera_auto_exposure_time_range_maximum_ = this->get_parameter("polarimetric_camera_auto_exposure_time_range_maximum").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_exposure_time_range_minimum", polarimetric_camera_auto_exposure_time_range_minimum_));
                    this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_exposure_time_range_maximum", polarimetric_camera_auto_exposure_time_range_maximum_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_polarimetric_camera_auto_exposure_time_range");
                // Service could not be called, reset parameters to value before change
                this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_exposure_time_range_minimum", polarimetric_camera_auto_exposure_time_range_minimum_));
                this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_exposure_time_range_maximum", polarimetric_camera_auto_exposure_time_range_minimum_));
            }
        }

        void exposureTimeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    polarimetric_camera_exposure_time_ = this->get_parameter("polarimetric_camera_exposure_time").as_double();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("polarimetric_camera_exposure_time", polarimetric_camera_exposure_time_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_polarimetric_camera_exposure_time");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("polarimetric_camera_exposure_time", polarimetric_camera_exposure_time_));
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
                    streaming_protocol_ = this->get_parameter("polarimetric_streaming_protocol").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("polarimetric_streaming_protocol", streaming_protocol_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_streaming_protocol");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("polarimetric_streaming_protocol", streaming_protocol_));
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

        rclcpp::Client<l3cam_interfaces::srv::EnablePolarimetricCameraStreamProcessedImage>::SharedPtr client_stream_processed_;
        rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraProcessType>::SharedPtr client_process_type_;
        rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness>::SharedPtr client_brightness_;
        rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel>::SharedPtr client_black_level_;
        rclcpp::Client<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain>::SharedPtr client_auto_gain_;
        rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange>::SharedPtr client_auto_gain_range_;
        rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraGain>::SharedPtr client_gain_;
        rclcpp::Client<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime>::SharedPtr client_auto_exposure_time_;
        rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange>::SharedPtr client_auto_exposure_time_range_;
        rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime>::SharedPtr client_exposure_time_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeStreamingProtocol>::SharedPtr client_streaming_protocol_;

        rclcpp::Service<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr srv_sensor_disconnected_;

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

        bool polarimetric_camera_stream_processed_;
        int polarimetric_camera_process_type_;
        int polarimetric_camera_brightness_;
        double polarimetric_camera_black_level_;
        bool polarimetric_camera_auto_gain_;
        double polarimetric_camera_auto_gain_range_minimum_;
        double polarimetric_camera_auto_gain_range_maximum_;
        double polarimetric_camera_gain_;
        bool polarimetric_camera_auto_exposure_time_;
        double polarimetric_camera_auto_exposure_time_range_minimum_;
        double polarimetric_camera_auto_exposure_time_range_maximum_;
        double polarimetric_camera_exposure_time_;
        int streaming_protocol_;

    }; // class PolarimetricConfiguration

} // namespace l3cam_ros2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::PolarimetricConfiguration> node = std::make_shared<l3cam_ros2::PolarimetricConfiguration>();

    // Check if Polarimetric is available
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
            for (int i = 0; i < resultGetSensors.get()->num_sensors; ++i)
            {
                if (resultGetSensors.get()->sensors[i].sensor_type == sensor_pol && resultGetSensors.get()->sensors[i].sensor_available)
                    sensor_is_available = true;
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Polarimetric camera configuration is available");
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
    requestGetRtspPipeline.get()->sensor_type = (int)sensorTypes::sensor_pol;
    auto resultGetRtspPipeline = node->client_get_rtsp_pipeline_->async_send_request(requestGetRtspPipeline);

    if (rclcpp::spin_until_future_complete(node, resultGetRtspPipeline) == rclcpp::FutureReturnCode::SUCCESS)
    {
        error = resultGetRtspPipeline.get()->error;

        if (!error)
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.read_only = true;
            node->declare_parameter("polarimetric_rtsp_pipeline", resultGetRtspPipeline.get()->pipeline, descriptor);
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
