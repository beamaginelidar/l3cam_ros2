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

#include "l3cam_interfaces/srv/change_polarimetric_camera_brightness.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_black_level.hpp"
#include "l3cam_interfaces/srv/enable_polarimetric_camera_auto_gain.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_auto_gain_range.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_gain.hpp"
#include "l3cam_interfaces/srv/enable_polarimetric_camera_auto_exposure_time.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_auto_exposure_time_range.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_exposure_time.hpp"

using namespace std::chrono_literals;

class PolarimetricConfiguration : public rclcpp::Node
{
public:
    PolarimetricConfiguration() : Node("polarimetric_configuration"){
        rcl_interfaces::msg::ParameterDescriptor intDescriptor;
        rcl_interfaces::msg::ParameterDescriptor floatDescriptor;
        rcl_interfaces::msg::IntegerRange intRange;
        rcl_interfaces::msg::FloatingPointRange floatRange;
        intRange.set__from_value(0).set__to_value(255).set__step(1);
        intDescriptor.integer_range = {intRange};
        this->declare_parameter("polarimetric_camera_brightness", 127, intDescriptor); // 0 - 255
        floatRange.set__from_value(0.0).set__to_value(12.5).set__step(0.1);
        floatDescriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_black_level", 6.0, floatDescriptor); // 0 - 12.5
        this->declare_parameter("polarimetric_camera_auto_gain", true);
        floatRange.set__from_value(0.0).set__to_value(48.0).set__step(0.1);
        floatDescriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_auto_gain_range_minimum", 0.0, floatDescriptor); // 0 - 48
        floatRange.set__from_value(0.0).set__to_value(48.0).set__step(0.1);
        floatDescriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_auto_gain_range_maximum", 48.0, floatDescriptor); // 0 - 48
        floatRange.set__from_value(0.0).set__to_value(48.0).set__step(0.1);
        floatDescriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_gain", 24.0, floatDescriptor); // 0 - 48
        this->declare_parameter("polarimetric_camera_auto_exposure_time", true);
        floatRange.set__from_value(33.5).set__to_value(66470.6).set__step(0.1);
        floatDescriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_auto_exposure_time_range_minimum", 33.5, floatDescriptor); // 33.5 - 66470.6
        floatRange.set__from_value(33.5).set__to_value(66470.6).set__step(0.1);
        floatDescriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_auto_exposure_time_range_maximum", 66470.6, floatDescriptor); // 33.5 - 66470.6
        floatRange.set__from_value(33.5).set__to_value(66470.6).set__step(0.001);
        floatDescriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_exposure_time", 33.5, floatDescriptor); // 33.5 - 66470.6

        polarimetric_camera_brightness = this->get_parameter("polarimetric_camera_brightness").as_int();
        polarimetric_camera_black_level = this->get_parameter("polarimetric_camera_black_level").as_double();
        polarimetric_camera_auto_gain = this->get_parameter("polarimetric_camera_auto_gain").as_bool();
        polarimetric_camera_auto_gain_range_minimum = this->get_parameter("polarimetric_camera_auto_gain_range_minimum").as_double();
        polarimetric_camera_auto_gain_range_maximum = this->get_parameter("polarimetric_camera_auto_gain_range_maximum").as_double();
        polarimetric_camera_gain = this->get_parameter("polarimetric_camera_gain").as_double();
        polarimetric_camera_auto_exposure_time = this->get_parameter("polarimetric_camera_auto_exposure_time").as_bool();
        polarimetric_camera_auto_exposure_time_range_minimum = this->get_parameter("polarimetric_camera_auto_exposure_time_range_minimum").as_double();
        polarimetric_camera_auto_exposure_time_range_maximum = this->get_parameter("polarimetric_camera_auto_exposure_time_range_maximum").as_double();
        polarimetric_camera_exposure_time = this->get_parameter("polarimetric_camera_exposure_time").as_double();

        clientBrightness = this->create_client<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness>("change_polarimetric_camera_brightness");
        clientBlackLevel = this->create_client<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel>("change_polarimetric_camera_black_level");
        clientAutoGain = this->create_client<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain>("enable_polarimetric_camera_auto_gain");
        clientAutoGainRange = this->create_client<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange>("change_polarimetric_camera_auto_gain_range");
        clientGain = this->create_client<l3cam_interfaces::srv::ChangePolarimetricCameraGain>("change_polarimetric_camera_gain");
        clientAutoExposureTime = this->create_client<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime>("enable_polarimetric_camera_auto_exposure_time");
        clientAutoExposureTimeRange = this->create_client<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange>("change_polarimetric_camera_auto_exposure_time_range");
        clientExposureTime = this->create_client<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime>("change_polarimetric_camera_exposure_time");

        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PolarimetricConfiguration::parametersCallback, this, std::placeholders::_1));
    }

private:
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters)
        {
            std::string param_name = param.get_name();
            if (param_name == "polarimetric_camera_brightness")
            {
                while (!clientBrightness->wait_for_service(1s))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                        break;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                }

                auto requestBrightness = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness::Request>();
                requestBrightness->brightness = param.as_int();

                auto resultBrightness = clientBrightness->async_send_request(
                    requestBrightness, std::bind(&PolarimetricConfiguration::brightnessResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "polarimetric_camera_black_level")
            {
                while (!clientBlackLevel->wait_for_service(1s))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                        break;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                }

                auto requestBlackLevel = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel::Request>();
                requestBlackLevel->black_level = param.as_double();

                auto resultBlackLevel = clientBlackLevel->async_send_request(
                    requestBlackLevel, std::bind(&PolarimetricConfiguration::blackLevelResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "polarimetric_camera_auto_gain")
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

                auto requestAutoGain = std::make_shared<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain::Request>();
                requestAutoGain->enabled = param.as_bool();

                auto resultAutoGain = clientAutoGain->async_send_request(
                    requestAutoGain, std::bind(&PolarimetricConfiguration::autoGainResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "polarimetric_camera_auto_gain_range_minimum")
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

                auto requestAutoGainRange = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange::Request>();
                requestAutoGainRange->min_gain = param.as_double();
                requestAutoGainRange->max_gain = polarimetric_camera_auto_gain_range_maximum;

                auto resultAutoGainRange = clientAutoGainRange->async_send_request(
                    requestAutoGainRange, std::bind(&PolarimetricConfiguration::autoGainRangeResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "polarimetric_camera_auto_gain_range_maximum")
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

                auto requestAutoGainRange = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange::Request>();
                requestAutoGainRange->min_gain = polarimetric_camera_auto_gain_range_minimum;
                requestAutoGainRange->max_gain = param.as_double();

                auto resultAutoGainRange = clientAutoGainRange->async_send_request(
                    requestAutoGainRange, std::bind(&PolarimetricConfiguration::autoGainRangeResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "polarimetric_camera_gain")
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

                auto requestGain = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraGain::Request>();
                requestGain->gain = param.as_double();

                auto resultGain = clientGain->async_send_request(
                    requestGain, std::bind(&PolarimetricConfiguration::gainResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "polarimetric_camera_auto_exposure_time")
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

                auto requestAutoExposureTime = std::make_shared<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime::Request>();
                requestAutoExposureTime->enabled = param.as_bool();

                auto resultAutoExposureTime = clientAutoExposureTime->async_send_request(
                    requestAutoExposureTime, std::bind(&PolarimetricConfiguration::autoExposureTimeResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "polarimetric_camera_auto_exposure_time_range_minimum")
            {
                while (!clientAutoExposureTimeRange->wait_for_service(1s))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                        break;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting aExposureTime...");
                }

                auto requestAutoExposureTimeRange = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange::Request>();
                requestAutoExposureTimeRange->min_exposure = param.as_double();
                requestAutoExposureTimeRange->max_exposure = polarimetric_camera_auto_exposure_time_range_maximum;

                auto resultAutoExposureTimeRange = clientAutoExposureTimeRange->async_send_request(
                    requestAutoExposureTimeRange, std::bind(&PolarimetricConfiguration::autoExposureTimeRangeResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "polarimetric_camera_auto_exposure_time_range_maximum")
            {
                while (!clientAutoExposureTimeRange->wait_for_service(1s))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                        break;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting aExposureTime...");
                }

                auto requestAutoExposureTimeRange = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange::Request>();
                requestAutoExposureTimeRange->min_exposure = polarimetric_camera_auto_exposure_time_range_minimum;
                requestAutoExposureTimeRange->max_exposure = param.as_double();

                auto resultAutoExposureTimeRange = clientAutoExposureTimeRange->async_send_request(
                    requestAutoExposureTimeRange, std::bind(&PolarimetricConfiguration::autoExposureTimeRangeResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "polarimetric_camera_exposure_time")
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

                auto requestExposureTime = std::make_shared<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime::Request>();
                requestExposureTime->exposure_time = param.as_double();

                auto resultExposureTime = clientExposureTime->async_send_request(
                    requestExposureTime, std::bind(&PolarimetricConfiguration::exposureTimeResponseCallback, this, std::placeholders::_1));
            }

        }

        return result;
    }

    void brightnessResponseCallback(
        rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness>::SharedFuture future)
    {
        auto status = future.wait_for(1s);
        if (status == std::future_status::ready)
        {
            int error = future.get()->error;
            if (!error)
                polarimetric_camera_brightness = this->get_parameter("polarimetric_camera_brightness").as_int();
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("polarimetric_camera_brightness", polarimetric_camera_brightness));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_polarimetric_camera_brightness");
            // this->set_parameter(rclcpp::Parameter("polarimetric_camera_brightness", polarimetric_camera_brightness));
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
                polarimetric_camera_black_level = this->get_parameter("polarimetric_camera_black_level").as_double();
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("polarimetric_camera_black_level", polarimetric_camera_black_level));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_polarimetric_camera_black_level");
            // this->set_parameter(rclcpp::Parameter("polarimetric_camera_black_level", polarimetric_camera_black_level));
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
                polarimetric_camera_auto_gain = this->get_parameter("polarimetric_camera_auto_gain").as_bool();
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_gain", polarimetric_camera_auto_gain));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_polarimetric_camera_auto_gain");
            // this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_gain", polarimetric_camera_auto_gain));
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
                polarimetric_camera_auto_gain_range_minimum = this->get_parameter("polarimetric_camera_auto_gain_range_minimum").as_double();
                polarimetric_camera_auto_gain_range_maximum = this->get_parameter("polarimetric_camera_auto_gain_range_maximum").as_double();
            }
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_gain_range_minimum", polarimetric_camera_auto_gain_range_minimum));
                // this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_gain_range_maximum", polarimetric_camera_auto_gain_range_maximum));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_polarimetric_camera_auto_gain_range");
            // this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_gain_range_minimum", polarimetric_camera_auto_gain_range_minimum));
            // this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_gain_range_maximum", polarimetric_camera_auto_gain_range_minimum));
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
                polarimetric_camera_gain = this->get_parameter("polarimetric_camera_gain").as_double();
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("polarimetric_camera_gain", polarimetric_camera_gain));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_polarimetric_camera_gain");
            // this->set_parameter(rclcpp::Parameter("polarimetric_camera_gain", polarimetric_camera_gain));
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
                polarimetric_camera_auto_exposure_time = this->get_parameter("polarimetric_camera_auto_exposure_time").as_bool();
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_exposure_time", polarimetric_camera_auto_exposure_time));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_polarimetric_camera_auto_exposure_time");
            // this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_exposure_time", polarimetric_camera_auto_exposure_time));
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
                polarimetric_camera_auto_exposure_time_range_minimum = this->get_parameter("polarimetric_camera_auto_exposure_time_range_minimum").as_double();
                polarimetric_camera_auto_exposure_time_range_maximum = this->get_parameter("polarimetric_camera_auto_exposure_time_range_maximum").as_double();
            }
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_exposure_time_range_minimum", polarimetric_camera_auto_exposure_time_range_minimum));
                // this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_exposure_time_range_maximum", polarimetric_camera_auto_exposure_time_range_maximum));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_polarimetric_camera_auto_exposure_time_range");
            // this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_exposure_time_range_minimum", polarimetric_camera_auto_exposure_time_range_minimum));
            // this->set_parameter(rclcpp::Parameter("polarimetric_camera_auto_exposure_time_range_maximum", polarimetric_camera_auto_exposure_time_range_minimum));
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
                polarimetric_camera_exposure_time = this->get_parameter("polarimetric_camera_exposure_time").as_double();
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("polarimetric_camera_exposure_time", polarimetric_camera_exposure_time));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_polarimetric_camera_exposure_time");
            // this->set_parameter(rclcpp::Parameter("polarimetric_camera_exposure_time", polarimetric_camera_exposure_time));
        }
    }

    int polarimetric_camera_brightness;
    double polarimetric_camera_black_level;
    bool polarimetric_camera_auto_gain;
    double polarimetric_camera_auto_gain_range_minimum;
    double polarimetric_camera_auto_gain_range_maximum;
    double polarimetric_camera_gain;
    bool polarimetric_camera_auto_exposure_time;
    double polarimetric_camera_auto_exposure_time_range_minimum;
    double polarimetric_camera_auto_exposure_time_range_maximum;
    double polarimetric_camera_exposure_time;

    rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness>::SharedPtr clientBrightness;
    rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel>::SharedPtr clientBlackLevel;
    rclcpp::Client<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain>::SharedPtr clientAutoGain;
    rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange>::SharedPtr clientAutoGainRange;
    rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraGain>::SharedPtr clientGain;
    rclcpp::Client<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime>::SharedPtr clientAutoExposureTime;
    rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange>::SharedPtr clientAutoExposureTimeRange;
    rclcpp::Client<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime>::SharedPtr clientExposureTime;

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    int error = L3CAM_OK;

    std::shared_ptr<PolarimetricConfiguration> node = std::make_shared<PolarimetricConfiguration>();

    // Check if Polarimetric is available
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

    auto requestGetSensors = std::make_shared<l3cam_interfaces::srv::GetSensorsAvailable::Request>();
    auto resultGetSensors = clientGetSensors->async_send_request(requestGetSensors);
    bool sensor_is_available = false;
    if (rclcpp::spin_until_future_complete(node, resultGetSensors) == rclcpp::FutureReturnCode::SUCCESS)
    {
        error = resultGetSensors.get()->error;

        if (!error)
            for (int i = 0; i < resultGetSensors.get()->num_sensors; ++i)
            {
                if (resultGetSensors.get()->sensors[i].sensor_type == sensor_pol)
                    sensor_is_available = true;
            }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
            return 1;
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_sensors_available");
        return 1;
    }

    if (sensor_is_available)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Polarimetric camera configuration is available");
    else
        return 0;

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
