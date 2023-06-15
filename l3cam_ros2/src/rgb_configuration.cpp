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

using namespace std::chrono_literals;

class RgbConfiguration : public rclcpp::Node
{
public:
    RgbConfiguration() : Node("rgb_configuration")
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(-15).set__to_value(15).set__step(1);
        descriptor.integer_range = {range};
        this->declare_parameter("rgb_camera_brightness", 0, descriptor); // -15 - 15
        this->declare_parameter("rgb_camera_contrast", 10, descriptor);  // 0 - 30
        range.set__from_value(0).set__to_value(60).set__step(1);
        descriptor.integer_range = {range};
        this->declare_parameter("rgb_camera_saturation", 16, descriptor); // 0 - 60
        range.set__from_value(0).set__to_value(127).set__step(1);
        descriptor.integer_range = {range};
        this->declare_parameter("rgb_camera_sharpness", 16, descriptor); // 0 - 127
        range.set__from_value(40).set__to_value(500).set__step(1);
        descriptor.integer_range = {range};
        this->declare_parameter("rgb_camera_gamma", 220, descriptor); // 40 - 500
        range.set__from_value(0).set__to_value(63).set__step(1);
        descriptor.integer_range = {range};
        this->declare_parameter("rgb_camera_gain", 0, descriptor); // 0 - 63
        this->declare_parameter("rgb_camera_auto_white_balance", true);
        range.set__from_value(1000).set__to_value(10000).set__step(1);
        descriptor.integer_range = {range};
        // descriptor.read_only = this->get_parameter("rgb_camera_auto_white_balance").as_bool();
        this->declare_parameter("rgb_camera_white_balance", 5000, descriptor); // 1000 - 10000
        this->declare_parameter("rgb_camera_auto_exposure_time", true);
        range.set__from_value(1).set__to_value(10000).set__step(1);
        descriptor.integer_range = {range};
        // descriptor.read_only = this->get_parameter("rgb_camera_auto_exposure_time").as_bool();
        this->declare_parameter("rgb_camera_exposure_time", 156, descriptor); // 1 - 10000

        /*range.set__from_value(1).set__to_value(3).set__step(1);
        descriptor.integer_range = {range};
        this->declare_parameter("rgb_camera_resolution", 3, descriptor); // see econResolutions
        range.set__from_value(1).set__to_value(16).set__step(1);
        descriptor.integer_range = {range};
        this->declare_parameter("rgb_camera_framerate", 10, descriptor); // 1 - 16*/

        rgb_camera_brightness = this->get_parameter("rgb_camera_brightness").as_int();
        rgb_camera_contrast = this->get_parameter("rgb_camera_contrast").as_int();
        rgb_camera_saturation = this->get_parameter("rgb_camera_saturation").as_int();
        rgb_camera_sharpness = this->get_parameter("rgb_camera_sharpness").as_int();
        rgb_camera_gamma = this->get_parameter("rgb_camera_gamma").as_int();
        rgb_camera_gain = this->get_parameter("rgb_camera_gain").as_int();
        rgb_camera_auto_white_balance = this->get_parameter("rgb_camera_auto_white_balance").as_bool();
        rgb_camera_white_balance = this->get_parameter("rgb_camera_white_balance").as_int();
        rgb_camera_auto_exposure_time = this->get_parameter("rgb_camera_auto_exposure_time").as_bool();
        rgb_camera_exposure_time = this->get_parameter("rgb_camera_exposure_time").as_int();

        // rgb_camera_resolution = (econResolutions)this->get_parameter("rgb_camera_resolution").as_int();
        // rgb_camera_framerate = this->get_parameter("rgb_camera_framerate").as_int();

        clientBrightness = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraBrightness>("change_rgb_camera_brightness");
        clientContrast = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraContrast>("change_rgb_camera_contrast");
        clientSaturation = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraSaturation>("change_rgb_camera_saturation");
        clientSharpness = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraSharpness>("change_rgb_camera_sharpness");
        clientGamma = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraGamma>("change_rgb_camera_gamma");
        clientGain = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraGain>("change_rgb_camera_gain");
        clientAutoWhiteBalance = this->create_client<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance>("enable_rgb_camera_auto_white_balance");
        clientWhiteBalance = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance>("change_rgb_camera_white_balance");
        clientAutoExposureTime = this->create_client<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime>("enable_rgb_camera_auto_exposure_time");
        clientExposureTime = this->create_client<l3cam_interfaces::srv::ChangeRgbCameraExposureTime>("change_rgb_camera_exposure_time");

        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&RgbConfiguration::parametersCallback, this, std::placeholders::_1));
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
            if (param_name == "rgb_camera_brightness")
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

                auto requestBrightness = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraBrightness::Request>();
                requestBrightness->brightness = param.as_int();

                auto resultBrightness = clientBrightness->async_send_request(
                    requestBrightness, std::bind(&RgbConfiguration::brightnessResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "rgb_camera_contrast")
            {
                while (!clientContrast->wait_for_service(1s))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                        break;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                }

                auto requestContrast = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraContrast::Request>();
                requestContrast->contrast = param.as_int();

                auto resultContrast = clientContrast->async_send_request(
                    requestContrast, std::bind(&RgbConfiguration::contrastResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "rgb_camera_saturation")
            {
                while (!clientSaturation->wait_for_service(1s))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                        break;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                }

                auto requestSaturation = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraSaturation::Request>();
                requestSaturation->saturation = param.as_int();

                auto resultSaturation = clientSaturation->async_send_request(
                    requestSaturation, std::bind(&RgbConfiguration::saturationResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "rgb_camera_sharpness")
            {
                while (!clientSharpness->wait_for_service(1s))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                        break;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                }

                auto requestSharpness = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraSharpness::Request>();
                requestSharpness->sharpness = param.as_int();

                auto resultSharpness = clientSharpness->async_send_request(
                    requestSharpness, std::bind(&RgbConfiguration::sharpnessResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "rgb_camera_gamma")
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

                auto requestGamma = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraGamma::Request>();
                requestGamma->gamma = param.as_int();

                auto resultGamma = clientGamma->async_send_request(
                    requestGamma, std::bind(&RgbConfiguration::gammaResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "rgb_camera_gain")
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

                auto requestGain = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraGain::Request>();
                requestGain->gain = param.as_int();

                auto resultGain = clientGain->async_send_request(
                    requestGain, std::bind(&RgbConfiguration::gainResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "rgb_camera_auto_white_balance")
            {
                while (!clientAutoWhiteBalance->wait_for_service(1s))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                        break;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                }

                auto requestAutoWhiteBalance = std::make_shared<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance::Request>();
                requestAutoWhiteBalance->enabled = param.as_bool();

                auto resultAutoWhiteBalance = clientAutoWhiteBalance->async_send_request(
                    requestAutoWhiteBalance, std::bind(&RgbConfiguration::autoWhiteBalanceResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "rgb_camera_white_balance")
            {
                while (!clientWhiteBalance->wait_for_service(1s))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                        break;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                }

                auto requestWhiteBalance = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance::Request>();
                requestWhiteBalance->white_balance = param.as_int();

                auto resultWhiteBalance = clientWhiteBalance->async_send_request(
                    requestWhiteBalance, std::bind(&RgbConfiguration::whiteBalanceResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "rgb_camera_auto_exposure_time")
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

                auto requestAutoExposureTime = std::make_shared<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime::Request>();
                requestAutoExposureTime->enabled = param.as_bool();

                auto resultAutoExposureTime = clientAutoExposureTime->async_send_request(
                    requestAutoExposureTime, std::bind(&RgbConfiguration::autoExposureTimeResponseCallback, this, std::placeholders::_1));
            }
            if (param_name == "rgb_camera_exposure_time")
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

                auto requestExposureTime = std::make_shared<l3cam_interfaces::srv::ChangeRgbCameraExposureTime::Request>();
                requestExposureTime->exposure_time = param.as_int();

                auto resultExposureTime = clientExposureTime->async_send_request(
                    requestExposureTime, std::bind(&RgbConfiguration::exposureTimeResponseCallback, this, std::placeholders::_1));
            }
        }

        return result;
    }

    void brightnessResponseCallback(
        rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraBrightness>::SharedFuture future)
    {
        auto status = future.wait_for(1s);
        if (status == std::future_status::ready)
        {
            int error = future.get()->error;
            if (!error)
                rgb_camera_brightness = this->get_parameter("rgb_camera_brightness").as_int();
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("rgb_camera_brightness", rgb_camera_brightness));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_rgb_camera_brightness");
            // this->set_parameter(rclcpp::Parameter("rgb_camera_brightness", rgb_camera_brightness));
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
                rgb_camera_contrast = this->get_parameter("rgb_camera_contrast").as_int();
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("rgb_camera_contrast", rgb_camera_contrast));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_rgb_camera_contrast");
            // this->set_parameter(rclcpp::Parameter("rgb_camera_contrast", rgb_camera_contrast));
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
                rgb_camera_saturation = this->get_parameter("rgb_camera_saturation").as_int();
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("rgb_camera_saturation", rgb_camera_saturation));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_rgb_camera_saturation");
            // this->set_parameter(rclcpp::Parameter("rgb_camera_saturation", rgb_camera_saturation));
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
                rgb_camera_sharpness = this->get_parameter("rgb_camera_sharpness").as_int();
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("rgb_camera_sharpness", rgb_camera_sharpness));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_rgb_camera_sharpness");
            // this->set_parameter(rclcpp::Parameter("rgb_camera_sharpness", rgb_camera_sharpness));
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
                rgb_camera_gamma = this->get_parameter("rgb_camera_gamma").as_int();
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("rgb_camera_gamma", rgb_camera_gamma));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_rgb_camera_gamma");
            // this->set_parameter(rclcpp::Parameter("rgb_camera_gamma", rgb_camera_gamma));
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
                rgb_camera_gain = this->get_parameter("rgb_camera_gain").as_int();
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("rgb_camera_gain", rgb_camera_gain));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_rgb_camera_gain");
            // this->set_parameter(rclcpp::Parameter("rgb_camera_gain", rgb_camera_gain));
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
                rgb_camera_auto_white_balance = this->get_parameter("rgb_camera_auto_white_balance").as_bool();
            }
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("rgb_camera_auto_white_balance", rgb_camera_auto_white_balance));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service enable_rgb_camera_auto_white_balance");
            // this->set_parameter(rclcpp::Parameter("rgb_camera_auto_white_balance", rgb_camera_auto_white_balance));
        }
    }

    void whiteBalanceResponseCallback(
        rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance>::SharedFuture future)
    {
        if (!rgb_camera_auto_white_balance)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                    rgb_camera_white_balance = this->get_parameter("rgb_camera_white_balance").as_int();
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                    // this->set_parameter(rclcpp::Parameter("rgb_camera_white_balance", rgb_camera_white_balance));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_rgb_camera_white_balance");
                // this->set_parameter(rclcpp::Parameter("rgb_camera_white_balance", rgb_camera_white_balance));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RGB camera auto white balance must be disabled to change white balance");
            // this->set_parameter(rclcpp::Parameter("rgb_camera_white_balance", rgb_camera_white_balance));
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
                rgb_camera_auto_exposure_time = this->get_parameter("rgb_camera_auto_exposure_time").as_bool();
            else
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                // this->set_parameter(rclcpp::Parameter("rgb_camera_auto_exposure_time", rgb_camera_auto_exposure_time));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service enable_rgb_camera_auto_exposure_time");
            // this->set_parameter(rclcpp::Parameter("rgb_camera_auto_exposure_time", rgb_camera_auto_exposure_time));
        }
    }

    void exposureTimeResponseCallback(
        rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraExposureTime>::SharedFuture future)
    {
        if (!rgb_camera_auto_exposure_time)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                    rgb_camera_exposure_time = this->get_parameter("rgb_camera_exposure_time").as_int();
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                    // this->set_parameter(rclcpp::Parameter("rgb_camera_exposure_time", rgb_camera_exposure_time));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_rgb_camera_exposure_time");
                // this->set_parameter(rclcpp::Parameter("rgb_camera_exposure_time", rgb_camera_exposure_time));
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RGB camera auto exposure time must be disabled to change exposure time");
            // this->set_parameter(rclcpp::Parameter("rgb_camera_exposure_time", rgb_camera_exposure_time));
        }
    }

    int rgb_camera_brightness;
    int rgb_camera_contrast;
    int rgb_camera_saturation;
    int rgb_camera_sharpness;
    int rgb_camera_gamma;
    int rgb_camera_gain;
    bool rgb_camera_auto_white_balance;
    int rgb_camera_white_balance;
    bool rgb_camera_auto_exposure_time;
    int rgb_camera_exposure_time;

    rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraBrightness>::SharedPtr clientBrightness;
    rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraContrast>::SharedPtr clientContrast;
    rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraSaturation>::SharedPtr clientSaturation;
    rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraSharpness>::SharedPtr clientSharpness;
    rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraGamma>::SharedPtr clientGamma;
    rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraGain>::SharedPtr clientGain;
    rclcpp::Client<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance>::SharedPtr clientAutoWhiteBalance;
    rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance>::SharedPtr clientWhiteBalance;
    rclcpp::Client<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime>::SharedPtr clientAutoExposureTime;
    rclcpp::Client<l3cam_interfaces::srv::ChangeRgbCameraExposureTime>::SharedPtr clientExposureTime;

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    int error = L3CAM_OK;

    std::shared_ptr<RgbConfiguration> node = std::make_shared<RgbConfiguration>();

    // Check if RGB is available
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
                if (resultGetSensors.get()->sensors[i].sensor_type == sensor_econ_rgb)
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RGB camera configuration is available");
    else
        return 0;

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
