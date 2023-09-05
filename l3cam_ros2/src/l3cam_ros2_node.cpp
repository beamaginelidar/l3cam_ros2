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

#include <libL3Cam.h>
#include <libL3Cam_allied.h>
#include <libL3Cam_econ.h>
#include <libL3Cam_polarimetric.h>
#include <libL3Cam_thermal.h>
#include <beamagine.h>
#include <beamErrors.h>

#include "l3cam_interfaces/msg/sensor.hpp"

#include "l3cam_interfaces/srv/get_version.hpp"
#include "l3cam_interfaces/srv/initialize.hpp"
#include "l3cam_interfaces/srv/terminate.hpp"
#include "l3cam_interfaces/srv/find_devices.hpp"
#include "l3cam_interfaces/srv/get_local_server_address.hpp"
#include "l3cam_interfaces/srv/get_device_status.hpp"
#include "l3cam_interfaces/srv/get_sensors_available.hpp"
#include "l3cam_interfaces/srv/get_network_configuration.hpp"
#include "l3cam_interfaces/srv/change_network_configuration.hpp"
#include "l3cam_interfaces/srv/power_off_device.hpp"
#include "l3cam_interfaces/srv/start_device.hpp"
#include "l3cam_interfaces/srv/stop_device.hpp"
#include "l3cam_interfaces/srv/start_stream.hpp"
#include "l3cam_interfaces/srv/stop_stream.hpp"

#include "l3cam_interfaces/srv/change_pointcloud_color.hpp"
#include "l3cam_interfaces/srv/change_pointcloud_color_range.hpp"
#include "l3cam_interfaces/srv/change_distance_range.hpp"
#include "l3cam_interfaces/srv/enable_auto_bias.hpp"
#include "l3cam_interfaces/srv/change_bias_value.hpp"

#include "l3cam_interfaces/srv/set_polarimetric_camera_default_settings.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_brightness.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_black_level.hpp"
#include "l3cam_interfaces/srv/enable_polarimetric_camera_auto_gain.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_auto_gain_range.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_gain.hpp"
#include "l3cam_interfaces/srv/enable_polarimetric_camera_auto_exposure_time.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_auto_exposure_time_range.hpp"
#include "l3cam_interfaces/srv/change_polarimetric_camera_exposure_time.hpp"

#include "l3cam_interfaces/srv/set_rgb_camera_default_settings.hpp"
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

#include "l3cam_interfaces/srv/change_thermal_camera_colormap.hpp"
#include "l3cam_interfaces/srv/change_thermal_camera_temperature_filter.hpp"
#include "l3cam_interfaces/srv/enable_thermal_camera_temperature_filter.hpp"

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

#include "l3cam_interfaces/srv/get_allied_camera_black_level.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_exposure_time.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_auto_exposure_time.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_auto_exposure_time_range.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_gain.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_auto_gain.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_auto_gain_range.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_gamma.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_saturation.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_sharpness.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_hue.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_intensity_auto_precedence.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_auto_white_balance.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_balance_ratio_selector.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_balance_ratio.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_balance_white_auto_rate.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_balance_white_auto_tolerance.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_auto_mode_region.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_intensity_controller_region.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_intensity_controller_target.hpp"
#include "l3cam_interfaces/srv/get_allied_camera_max_driver_buffers_count.hpp"

using namespace std::chrono_literals;

l3cam devices[1];
sensor av_sensors[6];

sensor *m_lidar_sensor = NULL;
sensor *m_rgb_sensor = NULL;
sensor *m_thermal_sensor = NULL;
sensor *m_polarimetric_sensor = NULL;
sensor *m_allied_wide_sensor = NULL;
sensor *m_allied_narrow_sensor = NULL;

//! SIGNALS
std::string getSignalDescription(int signal)
{
    std::string description = "";
    switch (signal)
    {
    case SIGQUIT:
        description = "SIGQUIT";
        break;
    case SIGINT:
        description = "SIGINT";
        break;
    case SIGTERM:
        description = "SIGTERM";
        break;
    case SIGSEGV:
        description = "SIGSEGV";
        break;
    case SIGHUP:
        description = "SIGHUP";
        break;
    case SIGILL:
        description = "SIGILL";
        break;
    case SIGABRT:
        description = "SIGABRT";
        break;
    case SIGFPE:
        description = "SIGFPE";
        break;
    }
    return description;
}

void handleSignalCaptured(int signal)
{
    // std::cout<<"handleSignalCaputred exiting for captured signal "<<signal<<"--"<<getSignalDescription(signal)<<std::endl;
    //! TODO(ebernal) gestionar aqui el cierre controlado
    if (signal == SIGINT)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Terminating...");
        STOP_STREAM(devices[0]);
        STOP_DEVICE(devices[0]);
        TERMINATE(devices[0]);
    }
    exit(0);
}

void setCapturedSignal(int signal)
{
    struct sigaction sig_action;
    sig_action.sa_flags = 0;
    sigemptyset(&sig_action.sa_mask);
    sig_action.sa_handler = handleSignalCaptured;

    if (sigaction(signal, &sig_action, NULL) == 1)
    {
        std::cout << "setCaputredSignal Error setting signal\n";
    }
}

void registerSystemSignals()
{
    setCapturedSignal(SIGQUIT);
    setCapturedSignal(SIGINT);
    setCapturedSignal(SIGTERM);
    setCapturedSignal(SIGSEGV);
    setCapturedSignal(SIGHUP);
    setCapturedSignal(SIGILL);
    setCapturedSignal(SIGABRT);
    setCapturedSignal(SIGFPE);
}

// Services
void getVersion(const std::shared_ptr<l3cam_interfaces::srv::GetVersion::Request> req,
                std::shared_ptr<l3cam_interfaces::srv::GetVersion::Response> res)
{
    res->version = GET_VERSION();
}

void initialize(const std::shared_ptr<l3cam_interfaces::srv::Initialize::Request> req,
                std::shared_ptr<l3cam_interfaces::srv::Initialize::Response> res)
{
    res->error = INITIALIZE(&req->local_address[0], &req->device_address[0]);
}

void terminate(const std::shared_ptr<l3cam_interfaces::srv::Terminate::Request> req,
               std::shared_ptr<l3cam_interfaces::srv::Terminate::Response> res)
{
    res->error = TERMINATE(devices[0]);
}

void findDevices(const std::shared_ptr<l3cam_interfaces::srv::FindDevices::Request> req,
                 std::shared_ptr<l3cam_interfaces::srv::FindDevices::Response> res)
{
    res->error = FIND_DEVICES(&devices[0], &res->num_devices);
}

void getLocalServerAddress(const std::shared_ptr<l3cam_interfaces::srv::GetLocalServerAddress::Request> req,
                           std::shared_ptr<l3cam_interfaces::srv::GetLocalServerAddress::Response> res)
{
    res->local_ip_address = GET_LOCAL_SERVER_ADDRESS(devices[0]);
}

void getDeviceStatus(const std::shared_ptr<l3cam_interfaces::srv::GetDeviceStatus::Request> req,
                     std::shared_ptr<l3cam_interfaces::srv::GetDeviceStatus::Response> res)
{
    res->error = GET_DEVICE_STATUS(devices[0], &res->system_status);
}

void getSensorsAvailable(const std::shared_ptr<l3cam_interfaces::srv::GetSensorsAvailable::Request> req,
                         std::shared_ptr<l3cam_interfaces::srv::GetSensorsAvailable::Response> res)
{
    res->error = GET_SENSORS_AVAILABLE(devices[0], av_sensors, &res->num_sensors);
    res->sensors.resize(res->num_sensors);
    for (int i = 0; i < res->num_sensors; ++i)
    {
        res->sensors[i].protocol = av_sensors[i].protocol;
        res->sensors[i].sensor_type = av_sensors[i].sensor_type;
        res->sensors[i].sensor_status = av_sensors[i].sensor_status;
        res->sensors[i].image_type = av_sensors[i].image_type;
        res->sensors[i].perception_enabled = av_sensors[i].perception_enabled;
    }
}

void getNetworkConfiguration(const std::shared_ptr<l3cam_interfaces::srv::GetNetworkConfiguration::Request> req,
                             std::shared_ptr<l3cam_interfaces::srv::GetNetworkConfiguration::Response> res)
{
    char *ip_address = NULL;
    char *netmask = NULL;
    char *gateway = NULL;
    res->error = GET_NETWORK_CONFIGURATION(devices[0], &ip_address, &netmask, &gateway);
    res->ip_address = ip_address;
    res->netmask = netmask;
    res->gateway = gateway;
}

void changeNetworkConfiguration(const std::shared_ptr<l3cam_interfaces::srv::ChangeNetworkConfiguration::Request> req,
                                std::shared_ptr<l3cam_interfaces::srv::ChangeNetworkConfiguration::Response> res)
{
    /*if (req->enable_dhcp)
        res->error = CHANGE_NETWORK_CONFIGURATION(devices[0], NULL, NULL, NULL, true);
    else
    {
        std::string ip_address = req->ip_address;
        std::string netmask = req->netmask;
        std::string gateway = req->gateway;
        res->error = CHANGE_NETWORK_CONFIGURATION(devices[0], (char *)ip_address.data(), (char *)netmask.data(), (char *)gateway.data(), false);
    }

    // TODO: Terminate, Initialize, etc.
    */
}

void powerOffDevice(const std::shared_ptr<l3cam_interfaces::srv::PowerOffDevice::Request> req,
                    std::shared_ptr<l3cam_interfaces::srv::PowerOffDevice::Response> res)
{
    POWER_OFF_DEVICE(devices[0]);
    res->error = 0;
}

void startDevice(const std::shared_ptr<l3cam_interfaces::srv::StartDevice::Request> req,
                 std::shared_ptr<l3cam_interfaces::srv::StartDevice::Response> res)
{
    res->error = START_DEVICE(devices[0]);
}

void stopDevice(const std::shared_ptr<l3cam_interfaces::srv::StopDevice::Request> req,
                std::shared_ptr<l3cam_interfaces::srv::StopDevice::Response> res)
{
    res->error = STOP_DEVICE(devices[0]);
}

void startStream(const std::shared_ptr<l3cam_interfaces::srv::StartStream::Request> req,
                 std::shared_ptr<l3cam_interfaces::srv::StartStream::Response> res)
{
    res->error = START_STREAM(devices[0]);
}

void stopStream(const std::shared_ptr<l3cam_interfaces::srv::StopStream::Request> req,
                std::shared_ptr<l3cam_interfaces::srv::StopStream::Response> res)
{
    res->error = STOP_STREAM(devices[0]);
}

// Point Cloud
void changePointcloudColor(const std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColor::Request> req,
                           std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColor::Response> res)
{
    res->error = CHANGE_POINT_CLOUD_COLOR(devices[0], req->visualization_color);
}

void changePointcloudColorRange(const std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColorRange::Request> req,
                                std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColorRange::Response> res)
{
    res->error = CHANGE_POINT_CLOUD_COLOR_RANGE(devices[0], req->min_value, req->max_value);
}

void changeDistanceRange(const std::shared_ptr<l3cam_interfaces::srv::ChangeDistanceRange::Request> req,
                         std::shared_ptr<l3cam_interfaces::srv::ChangeDistanceRange::Response> res)
{
    res->error = CHANGE_DISTANCE_RANGE(devices[0], req->min_value, req->max_value);
}

void enableAutoBias(const std::shared_ptr<l3cam_interfaces::srv::EnableAutoBias::Request> req,
                    std::shared_ptr<l3cam_interfaces::srv::EnableAutoBias::Response> res)
{
    ENABLE_AUTO_BIAS(devices[0], req->enabled);
}

void changeBiasValue(const std::shared_ptr<l3cam_interfaces::srv::ChangeBiasValue::Request> req,
                     std::shared_ptr<l3cam_interfaces::srv::ChangeBiasValue::Response> res)
{
    CHANGE_BIAS_VALUE(devices[0], req->index, req->bias);
}

// Polarimetric
void setPolarimetricCameraDefaultSettings(const std::shared_ptr<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings::Request> req,
                                          std::shared_ptr<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings::Response> res)
{
    res->error = SET_POLARIMETRIC_CAMERA_DEFAULT_SETTINGS(devices[0]);
}

void changePolarimetricCameraBrightness(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness::Response> res)
{
    res->error = CHANGE_POLARIMETRIC_CAMERA_BRIGHTNESS(devices[0], req->brightness);
}

void changePolarimetricCameraBlackLevel(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel::Response> res)
{
    res->error = CHANGE_POLARIMETRIC_CAMERA_BLACK_LEVEL(devices[0], req->black_level);
}

void enablePolarimetricCameraAutoGain(const std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain::Request> req,
                                      std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain::Response> res)
{
    res->error = ENABLE_POLARIMETRIC_CAMERA_AUTO_GAIN(devices[0], req->enabled);
}

void changePolarimetricCameraAutoGainRange(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange::Request> req,
                                           std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange::Response> res)
{
    res->error = CHANGE_POLARIMETRIC_CAMERA_AUTO_GAIN_RANGE(devices[0], req->min_gain, req->max_gain);
}

void changePolarimetricCameraGain(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraGain::Request> req,
                                  std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraGain::Response> res)
{
    res->error = CHANGE_POLARIMETRIC_CAMERA_GAIN(devices[0], req->gain);
}

void enablePolarimetricCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime::Request> req,
                                              std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime::Response> res)
{
    res->error = ENABLE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME(devices[0], req->enabled);
}

void changePolarimetricCameraAutoExposureTimeRange(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange::Request> req,
                                                   std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange::Response> res)
{
    res->error = CHANGE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], req->min_exposure, req->max_exposure);
}

void changePolarimetricCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime::Request> req,
                                          std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime::Response> res)
{
    res->error = CHANGE_POLARIMETRIC_CAMERA_EXPOSURE_TIME(devices[0], req->exposure_time);
}

// RGB
void setRgbCameraDefaultSettings(const std::shared_ptr<l3cam_interfaces::srv::SetRgbCameraDefaultSettings::Request> req,
                                 std::shared_ptr<l3cam_interfaces::srv::SetRgbCameraDefaultSettings::Response> res)
{
    res->error = SET_RGB_CAMERA_DEFAULT_SETTINGS(devices[0]);
}

void changeRgbCameraBrightness(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraBrightness::Request> req,
                               std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraBrightness::Response> res)
{
    res->error = CHANGE_RGB_CAMERA_BRIGHTNESS(devices[0], req->brightness);
}

void changeRgbCameraContrast(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraContrast::Request> req,
                             std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraContrast::Response> res)
{
    res->error = CHANGE_RGB_CAMERA_CONTRAST(devices[0], req->contrast);
}

void changeRgbCameraSaturation(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSaturation::Request> req,
                               std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSaturation::Response> res)
{
    res->error = CHANGE_RGB_CAMERA_SATURATION(devices[0], req->saturation);
}

void changeRgbCameraSharpness(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSharpness::Request> req,
                              std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSharpness::Response> res)
{
    res->error = CHANGE_RGB_CAMERA_SHARPNESS(devices[0], req->sharpness);
}

void changeRgbCameraGamma(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGamma::Request> req,
                          std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGamma::Response> res)
{
    res->error = CHANGE_RGB_CAMERA_GAMMA(devices[0], req->gamma);
}

void changeRgbCameraGain(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGain::Request> req,
                         std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGain::Response> res)
{
    res->error = CHANGE_RGB_CAMERA_GAIN(devices[0], req->gain);
}

void enableRgbCameraAutoWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance::Request> req,
                                     std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance::Response> res)
{
    res->error = ENABLE_RGB_CAMERA_AUTO_WHITE_BALANCE(devices[0], req->enabled);
}

void changeRgbCameraWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance::Request> req,
                                 std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance::Response> res)
{
    res->error = CHANGE_RGB_CAMERA_WHITE_BALANCE(devices[0], req->white_balance);
}

void enableRgbCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime::Request> req,
                                     std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime::Response> res)
{
    res->error = ENABLE_RGB_CAMERA_AUTO_EXPOSURE_TIME(devices[0], req->enabled);
}

void changeRgbCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraExposureTime::Request> req,
                                 std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraExposureTime::Response> res)
{
    res->error = CHANGE_RGB_CAMERA_EXPOSURE_TIME(devices[0], req->exposure_time);
}

// Thermal
void changeThermalCameraColormap(const std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraColormap::Request> req,
                                 std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraColormap::Response> res)
{
    res->error = CHANGE_THERMAL_CAMERA_COLORMAP(devices[0], (thermalTypes)req->colormap);
}

void changeThermalCameraTemperatureFilter(const std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter::Request> req,
                                          std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter::Response> res)
{
    res->error = CHANGE_THERMAL_CAMERA_TEMPERATURE_FILTER(devices[0], req->min_temperature, req->max_temperature);
}

void enableThermalCameraTemperatureFilter(const std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter::Request> req,
                                          std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter::Response> res)
{
    res->error = ENABLE_THERMAL_CAMERA_TEMPERATURE_FILTER(devices[0], req->enabled);
}

// Allied
void changeAlliedCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime::Request> req,
                                    std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(devices[0], *m_allied_wide_sensor, req->exposure_time);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(devices[0], *m_allied_narrow_sensor, req->exposure_time);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void enableAlliedCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(devices[0], *m_allied_wide_sensor, req->enabled);
        break;
    case 2:
        res->error = ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(devices[0], *m_allied_narrow_sensor, req->enabled);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void changeAlliedCameraAutoExposureTimeRange(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange::Request> req,
                                             std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], *m_allied_wide_sensor, req->auto_exposure_time_range_min, req->auto_exposure_time_range_max);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], *m_allied_narrow_sensor, req->auto_exposure_time_range_min, req->auto_exposure_time_range_max);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void changeAlliedCameraGain(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraGain::Request> req,
                            std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraGain::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_GAIN(devices[0], *m_allied_wide_sensor, req->gain);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_GAIN(devices[0], *m_allied_narrow_sensor, req->gain);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void enableAlliedCameraAutoGain(const std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoGain::Request> req,
                                std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoGain::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = ENABLE_ALLIED_CAMERA_AUTO_GAIN(devices[0], *m_allied_wide_sensor, req->enabled);
        break;
    case 2:
        res->error = ENABLE_ALLIED_CAMERA_AUTO_GAIN(devices[0], *m_allied_narrow_sensor, req->enabled);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void changeAlliedCameraAutoGainRange(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange::Request> req,
                                     std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(devices[0], *m_allied_wide_sensor, (float)req->auto_gain_range_min, (float)req->auto_gain_range_max);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(devices[0], *m_allied_narrow_sensor, (float)req->auto_gain_range_min, (float)req->auto_gain_range_max);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void changeAlliedCameraGamma(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraGamma::Request> req,
                             std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraGamma::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_GAMMA(devices[0], *m_allied_wide_sensor, req->gamma);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_GAMMA(devices[0], *m_allied_narrow_sensor, req->gamma);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void changeAlliedCameraSaturation(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraSaturation::Request> req,
                                  std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraSaturation::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_SATURATION(devices[0], *m_allied_wide_sensor, req->saturation);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_SATURATION(devices[0], *m_allied_narrow_sensor, req->saturation);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void changeAlliedCameraHue(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraHue::Request> req,
                           std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraHue::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_HUE(devices[0], *m_allied_wide_sensor, req->hue);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_HUE(devices[0], *m_allied_narrow_sensor, req->hue);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void changeAlliedCameraIntensityAutoPrecedence(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence::Request> req,
                                               std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_wide_sensor, req->intensity_auto_precedence);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_narrow_sensor, req->intensity_auto_precedence);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void enableAlliedCameraAutoWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(devices[0], *m_allied_wide_sensor, req->enabled);
        break;
    case 2:
        res->error = ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(devices[0], *m_allied_narrow_sensor, req->enabled);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void changeAlliedCameraBalanceRatioSelector(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector::Request> req,
                                            std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_wide_sensor, req->white_balance_ratio_selector);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_narrow_sensor, req->white_balance_ratio_selector);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void changeAlliedCameraBalanceRatio(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio::Request> req,
                                    std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO(devices[0], *m_allied_wide_sensor, req->balance_ratio);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO(devices[0], *m_allied_narrow_sensor, req->balance_ratio);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void changeAlliedCameraBalanceWhiteAutoRate(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate::Request> req,
                                            std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(devices[0], *m_allied_wide_sensor, req->white_balance_auto_rate);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(devices[0], *m_allied_narrow_sensor, req->white_balance_auto_rate);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void changeAlliedCameraBalanceWhiteAutoTolerance(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance::Request> req,
                                                 std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(devices[0], *m_allied_wide_sensor, req->white_balance_auto_tolerance);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(devices[0], *m_allied_narrow_sensor, req->white_balance_auto_tolerance);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void changeAlliedCameraIntensityControllerRegion(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion::Request> req,
                                                 std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_wide_sensor, req->intensity_controller_region);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_narrow_sensor, req->intensity_controller_region);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void changeAlliedCameraIntensityControllerTarget(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget::Request> req,
                                                 std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(devices[0], *m_allied_wide_sensor, req->intensity_controller_target);
        break;
    case 2:
        res->error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(devices[0], *m_allied_narrow_sensor, req->intensity_controller_target);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraBlackLevel(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBlackLevel::Request> req,
                               std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBlackLevel::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_BLACK_LEVEL(devices[0], *m_allied_wide_sensor, &res->black_level);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_BLACK_LEVEL(devices[0], *m_allied_narrow_sensor, &res->black_level);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraExposureTime::Request> req,
                                 std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraExposureTime::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_EXPOSURE_TIME_US(devices[0], *m_allied_wide_sensor, &res->exposure_time);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_EXPOSURE_TIME_US(devices[0], *m_allied_narrow_sensor, &res->exposure_time);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTime::Request> req,
                                     std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTime::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(devices[0], *m_allied_wide_sensor, &res->enabled);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(devices[0], *m_allied_narrow_sensor, &res->enabled);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraAutoExposureTimeRange(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTimeRange::Request> req,
                                          std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTimeRange::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], *m_allied_wide_sensor, &res->auto_exposure_time_range_min, &res->auto_exposure_time_range_max);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], *m_allied_narrow_sensor, &res->auto_exposure_time_range_min, &res->auto_exposure_time_range_max);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraGain(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraGain::Request> req,
                         std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraGain::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_GAIN(devices[0], *m_allied_wide_sensor, &res->gain);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_GAIN(devices[0], *m_allied_narrow_sensor, &res->gain);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraAutoGain(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoGain::Request> req,
                             std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoGain::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_AUTO_GAIN(devices[0], *m_allied_wide_sensor, &res->enabled);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_AUTO_GAIN(devices[0], *m_allied_narrow_sensor, &res->enabled);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraAutoGainRange(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoGainRange::Request> req,
                                  std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoGainRange::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_AUTO_GAIN_RANGE(devices[0], *m_allied_wide_sensor, &res->auto_gain_range_min, &res->auto_gain_range_max);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_AUTO_GAIN_RANGE(devices[0], *m_allied_narrow_sensor, &res->auto_gain_range_min, &res->auto_gain_range_max);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraGamma(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraGamma::Request> req,
                          std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraGamma::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_GAMMA(devices[0], *m_allied_wide_sensor, &res->gamma);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_GAMMA(devices[0], *m_allied_narrow_sensor, &res->gamma);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraSaturation(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraSaturation::Request> req,
                               std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraSaturation::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_SATURATION(devices[0], *m_allied_wide_sensor, &res->saturation);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_SATURATION(devices[0], *m_allied_narrow_sensor, &res->saturation);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraSharpness(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraSharpness::Request> req,
                              std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraSharpness::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_SHARPNESS(devices[0], *m_allied_wide_sensor, &res->sharpness);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_SHARPNESS(devices[0], *m_allied_narrow_sensor, &res->sharpness);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraHue(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraHue::Request> req,
                        std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraHue::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_HUE(devices[0], *m_allied_wide_sensor, &res->hue);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_HUE(devices[0], *m_allied_narrow_sensor, &res->hue);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraIntensityAutoPrecedence(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityAutoPrecedence::Request> req,
                                            std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityAutoPrecedence::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_wide_sensor, &res->intensity_auto_precedence);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_narrow_sensor, &res->intensity_auto_precedence);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraAutoWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoWhiteBalance::Request> req,
                                     std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoWhiteBalance::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_AUTO_WHITE_BALANCE(devices[0], *m_allied_wide_sensor, &res->enabled);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_AUTO_WHITE_BALANCE(devices[0], *m_allied_narrow_sensor, &res->enabled);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraBalanceRatioSelector(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceRatioSelector::Request> req,
                                         std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceRatioSelector::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_wide_sensor, &res->white_balance_ratio_selector);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_narrow_sensor, &res->white_balance_ratio_selector);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraBalanceRatio(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceRatio::Request> req,
                                 std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceRatio::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_BALANCE_RATIO(devices[0], *m_allied_wide_sensor, &res->balance_ratio);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_BALANCE_RATIO(devices[0], *m_allied_narrow_sensor, &res->balance_ratio);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraBalanceWhiteAutoRate(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoRate::Request> req,
                                         std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoRate::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(devices[0], *m_allied_wide_sensor, &res->white_balance_auto_rate);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(devices[0], *m_allied_narrow_sensor, &res->white_balance_auto_rate);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraBalanceWhiteAutoTolerance(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoTolerance::Request> req,
                                              std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoTolerance::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(devices[0], *m_allied_wide_sensor, &res->white_balance_auto_tolerance);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(devices[0], *m_allied_narrow_sensor, &res->white_balance_auto_tolerance);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraAutoModeRegion(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoModeRegion::Request> req,
                                   std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoModeRegion::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_AUTO_MODE_REGION(devices[0], *m_allied_wide_sensor, &res->height, &res->width);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_AUTO_MODE_REGION(devices[0], *m_allied_narrow_sensor, &res->height, &res->width);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraIntensityControllerRegion(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerRegion::Request> req,
                                              std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerRegion::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_wide_sensor, &res->intensity_controller_region);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_narrow_sensor, &res->intensity_controller_region);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraIntensityControllerTarget(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerTarget::Request> req,
                                              std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerTarget::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(devices[0], *m_allied_wide_sensor, &res->intensity_controller_target);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(devices[0], *m_allied_narrow_sensor, &res->intensity_controller_target);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

void getAlliedCameraMaxDriverBuffersCount(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraMaxDriverBuffersCount::Request> req,
                                          std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraMaxDriverBuffersCount::Response> res)
{
    switch (req->allied_type)
    {
    case 1:
        res->error = GET_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(devices[0], *m_allied_wide_sensor, &res->max_driver_buffers_count);
        break;
    case 2:
        res->error = GET_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(devices[0], *m_allied_narrow_sensor, &res->max_driver_buffers_count);
        break;
    default:
        res->error = L3CAM_VALUE_OUT_OF_RANGE;
    }
}

class L3Cam : public rclcpp::Node
{
public:
    L3Cam() : Node("l3cam_ros2_node")
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        rcl_interfaces::msg::IntegerRange intRange;
        rcl_interfaces::msg::FloatingPointRange floatRange;
        // Network
        this->declare_parameter("ip_address", "192.168.1.250");
        this->declare_parameter("netmask", "255.255.255.0");
        this->declare_parameter("gateway", "0.0.0.0");
        this->declare_parameter("dhcp", false);
        this->declare_parameter("local_address", "");
        this->declare_parameter("device_address", "");
        // Point Cloud
        intRange.set__from_value(0).set__to_value(13).set__step(1); // TODO: enumerate pointCloudColor
        descriptor.integer_range = {intRange};
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
        intRange.set__from_value(0).set__to_value(400000).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("pointcloud_color_range_minimum", 0, descriptor); // 0 - 400000
        intRange.set__from_value(0).set__to_value(400000).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("pointcloud_color_range_maximum", 400000, descriptor); // 0 - 400000
        intRange.set__from_value(0).set__to_value(400000).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("distance_range_minimum", 0, descriptor); // 0 - 400000
        intRange.set__from_value(0).set__to_value(400000).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("distance_range_maximum", 400000, descriptor); // 0 - 400000
        this->declare_parameter("auto_bias", true);
        intRange.set__from_value(700).set__to_value(3500).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("bias_value_right", 1580, descriptor); // 700 - 3500
        this->declare_parameter("bias_value_left", 1380, descriptor);  // 700 - 3500
        // Polarimetric
        intRange.set__from_value(0).set__to_value(255).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("polarimetric_camera_brightness", 127, descriptor); // 0 - 255
        floatRange.set__from_value(0.0).set__to_value(12.5).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_black_level", 6.0, descriptor); // 0 - 12.5
        this->declare_parameter("polarimetric_camera_auto_gain", true);
        floatRange.set__from_value(0.0).set__to_value(48.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_auto_gain_range_minimum", 0.0, descriptor); // 0 - 48
        floatRange.set__from_value(0.0).set__to_value(48.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_auto_gain_range_maximum", 48.0, descriptor); // 0 - 48
        floatRange.set__from_value(0.0).set__to_value(48.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_gain", 24.0, descriptor); // 0 - 48
        this->declare_parameter("polarimetric_camera_auto_exposure_time", true);
        floatRange.set__from_value(33.5).set__to_value(66470.6).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_auto_exposure_time_range_minimum", 33.5, descriptor); // 33.5 - 66470.6
        floatRange.set__from_value(33.5).set__to_value(66470.6).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_auto_exposure_time_range_maximum", 66470.6, descriptor); // 33.5 - 66470.6
        floatRange.set__from_value(33.5).set__to_value(66470.6).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("polarimetric_camera_exposure_time", 33.5, descriptor); // 33.5 - 66470.6
        // RGB
        intRange.set__from_value(-15).set__to_value(15).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_brightness", 0, descriptor); // -15 - 15
        this->declare_parameter("rgb_camera_contrast", 10, descriptor);  // 0 - 30
        intRange.set__from_value(0).set__to_value(60).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_saturation", 16, descriptor); // 0 - 60
        intRange.set__from_value(0).set__to_value(127).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_sharpness", 16, descriptor); // 0 - 127
        intRange.set__from_value(40).set__to_value(500).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_gamma", 220, descriptor); // 40 - 500
        intRange.set__from_value(0).set__to_value(63).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_gain", 0, descriptor); // 0 - 63
        this->declare_parameter("rgb_camera_auto_white_balance", true);
        intRange.set__from_value(1000).set__to_value(10000).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_white_balance", 5000, descriptor); // 1000 - 10000
        this->declare_parameter("rgb_camera_auto_exposure_time", true);
        intRange.set__from_value(1).set__to_value(10000).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_exposure_time", 156, descriptor); // 1 - 10000
        intRange.set__from_value(1).set__to_value(3).set__step(1);
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be: (econResolutions)\n"
            "\treso_640_480 = 1\n"
            "\treso_1280_720 = 2\n"
            "\treso_1920_1080 = 3\n";
        this->declare_parameter("rgb_camera_resolution", 3, descriptor); // see econResolutions
        descriptor.description = "";
        intRange.set__from_value(1).set__to_value(16).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_framerate", 10, descriptor); // 1 - 16
        // Thermal
        intRange.set__from_value(1).set__to_value(108).set__step(1); // TODO: enumerate thermalTypes
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be: (thermalTypes)\n"
            "\tthermal_WHITE = 1\n"
            "\tthermal_BLACK = 17\n"
            "\tthermal_IRON = 20\n"
            "\tthermal_COOL = 2\n"
            "\tthermal_AMBER = 9\n"
            "\tthermal_INDIGO = 10\n"
            "\tthermal_TYRIAN = 16\n"
            "\tthermal_GLORY = 8\n"
            "\tthermal_ENVY = 16\n"
            "\tthermal_WHITE_NEW = 100\n"
            "\tthermal_BLACK_NEW = 101\n"
            "\tthermal_SPECTRA = 102\n"
            "\tthermal_PRISM = 103\n"
            "\tthermal_TYRIAN_NEW = 104\n"
            "\tthermal_AMBER_NEW = 105\n"
            "\tthermal_IRON_NEW = 106\n"
            "\tthermal_HI = 107\n"
            "\tthermal_HILO = 108";
        this->declare_parameter("thermal_camera_colormap", 1, descriptor); // see thermalTypes
        descriptor.description = "";
        this->declare_parameter("thermal_camera_temperature_filter", false);
        intRange.set__from_value(-40).set__to_value(200).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("thermal_camera_temperature_filter_min", 0, descriptor); // -40 - 200
        intRange.set__from_value(-40).set__to_value(200).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("thermal_camera_temperature_filter_max", 50, descriptor); // -40 - 200
        // Allied Wide
        this->declare_parameter("allied_wide_camera_black_level", 0.0); // 0 - 4095
        floatRange.set__from_value(63.0).set__to_value(10000000.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_exposure_time", 4992.4, descriptor); // 63 - 10000000
        this->declare_parameter("allied_wide_camera_auto_exposure_time", false);
        floatRange.set__from_value(63.1).set__to_value(8999990.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_auto_exposure_time_range_min", 87.6, descriptor); // 63.1 - 8999990
        floatRange.set__from_value(87.6).set__to_value(10000000.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_auto_exposure_time_range_max", 8999990.0, descriptor); // 87.6 - 10000000
        floatRange.set__from_value(0.0).set__to_value(48.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_gain", 0.0, descriptor); // 0 - 48
        this->declare_parameter("allied_wide_camera_auto_gain", false);
        floatRange.set__from_value(0.0).set__to_value(48.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_auto_gain_range_min", 0.0, descriptor); // 0 - 48
        floatRange.set__from_value(0.0).set__to_value(48.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_auto_gain_range_max", 48.0, descriptor); // 0 - 48
        floatRange.set__from_value(0.4).set__to_value(2.4).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_gamma", 1.0, descriptor); // 0.4 - 2.4
        floatRange.set__from_value(0.0).set__to_value(2.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_saturation", 1.0, descriptor); // 0 - 2
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_sharpness", 0.0, descriptor); // -12 - 12
        floatRange.set__from_value(-40).set__to_value(40.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_hue", 0.0, descriptor); // -40 - 40
        intRange.set__from_value(0).set__to_value(1).set__step(1);          // TODO: enumerate
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be:\n"
            "\tMinimizeNoise = 0\n"
            "\tMinimizeBlur = 1";
        this->declare_parameter("allied_wide_camera_intensity_auto_precedence", 0, descriptor); // 0(MinimizeNoise) or 1(MinimizeBlur)
        descriptor.description = "";
        this->declare_parameter("allied_wide_camera_auto_white_balance", false);
        intRange.set__from_value(0).set__to_value(1).set__step(1); // TODO: enumerate
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be:\n"
            "\tRed = 0\n"
            "\tBlue = 1";
        this->declare_parameter("allied_wide_camera_balance_ratio_selector", 0, descriptor); // 0(Red), 1(Blue)
        descriptor.description = "";
        floatRange.set__from_value(0.0).set__to_value(8.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_balance_ratio", 2.4, descriptor); // 0 - 8
        floatRange.set__from_value(0.0).set__to_value(100.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_balance_white_auto_rate", 100.0, descriptor); // 0 - 100
        floatRange.set__from_value(0.0).set__to_value(50.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_balance_white_auto_tolerance", 5.0, descriptor); // 0 - 50
        intRange.set__from_value(00).set__to_value(1028).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("allied_wide_camera_auto_mode_region_height", 1280, descriptor); // 0 - 1028
        intRange.set__from_value(0).set__to_value(1232).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("allied_wide_camera_auto_mode_region_width", 1232, descriptor); // 0 - 1232
        intRange.set__from_value(0).set__to_value(4).set__step(1);                              // TODO: enumerate
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be:\n"
            "\tAutoMode = 0\n"
            "\tFullImage = 4";
        this->declare_parameter("allied_wide_camera_intensity_controller_region", 0, descriptor); // 0(AutoMode), 4(FullImage)
        descriptor.description = "";
        floatRange.set__from_value(10).set__to_value(90).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_intensity_controller_target", 50.0, descriptor); // 10 - 90
        intRange.set__from_value(1).set__to_value(4096).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("allied_wide_camera_max_driver_buffers_count", 64, descriptor); // 1 - 4096
        // Allied Narrow
        this->declare_parameter("allied_narrow_camera_black_level", 0.0); // 0 - 4095
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
        floatRange.set__from_value(-12.0).set__to_value(12.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_sharpness", 0.0, descriptor); // -12 - 12
        floatRange.set__from_value(-40).set__to_value(40.0).set__step(0.1);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_hue", 0.0, descriptor); // -40 - 40
        intRange.set__from_value(0).set__to_value(1).set__step(1);            // TODO: enumerate
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be:\n"
            "\tMinimizeNoise = 0\n"
            "\tMinimizeBlur = 1";
        this->declare_parameter("allied_narrow_camera_intensity_auto_precedence", 0, descriptor); // 0(MinimizeNoise) or 1(MinimizeBlur)
        descriptor.description = "";
        this->declare_parameter("allied_narrow_camera_auto_white_balance", false);
        intRange.set__from_value(0).set__to_value(1).set__step(1); // TODO: enumerate
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
        intRange.set__from_value(0).set__to_value(2056).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("allied_narrow_camera_auto_mode_region_height", 2056, descriptor); // 0 - 2056
        intRange.set__from_value(0).set__to_value(2464).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("allied_narrow_camera_auto_mode_region_width", 2464, descriptor); // 0 - 2464
        intRange.set__from_value(0).set__to_value(4).set__step(1);                                // TODO: enumerate
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
        intRange.set__from_value(0).set__to_value(4096).set__step(1);
        descriptor.integer_range = {intRange};
        this->declare_parameter("allied_narrow_camera_max_driver_buffers_count", 64, descriptor); // 1 - 4096
    }

    void loadDefaultParams()
    {
        /*std::string ip_address = this->get_parameter("ip_address").as_string().data();
        std::string netmask = this->get_parameter("netmask").as_string().data();
        std::string gateway = this->get_parameter("gateway").as_string().data();
        printDefaultError(CHANGE_NETWORK_CONFIGURATION(devices[0],
                &ip_address[0],
                &netmask[0],
                &gateway[0],
                this->get_parameter("dhcp").as_bool()));*/
        if (m_lidar_sensor != NULL)
        {
            printDefaultError(CHANGE_POINT_CLOUD_COLOR(devices[0],
                                                       this->get_parameter("pointcloud_color").as_int()));
            printDefaultError(CHANGE_POINT_CLOUD_COLOR_RANGE(devices[0],
                                                             this->get_parameter("pointcloud_color_range_minimum").as_int(),
                                                             this->get_parameter("pointcloud_color_range_maximum").as_int()));
            printDefaultError(CHANGE_DISTANCE_RANGE(devices[0],
                                                    this->get_parameter("distance_range_minimum").as_int(),
                                                    this->get_parameter("distance_range_maximum").as_int()));
            ENABLE_AUTO_BIAS(devices[0], this->get_parameter("auto_bias").as_bool());
            if (!this->get_parameter("auto_bias").as_bool())
            { //! Values might not match after disabling auto_bias
                CHANGE_BIAS_VALUE(devices[0], 1, this->get_parameter("bias_value_right").as_int());
                CHANGE_BIAS_VALUE(devices[0], 2, this->get_parameter("bias_value_left").as_int());
            }
        }
        if (m_polarimetric_sensor != NULL)
        {
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_BRIGHTNESS(devices[0],
                                                                    this->get_parameter("polarimetric_camera_brightness").as_int()));
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_BLACK_LEVEL(devices[0],
                                                                     this->get_parameter("polarimetric_camera_black_level").as_double()));
            printDefaultError(ENABLE_POLARIMETRIC_CAMERA_AUTO_GAIN(devices[0],
                                                                   this->get_parameter("polarimetric_camera_auto_gain").as_bool()));
            if (this->get_parameter("polarimetric_camera_auto_gain").as_bool())
            { //! Values might not match after enabling polarimetric_camera_auto_gain
                printDefaultError(CHANGE_POLARIMETRIC_CAMERA_AUTO_GAIN_RANGE(devices[0],
                                                                             this->get_parameter("polarimetric_camera_auto_gain_range_minimum").as_double(),
                                                                             this->get_parameter("polarimetric_camera_auto_gain_range_maximum").as_double()));
            }
            else
            { //! Values might not match after disabling polarimetric_camera_auto_gain
                printDefaultError(CHANGE_POLARIMETRIC_CAMERA_GAIN(devices[0],
                                                                  this->get_parameter("polarimetric_camera_gain").as_double()));
            }
            printDefaultError(ENABLE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME(devices[0],
                                                                            this->get_parameter("polarimetric_camera_auto_exposure_time").as_bool()));
            if (this->get_parameter("polarimetric_camera_auto_exposure_time").as_bool())
            { //! Values might not match after enabling polarimetric_camera_auto_exposure_time
                printDefaultError(CHANGE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0],
                                                                                      this->get_parameter("polarimetric_camera_auto_exposure_time_range_minimum").as_double(),
                                                                                      this->get_parameter("polarimetric_camera_auto_exposure_time_range_maximum").as_double()));
            }
            else
            { //! Values might not match after disabling polarimetric_camera_auto_exposure_time
                printDefaultError(CHANGE_POLARIMETRIC_CAMERA_EXPOSURE_TIME(devices[0],
                                                                           this->get_parameter("polarimetric_camera_exposure_time").as_double()));
            }
        }
        if (m_rgb_sensor != NULL)
        {
            printDefaultError(CHANGE_RGB_CAMERA_BRIGHTNESS(devices[0],
                                                           this->get_parameter("rgb_camera_brightness").as_int()));
            printDefaultError(CHANGE_RGB_CAMERA_CONTRAST(devices[0],
                                                         this->get_parameter("rgb_camera_contrast").as_int()));
            printDefaultError(CHANGE_RGB_CAMERA_SATURATION(devices[0],
                                                           this->get_parameter("rgb_camera_saturation").as_int()));
            printDefaultError(CHANGE_RGB_CAMERA_SHARPNESS(devices[0],
                                                          this->get_parameter("rgb_camera_sharpness").as_int()));
            printDefaultError(CHANGE_RGB_CAMERA_GAMMA(devices[0],
                                                      this->get_parameter("rgb_camera_gamma").as_int()));
            printDefaultError(CHANGE_RGB_CAMERA_GAIN(devices[0],
                                                     this->get_parameter("rgb_camera_gain").as_int()));
            printDefaultError(ENABLE_RGB_CAMERA_AUTO_WHITE_BALANCE(devices[0],
                                                                   this->get_parameter("rgb_camera_auto_white_balance").as_bool()));
            if (!this->get_parameter("rgb_camera_auto_white_balance").as_bool())
            { //! Values might not match after disabling rgb_camera_auto_white_balance
                printDefaultError(CHANGE_RGB_CAMERA_WHITE_BALANCE(devices[0],
                                                                  this->get_parameter("rgb_camera_white_balance").as_int()));
            }
            printDefaultError(ENABLE_RGB_CAMERA_AUTO_EXPOSURE_TIME(devices[0],
                                                                   this->get_parameter("rgb_camera_auto_exposure_time").as_bool()));
            if (!this->get_parameter("rgb_camera_auto_exposure_time").as_bool())
            { //! Values might not match after disabling rgb_camera_auto_exposure_time
                printDefaultError(CHANGE_RGB_CAMERA_EXPOSURE_TIME(devices[0],
                                                                  this->get_parameter("rgb_camera_exposure_time").as_int()));
            }
            printDefaultError(CHANGE_RGB_CAMERA_RESOLUTION(devices[0],
                                                           (econResolutions)this->get_parameter("rgb_camera_resolution").as_int()));
            printDefaultError(CHANGE_RGB_CAMERA_FRAMERATE(devices[0],
                                                          this->get_parameter("rgb_camera_framerate").as_int()));
        }
        if (m_thermal_sensor != NULL)
        {
            printDefaultError(CHANGE_THERMAL_CAMERA_COLORMAP(devices[0],
                                                             (thermalTypes)this->get_parameter("thermal_camera_colormap").as_int()));
            printDefaultError(ENABLE_THERMAL_CAMERA_TEMPERATURE_FILTER(devices[0],
                                                                       this->get_parameter("thermal_camera_temperature_filter").as_bool()));
            printDefaultError(CHANGE_THERMAL_CAMERA_TEMPERATURE_FILTER(devices[0],
                                                                       this->get_parameter("thermal_camera_temperature_filter_min").as_int(),
                                                                       this->get_parameter("thermal_camera_temperature_filter_max").as_int()));
        }
        if (m_allied_wide_sensor != NULL)
        {
            printDefaultError(CHANGE_ALLIED_CAMERA_BLACK_LEVEL(devices[0], *m_allied_wide_sensor,
                                                               this->get_parameter("allied_wide_camera_black_level").as_double()));
            printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(devices[0], *m_allied_wide_sensor,
                                                                      this->get_parameter("allied_wide_camera_auto_exposure_time").as_bool()));
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], *m_allied_wide_sensor,
                                                                            this->get_parameter("allied_wide_camera_auto_exposure_time_range_min").as_double(),
                                                                            this->get_parameter("allied_wide_camera_auto_exposure_time_range_max").as_double()));
            if (!this->get_parameter("allied_wide_camera_auto_exposure_time").as_bool())
            { //! Values might not match after disabling allied_wide_camera_auto_exposure_time
                printDefaultError(CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(devices[0], *m_allied_wide_sensor,
                                                                        this->get_parameter("allied_wide_camera_exposure_time").as_double()));
            }
            printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_GAIN(devices[0], *m_allied_wide_sensor,
                                                             this->get_parameter("allied_wide_camera_auto_gain").as_bool()));
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(devices[0], *m_allied_wide_sensor,
                                                                   (float)this->get_parameter("allied_wide_camera_auto_gain_range_min").as_double(),
                                                                   (float)this->get_parameter("allied_wide_camera_auto_gain_range_max").as_double()));
            if (!this->get_parameter("allied_wide_camera_auto_gain").as_bool())
            { //! Values might not match after disabling allied_wide_camera_auto_gain
                printDefaultError(CHANGE_ALLIED_CAMERA_GAIN(devices[0], *m_allied_wide_sensor,
                                                            this->get_parameter("allied_wide_camera_gain").as_double()));
            }
            printDefaultError(CHANGE_ALLIED_CAMERA_GAMMA(devices[0], *m_allied_wide_sensor,
                                                         this->get_parameter("allied_wide_camera_gamma").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_SATURATION(devices[0], *m_allied_wide_sensor,
                                                              this->get_parameter("allied_wide_camera_saturation").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_SHARPNESS(devices[0], *m_allied_wide_sensor,
                                                             this->get_parameter("allied_wide_camera_sharpness").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_HUE(devices[0], *m_allied_wide_sensor,
                                                       this->get_parameter("allied_wide_camera_hue").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_wide_sensor,
                                                                             this->get_parameter("allied_wide_camera_intensity_auto_precedence").as_int()));
            printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(devices[0], *m_allied_wide_sensor,
                                                                      this->get_parameter("allied_wide_camera_auto_white_balance").as_bool()));
            printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_wide_sensor,
                                                                          this->get_parameter("allied_wide_camera_balance_ratio_selector").as_int()));
            printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO(devices[0], *m_allied_wide_sensor,
                                                                 this->get_parameter("allied_wide_camera_balance_ratio").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(devices[0], *m_allied_wide_sensor,
                                                                           this->get_parameter("allied_wide_camera_balance_white_auto_rate").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(devices[0], *m_allied_wide_sensor,
                                                                                this->get_parameter("allied_wide_camera_balance_white_auto_tolerance").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_MODE_REGION(devices[0], *m_allied_wide_sensor,
                                                                    this->get_parameter("allied_wide_camera_auto_mode_region_height").as_int(),
                                                                    this->get_parameter("allied_wide_camera_auto_mode_region_width").as_int()));
            printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_wide_sensor,
                                                                               this->get_parameter("allied_wide_camera_intensity_controller_region").as_int()));
            printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(devices[0], *m_allied_wide_sensor,
                                                                               this->get_parameter("allied_wide_camera_intensity_controller_target").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(devices[0], *m_allied_wide_sensor,
                                                                            this->get_parameter("allied_wide_camera_max_driver_buffers_count").as_int()));
        }
        if (m_allied_narrow_sensor != NULL)
        {
            printDefaultError(CHANGE_ALLIED_CAMERA_BLACK_LEVEL(devices[0], *m_allied_narrow_sensor,
                                                               this->get_parameter("allied_narrow_camera_black_level").as_double()));
            printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(devices[0], *m_allied_narrow_sensor,
                                                                      this->get_parameter("allied_narrow_camera_auto_exposure_time").as_bool()));
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], *m_allied_narrow_sensor,
                                                                            this->get_parameter("allied_narrow_camera_auto_exposure_time_range_min").as_double(),
                                                                            this->get_parameter("allied_narrow_camera_auto_exposure_time_range_max").as_double()));
            if (!this->get_parameter("allied_narrow_camera_auto_exposure_time").as_bool())
            { //! Values might not match after disabling allied_narrow_camera_auto_exposure_time
                printDefaultError(CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(devices[0], *m_allied_narrow_sensor,
                                                                        this->get_parameter("allied_narrow_camera_exposure_time").as_double()));
            }
            printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_GAIN(devices[0], *m_allied_narrow_sensor,
                                                             this->get_parameter("allied_narrow_camera_auto_gain").as_bool()));
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(devices[0], *m_allied_narrow_sensor,
                                                                   (float)this->get_parameter("allied_narrow_camera_auto_gain_range_min").as_double(),
                                                                   (float)this->get_parameter("allied_narrow_camera_auto_gain_range_max").as_double()));
            if (!this->get_parameter("allied_narrow_camera_auto_gain").as_bool())
            { //! Values might not match after disabling allied_narrow_camera_auto_gain
                printDefaultError(CHANGE_ALLIED_CAMERA_GAIN(devices[0], *m_allied_narrow_sensor,
                                                            this->get_parameter("allied_narrow_camera_gain").as_double()));
            }
            printDefaultError(CHANGE_ALLIED_CAMERA_GAMMA(devices[0], *m_allied_narrow_sensor,
                                                         this->get_parameter("allied_narrow_camera_gamma").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_SATURATION(devices[0], *m_allied_narrow_sensor,
                                                              this->get_parameter("allied_narrow_camera_saturation").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_SHARPNESS(devices[0], *m_allied_narrow_sensor,
                                                             this->get_parameter("allied_narrow_camera_sharpness").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_HUE(devices[0], *m_allied_narrow_sensor,
                                                       this->get_parameter("allied_narrow_camera_hue").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(devices[0], *m_allied_narrow_sensor,
                                                                             this->get_parameter("allied_narrow_camera_intensity_auto_precedence").as_int()));
            printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(devices[0], *m_allied_narrow_sensor,
                                                                      this->get_parameter("allied_narrow_camera_auto_white_balance").as_bool()));
            printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(devices[0], *m_allied_narrow_sensor,
                                                                          this->get_parameter("allied_narrow_camera_balance_ratio_selector").as_int()));
            printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO(devices[0], *m_allied_narrow_sensor,
                                                                 this->get_parameter("allied_narrow_camera_balance_ratio").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(devices[0], *m_allied_narrow_sensor,
                                                                           this->get_parameter("allied_narrow_camera_balance_white_auto_rate").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(devices[0], *m_allied_narrow_sensor,
                                                                                this->get_parameter("allied_narrow_camera_balance_white_auto_tolerance").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_MODE_REGION(devices[0], *m_allied_narrow_sensor,
                                                                    this->get_parameter("allied_narrow_camera_auto_mode_region_height").as_int(),
                                                                    this->get_parameter("allied_narrow_camera_auto_mode_region_width").as_int()));
            printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(devices[0], *m_allied_narrow_sensor,
                                                                               this->get_parameter("allied_narrow_camera_intensity_controller_region").as_int()));
            printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(devices[0], *m_allied_narrow_sensor,
                                                                               this->get_parameter("allied_narrow_camera_intensity_controller_target").as_double()));
            printDefaultError(CHANGE_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(devices[0], *m_allied_narrow_sensor,
                                                                            this->get_parameter("allied_narrow_camera_max_driver_buffers_count").as_int()));
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Default parameters loaded");
    }

private:
    void printDefaultError(int error)
    {
        if (error != L3CAM_OK)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
                                "ERROR SETTING DEFAULT PARAMETER (" << error << ") " << getBeamErrorDescription(error));
        }
    }
};

int main(int argc, char **argv)
{
    std::cout << "L3Cam version " << GET_VERSION() << std::endl;

    rclcpp::init(argc, argv);
    std::shared_ptr<L3Cam> node = std::make_shared<L3Cam>();

    // Initialize services
    rclcpp::Service<l3cam_interfaces::srv::GetVersion>::SharedPtr srvGetVersion =
        node->create_service<l3cam_interfaces::srv::GetVersion>("get_version", &getVersion);
    rclcpp::Service<l3cam_interfaces::srv::Initialize>::SharedPtr srvInitialize =
        node->create_service<l3cam_interfaces::srv::Initialize>("initialize", &initialize);
    rclcpp::Service<l3cam_interfaces::srv::Terminate>::SharedPtr srvTerminate =
        node->create_service<l3cam_interfaces::srv::Terminate>("terminate", &terminate);
    rclcpp::Service<l3cam_interfaces::srv::FindDevices>::SharedPtr srvFindDevices =
        node->create_service<l3cam_interfaces::srv::FindDevices>("find_devices", &findDevices);
    rclcpp::Service<l3cam_interfaces::srv::GetLocalServerAddress>::SharedPtr srvGetLocalServerAddress =
        node->create_service<l3cam_interfaces::srv::GetLocalServerAddress>("get_local_server_address", &getLocalServerAddress);
    rclcpp::Service<l3cam_interfaces::srv::GetDeviceStatus>::SharedPtr srvGetDeviceStatus =
        node->create_service<l3cam_interfaces::srv::GetDeviceStatus>("get_device_status", &getDeviceStatus);
    rclcpp::Service<l3cam_interfaces::srv::GetSensorsAvailable>::SharedPtr srvGetSensorsAvailable =
        node->create_service<l3cam_interfaces::srv::GetSensorsAvailable>("get_sensors_available", &getSensorsAvailable);
    rclcpp::Service<l3cam_interfaces::srv::GetNetworkConfiguration>::SharedPtr srvGetNetworkConfiguration =
        node->create_service<l3cam_interfaces::srv::GetNetworkConfiguration>("get_network_configuration", &getNetworkConfiguration);
    rclcpp::Service<l3cam_interfaces::srv::ChangeNetworkConfiguration>::SharedPtr srvChangeNetworkConfiguration =
        node->create_service<l3cam_interfaces::srv::ChangeNetworkConfiguration>("change_network_configuration", &changeNetworkConfiguration);
    rclcpp::Service<l3cam_interfaces::srv::PowerOffDevice>::SharedPtr srvPowerOffDevice =
        node->create_service<l3cam_interfaces::srv::PowerOffDevice>("power_off_device", &powerOffDevice);
    rclcpp::Service<l3cam_interfaces::srv::StartDevice>::SharedPtr srvStartDevice =
        node->create_service<l3cam_interfaces::srv::StartDevice>("start_device", &startDevice);
    rclcpp::Service<l3cam_interfaces::srv::StopDevice>::SharedPtr srvStopDevice =
        node->create_service<l3cam_interfaces::srv::StopDevice>("stop_device", &stopDevice);
    rclcpp::Service<l3cam_interfaces::srv::StartStream>::SharedPtr srvStartStream =
        node->create_service<l3cam_interfaces::srv::StartStream>("start_stream", &startStream);
    rclcpp::Service<l3cam_interfaces::srv::StopStream>::SharedPtr srvStopStream =
        node->create_service<l3cam_interfaces::srv::StopStream>("stop_stream", &stopStream);

    rclcpp::Service<l3cam_interfaces::srv::ChangePointcloudColor>::SharedPtr srvChangePointcloudColor =
        node->create_service<l3cam_interfaces::srv::ChangePointcloudColor>("change_pointcloud_color", &changePointcloudColor);
    rclcpp::Service<l3cam_interfaces::srv::ChangePointcloudColorRange>::SharedPtr srvChangePointcloudColorRange =
        node->create_service<l3cam_interfaces::srv::ChangePointcloudColorRange>("change_pointcloud_color_range", &changePointcloudColorRange);
    rclcpp::Service<l3cam_interfaces::srv::ChangeDistanceRange>::SharedPtr srvChangeDistanceRange =
        node->create_service<l3cam_interfaces::srv::ChangeDistanceRange>("change_distance_range", &changeDistanceRange);
    rclcpp::Service<l3cam_interfaces::srv::EnableAutoBias>::SharedPtr srvEnableAutoBias =
        node->create_service<l3cam_interfaces::srv::EnableAutoBias>("enable_auto_bias", &enableAutoBias);
    rclcpp::Service<l3cam_interfaces::srv::ChangeBiasValue>::SharedPtr srvChangeBiasValue =
        node->create_service<l3cam_interfaces::srv::ChangeBiasValue>("change_bias_value", &changeBiasValue);

    rclcpp::Service<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings>::SharedPtr srvSetPolarimetricCameraDefaultSettings =
        node->create_service<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings>("set_polarimetric_camera_default_settings", &setPolarimetricCameraDefaultSettings);
    rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness>::SharedPtr srvChangePolarimetricCameraBrightness =
        node->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness>("change_polarimetric_camera_brightness", &changePolarimetricCameraBrightness);
    rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel>::SharedPtr srvChangePolarimetricCameraBlackLevel =
        node->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel>("change_polarimetric_camera_black_level", &changePolarimetricCameraBlackLevel);
    rclcpp::Service<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain>::SharedPtr srvEnablePolarimetricCameraAutoGain =
        node->create_service<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain>("enable_polarimetric_camera_auto_gain", &enablePolarimetricCameraAutoGain);
    rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange>::SharedPtr srvChangePolarimetricCameraAutoGainRange =
        node->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange>("change_polarimetric_camera_auto_gain_range", &changePolarimetricCameraAutoGainRange);
    rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraGain>::SharedPtr srvChangePolarimetricCameraGain =
        node->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraGain>("change_polarimetric_camera_gain", &changePolarimetricCameraGain);
    rclcpp::Service<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime>::SharedPtr srvEnablePolarimetricCameraAutoExposureTime =
        node->create_service<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime>("enable_polarimetric_camera_auto_exposure_time", &enablePolarimetricCameraAutoExposureTime);
    rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange>::SharedPtr srvChangePolarimetricCameraAutoExposureTimeRange =
        node->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange>("change_polarimetric_camera_auto_exposure_time_range", &changePolarimetricCameraAutoExposureTimeRange);
    rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime>::SharedPtr srvChangePolarimetricCameraExposureTime =
        node->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime>("change_polarimetric_camera_exposure_time", &changePolarimetricCameraExposureTime);

    rclcpp::Service<l3cam_interfaces::srv::SetRgbCameraDefaultSettings>::SharedPtr srvSetRgbCameraDefaultSettings =
        node->create_service<l3cam_interfaces::srv::SetRgbCameraDefaultSettings>("set_rgb_camera_default_settings", &setRgbCameraDefaultSettings);
    rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraBrightness>::SharedPtr srvChangeRgbCameraBrightness =
        node->create_service<l3cam_interfaces::srv::ChangeRgbCameraBrightness>("change_rgb_camera_brightness", &changeRgbCameraBrightness);
    rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraContrast>::SharedPtr srvChangeRgbCameraContrast =
        node->create_service<l3cam_interfaces::srv::ChangeRgbCameraContrast>("change_rgb_camera_contrast", &changeRgbCameraContrast);
    rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraSaturation>::SharedPtr srvChangeRgbCameraSaturation =
        node->create_service<l3cam_interfaces::srv::ChangeRgbCameraSaturation>("change_rgb_camera_saturation", &changeRgbCameraSaturation);
    rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraSharpness>::SharedPtr srvChangeRgbCameraSharpness =
        node->create_service<l3cam_interfaces::srv::ChangeRgbCameraSharpness>("change_rgb_camera_sharpness", &changeRgbCameraSharpness);
    rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraGamma>::SharedPtr srvChangeRgbCameraGamma =
        node->create_service<l3cam_interfaces::srv::ChangeRgbCameraGamma>("change_rgb_camera_gamma", &changeRgbCameraGamma);
    rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraGain>::SharedPtr srvChangeRgbCameraGain =
        node->create_service<l3cam_interfaces::srv::ChangeRgbCameraGain>("change_rgb_camera_gain", &changeRgbCameraGain);
    rclcpp::Service<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance>::SharedPtr srvEnableRgbCameraAutoWhiteBalance =
        node->create_service<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance>("enable_rgb_camera_auto_white_balance", &enableRgbCameraAutoWhiteBalance);
    rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance>::SharedPtr srvChangeRgbCameraWhiteBalance =
        node->create_service<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance>("change_rgb_camera_white_balance", &changeRgbCameraWhiteBalance);
    rclcpp::Service<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime>::SharedPtr srvEnableRgbCameraAutoExposureTime =
        node->create_service<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime>("enable_rgb_camera_auto_exposure_time", &enableRgbCameraAutoExposureTime);
    rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraExposureTime>::SharedPtr srvChangeRgbCameraExposureTime =
        node->create_service<l3cam_interfaces::srv::ChangeRgbCameraExposureTime>("change_rgb_camera_exposure_time", &changeRgbCameraExposureTime);

    rclcpp::Service<l3cam_interfaces::srv::ChangeThermalCameraColormap>::SharedPtr srvChangeThermalCameraColormap =
        node->create_service<l3cam_interfaces::srv::ChangeThermalCameraColormap>("change_thermal_camera_colormap", &changeThermalCameraColormap);
    rclcpp::Service<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter>::SharedPtr srvChangeThermalCameraTemperatureFilter =
        node->create_service<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter>("change_thermal_camera_temperature_filter", &changeThermalCameraTemperatureFilter);
    rclcpp::Service<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter>::SharedPtr srvEnableThermalCameraTemperatureFilter =
        node->create_service<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter>("enable_thermal_camera_temperature_filter", &enableThermalCameraTemperatureFilter);

    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime>::SharedPtr srvChangeAlliedCameraExposureTime =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime>("change_allied_camera_exposure_time", &changeAlliedCameraExposureTime);
    rclcpp::Service<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime>::SharedPtr srvEnableAlliedCameraAutoExposureTime =
        node->create_service<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime>("enable_allied_camera_auto_exposure_time", &enableAlliedCameraAutoExposureTime);
    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange>::SharedPtr srvChangeAlliedCameraAutoExposureTimeRange =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange>("change_allied_camera_auto_exposure_time_range", &changeAlliedCameraAutoExposureTimeRange);
    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraGain>::SharedPtr srvChangeAlliedCameraGain =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraGain>("change_allied_camera_gain", &changeAlliedCameraGain);
    rclcpp::Service<l3cam_interfaces::srv::EnableAlliedCameraAutoGain>::SharedPtr srvEnableAlliedCameraAutoGain =
        node->create_service<l3cam_interfaces::srv::EnableAlliedCameraAutoGain>("enable_allied_camera_auto_gain", &enableAlliedCameraAutoGain);
    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange>::SharedPtr srvChangeAlliedCameraAutoGainRange =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange>("change_allied_camera_auto_gain_range", &changeAlliedCameraAutoGainRange);
    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraGamma>::SharedPtr srvChangeAlliedCameraGamma =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraGamma>("change_allied_camera_gamma", &changeAlliedCameraGamma);
    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraSaturation>::SharedPtr srvChangeAlliedCameraSaturation =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraSaturation>("change_allied_camera_saturation", &changeAlliedCameraSaturation);
    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraHue>::SharedPtr srvChangeAlliedCameraHue =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraHue>("change_allied_camera_hue", &changeAlliedCameraHue);
    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence>::SharedPtr srvChangeAlliedCameraIntensityAutoPrecedence =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence>("change_allied_camera_intensity_auto_precedence", &changeAlliedCameraIntensityAutoPrecedence);
    rclcpp::Service<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance>::SharedPtr srvEnableAlliedCameraAutoWhiteBalance =
        node->create_service<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance>("enable_allied_camera_auto_white_balance", &enableAlliedCameraAutoWhiteBalance);
    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector>::SharedPtr srvChangeAlliedCameraBalanceRatioSelector =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector>("change_allied_camera_balance_ratio_selector", &changeAlliedCameraBalanceRatioSelector);
    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio>::SharedPtr srvChangeAlliedCameraBalanceRatio =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio>("change_allied_camera_balance_ratio", &changeAlliedCameraBalanceRatio);
    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate>::SharedPtr srvChangeAlliedCameraBalanceWhiteAutoRate =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate>("change_allied_camera_balance_white_auto_rate", &changeAlliedCameraBalanceWhiteAutoRate);
    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance>::SharedPtr srvChangeAlliedCameraBalanceWhiteAutoTolerance =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance>("change_allied_camera_balance_white_auto_tolerance", &changeAlliedCameraBalanceWhiteAutoTolerance);
    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion>::SharedPtr srvChangeAlliedCameraIntensityControllerRegion =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion>("change_allied_camera_intensity_controller_region", &changeAlliedCameraIntensityControllerRegion);
    rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget>::SharedPtr srvChangeAlliedCameraIntensityControllerTarget =
        node->create_service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget>("change_allied_camera_intensity_controller_target", &changeAlliedCameraIntensityControllerTarget);

    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBlackLevel>::SharedPtr srvGetAlliedCameraBlackLevel =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraBlackLevel>("get_allied_camera_black_level", &getAlliedCameraBlackLevel);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraExposureTime>::SharedPtr srvGetAlliedCameraExposureTime =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraExposureTime>("get_allied_camera_exposure_time", &getAlliedCameraExposureTime);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTime>::SharedPtr srvGetAlliedCameraAutoExposureTime =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTime>("get_allied_camera_auto_exposure_time", &getAlliedCameraAutoExposureTime);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTimeRange>::SharedPtr srvGetAlliedCameraAutoExposureTimeRange =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTimeRange>("get_allied_camera_auto_exposure_time_range", &getAlliedCameraAutoExposureTimeRange);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraGain>::SharedPtr srvGetAlliedCameraGain =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraGain>("get_allied_camera_gain", &getAlliedCameraGain);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoGain>::SharedPtr srvGetAlliedCameraAutoGain =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoGain>("get_allied_camera_auto_gain", &getAlliedCameraAutoGain);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoGainRange>::SharedPtr srvGetAlliedCameraAutoGainRange =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoGainRange>("get_allied_camera_auto_gain_range", &getAlliedCameraAutoGainRange);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraGamma>::SharedPtr srvGetAlliedCameraGamma =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraGamma>("get_allied_camera_gamma", &getAlliedCameraGamma);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraSaturation>::SharedPtr srvGetAlliedCameraSaturation =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraSaturation>("get_allied_camera_saturation", &getAlliedCameraSaturation);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraSharpness>::SharedPtr srvGetAlliedCameraSharpness =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraSharpness>("get_allied_camera_sharpness", &getAlliedCameraSharpness);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraHue>::SharedPtr srvGetAlliedCameraHue =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraHue>("get_allied_camera_hue", &getAlliedCameraHue);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraIntensityAutoPrecedence>::SharedPtr srvGetAlliedCameraIntensityAutoPrecedence =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraIntensityAutoPrecedence>("get_allied_camera_intensity_auto_precedence", &getAlliedCameraIntensityAutoPrecedence);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoWhiteBalance>::SharedPtr srvGetAlliedCameraAutoWhiteBalance =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoWhiteBalance>("get_allied_camera_auto_white_balance", &getAlliedCameraAutoWhiteBalance);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBalanceRatioSelector>::SharedPtr srvGetAlliedCameraBalanceRatioSelector =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraBalanceRatioSelector>("get_allied_camera_balance_ratio_selector", &getAlliedCameraBalanceRatioSelector);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBalanceRatio>::SharedPtr srvGetAlliedCameraBalanceRatio =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraBalanceRatio>("get_allied_camera_balance_ratio", &getAlliedCameraBalanceRatio);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoRate>::SharedPtr srvGetAlliedCameraBalanceWhiteAutoRate =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoRate>("get_allied_camera_balance_white_auto_rate", &getAlliedCameraBalanceWhiteAutoRate);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoTolerance>::SharedPtr srvGetAlliedCameraBalanceWhiteAutoTolerance =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoTolerance>("get_allied_camera_balance_white_auto_tolerance", &getAlliedCameraBalanceWhiteAutoTolerance);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoModeRegion>::SharedPtr srvGetAlliedCameraAutoModeRegion =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoModeRegion>("get_allied_camera_auto_mode_region", &getAlliedCameraAutoModeRegion);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerRegion>::SharedPtr srvGetAlliedCameraIntensityControllerRegion =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerRegion>("get_allied_camera_intensity_controller_region", &getAlliedCameraIntensityControllerRegion);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerTarget>::SharedPtr srvGetAlliedCameraIntensityControllerTarget =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerTarget>("get_allied_camera_intensity_controller_target", &getAlliedCameraIntensityControllerTarget);
    rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraMaxDriverBuffersCount>::SharedPtr srvGetAlliedCameraMaxDriverBuffersCount =
        node->create_service<l3cam_interfaces::srv::GetAlliedCameraMaxDriverBuffersCount>("get_allied_camera_max_driver_buffers_count", &getAlliedCameraMaxDriverBuffersCount);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Services ready");

    // Initialize L3Cam
    int error = L3CAM_OK;

    registerSystemSignals();

    std::string local_address = node->get_parameter("local_address").as_string();
    std::string device_address = node->get_parameter("device_address").as_string();
    error = INITIALIZE((local_address == "") ? NULL : &local_address[0], (device_address == "") ? NULL : &device_address[0]);
    if (error)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "(INTIALIZE error " << error << ") " << getBeamErrorDescription(error));
        TERMINATE(devices[0]);
        return 1;
    }

    int num_devices = 0;
    // ros::Rate loop_rate(100);
    while (num_devices == 0 /*&& ros::ok()*/)
    {
        error = FIND_DEVICES(devices, &num_devices);
        // loop_rate.sleep();
    }
    if (error)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "(FIND_DEVICES error " << error << ") " << getBeamErrorDescription(error));
        TERMINATE(devices[0]);
        return 1;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Device found " << devices[0].ip_address << " model " << devices[0].model << " serial number " << devices[0].serial_number);

    int status = 0;
    error = GET_DEVICE_STATUS(devices[0], &status);
    if (error)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "(GET_DEVICE_STATUS error " << error << ") " << getBeamErrorDescription(error));
        TERMINATE(devices[0]);
        return 1;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Device status " << status);

    int num_sensors = 0;
    error = GET_SENSORS_AVAILABLE(devices[0], av_sensors, &num_sensors);
    if (error)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "(GET_SENSORS_AVAILABLE error " << error << ") " << getBeamErrorDescription(error));
        TERMINATE(devices[0]);
        return 1;
    }
    for (int i = 0; i < num_sensors; ++i)
    {
        switch (av_sensors[i].sensor_type)
        {
        case sensor_lidar:
            m_lidar_sensor = &av_sensors[i];
            break;
        case sensor_econ_rgb:
            m_rgb_sensor = &av_sensors[i];
            break;
        case sensor_thermal:
            m_thermal_sensor = &av_sensors[i];
            break;
        case sensor_pol:
            m_polarimetric_sensor = &av_sensors[i];
            break;
        case sensor_allied_wide:
            m_allied_wide_sensor = &av_sensors[i];
            break;
        case sensor_allied_narrow:
            m_allied_narrow_sensor = &av_sensors[i];
            break;
        }
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), num_sensors << ((num_sensors == 1) ? " sensor" : " sensors") << " available");

    error = START_DEVICE(devices[0]);
    if (error)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "(START_DEVICE error " << error << ") " << getBeamErrorDescription(error));
        TERMINATE(devices[0]);
        return 1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Device started");

    node.get()->loadDefaultParams();

    error = START_STREAM(devices[0]);
    if (error)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "(START_STREAM error " << error << ") " << getBeamErrorDescription(error));
        STOP_DEVICE(devices[0]);
        TERMINATE(devices[0]);
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Device ready\n");

    while (rclcpp::ok())
    {
        error = FIND_DEVICES(devices, &num_devices);
        if (error)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
            break;
        }
        if (num_devices == 0)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Device disconnected");
            break;
        }

        rclcpp::spin_some(node); // spin_once(?)
    }

    rclcpp::shutdown();
    return 0;
}
