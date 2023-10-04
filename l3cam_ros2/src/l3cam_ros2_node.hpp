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

#include <beamagine.h>

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

#include "l3cam_interfaces/srv/sensor_disconnected.hpp"

#define ROS2_BMG_UNUSED(x) (void)x;

namespace l3cam_ros2
{
    class L3Cam : public rclcpp::Node
    {
    public:
        explicit L3Cam();

        int initializeDevice();
        int startDeviceStream();

        l3cam devices[1];

    private:
        void declareParameters();
        void initializeServices();

        inline void printDefaultError(int error, std::string param);
        void loadDefaultParams();

        void timer_callback();

        void getVersion(const std::shared_ptr<l3cam_interfaces::srv::GetVersion::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetVersion::Response> res);
        void initialize(const std::shared_ptr<l3cam_interfaces::srv::Initialize::Request> req, std::shared_ptr<l3cam_interfaces::srv::Initialize::Response> res);
        void terminate(const std::shared_ptr<l3cam_interfaces::srv::Terminate::Request> req, std::shared_ptr<l3cam_interfaces::srv::Terminate::Response> res);
        void findDevices(const std::shared_ptr<l3cam_interfaces::srv::FindDevices::Request> req, std::shared_ptr<l3cam_interfaces::srv::FindDevices::Response> res);
        void getLocalServerAddress(const std::shared_ptr<l3cam_interfaces::srv::GetLocalServerAddress::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetLocalServerAddress::Response> res);
        void getDeviceStatus(const std::shared_ptr<l3cam_interfaces::srv::GetDeviceStatus::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetDeviceStatus::Response> res);
        void getSensorsAvailable(const std::shared_ptr<l3cam_interfaces::srv::GetSensorsAvailable::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetSensorsAvailable::Response> res);
        void getNetworkConfiguration(const std::shared_ptr<l3cam_interfaces::srv::GetNetworkConfiguration::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetNetworkConfiguration::Response> res);
        void changeNetworkConfiguration(const std::shared_ptr<l3cam_interfaces::srv::ChangeNetworkConfiguration::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeNetworkConfiguration::Response> res);
        void powerOffDevice(const std::shared_ptr<l3cam_interfaces::srv::PowerOffDevice::Request> req, std::shared_ptr<l3cam_interfaces::srv::PowerOffDevice::Response> res);
        void startDevice(const std::shared_ptr<l3cam_interfaces::srv::StartDevice::Request> req, std::shared_ptr<l3cam_interfaces::srv::StartDevice::Response> res);
        void stopDevice(const std::shared_ptr<l3cam_interfaces::srv::StopDevice::Request> req, std::shared_ptr<l3cam_interfaces::srv::StopDevice::Response> res);
        void startStream(const std::shared_ptr<l3cam_interfaces::srv::StartStream::Request> req, std::shared_ptr<l3cam_interfaces::srv::StartStream::Response> res);
        void stopStream(const std::shared_ptr<l3cam_interfaces::srv::StopStream::Request> req, std::shared_ptr<l3cam_interfaces::srv::StopStream::Response> res);
        void changePointcloudColor(const std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColor::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColor::Response> res);
        void changePointcloudColorRange(const std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColorRange::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColorRange::Response> res);
        void changeDistanceRange(const std::shared_ptr<l3cam_interfaces::srv::ChangeDistanceRange::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeDistanceRange::Response> res);
        void enableAutoBias(const std::shared_ptr<l3cam_interfaces::srv::EnableAutoBias::Request> req, std::shared_ptr<l3cam_interfaces::srv::EnableAutoBias::Response> res);
        void changeBiasValue(const std::shared_ptr<l3cam_interfaces::srv::ChangeBiasValue::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeBiasValue::Response> res);
        void setPolarimetricCameraDefaultSettings(const std::shared_ptr<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings::Request> req, std::shared_ptr<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings::Response> res);
        void changePolarimetricCameraBrightness(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness::Response> res);
        void changePolarimetricCameraBlackLevel(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel::Response> res);
        void enablePolarimetricCameraAutoGain(const std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain::Request> req, std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain::Response> res);
        void changePolarimetricCameraAutoGainRange(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange::Response> res);
        void changePolarimetricCameraGain(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraGain::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraGain::Response> res);
        void enablePolarimetricCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime::Request> req, std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime::Response> res);
        void changePolarimetricCameraAutoExposureTimeRange(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange::Response> res);
        void changePolarimetricCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime::Response> res);
        void setRgbCameraDefaultSettings(const std::shared_ptr<l3cam_interfaces::srv::SetRgbCameraDefaultSettings::Request> req, std::shared_ptr<l3cam_interfaces::srv::SetRgbCameraDefaultSettings::Response> res);
        void changeRgbCameraBrightness(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraBrightness::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraBrightness::Response> res);
        void changeRgbCameraContrast(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraContrast::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraContrast::Response> res);
        void changeRgbCameraSaturation(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSaturation::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSaturation::Response> res);
        void changeRgbCameraSharpness(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSharpness::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSharpness::Response> res);
        void changeRgbCameraGamma(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGamma::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGamma::Response> res);
        void changeRgbCameraGain(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGain::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGain::Response> res);
        void enableRgbCameraAutoWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance::Request> req, std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance::Response> res);
        void changeRgbCameraWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance::Response> res);
        void enableRgbCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime::Request> req, std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime::Response> res);
        void changeRgbCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraExposureTime::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraExposureTime::Response> res);
        void changeThermalCameraColormap(const std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraColormap::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraColormap::Response> res);
        void changeThermalCameraTemperatureFilter(const std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter::Response> res);
        void enableThermalCameraTemperatureFilter(const std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter::Request> req, std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter::Response> res);
        void changeAlliedCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime::Response> res);
        void enableAlliedCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime::Request> req, std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime::Response> res);
        void changeAlliedCameraAutoExposureTimeRange(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange::Response> res);
        void changeAlliedCameraGain(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraGain::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraGain::Response> res);
        void enableAlliedCameraAutoGain(const std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoGain::Request> req, std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoGain::Response> res);
        void changeAlliedCameraAutoGainRange(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange::Response> res);
        void changeAlliedCameraGamma(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraGamma::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraGamma::Response> res);
        void changeAlliedCameraSaturation(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraSaturation::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraSaturation::Response> res);
        void changeAlliedCameraHue(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraHue::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraHue::Response> res);
        void changeAlliedCameraIntensityAutoPrecedence(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence::Response> res);
        void enableAlliedCameraAutoWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance::Request> req, std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance::Response> res);
        void changeAlliedCameraBalanceRatioSelector(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector::Response> res);
        void changeAlliedCameraBalanceRatio(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio::Response> res);
        void changeAlliedCameraBalanceWhiteAutoRate(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate::Response> res);
        void changeAlliedCameraBalanceWhiteAutoTolerance(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance::Response> res);
        void changeAlliedCameraIntensityControllerRegion(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion::Response> res);
        void changeAlliedCameraIntensityControllerTarget(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget::Response> res);
        void getAlliedCameraBlackLevel(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBlackLevel::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBlackLevel::Response> res);
        void getAlliedCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraExposureTime::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraExposureTime::Response> res);
        void getAlliedCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTime::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTime::Response> res);
        void getAlliedCameraAutoExposureTimeRange(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTimeRange::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTimeRange::Response> res);
        void getAlliedCameraGain(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraGain::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraGain::Response> res);
        void getAlliedCameraAutoGain(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoGain::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoGain::Response> res);
        void getAlliedCameraAutoGainRange(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoGainRange::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoGainRange::Response> res);
        void getAlliedCameraGamma(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraGamma::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraGamma::Response> res);
        void getAlliedCameraSaturation(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraSaturation::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraSaturation::Response> res);
        void getAlliedCameraSharpness(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraSharpness::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraSharpness::Response> res);
        void getAlliedCameraHue(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraHue::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraHue::Response> res);
        void getAlliedCameraIntensityAutoPrecedence(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityAutoPrecedence::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityAutoPrecedence::Response> res);
        void getAlliedCameraAutoWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoWhiteBalance::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoWhiteBalance::Response> res);
        void getAlliedCameraBalanceRatioSelector(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceRatioSelector::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceRatioSelector::Response> res);
        void getAlliedCameraBalanceRatio(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceRatio::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceRatio::Response> res);
        void getAlliedCameraBalanceWhiteAutoRate(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoRate::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoRate::Response> res);
        void getAlliedCameraBalanceWhiteAutoTolerance(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoTolerance::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoTolerance::Response> res);
        void getAlliedCameraAutoModeRegion(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoModeRegion::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoModeRegion::Response> res);
        void getAlliedCameraIntensityControllerRegion(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerRegion::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerRegion::Response> res);
        void getAlliedCameraIntensityControllerTarget(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerTarget::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerTarget::Response> res);
        void getAlliedCameraMaxDriverBuffersCount(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraMaxDriverBuffersCount::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraMaxDriverBuffersCount::Response> res);

        void lidarDisconnected(int code);
        void polDisconnected(int code);
        void rgbDisconnected(int code);
        void wideDisconnected(int code);
        void alliedNarrowDisconnect(int code);
        void thermalDisconnected(int code);

        static void errorNotification(const int32_t *error);

        rclcpp::Service<l3cam_interfaces::srv::GetVersion>::SharedPtr srvGetVersion;
        rclcpp::Service<l3cam_interfaces::srv::Initialize>::SharedPtr srvInitialize;
        rclcpp::Service<l3cam_interfaces::srv::Terminate>::SharedPtr srvTerminate;
        rclcpp::Service<l3cam_interfaces::srv::FindDevices>::SharedPtr srvFindDevices;
        rclcpp::Service<l3cam_interfaces::srv::GetLocalServerAddress>::SharedPtr srvGetLocalServerAddress;
        rclcpp::Service<l3cam_interfaces::srv::GetDeviceStatus>::SharedPtr srvGetDeviceStatus;
        rclcpp::Service<l3cam_interfaces::srv::GetSensorsAvailable>::SharedPtr srvGetSensorsAvailable;
        rclcpp::Service<l3cam_interfaces::srv::GetNetworkConfiguration>::SharedPtr srvGetNetworkConfiguration;
        rclcpp::Service<l3cam_interfaces::srv::ChangeNetworkConfiguration>::SharedPtr srvChangeNetworkConfiguration;
        rclcpp::Service<l3cam_interfaces::srv::PowerOffDevice>::SharedPtr srvPowerOffDevice;
        rclcpp::Service<l3cam_interfaces::srv::StartDevice>::SharedPtr srvStartDevice;
        rclcpp::Service<l3cam_interfaces::srv::StopDevice>::SharedPtr srvStopDevice;
        rclcpp::Service<l3cam_interfaces::srv::StartStream>::SharedPtr srvStartStream;
        rclcpp::Service<l3cam_interfaces::srv::StopStream>::SharedPtr srvStopStream;

        rclcpp::Service<l3cam_interfaces::srv::ChangePointcloudColor>::SharedPtr srvChangePointcloudColor;
        rclcpp::Service<l3cam_interfaces::srv::ChangePointcloudColorRange>::SharedPtr srvChangePointcloudColorRange;
        rclcpp::Service<l3cam_interfaces::srv::ChangeDistanceRange>::SharedPtr srvChangeDistanceRange;
        rclcpp::Service<l3cam_interfaces::srv::EnableAutoBias>::SharedPtr srvEnableAutoBias;
        rclcpp::Service<l3cam_interfaces::srv::ChangeBiasValue>::SharedPtr srvChangeBiasValue;

        rclcpp::Service<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings>::SharedPtr srvSetPolarimetricCameraDefaultSettings;
        rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness>::SharedPtr srvChangePolarimetricCameraBrightness;
        rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel>::SharedPtr srvChangePolarimetricCameraBlackLevel;
        rclcpp::Service<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain>::SharedPtr srvEnablePolarimetricCameraAutoGain;
        rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange>::SharedPtr srvChangePolarimetricCameraAutoGainRange;
        rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraGain>::SharedPtr srvChangePolarimetricCameraGain;
        rclcpp::Service<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime>::SharedPtr srvEnablePolarimetricCameraAutoExposureTime;
        rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange>::SharedPtr srvChangePolarimetricCameraAutoExposureTimeRange;
        rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime>::SharedPtr srvChangePolarimetricCameraExposureTime;

        rclcpp::Service<l3cam_interfaces::srv::SetRgbCameraDefaultSettings>::SharedPtr srvSetRgbCameraDefaultSettings;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraBrightness>::SharedPtr srvChangeRgbCameraBrightness;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraContrast>::SharedPtr srvChangeRgbCameraContrast;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraSaturation>::SharedPtr srvChangeRgbCameraSaturation;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraSharpness>::SharedPtr srvChangeRgbCameraSharpness;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraGamma>::SharedPtr srvChangeRgbCameraGamma;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraGain>::SharedPtr srvChangeRgbCameraGain;
        rclcpp::Service<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance>::SharedPtr srvEnableRgbCameraAutoWhiteBalance;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance>::SharedPtr srvChangeRgbCameraWhiteBalance;
        rclcpp::Service<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime>::SharedPtr srvEnableRgbCameraAutoExposureTime;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraExposureTime>::SharedPtr srvChangeRgbCameraExposureTime;

        rclcpp::Service<l3cam_interfaces::srv::ChangeThermalCameraColormap>::SharedPtr srvChangeThermalCameraColormap;
        rclcpp::Service<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter>::SharedPtr srvChangeThermalCameraTemperatureFilter;
        rclcpp::Service<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter>::SharedPtr srvEnableThermalCameraTemperatureFilter;

        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime>::SharedPtr srvChangeAlliedCameraExposureTime;
        rclcpp::Service<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime>::SharedPtr srvEnableAlliedCameraAutoExposureTime;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange>::SharedPtr srvChangeAlliedCameraAutoExposureTimeRange;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraGain>::SharedPtr srvChangeAlliedCameraGain;
        rclcpp::Service<l3cam_interfaces::srv::EnableAlliedCameraAutoGain>::SharedPtr srvEnableAlliedCameraAutoGain;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange>::SharedPtr srvChangeAlliedCameraAutoGainRange;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraGamma>::SharedPtr srvChangeAlliedCameraGamma;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraSaturation>::SharedPtr srvChangeAlliedCameraSaturation;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraHue>::SharedPtr srvChangeAlliedCameraHue;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence>::SharedPtr srvChangeAlliedCameraIntensityAutoPrecedence;
        rclcpp::Service<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance>::SharedPtr srvEnableAlliedCameraAutoWhiteBalance;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector>::SharedPtr srvChangeAlliedCameraBalanceRatioSelector;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio>::SharedPtr srvChangeAlliedCameraBalanceRatio;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate>::SharedPtr srvChangeAlliedCameraBalanceWhiteAutoRate;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance>::SharedPtr srvChangeAlliedCameraBalanceWhiteAutoTolerance;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion>::SharedPtr srvChangeAlliedCameraIntensityControllerRegion;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget>::SharedPtr srvChangeAlliedCameraIntensityControllerTarget;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBlackLevel>::SharedPtr srvGetAlliedCameraBlackLevel;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraExposureTime>::SharedPtr srvGetAlliedCameraExposureTime;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTime>::SharedPtr srvGetAlliedCameraAutoExposureTime;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTimeRange>::SharedPtr srvGetAlliedCameraAutoExposureTimeRange;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraGain>::SharedPtr srvGetAlliedCameraGain;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoGain>::SharedPtr srvGetAlliedCameraAutoGain;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoGainRange>::SharedPtr srvGetAlliedCameraAutoGainRange;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraGamma>::SharedPtr srvGetAlliedCameraGamma;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraSaturation>::SharedPtr srvGetAlliedCameraSaturation;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraSharpness>::SharedPtr srvGetAlliedCameraSharpness;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraHue>::SharedPtr srvGetAlliedCameraHue;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraIntensityAutoPrecedence>::SharedPtr srvGetAlliedCameraIntensityAutoPrecedence;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoWhiteBalance>::SharedPtr srvGetAlliedCameraAutoWhiteBalance;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBalanceRatioSelector>::SharedPtr srvGetAlliedCameraBalanceRatioSelector;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBalanceRatio>::SharedPtr srvGetAlliedCameraBalanceRatio;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoRate>::SharedPtr srvGetAlliedCameraBalanceWhiteAutoRate;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoTolerance>::SharedPtr srvGetAlliedCameraBalanceWhiteAutoTolerance;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoModeRegion>::SharedPtr srvGetAlliedCameraAutoModeRegion;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerRegion>::SharedPtr srvGetAlliedCameraIntensityControllerRegion;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerTarget>::SharedPtr srvGetAlliedCameraIntensityControllerTarget;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraMaxDriverBuffersCount>::SharedPtr srvGetAlliedCameraMaxDriverBuffersCount;

        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr clientPointCloudStreamDisconnected;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr clientPointCloudConfigurationDisconnected;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr clientPolWideStreamDisconnected;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr clientPolConfigurationDisconnected;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr clientRgbNarrowStreamDisconnected;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr clientRgbConfigurationDisconnected;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr clientThermalStreamDisconnected;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr clientThermalConfigurationDisconnected;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr clientWideConfigurationDisconnected;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr clientNarrowConfigurationDisconnected;

        rclcpp::TimerBase::SharedPtr timer_;

        sensor av_sensors[6];

        sensor *m_lidar_sensor = NULL;
        sensor *m_rgb_sensor = NULL;
        sensor *m_thermal_sensor = NULL;
        sensor *m_polarimetric_sensor = NULL;
        sensor *m_allied_wide_sensor = NULL;
        sensor *m_allied_narrow_sensor = NULL;

        int num_devices;

    }; // class L3Cam

} // namespace l3cam_ros2
