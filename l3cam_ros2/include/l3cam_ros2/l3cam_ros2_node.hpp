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
#include <l3cam_ros2_utils.hpp>

#include "l3cam_interfaces/srv/lib_l3cam_status.hpp"
#include "l3cam_interfaces/srv/get_version.hpp"
#include "l3cam_interfaces/srv/initialize.hpp"
#include "l3cam_interfaces/srv/terminate.hpp"
#include "l3cam_interfaces/srv/find_devices.hpp"
#include "l3cam_interfaces/srv/get_local_server_address.hpp"
#include "l3cam_interfaces/srv/get_device_info.hpp"
#include "l3cam_interfaces/srv/get_device_status.hpp"
#include "l3cam_interfaces/srv/get_sensors_available.hpp"
#include "l3cam_interfaces/srv/change_streaming_protocol.hpp"
#include "l3cam_interfaces/srv/get_rtsp_pipeline.hpp"
#include "l3cam_interfaces/srv/get_network_configuration.hpp"
#include "l3cam_interfaces/srv/change_network_configuration.hpp"
#include "l3cam_interfaces/srv/power_off_device.hpp"
#include "l3cam_interfaces/srv/start_device.hpp"
#include "l3cam_interfaces/srv/stop_device.hpp"
#include "l3cam_interfaces/srv/start_stream.hpp"
#include "l3cam_interfaces/srv/stop_stream.hpp"
#include "l3cam_interfaces/srv/get_device_temperatures.hpp"

#include "l3cam_interfaces/srv/change_pointcloud_color.hpp"
#include "l3cam_interfaces/srv/change_pointcloud_color_range.hpp"
#include "l3cam_interfaces/srv/change_distance_range.hpp"
#include "l3cam_interfaces/srv/set_bias_short_range.hpp"
#include "l3cam_interfaces/srv/enable_auto_bias.hpp"
#include "l3cam_interfaces/srv/change_bias_value.hpp"
#include "l3cam_interfaces/srv/change_autobias_value.hpp"
#include "l3cam_interfaces/srv/get_autobias_value.hpp"

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
#include "l3cam_interfaces/srv/enable_thermal_camera_temperature_filter.hpp"
#include "l3cam_interfaces/srv/change_thermal_camera_temperature_filter.hpp"
#include "l3cam_interfaces/srv/change_thermal_camera_processing_pipeline.hpp"
#include "l3cam_interfaces/srv/enable_thermal_camera_temperature_data_udp.hpp"

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

namespace l3cam_ros2
{

    class L3Cam : public rclcpp::Node
    {
    public:
        explicit L3Cam();

        int initializeDevice();
        int startDeviceStream();
        void disconnectAll(int code);

        l3cam m_devices[1];
        LibL3CamStatus m_status;

    private:
        void declareParameters();
        void declareNetworkParameters();
        void declareLidarParameters();
        void declarePolarimetricParameters();
        void declareRgbParameters();
        void declareThermalParameters();
        void declareAlliedWideParameters();
        void declareAlliedNarrowParameters();

        void initializeServices();
        void initializeLidarServices();
        void initializePolarimetricServices();
        void initializeRgbServices();
        void initializeThermalServices();
        void initializeAlliedWideServices();
        void initializeAlliedNarrowServices();

        inline void printDefaultError(int error, std::string param);

        void loadDefaultParams();
        void loadNetworkDefaultParams();
        void loadLidarDefaultParams();
        void loadPolarimetricDefaultParams();
        void loadRgbDefaultParams();
        void loadThermalDefaultParams();
        void loadAlliedWideDefaultParams();
        void loadAlliedNarrowDefaultParams();

        void libL3camStatus(const std::shared_ptr<l3cam_interfaces::srv::LibL3camStatus::Request> req, std::shared_ptr<l3cam_interfaces::srv::LibL3camStatus::Response> res);
        void getVersion(const std::shared_ptr<l3cam_interfaces::srv::GetVersion::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetVersion::Response> res);
        void initialize(const std::shared_ptr<l3cam_interfaces::srv::Initialize::Request> req, std::shared_ptr<l3cam_interfaces::srv::Initialize::Response> res);
        void terminate(const std::shared_ptr<l3cam_interfaces::srv::Terminate::Request> req, std::shared_ptr<l3cam_interfaces::srv::Terminate::Response> res);
        void findDevices(const std::shared_ptr<l3cam_interfaces::srv::FindDevices::Request> req, std::shared_ptr<l3cam_interfaces::srv::FindDevices::Response> res);
        void getLocalServerAddress(const std::shared_ptr<l3cam_interfaces::srv::GetLocalServerAddress::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetLocalServerAddress::Response> res);
        void getDeviceInfo(const std::shared_ptr<l3cam_interfaces::srv::GetDeviceInfo::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetDeviceInfo::Response> res);
        void getDeviceStatus(const std::shared_ptr<l3cam_interfaces::srv::GetDeviceStatus::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetDeviceStatus::Response> res);
        void getSensorsAvailable(const std::shared_ptr<l3cam_interfaces::srv::GetSensorsAvailable::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetSensorsAvailable::Response> res);
        void changeStreamingProtocol(const std::shared_ptr<l3cam_interfaces::srv::ChangeStreamingProtocol::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeStreamingProtocol::Response> res);
        void getRtspPipeline(const std::shared_ptr<l3cam_interfaces::srv::GetRtspPipeline::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetRtspPipeline::Response> res);
        void getNetworkConfiguration(const std::shared_ptr<l3cam_interfaces::srv::GetNetworkConfiguration::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetNetworkConfiguration::Response> res);
        void changeNetworkConfiguration(const std::shared_ptr<l3cam_interfaces::srv::ChangeNetworkConfiguration::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeNetworkConfiguration::Response> res);
        void powerOffDevice(const std::shared_ptr<l3cam_interfaces::srv::PowerOffDevice::Request> req, std::shared_ptr<l3cam_interfaces::srv::PowerOffDevice::Response> res);
        void startDevice(const std::shared_ptr<l3cam_interfaces::srv::StartDevice::Request> req, std::shared_ptr<l3cam_interfaces::srv::StartDevice::Response> res);
        void stopDevice(const std::shared_ptr<l3cam_interfaces::srv::StopDevice::Request> req, std::shared_ptr<l3cam_interfaces::srv::StopDevice::Response> res);
        void startStream(const std::shared_ptr<l3cam_interfaces::srv::StartStream::Request> req, std::shared_ptr<l3cam_interfaces::srv::StartStream::Response> res);
        void stopStream(const std::shared_ptr<l3cam_interfaces::srv::StopStream::Request> req, std::shared_ptr<l3cam_interfaces::srv::StopStream::Response> res);
        void getDeviceTemperatures(const std::shared_ptr<l3cam_interfaces::srv::GetDeviceTemperatures::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetDeviceTemperatures::Response> res);
        void changePointcloudColor(const std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColor::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColor::Response> res);
        void changePointcloudColorRange(const std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColorRange::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColorRange::Response> res);
        void changeDistanceRange(const std::shared_ptr<l3cam_interfaces::srv::ChangeDistanceRange::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeDistanceRange::Response> res);
        void setBiasShortRange(const std::shared_ptr<l3cam_interfaces::srv::SetBiasShortRange::Request> req, std::shared_ptr<l3cam_interfaces::srv::SetBiasShortRange::Response> res);
        void enableAutoBias(const std::shared_ptr<l3cam_interfaces::srv::EnableAutoBias::Request> req, std::shared_ptr<l3cam_interfaces::srv::EnableAutoBias::Response> res);
        void changeBiasValue(const std::shared_ptr<l3cam_interfaces::srv::ChangeBiasValue::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeBiasValue::Response> res);
        void changeAutobiasValue(const std::shared_ptr<l3cam_interfaces::srv::ChangeAutobiasValue::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeAutobiasValue::Response> res);
        void getAutobiasValue(const std::shared_ptr<l3cam_interfaces::srv::GetAutobiasValue::Request> req, std::shared_ptr<l3cam_interfaces::srv::GetAutobiasValue::Response> res);
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
        void enableThermalCameraTemperatureFilter(const std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter::Request> req, std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter::Response> res);
        void changeThermalCameraTemperatureFilter(const std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter::Response> res);
        void changeThermalCameraProcessingPipeline(const std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraProcessingPipeline::Request> req, std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraProcessingPipeline::Response> res);
        void enableThermalCameraTemperatureDataUdp(const std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureDataUdp::Request> req, std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureDataUdp::Response> res);
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

        void networkDisconnected(int code);
        void lidarDisconnected(int code);
        void polDisconnected(int code);
        void rgbDisconnected(int code);
        void thermalDisconnected(int code);
        void alliedwideDisconnected(int code);
        void alliedNarrowDisconnect(int code);

        static void errorNotification(const int32_t *error);

        rclcpp::Service<l3cam_interfaces::srv::LibL3camStatus>::SharedPtr srv_libl3cam_status_;
        rclcpp::Service<l3cam_interfaces::srv::GetVersion>::SharedPtr srv_get_version_;
        rclcpp::Service<l3cam_interfaces::srv::Initialize>::SharedPtr srv_initialize_;
        rclcpp::Service<l3cam_interfaces::srv::Terminate>::SharedPtr srv_terminate_;
        rclcpp::Service<l3cam_interfaces::srv::FindDevices>::SharedPtr srv_find_devices_;
        rclcpp::Service<l3cam_interfaces::srv::GetLocalServerAddress>::SharedPtr srv_get_local_server_address_;
        rclcpp::Service<l3cam_interfaces::srv::GetDeviceInfo>::SharedPtr srv_get_device_info_;
        rclcpp::Service<l3cam_interfaces::srv::GetDeviceStatus>::SharedPtr srv_get_device_status_;
        rclcpp::Service<l3cam_interfaces::srv::GetSensorsAvailable>::SharedPtr srv_get_sensors_available_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeStreamingProtocol>::SharedPtr srv_change_streaming_protocol_;
        rclcpp::Service<l3cam_interfaces::srv::GetRtspPipeline>::SharedPtr srv_get_rtsp_pipeline_;
        rclcpp::Service<l3cam_interfaces::srv::GetNetworkConfiguration>::SharedPtr srv_get_network_configuration_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeNetworkConfiguration>::SharedPtr srv_change_network_configuration_;
        rclcpp::Service<l3cam_interfaces::srv::PowerOffDevice>::SharedPtr srv_power_off_device_;
        rclcpp::Service<l3cam_interfaces::srv::StartDevice>::SharedPtr srv_start_device_;
        rclcpp::Service<l3cam_interfaces::srv::StopDevice>::SharedPtr srv_stop_device_;
        rclcpp::Service<l3cam_interfaces::srv::StartStream>::SharedPtr srv_start_stream_;
        rclcpp::Service<l3cam_interfaces::srv::StopStream>::SharedPtr srv_stop_stream_;
        rclcpp::Service<l3cam_interfaces::srv::GetDeviceTemperatures>::SharedPtr srv_get_device_temperatures_;

        rclcpp::Service<l3cam_interfaces::srv::ChangePointcloudColor>::SharedPtr srv_change_pointcloud_color_;
        rclcpp::Service<l3cam_interfaces::srv::ChangePointcloudColorRange>::SharedPtr srv_change_pointcloud_color_range_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeDistanceRange>::SharedPtr srv_change_distance_range_;
        rclcpp::Service<l3cam_interfaces::srv::SetBiasShortRange>::SharedPtr srv_set_bias_short_range_;
        rclcpp::Service<l3cam_interfaces::srv::EnableAutoBias>::SharedPtr srv_enable_auto_bias_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeBiasValue>::SharedPtr srv_change_bias_value_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAutobiasValue>::SharedPtr srv_change_autobias_value_;
        rclcpp::Service<l3cam_interfaces::srv::GetAutobiasValue>::SharedPtr srv_get_autobias_value_;

        rclcpp::Service<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings>::SharedPtr srv_set_polarimetric_camera_default_settings_;
        rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness>::SharedPtr srv_change_polarimetric_camera_brightness_;
        rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel>::SharedPtr srv_change_polarimetric_camera_black_level_;
        rclcpp::Service<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain>::SharedPtr srv_enable_polarimetric_camera_auto_gain_;
        rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange>::SharedPtr srv_change_polarimetric_camera_auto_gain_range_;
        rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraGain>::SharedPtr srv_change_polarimetric_camera_gain_;
        rclcpp::Service<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime>::SharedPtr srv_enable_polarimetric_camera_auto_exposure_time_;
        rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange>::SharedPtr srv_change_polarimetric_camera_auto_exposure_time_range_;
        rclcpp::Service<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime>::SharedPtr srv_change_polarimetric_camera_exposure_time_;

        rclcpp::Service<l3cam_interfaces::srv::SetRgbCameraDefaultSettings>::SharedPtr srv_set_rgb_camera_default_settings_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraBrightness>::SharedPtr srv_change_rgb_camera_brightness_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraContrast>::SharedPtr srv_change_rgb_camera_contrast_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraSaturation>::SharedPtr srv_change_rgb_camera_saturation_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraSharpness>::SharedPtr srv_change_rgb_camera_sharpness_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraGamma>::SharedPtr srv_change_rgb_camera_gamma_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraGain>::SharedPtr srv_change_rgb_camera_gain_;
        rclcpp::Service<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance>::SharedPtr srv_enable_rgb_camera_auto_white_balance_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance>::SharedPtr srv_change_rgb_camera_white_balance_;
        rclcpp::Service<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime>::SharedPtr srv_enable_rgb_camera_auto_exposure_time_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeRgbCameraExposureTime>::SharedPtr srv_change_rgb_camera_exposure_time_;

        rclcpp::Service<l3cam_interfaces::srv::ChangeThermalCameraColormap>::SharedPtr srv_change_thermal_camera_colormap_;
        rclcpp::Service<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter>::SharedPtr srv_enable_thermal_camera_temperature_filter_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter>::SharedPtr srv_change_thermal_camera_temperature_filter_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeThermalCameraProcessingPipeline>::SharedPtr srv_change_thermal_camera_processing_pipeline_;
        rclcpp::Service<l3cam_interfaces::srv::EnableThermalCameraTemperatureDataUdp>::SharedPtr srv_enable_thermal_camera_temperature_data_udp_;

        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime>::SharedPtr srv_change_allied_camera_exposure_time_;
        rclcpp::Service<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime>::SharedPtr srv_enable_allied_camera_auto_exposure_time_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange>::SharedPtr srv_change_allied_camera_auto_exposure_time_range_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraGain>::SharedPtr srv_change_allied_camera_gain_;
        rclcpp::Service<l3cam_interfaces::srv::EnableAlliedCameraAutoGain>::SharedPtr srv_enable_allied_camera_auto_gain_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange>::SharedPtr srv_change_allied_camera_auto_gain_range_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraGamma>::SharedPtr srv_change_allied_camera_gamma_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraSaturation>::SharedPtr srv_change_allied_camera_saturation_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraHue>::SharedPtr srv_change_allied_camera_hue_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence>::SharedPtr srv_change_allied_camera_intensity_auto_precedence_;
        rclcpp::Service<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance>::SharedPtr srv_enable_allied_camera_auto_white_balance_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector>::SharedPtr srv_change_allied_camera_balance_ratio_selector_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio>::SharedPtr srv_change_allied_camera_balance_ratio_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate>::SharedPtr srv_change_allied_camera_balance_white_auto_rate_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance>::SharedPtr srv_change_allied_camera_balance_white_auto_tolerance_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion>::SharedPtr srv_change_allied_camera_intensity_controller_region_;
        rclcpp::Service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget>::SharedPtr srv_change_allied_camera_intensity_controller_target_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBlackLevel>::SharedPtr srv_get_allied_camera_black_level_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraExposureTime>::SharedPtr srv_get_allied_camera_exposure_time_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTime>::SharedPtr srv_get_allied_camera_auto_exposure_time_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTimeRange>::SharedPtr srv_get_allied_camera_auto_exposure_time_range_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraGain>::SharedPtr srv_get_allied_camera_gain_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoGain>::SharedPtr srv_get_allied_camera_auto_gain_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoGainRange>::SharedPtr srv_get_allied_camera_auto_gain_range_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraGamma>::SharedPtr srv_get_allied_camera_gamma_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraSaturation>::SharedPtr srv_get_allied_camera_saturation_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraSharpness>::SharedPtr srv_get_allied_camera_sharpness_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraHue>::SharedPtr srv_get_allied_camera_hue_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraIntensityAutoPrecedence>::SharedPtr srv_get_allied_camera_intensity_auto_precedence_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoWhiteBalance>::SharedPtr srv_get_allied_camera_auto_white_balance_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBalanceRatioSelector>::SharedPtr srv_get_allied_camera_balance_ratio_selector_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBalanceRatio>::SharedPtr srv_get_allied_camera_balance_ratio_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoRate>::SharedPtr srv_get_allied_camera_balance_white_auto_rate_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoTolerance>::SharedPtr srv_get_allied_camera_balance_white_auto_tolerance_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraAutoModeRegion>::SharedPtr srv_get_allied_camera_auto_mode_region_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerRegion>::SharedPtr srv_get_allied_camera_intensity_controller_region_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerTarget>::SharedPtr srv_get_allied_camera_intensity_controller_target_;
        rclcpp::Service<l3cam_interfaces::srv::GetAlliedCameraMaxDriverBuffersCount>::SharedPtr srv_get_allied_camera_max_driver_buffers_count_;

        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr client_network_disconnected_;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr client_lidar_stream_disconnected_;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr client_lidar_configuration_disconnected_;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr client_pol_wide_stream_disconnected_;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr client_pol_configuration_disconnected_;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr client_rgb_narrow_stream_disconnected_;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr client_rgb_configuration_disconnected_;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr client_thermal_stream_disconnected_;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr client_thermal_configuration_disconnected_;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr client_wide_configuration_disconnected_;
        rclcpp::Client<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr client_narrow_configuration_disconnected_;

        sensor m_av_sensors[6];

        sensor *m_lidar_sensor = NULL;
        sensor *m_rgb_sensor = NULL;
        sensor *m_econ_wide_sensor = NULL;
        sensor *m_thermal_sensor = NULL;
        sensor *m_polarimetric_sensor = NULL;
        sensor *m_allied_wide_sensor = NULL;
        sensor *m_allied_narrow_sensor = NULL;

        int m_num_devices;

    }; // class L3Cam

} // namespace l3cam_ros2
