cmake_minimum_required(VERSION 3.5)
project(l3cam_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/ChangeAlliedCameraAutoExposureTimeRange.srv"
  "srv/ChangeAlliedCameraAutoGainRange.srv"
  "srv/ChangeAlliedCameraBalanceRatioSelector.srv"
  "srv/ChangeAlliedCameraBalanceRatio.srv"
  "srv/ChangeAlliedCameraBalanceWhiteAutoRate.srv"
  "srv/ChangeAlliedCameraBalanceWhiteAutoTolerance.srv"
  "srv/ChangeAlliedCameraExposureTime.srv"
  "srv/ChangeAlliedCameraGain.srv"
  "srv/ChangeAlliedCameraGamma.srv"
  "srv/ChangeAlliedCameraHue.srv"
  "srv/ChangeAlliedCameraIntensityAutoPrecedence.srv"
  "srv/ChangeAlliedCameraIntensityControllerRegion.srv"
  "srv/ChangeAlliedCameraIntensityControllerTarget.srv"
  "srv/ChangeAlliedCameraSaturation.srv"
  "srv/ChangeAutobiasValue.srv"
  "srv/ChangeBiasValue.srv"
  "srv/ChangeDistanceRange.srv"
  "srv/ChangeNetworkConfiguration.srv"
  "srv/ChangePointcloudColorRange.srv"
  "srv/ChangePointcloudColor.srv"
  "srv/ChangePolarimetricCameraAutoExposureTimeRange.srv"
  "srv/ChangePolarimetricCameraAutoGainRange.srv"
  "srv/ChangePolarimetricCameraBlackLevel.srv"
  "srv/ChangePolarimetricCameraBrightness.srv"
  "srv/ChangePolarimetricCameraExposureTime.srv"
  "srv/ChangePolarimetricCameraGain.srv"
  "srv/ChangeRgbCameraBrightness.srv"
  "srv/ChangeRgbCameraContrast.srv"
  "srv/ChangeRgbCameraExposureTime.srv"
  "srv/ChangeRgbCameraGain.srv"
  "srv/ChangeRgbCameraGamma.srv"
  "srv/ChangeRgbCameraSaturation.srv"
  "srv/ChangeRgbCameraSharpness.srv"
  "srv/ChangeRgbCameraWhiteBalance.srv"
  "srv/ChangeStreamingProtocol.srv"
  "srv/ChangeThermalCameraColormap.srv"
  "srv/ChangeThermalCameraProcessingPipeline.srv"
  "srv/ChangeThermalCameraTemperatureFilter.srv"
  "srv/EnableAlliedCameraAutoExposureTime.srv"
  "srv/EnableAlliedCameraAutoGain.srv"
  "srv/EnableAlliedCameraAutoWhiteBalance.srv"
  "srv/EnableAutoBias.srv"
  "srv/EnablePolarimetricCameraAutoExposureTime.srv"
  "srv/EnablePolarimetricCameraAutoGain.srv"
  "srv/EnableRgbCameraAutoExposureTime.srv"
  "srv/EnableRgbCameraAutoWhiteBalance.srv"
  "srv/EnableThermalCameraTemperatureDataUdp.srv"
  "srv/EnableThermalCameraTemperatureFilter.srv"
  "srv/FindDevices.srv"
  "srv/GetAlliedCameraAutoExposureTimeRange.srv"
  "srv/GetAlliedCameraAutoExposureTime.srv"
  "srv/GetAlliedCameraAutoGainRange.srv"
  "srv/GetAlliedCameraAutoGain.srv"
  "srv/GetAlliedCameraAutoModeRegion.srv"
  "srv/GetAlliedCameraAutoWhiteBalance.srv"
  "srv/GetAlliedCameraBalanceRatioSelector.srv"
  "srv/GetAlliedCameraBalanceRatio.srv"
  "srv/GetAlliedCameraBalanceWhiteAutoRate.srv"
  "srv/GetAlliedCameraBalanceWhiteAutoTolerance.srv"
  "srv/GetAlliedCameraBlackLevel.srv"
  "srv/GetAlliedCameraExposureTime.srv"
  "srv/GetAlliedCameraGain.srv"
  "srv/GetAlliedCameraGamma.srv"
  "srv/GetAlliedCameraHue.srv"
  "srv/GetAlliedCameraIntensityAutoPrecedence.srv"
  "srv/GetAlliedCameraIntensityControllerRegion.srv"
  "srv/GetAlliedCameraIntensityControllerTarget.srv"
  "srv/GetAlliedCameraMaxDriverBuffersCount.srv"
  "srv/GetAlliedCameraSaturation.srv"
  "srv/GetAlliedCameraSharpness.srv"
  "srv/GetAutobiasValue.srv"
  "srv/GetDeviceInfo.srv"
  "srv/GetDeviceStatus.srv"
  "srv/GetDeviceTemperatures.srv"
  "srv/GetLocalServerAddress.srv"
  "srv/GetNetworkConfiguration.srv"
  "srv/GetRtspPipeline.srv"
  "srv/GetSensorsAvailable.srv"
  "srv/GetVersion.srv"
  "srv/Initialize.srv"
  "srv/LibL3camStatus.srv"
  "srv/PowerOffDevice.srv"
  "srv/SensorDisconnected.srv"
  "srv/SetBiasShortRange.srv"
  "srv/SetPolarimetricCameraDefaultSettings.srv"
  "srv/SetRgbCameraDefaultSettings.srv"
  "srv/StartDevice.srv"
  "srv/StartStream.srv"
  "srv/StopDevice.srv"
  "srv/StopStream.srv"
  "srv/Terminate.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sensor.msg"
  ${srv_files}
)

ament_package()
