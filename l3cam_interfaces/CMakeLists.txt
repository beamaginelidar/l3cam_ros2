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
  "srv/LibL3camStatus.srv"
  "srv/GetVersion.srv"
  "srv/Initialize.srv"
  "srv/Terminate.srv"
  "srv/FindDevices.srv"
  "srv/GetLocalServerAddress.srv"
  "srv/GetDeviceInfo.srv"
  "srv/GetDeviceStatus.srv"
  "srv/GetSensorsAvailable.srv"
  "srv/ChangeStreamingProtocol.srv"
  "srv/GetRtspPipeline.srv"
  "srv/GetNetworkConfiguration.srv"
  "srv/ChangeNetworkConfiguration.srv"
  "srv/PowerOffDevice.srv"
  "srv/StartDevice.srv"
  "srv/StopDevice.srv"
  "srv/StartStream.srv"
  "srv/StopStream.srv"
  "srv/GetDeviceTemperatures.srv"
  "srv/ChangePointcloudColor.srv"
  "srv/ChangePointcloudColorRange.srv"
  "srv/ChangeDistanceRange.srv"
  "srv/EnableAutoBias.srv"
  "srv/ChangeBiasValue.srv"
  "srv/SetPolarimetricCameraDefaultSettings.srv"
  "srv/ChangePolarimetricCameraBrightness.srv"
  "srv/ChangePolarimetricCameraBlackLevel.srv"
  "srv/EnablePolarimetricCameraAutoGain.srv"
  "srv/ChangePolarimetricCameraAutoGainRange.srv"
  "srv/ChangePolarimetricCameraGain.srv"
  "srv/EnablePolarimetricCameraAutoExposureTime.srv"
  "srv/ChangePolarimetricCameraAutoExposureTimeRange.srv"
  "srv/ChangePolarimetricCameraExposureTime.srv"
  "srv/SetRgbCameraDefaultSettings.srv"
  "srv/ChangeRgbCameraBrightness.srv"
  "srv/ChangeRgbCameraContrast.srv"
  "srv/ChangeRgbCameraSaturation.srv"
  "srv/ChangeRgbCameraSharpness.srv"
  "srv/ChangeRgbCameraGamma.srv"
  "srv/ChangeRgbCameraGain.srv"
  "srv/EnableRgbCameraAutoWhiteBalance.srv"
  "srv/ChangeRgbCameraWhiteBalance.srv"
  "srv/EnableRgbCameraAutoExposureTime.srv"
  "srv/ChangeRgbCameraExposureTime.srv"
  "srv/ChangeThermalCameraColormap.srv"
  "srv/EnableThermalCameraTemperatureFilter.srv"
  "srv/ChangeThermalCameraTemperatureFilter.srv"
  "srv/ChangeThermalCameraPipeline.srv"
  "srv/EnableThermalCameraTemperatureDataUdp.srv"
  "srv/ChangeAlliedCameraExposureTime.srv"
  "srv/EnableAlliedCameraAutoExposureTime.srv"
  "srv/ChangeAlliedCameraAutoExposureTimeRange.srv"
  "srv/ChangeAlliedCameraGain.srv"
  "srv/EnableAlliedCameraAutoGain.srv"
  "srv/ChangeAlliedCameraAutoGainRange.srv"
  "srv/ChangeAlliedCameraGamma.srv"
  "srv/ChangeAlliedCameraSaturation.srv"
  "srv/ChangeAlliedCameraHue.srv"
  "srv/ChangeAlliedCameraIntensityAutoPrecedence.srv"
  "srv/EnableAlliedCameraAutoWhiteBalance.srv"
  "srv/ChangeAlliedCameraBalanceRatioSelector.srv"
  "srv/ChangeAlliedCameraBalanceRatio.srv"
  "srv/ChangeAlliedCameraBalanceWhiteAutoRate.srv"
  "srv/ChangeAlliedCameraBalanceWhiteAutoTolerance.srv"
  "srv/ChangeAlliedCameraIntensityControllerRegion.srv"
  "srv/ChangeAlliedCameraIntensityControllerTarget.srv"
  "srv/GetAlliedCameraBlackLevel.srv"
  "srv/GetAlliedCameraExposureTime.srv"
  "srv/GetAlliedCameraAutoExposureTime.srv"
  "srv/GetAlliedCameraAutoExposureTimeRange.srv"
  "srv/GetAlliedCameraGain.srv"
  "srv/GetAlliedCameraAutoGain.srv"
  "srv/GetAlliedCameraAutoGainRange.srv"
  "srv/GetAlliedCameraGamma.srv"
  "srv/GetAlliedCameraSaturation.srv"
  "srv/GetAlliedCameraSharpness.srv"
  "srv/GetAlliedCameraHue.srv"
  "srv/GetAlliedCameraIntensityAutoPrecedence.srv"
  "srv/GetAlliedCameraAutoWhiteBalance.srv"
  "srv/GetAlliedCameraBalanceRatioSelector.srv"
  "srv/GetAlliedCameraBalanceRatio.srv"
  "srv/GetAlliedCameraBalanceWhiteAutoRate.srv"
  "srv/GetAlliedCameraBalanceWhiteAutoTolerance.srv"
  "srv/GetAlliedCameraAutoModeRegion.srv"
  "srv/GetAlliedCameraIntensityControllerRegion.srv"
  "srv/GetAlliedCameraIntensityControllerTarget.srv"
  "srv/GetAlliedCameraMaxDriverBuffersCount.srv"
  "srv/SensorDisconnected.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sensor.msg"
  ${srv_files}
)

ament_package()
