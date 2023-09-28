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

#include "l3cam_ros2_node.hpp"

#include <libL3Cam.h>
#include <libL3Cam_allied.h>
#include <libL3Cam_econ.h>
#include <libL3Cam_polarimetric.h>
#include <libL3Cam_thermal.h>
#include <beamErrors.h>

#include "l3cam_interfaces/msg/sensor.hpp"

using namespace std::chrono_literals;

namespace l3cam_ros2
{
    std::shared_ptr<L3Cam> node;

    L3Cam::L3Cam() : Node("l3cam_ros2_node")
    {
        // Register callback for sensor disconnection errors
        registerErrorCallback(errorNotification);

        declareParameters();
        initializeServices();

        timer_ = this->create_wall_timer(500ms, std::bind(&L3Cam::timer_callback, this));
    }

    int L3Cam::initializeDevice()
    {
        // Initialize L3Cam
        int error = L3CAM_OK;

        std::string local_address = this->get_parameter("local_address").as_string();
        std::string device_address = this->get_parameter("device_address").as_string();
        error = INITIALIZE((local_address == "") ? NULL : &local_address[0], (device_address == "") ? NULL : &device_address[0]);
        if (error)
            return error;

        num_devices = 0;
        while (num_devices == 0)
        {
            error = FIND_DEVICES(devices, &num_devices);
        }
        if (error)
            return error;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Device found " << std::string(devices[0].ip_address)
                                                                         << ", model " << (int)devices[0].model
                                                                         << ", serial number " << std::string(devices[0].serial_number));

        int status = 0;
        error = GET_DEVICE_STATUS(devices[0], &status);
        if (error)
            return error;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Device status " << status);

        int num_sensors = 0;
        error = GET_SENSORS_AVAILABLE(devices[0], av_sensors, &num_sensors);
        if (error)
            return error;

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

        return L3CAM_OK;
    }

    int L3Cam::startDeviceStream()
    {
        int error = L3CAM_OK;

        error = START_DEVICE(devices[0]);
        if (error)
            return error;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Device started");

        loadDefaultParams();

        error = START_STREAM(devices[0]);
        if (error)
            return error;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Device streaming ready\n");

        return L3CAM_OK;
    }

    void L3Cam::declareParameters()
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
        this->declare_parameter("allied_wide_camera_auto_mode_region_height", 1028, descriptor); // 0 - 1028
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

    void L3Cam::initializeServices()
    {
        // Initialize services
        srvGetVersion = this->create_service<l3cam_interfaces::srv::GetVersion>(
            "get_version",
            std::bind(&L3Cam::getVersion, this, std::placeholders::_1, std::placeholders::_2));
        srvInitialize = this->create_service<l3cam_interfaces::srv::Initialize>(
            "initialize",
            std::bind(&L3Cam::initialize, this, std::placeholders::_1, std::placeholders::_2));
        srvTerminate = this->create_service<l3cam_interfaces::srv::Terminate>(
            "terminate",
            std::bind(&L3Cam::terminate, this, std::placeholders::_1, std::placeholders::_2));
        srvFindDevices = this->create_service<l3cam_interfaces::srv::FindDevices>(
            "find_devices",
            std::bind(&L3Cam::findDevices, this, std::placeholders::_1, std::placeholders::_2));
        srvGetLocalServerAddress = this->create_service<l3cam_interfaces::srv::GetLocalServerAddress>(
            "get_local_server_address",
            std::bind(&L3Cam::getLocalServerAddress, this, std::placeholders::_1, std::placeholders::_2));
        srvGetDeviceStatus = this->create_service<l3cam_interfaces::srv::GetDeviceStatus>(
            "get_device_status",
            std::bind(&L3Cam::getDeviceStatus, this, std::placeholders::_1, std::placeholders::_2));
        srvGetSensorsAvailable = this->create_service<l3cam_interfaces::srv::GetSensorsAvailable>(
            "get_sensors_available",
            std::bind(&L3Cam::getSensorsAvailable, this, std::placeholders::_1, std::placeholders::_2));
        srvGetNetworkConfiguration = this->create_service<l3cam_interfaces::srv::GetNetworkConfiguration>(
            "get_network_configuration",
            std::bind(&L3Cam::getNetworkConfiguration, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeNetworkConfiguration = this->create_service<l3cam_interfaces::srv::ChangeNetworkConfiguration>(
            "change_network_configuration",
            std::bind(&L3Cam::changeNetworkConfiguration, this, std::placeholders::_1, std::placeholders::_2));
        srvPowerOffDevice = this->create_service<l3cam_interfaces::srv::PowerOffDevice>(
            "power_off_device",
            std::bind(&L3Cam::powerOffDevice, this, std::placeholders::_1, std::placeholders::_2));
        srvStartDevice = this->create_service<l3cam_interfaces::srv::StartDevice>(
            "start_device",
            std::bind(&L3Cam::startDevice, this, std::placeholders::_1, std::placeholders::_2));
        srvStopDevice = this->create_service<l3cam_interfaces::srv::StopDevice>(
            "stop_device",
            std::bind(&L3Cam::stopDevice, this, std::placeholders::_1, std::placeholders::_2));
        srvStartStream = this->create_service<l3cam_interfaces::srv::StartStream>(
            "start_stream",
            std::bind(&L3Cam::startStream, this, std::placeholders::_1, std::placeholders::_2));
        srvStopStream = this->create_service<l3cam_interfaces::srv::StopStream>(
            "stop_stream",
            std::bind(&L3Cam::stopStream, this, std::placeholders::_1, std::placeholders::_2));

        srvChangePointcloudColor = this->create_service<l3cam_interfaces::srv::ChangePointcloudColor>(
            "change_pointcloud_color",
            std::bind(&L3Cam::changePointcloudColor, this, std::placeholders::_1, std::placeholders::_2));
        srvChangePointcloudColorRange = this->create_service<l3cam_interfaces::srv::ChangePointcloudColorRange>(
            "change_pointcloud_color_range",
            std::bind(&L3Cam::changePointcloudColorRange, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeDistanceRange = this->create_service<l3cam_interfaces::srv::ChangeDistanceRange>(
            "change_distance_range",
            std::bind(&L3Cam::changeDistanceRange, this, std::placeholders::_1, std::placeholders::_2));
        srvEnableAutoBias = this->create_service<l3cam_interfaces::srv::EnableAutoBias>(
            "enable_auto_bias",
            std::bind(&L3Cam::enableAutoBias, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeBiasValue = this->create_service<l3cam_interfaces::srv::ChangeBiasValue>(
            "change_bias_value",
            std::bind(&L3Cam::changeBiasValue, this, std::placeholders::_1, std::placeholders::_2));

        srvSetPolarimetricCameraDefaultSettings = this->create_service<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings>(
            "set_polarimetric_camera_default_settings",
            std::bind(&L3Cam::setPolarimetricCameraDefaultSettings, this, std::placeholders::_1, std::placeholders::_2));
        srvChangePolarimetricCameraBrightness = this->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness>(
            "change_polarimetric_camera_brightness",
            std::bind(&L3Cam::changePolarimetricCameraBrightness, this, std::placeholders::_1, std::placeholders::_2));
        srvChangePolarimetricCameraBlackLevel = this->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel>(
            "change_polarimetric_camera_black_level",
            std::bind(&L3Cam::changePolarimetricCameraBlackLevel, this, std::placeholders::_1, std::placeholders::_2));
        srvEnablePolarimetricCameraAutoGain = this->create_service<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain>(
            "enable_polarimetric_camera_auto_gain",
            std::bind(&L3Cam::enablePolarimetricCameraAutoGain, this, std::placeholders::_1, std::placeholders::_2));
        srvChangePolarimetricCameraAutoGainRange = this->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange>(
            "change_polarimetric_camera_auto_gain_range",
            std::bind(&L3Cam::changePolarimetricCameraAutoGainRange, this, std::placeholders::_1, std::placeholders::_2));
        srvChangePolarimetricCameraGain = this->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraGain>(
            "change_polarimetric_camera_gain",
            std::bind(&L3Cam::changePolarimetricCameraGain, this, std::placeholders::_1, std::placeholders::_2));
        srvEnablePolarimetricCameraAutoExposureTime = this->create_service<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime>(
            "enable_polarimetric_camera_auto_exposure_time",
            std::bind(&L3Cam::enablePolarimetricCameraAutoExposureTime, this, std::placeholders::_1, std::placeholders::_2));
        srvChangePolarimetricCameraAutoExposureTimeRange = this->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange>(
            "change_polarimetric_camera_auto_exposure_time_range",
            std::bind(&L3Cam::changePolarimetricCameraAutoExposureTimeRange, this, std::placeholders::_1, std::placeholders::_2));
        srvChangePolarimetricCameraExposureTime = this->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime>(
            "change_polarimetric_camera_exposure_time",
            std::bind(&L3Cam::changePolarimetricCameraExposureTime, this, std::placeholders::_1, std::placeholders::_2));

        srvSetRgbCameraDefaultSettings = this->create_service<l3cam_interfaces::srv::SetRgbCameraDefaultSettings>(
            "set_rgb_camera_default_settings",
            std::bind(&L3Cam::setRgbCameraDefaultSettings, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeRgbCameraBrightness = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraBrightness>(
            "change_rgb_camera_brightness",
            std::bind(&L3Cam::changeRgbCameraBrightness, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeRgbCameraContrast = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraContrast>(
            "change_rgb_camera_contrast",
            std::bind(&L3Cam::changeRgbCameraContrast, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeRgbCameraSaturation = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraSaturation>(
            "change_rgb_camera_saturation",
            std::bind(&L3Cam::changeRgbCameraSaturation, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeRgbCameraSharpness = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraSharpness>(
            "change_rgb_camera_sharpness",
            std::bind(&L3Cam::changeRgbCameraSharpness, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeRgbCameraGamma = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraGamma>(
            "change_rgb_camera_gamma",
            std::bind(&L3Cam::changeRgbCameraGamma, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeRgbCameraGain = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraGain>(
            "change_rgb_camera_gain",
            std::bind(&L3Cam::changeRgbCameraGain, this, std::placeholders::_1, std::placeholders::_2));
        srvEnableRgbCameraAutoWhiteBalance = this->create_service<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance>(
            "enable_rgb_camera_auto_white_balance",
            std::bind(&L3Cam::enableRgbCameraAutoWhiteBalance, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeRgbCameraWhiteBalance = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance>(
            "change_rgb_camera_white_balance",
            std::bind(&L3Cam::changeRgbCameraWhiteBalance, this, std::placeholders::_1, std::placeholders::_2));
        srvEnableRgbCameraAutoExposureTime = this->create_service<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime>(
            "enable_rgb_camera_auto_exposure_time",
            std::bind(&L3Cam::enableRgbCameraAutoExposureTime, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeRgbCameraExposureTime = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraExposureTime>(
            "change_rgb_camera_exposure_time",
            std::bind(&L3Cam::changeRgbCameraExposureTime, this, std::placeholders::_1, std::placeholders::_2));

        srvChangeThermalCameraColormap = this->create_service<l3cam_interfaces::srv::ChangeThermalCameraColormap>(
            "change_thermal_camera_colormap",
            std::bind(&L3Cam::changeThermalCameraColormap, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeThermalCameraTemperatureFilter = this->create_service<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter>(
            "change_thermal_camera_temperature_filter",
            std::bind(&L3Cam::changeThermalCameraTemperatureFilter, this, std::placeholders::_1, std::placeholders::_2));
        srvEnableThermalCameraTemperatureFilter = this->create_service<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter>(
            "enable_thermal_camera_temperature_filter",
            std::bind(&L3Cam::enableThermalCameraTemperatureFilter, this, std::placeholders::_1, std::placeholders::_2));

        srvChangeAlliedCameraExposureTime = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime>(
            "change_allied_camera_exposure_time",
            std::bind(&L3Cam::changeAlliedCameraExposureTime, this, std::placeholders::_1, std::placeholders::_2));
        srvEnableAlliedCameraAutoExposureTime = this->create_service<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime>(
            "enable_allied_camera_auto_exposure_time",
            std::bind(&L3Cam::enableAlliedCameraAutoExposureTime, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeAlliedCameraAutoExposureTimeRange = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange>(
            "change_allied_camera_auto_exposure_time_range",
            std::bind(&L3Cam::changeAlliedCameraAutoExposureTimeRange, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeAlliedCameraGain = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraGain>(
            "change_allied_camera_gain",
            std::bind(&L3Cam::changeAlliedCameraGain, this, std::placeholders::_1, std::placeholders::_2));
        srvEnableAlliedCameraAutoGain = this->create_service<l3cam_interfaces::srv::EnableAlliedCameraAutoGain>(
            "enable_allied_camera_auto_gain",
            std::bind(&L3Cam::enableAlliedCameraAutoGain, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeAlliedCameraAutoGainRange = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange>(
            "change_allied_camera_auto_gain_range",
            std::bind(&L3Cam::changeAlliedCameraAutoGainRange, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeAlliedCameraGamma = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraGamma>(
            "change_allied_camera_gamma",
            std::bind(&L3Cam::changeAlliedCameraGamma, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeAlliedCameraSaturation = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraSaturation>(
            "change_allied_camera_saturation",
            std::bind(&L3Cam::changeAlliedCameraSaturation, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeAlliedCameraHue = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraHue>(
            "change_allied_camera_hue",
            std::bind(&L3Cam::changeAlliedCameraHue, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeAlliedCameraIntensityAutoPrecedence = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence>(
            "change_allied_camera_intensity_auto_precedence",
            std::bind(&L3Cam::changeAlliedCameraIntensityAutoPrecedence, this, std::placeholders::_1, std::placeholders::_2));
        srvEnableAlliedCameraAutoWhiteBalance = this->create_service<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance>(
            "enable_allied_camera_auto_white_balance",
            std::bind(&L3Cam::enableAlliedCameraAutoWhiteBalance, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeAlliedCameraBalanceRatioSelector = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector>(
            "change_allied_camera_balance_ratio_selector",
            std::bind(&L3Cam::changeAlliedCameraBalanceRatioSelector, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeAlliedCameraBalanceRatio = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio>(
            "change_allied_camera_balance_ratio",
            std::bind(&L3Cam::changeAlliedCameraBalanceRatio, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeAlliedCameraBalanceWhiteAutoRate = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate>(
            "change_allied_camera_balance_white_auto_rate",
            std::bind(&L3Cam::changeAlliedCameraBalanceWhiteAutoRate, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeAlliedCameraBalanceWhiteAutoTolerance = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance>(
            "change_allied_camera_balance_white_auto_tolerance",
            std::bind(&L3Cam::changeAlliedCameraBalanceWhiteAutoTolerance, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeAlliedCameraIntensityControllerRegion = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion>(
            "change_allied_camera_intensity_controller_region",
            std::bind(&L3Cam::changeAlliedCameraIntensityControllerRegion, this, std::placeholders::_1, std::placeholders::_2));
        srvChangeAlliedCameraIntensityControllerTarget = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget>(
            "change_allied_camera_intensity_controller_target",
            std::bind(&L3Cam::changeAlliedCameraIntensityControllerTarget, this, std::placeholders::_1, std::placeholders::_2));

        srvGetAlliedCameraBlackLevel = this->create_service<l3cam_interfaces::srv::GetAlliedCameraBlackLevel>(
            "get_allied_camera_black_level",
            std::bind(&L3Cam::getAlliedCameraBlackLevel, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraExposureTime = this->create_service<l3cam_interfaces::srv::GetAlliedCameraExposureTime>(
            "get_allied_camera_exposure_time",
            std::bind(&L3Cam::getAlliedCameraExposureTime, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraAutoExposureTime = this->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTime>(
            "get_allied_camera_auto_exposure_time",
            std::bind(&L3Cam::getAlliedCameraAutoExposureTime, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraAutoExposureTimeRange = this->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTimeRange>(
            "get_allied_camera_auto_exposure_time_range",
            std::bind(&L3Cam::getAlliedCameraAutoExposureTimeRange, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraGain = this->create_service<l3cam_interfaces::srv::GetAlliedCameraGain>(
            "get_allied_camera_gain",
            std::bind(&L3Cam::getAlliedCameraGain, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraAutoGain = this->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoGain>(
            "get_allied_camera_auto_gain",
            std::bind(&L3Cam::getAlliedCameraAutoGain, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraAutoGainRange = this->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoGainRange>(
            "get_allied_camera_auto_gain_range",
            std::bind(&L3Cam::getAlliedCameraAutoGainRange, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraGamma = this->create_service<l3cam_interfaces::srv::GetAlliedCameraGamma>(
            "get_allied_camera_gamma",
            std::bind(&L3Cam::getAlliedCameraGamma, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraSaturation = this->create_service<l3cam_interfaces::srv::GetAlliedCameraSaturation>(
            "get_allied_camera_saturation",
            std::bind(&L3Cam::getAlliedCameraSaturation, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraSharpness = this->create_service<l3cam_interfaces::srv::GetAlliedCameraSharpness>(
            "get_allied_camera_sharpness",
            std::bind(&L3Cam::getAlliedCameraSharpness, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraHue = this->create_service<l3cam_interfaces::srv::GetAlliedCameraHue>(
            "get_allied_camera_hue",
            std::bind(&L3Cam::getAlliedCameraHue, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraIntensityAutoPrecedence = this->create_service<l3cam_interfaces::srv::GetAlliedCameraIntensityAutoPrecedence>(
            "get_allied_camera_intensity_auto_precedence",
            std::bind(&L3Cam::getAlliedCameraIntensityAutoPrecedence, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraAutoWhiteBalance = this->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoWhiteBalance>(
            "get_allied_camera_auto_white_balance",
            std::bind(&L3Cam::getAlliedCameraAutoWhiteBalance, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraBalanceRatioSelector = this->create_service<l3cam_interfaces::srv::GetAlliedCameraBalanceRatioSelector>(
            "get_allied_camera_balance_ratio_selector",
            std::bind(&L3Cam::getAlliedCameraBalanceRatioSelector, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraBalanceRatio = this->create_service<l3cam_interfaces::srv::GetAlliedCameraBalanceRatio>(
            "get_allied_camera_balance_ratio",
            std::bind(&L3Cam::getAlliedCameraBalanceRatio, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraBalanceWhiteAutoRate = this->create_service<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoRate>(
            "get_allied_camera_balance_white_auto_rate",
            std::bind(&L3Cam::getAlliedCameraBalanceWhiteAutoRate, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraBalanceWhiteAutoTolerance = this->create_service<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoTolerance>(
            "get_allied_camera_balance_white_auto_tolerance",
            std::bind(&L3Cam::getAlliedCameraBalanceWhiteAutoTolerance, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraAutoModeRegion = this->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoModeRegion>(
            "get_allied_camera_auto_mode_region",
            std::bind(&L3Cam::getAlliedCameraAutoModeRegion, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraIntensityControllerRegion = this->create_service<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerRegion>(
            "get_allied_camera_intensity_controller_region",
            std::bind(&L3Cam::getAlliedCameraIntensityControllerRegion, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraIntensityControllerTarget = this->create_service<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerTarget>(
            "get_allied_camera_intensity_controller_target",
            std::bind(&L3Cam::getAlliedCameraIntensityControllerTarget, this, std::placeholders::_1, std::placeholders::_2));
        srvGetAlliedCameraMaxDriverBuffersCount = this->create_service<l3cam_interfaces::srv::GetAlliedCameraMaxDriverBuffersCount>(
            "get_allied_camera_max_driver_buffers_count",
            std::bind(&L3Cam::getAlliedCameraMaxDriverBuffersCount, this, std::placeholders::_1, std::placeholders::_2));

        // Disconnection service clients
        clientPointCloudStreamDisconnected = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("pointcloud_stream_disconnected");
        clientPointCloudConfigurationDisconnected = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("pointcloud_configuration_disconnected");
        clientPolWideStreamDisconnected = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("polarimetric_wide_stream_disconnected");
        clientPolConfigurationDisconnected = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("polarimetric_configuration_disconnected");
        clientRgbNarrowStreamDisconnected = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("rgb_narrow_stream_disconnected");
        clientRgbConfigurationDisconnected = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("rgb_configuration_disconnected");
        clientThermalStreamDisconnected = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("thermal_stream_disconnected");
        clientThermalConfigurationDisconnected = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("thermal_configuration_disconnected");
        clientWideConfigurationDisconnected = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("allied_wide_configuration_disconnected");
        clientNarrowConfigurationDisconnected = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("allied_narrow_configuration_disconnected");

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Services ready");
    }

    inline void L3Cam::printDefaultError(int error)
    {
        if (error != L3CAM_OK)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
                                "ERROR " << error << " while setting default parameter: " << getBeamErrorDescription(error));
        }
    }

    void L3Cam::loadDefaultParams()
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

    void L3Cam::timer_callback()
    {
        int error = FIND_DEVICES(devices, &num_devices);
        if (error)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while finding devices: " << getBeamErrorDescription(error));
            STOP_STREAM(devices[0]);
            STOP_DEVICE(devices[0]);
            TERMINATE(devices[0]);
            rclcpp::shutdown();
        }
        else if (num_devices == 0)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Device disconnected");
            STOP_STREAM(devices[0]);
            STOP_DEVICE(devices[0]);
            TERMINATE(devices[0]);
            rclcpp::shutdown();
        }
    }

    // Service callbacks
    void L3Cam::getVersion(const std::shared_ptr<l3cam_interfaces::srv::GetVersion::Request> req,
                           std::shared_ptr<l3cam_interfaces::srv::GetVersion::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->version = GET_VERSION();
    }

    void L3Cam::initialize(const std::shared_ptr<l3cam_interfaces::srv::Initialize::Request> req,
                           std::shared_ptr<l3cam_interfaces::srv::Initialize::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = INITIALIZE(&req->local_address[0], &req->device_address[0]);
    }

    void L3Cam::terminate(const std::shared_ptr<l3cam_interfaces::srv::Terminate::Request> req,
                          std::shared_ptr<l3cam_interfaces::srv::Terminate::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = TERMINATE(devices[0]);
    }

    void L3Cam::findDevices(const std::shared_ptr<l3cam_interfaces::srv::FindDevices::Request> req,
                            std::shared_ptr<l3cam_interfaces::srv::FindDevices::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = FIND_DEVICES(&devices[0], &res->num_devices);
    }

    void L3Cam::getLocalServerAddress(const std::shared_ptr<l3cam_interfaces::srv::GetLocalServerAddress::Request> req,
                                      std::shared_ptr<l3cam_interfaces::srv::GetLocalServerAddress::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->local_ip_address = GET_LOCAL_SERVER_ADDRESS(devices[0]);
    }

    void L3Cam::getDeviceStatus(const std::shared_ptr<l3cam_interfaces::srv::GetDeviceStatus::Request> req,
                                std::shared_ptr<l3cam_interfaces::srv::GetDeviceStatus::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = GET_DEVICE_STATUS(devices[0], &res->system_status);
    }

    void L3Cam::getSensorsAvailable(const std::shared_ptr<l3cam_interfaces::srv::GetSensorsAvailable::Request> req,
                                    std::shared_ptr<l3cam_interfaces::srv::GetSensorsAvailable::Response> res)
    {
        ROS2_BMG_UNUSED(req);
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

    void L3Cam::getNetworkConfiguration(const std::shared_ptr<l3cam_interfaces::srv::GetNetworkConfiguration::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::GetNetworkConfiguration::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        char *ip_address = NULL;
        char *netmask = NULL;
        char *gateway = NULL;
        res->error = GET_NETWORK_CONFIGURATION(devices[0], &ip_address, &netmask, &gateway);
        res->ip_address = ip_address;
        res->netmask = netmask;
        res->gateway = gateway;
    }

    void L3Cam::changeNetworkConfiguration(const std::shared_ptr<l3cam_interfaces::srv::ChangeNetworkConfiguration::Request> req,
                                           std::shared_ptr<l3cam_interfaces::srv::ChangeNetworkConfiguration::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        ROS2_BMG_UNUSED(res);
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

    void L3Cam::powerOffDevice(const std::shared_ptr<l3cam_interfaces::srv::PowerOffDevice::Request> req,
                               std::shared_ptr<l3cam_interfaces::srv::PowerOffDevice::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        POWER_OFF_DEVICE(devices[0]);
        res->error = 0;
    }

    void L3Cam::startDevice(const std::shared_ptr<l3cam_interfaces::srv::StartDevice::Request> req,
                            std::shared_ptr<l3cam_interfaces::srv::StartDevice::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = START_DEVICE(devices[0]);
    }

    void L3Cam::stopDevice(const std::shared_ptr<l3cam_interfaces::srv::StopDevice::Request> req,
                           std::shared_ptr<l3cam_interfaces::srv::StopDevice::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = STOP_DEVICE(devices[0]);
    }

    void L3Cam::startStream(const std::shared_ptr<l3cam_interfaces::srv::StartStream::Request> req,
                            std::shared_ptr<l3cam_interfaces::srv::StartStream::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = START_STREAM(devices[0]);
    }

    void L3Cam::stopStream(const std::shared_ptr<l3cam_interfaces::srv::StopStream::Request> req,
                           std::shared_ptr<l3cam_interfaces::srv::StopStream::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = STOP_STREAM(devices[0]);
    }

    // Point Cloud
    void L3Cam::changePointcloudColor(const std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColor::Request> req,
                                      std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColor::Response> res)
    {
        res->error = CHANGE_POINT_CLOUD_COLOR(devices[0], req->visualization_color);
    }

    void L3Cam::changePointcloudColorRange(const std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColorRange::Request> req,
                                           std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColorRange::Response> res)
    {
        res->error = CHANGE_POINT_CLOUD_COLOR_RANGE(devices[0], req->min_value, req->max_value);
    }

    void L3Cam::changeDistanceRange(const std::shared_ptr<l3cam_interfaces::srv::ChangeDistanceRange::Request> req,
                                    std::shared_ptr<l3cam_interfaces::srv::ChangeDistanceRange::Response> res)
    {
        res->error = CHANGE_DISTANCE_RANGE(devices[0], req->min_value, req->max_value);
    }

    void L3Cam::enableAutoBias(const std::shared_ptr<l3cam_interfaces::srv::EnableAutoBias::Request> req,
                               std::shared_ptr<l3cam_interfaces::srv::EnableAutoBias::Response> res)
    {
        ROS2_BMG_UNUSED(res);
        ENABLE_AUTO_BIAS(devices[0], req->enabled);
    }

    void L3Cam::changeBiasValue(const std::shared_ptr<l3cam_interfaces::srv::ChangeBiasValue::Request> req,
                                std::shared_ptr<l3cam_interfaces::srv::ChangeBiasValue::Response> res)
    {
        ROS2_BMG_UNUSED(res);
        CHANGE_BIAS_VALUE(devices[0], req->index, req->bias);
    }

    // Polarimetric
    void L3Cam::setPolarimetricCameraDefaultSettings(const std::shared_ptr<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings::Request> req,
                                                     std::shared_ptr<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = SET_POLARIMETRIC_CAMERA_DEFAULT_SETTINGS(devices[0]);
    }

    void L3Cam::changePolarimetricCameraBrightness(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness::Request> req,
                                                   std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness::Response> res)
    {
        res->error = CHANGE_POLARIMETRIC_CAMERA_BRIGHTNESS(devices[0], req->brightness);
    }

    void L3Cam::changePolarimetricCameraBlackLevel(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel::Request> req,
                                                   std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel::Response> res)
    {
        res->error = CHANGE_POLARIMETRIC_CAMERA_BLACK_LEVEL(devices[0], req->black_level);
    }

    void L3Cam::enablePolarimetricCameraAutoGain(const std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain::Request> req,
                                                 std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain::Response> res)
    {
        res->error = ENABLE_POLARIMETRIC_CAMERA_AUTO_GAIN(devices[0], req->enabled);
    }

    void L3Cam::changePolarimetricCameraAutoGainRange(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange::Request> req,
                                                      std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange::Response> res)
    {
        res->error = CHANGE_POLARIMETRIC_CAMERA_AUTO_GAIN_RANGE(devices[0], req->min_gain, req->max_gain);
    }

    void L3Cam::changePolarimetricCameraGain(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraGain::Request> req,
                                             std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraGain::Response> res)
    {
        res->error = CHANGE_POLARIMETRIC_CAMERA_GAIN(devices[0], req->gain);
    }

    void L3Cam::enablePolarimetricCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime::Request> req,
                                                         std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime::Response> res)
    {
        res->error = ENABLE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME(devices[0], req->enabled);
    }

    void L3Cam::changePolarimetricCameraAutoExposureTimeRange(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange::Request> req,
                                                              std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange::Response> res)
    {
        res->error = CHANGE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME_RANGE(devices[0], req->min_exposure, req->max_exposure);
    }

    void L3Cam::changePolarimetricCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime::Request> req,
                                                     std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime::Response> res)
    {
        res->error = CHANGE_POLARIMETRIC_CAMERA_EXPOSURE_TIME(devices[0], req->exposure_time);
    }

    // RGB
    void L3Cam::setRgbCameraDefaultSettings(const std::shared_ptr<l3cam_interfaces::srv::SetRgbCameraDefaultSettings::Request> req,
                                            std::shared_ptr<l3cam_interfaces::srv::SetRgbCameraDefaultSettings::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = SET_RGB_CAMERA_DEFAULT_SETTINGS(devices[0]);
    }

    void L3Cam::changeRgbCameraBrightness(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraBrightness::Request> req,
                                          std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraBrightness::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_BRIGHTNESS(devices[0], req->brightness);
        rgbDisconnected(0);
    }

    void L3Cam::changeRgbCameraContrast(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraContrast::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraContrast::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_CONTRAST(devices[0], req->contrast);
    }

    void L3Cam::changeRgbCameraSaturation(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSaturation::Request> req,
                                          std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSaturation::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_SATURATION(devices[0], req->saturation);
    }

    void L3Cam::changeRgbCameraSharpness(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSharpness::Request> req,
                                         std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSharpness::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_SHARPNESS(devices[0], req->sharpness);
    }

    void L3Cam::changeRgbCameraGamma(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGamma::Request> req,
                                     std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGamma::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_GAMMA(devices[0], req->gamma);
    }

    void L3Cam::changeRgbCameraGain(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGain::Request> req,
                                    std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGain::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_GAIN(devices[0], req->gain);
    }

    void L3Cam::enableRgbCameraAutoWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance::Request> req,
                                                std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance::Response> res)
    {
        res->error = ENABLE_RGB_CAMERA_AUTO_WHITE_BALANCE(devices[0], req->enabled);
    }

    void L3Cam::changeRgbCameraWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance::Request> req,
                                            std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_WHITE_BALANCE(devices[0], req->white_balance);
    }

    void L3Cam::enableRgbCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime::Request> req,
                                                std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime::Response> res)
    {
        res->error = ENABLE_RGB_CAMERA_AUTO_EXPOSURE_TIME(devices[0], req->enabled);
    }

    void L3Cam::changeRgbCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraExposureTime::Request> req,
                                            std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraExposureTime::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_EXPOSURE_TIME(devices[0], req->exposure_time);
    }

    // Thermal
    void L3Cam::changeThermalCameraColormap(const std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraColormap::Request> req,
                                            std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraColormap::Response> res)
    {
        res->error = CHANGE_THERMAL_CAMERA_COLORMAP(devices[0], (thermalTypes)req->colormap);
    }

    void L3Cam::changeThermalCameraTemperatureFilter(const std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter::Request> req,
                                                     std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter::Response> res)
    {
        res->error = CHANGE_THERMAL_CAMERA_TEMPERATURE_FILTER(devices[0], req->min_temperature, req->max_temperature);
    }

    void L3Cam::enableThermalCameraTemperatureFilter(const std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter::Request> req,
                                                     std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter::Response> res)
    {
        res->error = ENABLE_THERMAL_CAMERA_TEMPERATURE_FILTER(devices[0], req->enabled);
    }

    // Allied
    void L3Cam::changeAlliedCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime::Request> req,
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

    void L3Cam::enableAlliedCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime::Request> req,
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

    void L3Cam::changeAlliedCameraAutoExposureTimeRange(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange::Request> req,
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

    void L3Cam::changeAlliedCameraGain(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraGain::Request> req,
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

    void L3Cam::enableAlliedCameraAutoGain(const std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoGain::Request> req,
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

    void L3Cam::changeAlliedCameraAutoGainRange(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange::Request> req,
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

    void L3Cam::changeAlliedCameraGamma(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraGamma::Request> req,
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

    void L3Cam::changeAlliedCameraSaturation(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraSaturation::Request> req,
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

    void L3Cam::changeAlliedCameraHue(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraHue::Request> req,
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

    void L3Cam::changeAlliedCameraIntensityAutoPrecedence(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence::Request> req,
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

    void L3Cam::enableAlliedCameraAutoWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance::Request> req,
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

    void L3Cam::changeAlliedCameraBalanceRatioSelector(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector::Request> req,
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

    void L3Cam::changeAlliedCameraBalanceRatio(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio::Request> req,
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

    void L3Cam::changeAlliedCameraBalanceWhiteAutoRate(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate::Request> req,
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

    void L3Cam::changeAlliedCameraBalanceWhiteAutoTolerance(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance::Request> req,
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

    void L3Cam::changeAlliedCameraIntensityControllerRegion(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion::Request> req,
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

    void L3Cam::changeAlliedCameraIntensityControllerTarget(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget::Request> req,
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

    void L3Cam::getAlliedCameraBlackLevel(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBlackLevel::Request> req,
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

    void L3Cam::getAlliedCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraExposureTime::Request> req,
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

    void L3Cam::getAlliedCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTime::Request> req,
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

    void L3Cam::getAlliedCameraAutoExposureTimeRange(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTimeRange::Request> req,
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

    void L3Cam::getAlliedCameraGain(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraGain::Request> req,
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

    void L3Cam::getAlliedCameraAutoGain(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoGain::Request> req,
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

    void L3Cam::getAlliedCameraAutoGainRange(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoGainRange::Request> req,
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

    void L3Cam::getAlliedCameraGamma(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraGamma::Request> req,
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

    void L3Cam::getAlliedCameraSaturation(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraSaturation::Request> req,
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

    void L3Cam::getAlliedCameraSharpness(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraSharpness::Request> req,
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

    void L3Cam::getAlliedCameraHue(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraHue::Request> req,
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

    void L3Cam::getAlliedCameraIntensityAutoPrecedence(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityAutoPrecedence::Request> req,
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

    void L3Cam::getAlliedCameraAutoWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoWhiteBalance::Request> req,
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

    void L3Cam::getAlliedCameraBalanceRatioSelector(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceRatioSelector::Request> req,
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

    void L3Cam::getAlliedCameraBalanceRatio(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceRatio::Request> req,
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

    void L3Cam::getAlliedCameraBalanceWhiteAutoRate(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoRate::Request> req,
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

    void L3Cam::getAlliedCameraBalanceWhiteAutoTolerance(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoTolerance::Request> req,
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

    void L3Cam::getAlliedCameraAutoModeRegion(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraAutoModeRegion::Request> req,
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

    void L3Cam::getAlliedCameraIntensityControllerRegion(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerRegion::Request> req,
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

    void L3Cam::getAlliedCameraIntensityControllerTarget(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerTarget::Request> req,
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

    void L3Cam::getAlliedCameraMaxDriverBuffersCount(const std::shared_ptr<l3cam_interfaces::srv::GetAlliedCameraMaxDriverBuffersCount::Request> req,
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

    void L3Cam::lidarDisconnected(int code)
    {
        // Stream
        while (!clientPointCloudStreamDisconnected->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                break;
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto requestPointCloudStreamDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestPointCloudStreamDisconnected->code = code;

        auto resultPointCloudStreamDisconnected = clientPointCloudStreamDisconnected->async_send_request(requestPointCloudStreamDisconnected);

        // Configuration
        while (!clientPointCloudConfigurationDisconnected->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                break;
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto requestPointCloudConfigurationDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestPointCloudConfigurationDisconnected->code = code;

        auto resultPointCloudConfigurationDisconnected = clientPointCloudConfigurationDisconnected->async_send_request(requestPointCloudConfigurationDisconnected);
    }

    void L3Cam::polDisconnected(int code)
    {
        // Stream
        while (!clientPolWideStreamDisconnected->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                break;
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto requestPolWideStreamDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestPolWideStreamDisconnected->code = code;

        auto resultPolWideStreamDisconnected = clientPolWideStreamDisconnected->async_send_request(requestPolWideStreamDisconnected);

        // Configuration
        while (!clientPolConfigurationDisconnected->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                break;
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto requestPolConfigurationDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestPolConfigurationDisconnected->code = code;

        auto resultPolConfigurationDisconnected = clientPolConfigurationDisconnected->async_send_request(requestPolConfigurationDisconnected);
    }

    void L3Cam::rgbDisconnected(int code)
    {
        // Stream
        while (!clientRgbNarrowStreamDisconnected->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                break;
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto requestRgbNarrowStreamDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestRgbNarrowStreamDisconnected->code = code;

        auto resultRgbNarrowStreamDisconnected = clientRgbNarrowStreamDisconnected->async_send_request(requestRgbNarrowStreamDisconnected);

        // Configuration
        while (!clientRgbConfigurationDisconnected->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                break;
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto requestRgbConfigurationDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestRgbConfigurationDisconnected->code = code;

        auto resultRgbConfigurationDisconnected = clientRgbConfigurationDisconnected->async_send_request(requestRgbConfigurationDisconnected);
    }

    void L3Cam::wideDisconnected(int code)
    {
        // Stream
        while (!clientPolWideStreamDisconnected->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                break;
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto requestPolWideStreamDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestPolWideStreamDisconnected->code = code;

        auto resultPolWideStreamDisconnected = clientPolWideStreamDisconnected->async_send_request(requestPolWideStreamDisconnected);

        // Configuration
        while (!clientWideConfigurationDisconnected->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                break;
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto requestWideConfigurationDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestWideConfigurationDisconnected->code = code;

        auto resultWideConfigurationDisconnected = clientWideConfigurationDisconnected->async_send_request(requestWideConfigurationDisconnected);
    }

    void L3Cam::alliedNarrowDisconnect(int code)
    {
        // Stream
        while (!clientRgbNarrowStreamDisconnected->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                break;
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto requestRgbStreamDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestRgbStreamDisconnected->code = code;

        auto resultRgbStreamDisconnected = clientRgbNarrowStreamDisconnected->async_send_request(requestRgbStreamDisconnected);

        // Configuration
        while (!clientNarrowConfigurationDisconnected->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                break;
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto requestNarrowConfigurationDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestNarrowConfigurationDisconnected->code = code;

        auto resultNarrowConfigurationDisconnected = clientNarrowConfigurationDisconnected->async_send_request(requestNarrowConfigurationDisconnected);
    }

    void L3Cam::thermalDisconnected(int code)
    {
        // Stream
        while (!clientThermalStreamDisconnected->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                break;
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto requestThermalStreamDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestThermalStreamDisconnected->code = code;

        auto resultThermalStreamDisconnected = clientThermalStreamDisconnected->async_send_request(requestThermalStreamDisconnected);

        // Configuration
        while (!clientThermalConfigurationDisconnected->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                break;
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto requestThermalConfigurationDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestThermalConfigurationDisconnected->code = code;

        auto resultThermalConfigurationDisconnected = clientThermalConfigurationDisconnected->async_send_request(requestThermalConfigurationDisconnected);
    }

    void L3Cam::errorNotification(const int32_t *error)
    {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error notification received");
        int errort = *error;

        switch (errort)
        {
        case ERROR_LIDAR_TIMED_OUT:
            node->lidarDisconnected(errort);
            break;
        case ERROR_THERMAL_CAMERA_TIMEOUT:
            node->thermalDisconnected(errort);
            break;
        }
    }

} // namespace l3cam_ros2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "L3Cam version " << GET_VERSION() << "\n");

    l3cam_ros2::node = std::make_shared<l3cam_ros2::L3Cam>();

    int error = L3CAM_OK;
    error = l3cam_ros2::node->initializeDevice();
    if (error)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR " << error << " while initializing device: " << getBeamErrorDescription(error));
        TERMINATE(l3cam_ros2::node->devices[0]);
        return 1;
    }

    error = l3cam_ros2::node->startDeviceStream();
    if (error)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR " << error << " while starting device stream: " << getBeamErrorDescription(error));
        STOP_DEVICE(l3cam_ros2::node->devices[0]);
        TERMINATE(l3cam_ros2::node->devices[0]);
        return 1;
    }

    rclcpp::spin(l3cam_ros2::node);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Terminating...");
    // Before exiting stop stream, device and terminate
    STOP_STREAM(l3cam_ros2::node->devices[0]);
    STOP_DEVICE(l3cam_ros2::node->devices[0]);
    TERMINATE(l3cam_ros2::node->devices[0]);

    rclcpp::shutdown();
    return 0;
}
