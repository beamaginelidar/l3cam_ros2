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

#include <unistd.h>

using namespace std::chrono_literals;

std::shared_ptr<l3cam_ros2::L3Cam> node;

namespace l3cam_ros2
{
    L3Cam::L3Cam() : Node("l3cam_ros2_node")
    {
        // Register callback for sensor disconnection errors
        registerErrorCallback(errorNotification);

        declareParameters();

        // L3Cam node status
        m_status = LibL3CamStatus::undefined;
        srv_libl3cam_status_ = this->create_service<l3cam_interfaces::srv::LibL3camStatus>(
            "libl3cam_status",
            std::bind(&L3Cam::libL3camStatus, this, std::placeholders::_1, std::placeholders::_2));
    }

    int L3Cam::initializeDevice()
    {
        // Initialize L3Cam
        int error = L3CAM_OK;

        std::string local_address = this->get_parameter("local_address").as_string();
        std::string device_address = this->get_parameter("device_address").as_string();

        if (local_address == "" || device_address == "")
        {
            error = INITIALIZE(NULL, NULL);
        }
        else
        {
            error = INITIALIZE(&local_address[0], &device_address[0]);
        }

        if (error)
            return error;

        m_num_devices = 0;
        int i = 0;
        while (m_num_devices == 0 && error == L3CAM_OK)
        {
            error = FIND_DEVICES(m_devices, &m_num_devices);

            if (!rclcpp::ok())
            {
                return L3CAM_ROS2_RCLCPP_INTERRUPTED;
            }

            if (i >= this->get_parameter("timeout_secs").as_int() * 2)
                return L3CAM_ROS2_FIND_DEVICES_TIMEOUT_ERROR;
            usleep(500000);
            ++i;
        }
        this->undeclare_parameter("timeout_secs");

        if (error)
            return error;
        m_status = LibL3CamStatus::connected;
        RCLCPP_INFO_STREAM(this->get_logger(), "Device found " << std::string(m_devices[0].ip_address)
                                                               << ", model " << (int)m_devices[0].model
                                                               << ", serial number " << std::string(m_devices[0].serial_number)
                                                               << ", app version " << std::string(m_devices[0].app_version));

        int status = 0;
        error = GET_DEVICE_STATUS(m_devices[0], &status);
        if (error)
            return error;
        RCLCPP_INFO_STREAM(this->get_logger(), "Device status " << status);

        int num_sensors = 0;
        error = GET_SENSORS_AVAILABLE(m_devices[0], m_av_sensors, &num_sensors);
        if (error)
            return error;

        for (int i = 0; i < num_sensors; ++i)
        {
            switch (m_av_sensors[i].sensor_type)
            {
            case sensor_lidar:
                m_lidar_sensor = &m_av_sensors[i];
                break;
            case sensor_econ_rgb:
                m_rgb_sensor = &m_av_sensors[i];
                break;
            case sensor_thermal:
                m_thermal_sensor = &m_av_sensors[i];
                break;
            case sensor_pol:
                m_polarimetric_sensor = &m_av_sensors[i];
                break;
            case sensor_allied_wide:
                m_allied_wide_sensor = &m_av_sensors[i];
                break;
            case sensor_allied_narrow:
                m_allied_narrow_sensor = &m_av_sensors[i];
                break;
            }
        }

        RCLCPP_INFO_STREAM(this->get_logger(), num_sensors << ((num_sensors == 1) ? " sensor" : " sensors") << " available");

        initializeServices();

        return L3CAM_OK;
    }

    int L3Cam::startDeviceStream()
    {
        int error = L3CAM_OK;

        error = START_DEVICE(m_devices[0]);
        if (error)
        {
            return error;
        }

        m_status = LibL3CamStatus::started;
        RCLCPP_INFO(this->get_logger(), "Device started");

        loadDefaultParams();

        error = START_STREAM(m_devices[0]);
        if (error)
            return error;

        m_status = LibL3CamStatus::streaming;
        RCLCPP_INFO(this->get_logger(), "Device streaming ready\n");

        return L3CAM_OK;
    }

    void L3Cam::disconnectAll(int code)
    {
        networkDisconnected(code);
        if (m_lidar_sensor != NULL && m_lidar_sensor->sensor_available)
        {
            lidarDisconnected(code);
        }
        if (m_polarimetric_sensor != NULL && m_polarimetric_sensor->sensor_available)
        {
            polDisconnected(code);
        }
        if (m_rgb_sensor != NULL && m_rgb_sensor->sensor_available)
        {
            rgbDisconnected(code);
        }
        if (m_allied_wide_sensor != NULL && m_allied_wide_sensor->sensor_available)
        {
            alliedwideDisconnected(code);
        }
        if (m_allied_narrow_sensor != NULL && m_allied_narrow_sensor->sensor_available)
        {
            alliedNarrowDisconnect(code);
        }
        if (m_thermal_sensor != NULL && m_thermal_sensor->sensor_available)
        {
            thermalDisconnected(code);
        }
    }

    // Declare parameters
    void L3Cam::declareParameters()
    {
        this->declare_parameter("timeout_secs", 60);
        declareNetworkParameters();
        declareLidarParameters();
        declarePolarimetricParameters();
        declareRgbParameters();
        declareThermalParameters();
        declareAlliedWideParameters();
        declareAlliedNarrowParameters();
    }

    void L3Cam::declareNetworkParameters()
    {
        this->declare_parameter("ip_address", "192.168.1.250");
        this->declare_parameter("netmask", "255.255.255.0");
        this->declare_parameter("gateway", "0.0.0.0");
        this->declare_parameter("dhcp", false);
        this->declare_parameter("local_address", "");
        this->declare_parameter("device_address", "");
    }

    void L3Cam::declareLidarParameters()
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        rcl_interfaces::msg::IntegerRange intRange;

        intRange.set__from_value(0).set__to_value(13); // TBD: dynamic reconfigure enumerate pointCloudColor
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
        intRange.set__from_value(0).set__to_value(300000);
        descriptor.integer_range = {intRange};
        this->declare_parameter("pointcloud_color_range_minimum", 0, descriptor); // 0 - 300000
        intRange.set__from_value(0).set__to_value(300000);
        descriptor.integer_range = {intRange};
        this->declare_parameter("pointcloud_color_range_maximum", 300000, descriptor); // 0 - 300000
        intRange.set__from_value(0).set__to_value(300000);
        descriptor.integer_range = {intRange};
        this->declare_parameter("distance_range_minimum", 0, descriptor); // 0 - 300000
        intRange.set__from_value(0).set__to_value(300000);
        descriptor.integer_range = {intRange};
        this->declare_parameter("distance_range_maximum", 300000, descriptor); // 0 - 300000
        this->declare_parameter("auto_bias", true);
        intRange.set__from_value(700).set__to_value(3500);
        descriptor.integer_range = {intRange};
        this->declare_parameter("bias_value_right", 1580, descriptor); // 700 - 3500
        this->declare_parameter("bias_value_left", 1380, descriptor);  // 700 - 3500
        intRange.set__from_value(0).set__to_value(1);
        descriptor.integer_range = {intRange};
        descriptor.description = 
            "Value must be:\n"
            "\tprotocol_raw_udp = 0\n"
            "\tprotocol_gstreamer = 1";
        this->declare_parameter("lidar_streaming_protocol", 0, descriptor); // 0(protocol_raw_udp), 1(protocol_gstreamer)
        descriptor.description = "";
        descriptor.read_only = true;
        this->declare_parameter("lidar_rtsp_pipeline", "", descriptor);
        descriptor.read_only = false;
    }

    void L3Cam::declarePolarimetricParameters()
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        rcl_interfaces::msg::IntegerRange intRange;
        rcl_interfaces::msg::FloatingPointRange floatRange;

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
        descriptor.description = "";
        descriptor.read_only = true;
        this->declare_parameter("polarimetric_rtsp_pipeline", "", descriptor);
        descriptor.read_only = false;
    }

    void L3Cam::declareRgbParameters()
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        rcl_interfaces::msg::IntegerRange intRange;

        intRange.set__from_value(-15).set__to_value(15);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_brightness", 0, descriptor); // -15 - 15
        this->declare_parameter("rgb_camera_contrast", 10, descriptor);  // 0 - 30
        intRange.set__from_value(0).set__to_value(60);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_saturation", 16, descriptor); // 0 - 60
        intRange.set__from_value(0).set__to_value(127);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_sharpness", 16, descriptor); // 0 - 127
        intRange.set__from_value(40).set__to_value(500);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_gamma", 220, descriptor); // 40 - 500
        intRange.set__from_value(0).set__to_value(63);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_gain", 0, descriptor); // 0 - 63
        this->declare_parameter("rgb_camera_auto_white_balance", true);
        intRange.set__from_value(1000).set__to_value(10000);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_white_balance", 5000, descriptor); // 1000 - 10000
        this->declare_parameter("rgb_camera_auto_exposure_time", true);
        intRange.set__from_value(1).set__to_value(10000);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_exposure_time", 156, descriptor); // 1 - 10000
        intRange.set__from_value(1).set__to_value(3);
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be: (econResolutions)\n"
            "\treso_640_480 = 1\n"
            "\treso_1280_720 = 2\n"
            "\treso_1920_1080 = 3\n";
        this->declare_parameter("rgb_camera_resolution", 3, descriptor); // see econResolutions
        descriptor.description = "";
        intRange.set__from_value(1).set__to_value(16);
        descriptor.integer_range = {intRange};
        this->declare_parameter("rgb_camera_framerate", 10, descriptor); // 1 - 16
        intRange.set__from_value(0).set__to_value(1);
        descriptor.integer_range = {intRange};
        descriptor.description = 
            "Value must be:\n"
            "\tprotocol_raw_udp = 0\n"
            "\tprotocol_gstreamer = 1";
        this->declare_parameter("rgb_streaming_protocol", 0, descriptor); // 0(protocol_raw_udp), 1(protocol_gstreamer)
        descriptor.description = "";
        descriptor.read_only = true;
        this->declare_parameter("rgb_rtsp_pipeline", "", descriptor);
        descriptor.read_only = false;
    }

    void L3Cam::declareThermalParameters()
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        rcl_interfaces::msg::IntegerRange intRange;
        intRange.set__from_value(1).set__to_value(108); // TBD: dynamic reconfigure enumerate newThermalTypes
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be: (newThermalTypes)\n"
            "\tnew_thermal_WHITE_HOT = 0\n"
            "\tnew_thermal_BLACK_HOT = 1\n"
            "\tnew_thermal_SPECTRA = 2\n"
            "\tnew_thermal_PRISM = 3\n"
            "\tnew_thermal_TYRIAN = 4\n"
            "\tnew_thermal_IRON = 5\n"
            "\tnew_thermal_AMBER = 6\n"
            "\tnew_thermal_HI = 7\n"
            "\tnew_thermal_GREEN = 8";
        this->declare_parameter("thermal_camera_colormap", 1, descriptor); // see newThermalTypes
        descriptor.description = "";
        this->declare_parameter("thermal_camera_temperature_filter", false);
        intRange.set__from_value(-40).set__to_value(200);
        descriptor.integer_range = {intRange};
        this->declare_parameter("thermal_camera_temperature_filter_min", 0, descriptor); // -40 - 200
        intRange.set__from_value(-40).set__to_value(200);
        descriptor.integer_range = {intRange};
        this->declare_parameter("thermal_camera_temperature_filter_max", 50, descriptor); // -40 - 200
        intRange.set__from_value(0).set__to_value(2); // TBD: dynamic reconfigure enumerate thermalPipelines
        descriptor.integer_range = {intRange};
        descriptor.description = 
            "Value must be: (thermalPipelines)\n"
            "\tthermal_LITE = 0\n"
            "\tthermal_LEGACY = 1\n"
            "\tthermal_SEEK = 2";
        this->declare_parameter("thermal_camera_processing_pipeline", 1, descriptor); // 0 - 2
        descriptor.description = "";
        this->declare_parameter("thermal_camera_temperature_data_udp", false);
        intRange.set__from_value(0).set__to_value(1);
        descriptor.integer_range = {intRange};
        descriptor.description = 
            "Value must be:\n"
            "\tprotocol_raw_udp = 0\n"
            "\tprotocol_gstreamer = 1";
        this->declare_parameter("thermal_streaming_protocol", 0, descriptor); // 0(protocol_raw_udp), 1(protocol_gstreamer)
        descriptor.description = "";
        descriptor.read_only = true;
        this->declare_parameter("thermal_rtsp_pipeline", "", descriptor);
        descriptor.read_only = false;
    }

    void L3Cam::declareAlliedWideParameters()
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        rcl_interfaces::msg::IntegerRange intRange;
        rcl_interfaces::msg::FloatingPointRange floatRange;

        this->declare_parameter("allied_wide_camera_black_level", 0.0); // 0 - 4095
        floatRange.set__from_value(63.0).set__to_value(10000000.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_exposure_time", 4992.4, descriptor); // 63 - 10000000
        this->declare_parameter("allied_wide_camera_auto_exposure_time", false);
        floatRange.set__from_value(63.1).set__to_value(8999990.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_auto_exposure_time_range_min", 87.6, descriptor); // 63.1 - 8999990
        floatRange.set__from_value(87.6).set__to_value(10000000.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_auto_exposure_time_range_max", 8999990.0, descriptor); // 87.6 - 10000000
        floatRange.set__from_value(0.0).set__to_value(48.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_gain", 0.0, descriptor); // 0 - 48
        this->declare_parameter("allied_wide_camera_auto_gain", false);
        floatRange.set__from_value(0.0).set__to_value(48.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_auto_gain_range_min", 0.0, descriptor); // 0 - 48
        floatRange.set__from_value(0.0).set__to_value(48.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_auto_gain_range_max", 48.0, descriptor); // 0 - 48
        floatRange.set__from_value(0.4).set__to_value(2.4);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_gamma", 1.0, descriptor); // 0.4 - 2.4
        floatRange.set__from_value(0.0).set__to_value(2.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_saturation", 1.0, descriptor); // 0 - 2
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_sharpness", 0.0, descriptor); // -12 - 12
        floatRange.set__from_value(-40).set__to_value(40.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_hue", 0.0, descriptor); // -40 - 40
        intRange.set__from_value(0).set__to_value(1);                       // TBD: dynamic reconfigure enumerate
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be:\n"
            "\tMinimizeNoise = 0\n"
            "\tMinimizeBlur = 1";
        this->declare_parameter("allied_wide_camera_intensity_auto_precedence", 0, descriptor); // 0(MinimizeNoise) or 1(MinimizeBlur)
        descriptor.description = "";
        this->declare_parameter("allied_wide_camera_auto_white_balance", false);
        intRange.set__from_value(0).set__to_value(1); // TBD: dynamic reconfigure enumerate
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be:\n"
            "\tRed = 0\n"
            "\tBlue = 1";
        this->declare_parameter("allied_wide_camera_balance_ratio_selector", 0, descriptor); // 0(Red), 1(Blue)
        descriptor.description = "";
        floatRange.set__from_value(0.0).set__to_value(8.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_balance_ratio", 2.4, descriptor); // 0 - 8
        floatRange.set__from_value(0.0).set__to_value(100.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_balance_white_auto_rate", 100.0, descriptor); // 0 - 100
        floatRange.set__from_value(0.0).set__to_value(50.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_balance_white_auto_tolerance", 5.0, descriptor); // 0 - 50
        intRange.set__from_value(00).set__to_value(1028);
        descriptor.integer_range = {intRange};
        this->declare_parameter("allied_wide_camera_auto_mode_region_height", 0, descriptor); // 0 - 1028
        intRange.set__from_value(0).set__to_value(1232);
        descriptor.integer_range = {intRange};
        this->declare_parameter("allied_wide_camera_auto_mode_region_width", 0, descriptor); // 0 - 1232
        intRange.set__from_value(0).set__to_value(4);                                        // TBD: dynamic reconfigure enumerate
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be:\n"
            "\tAutoMode = 0\n"
            "\tFullImage = 4";
        this->declare_parameter("allied_wide_camera_intensity_controller_region", 4, descriptor); // 0(AutoMode), 4(FullImage)
        descriptor.description = "";
        floatRange.set__from_value(10).set__to_value(90);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_wide_camera_intensity_controller_target", 50.0, descriptor); // 10 - 90
        intRange.set__from_value(1).set__to_value(4096);
        descriptor.integer_range = {intRange};
        this->declare_parameter("allied_wide_camera_max_driver_buffers_count", 64, descriptor); // 1 - 4096
        intRange.set__from_value(0).set__to_value(1);
        descriptor.integer_range = {intRange};
        descriptor.description = 
            "Value must be:\n"
            "\tprotocol_raw_udp = 0\n"
            "\tprotocol_gstreamer = 1";
        this->declare_parameter("allied_wide_streaming_protocol", 0, descriptor); // 0(protocol_raw_udp), 1(protocol_gstreamer)
        descriptor.description = "";
        descriptor.read_only = true;
        this->declare_parameter("allied_wide_rtsp_pipeline", "", descriptor);
        descriptor.read_only = false;
    }

    void L3Cam::declareAlliedNarrowParameters()
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        rcl_interfaces::msg::IntegerRange intRange;
        rcl_interfaces::msg::FloatingPointRange floatRange;

        this->declare_parameter("allied_narrow_camera_black_level", 0.0); // 0 - 4095
        floatRange.set__from_value(63.0).set__to_value(10000000.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_exposure_time", 4992.4, descriptor); // 63 - 10000000
        this->declare_parameter("allied_narrow_camera_auto_exposure_time", false);
        floatRange.set__from_value(63.1).set__to_value(8999990.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_auto_exposure_time_range_min", 87.6, descriptor); // 63.1 - 8999990
        floatRange.set__from_value(87.6).set__to_value(10000000.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_auto_exposure_time_range_max", 8999990.0, descriptor); // 87.6 - 10000000
        floatRange.set__from_value(0.0).set__to_value(48.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_gain", 0.0, descriptor); // 0 - 48
        this->declare_parameter("allied_narrow_camera_auto_gain", false);
        floatRange.set__from_value(0.0).set__to_value(48.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_auto_gain_range_min", 0.0, descriptor); // 0 - 48
        floatRange.set__from_value(0.0).set__to_value(48.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_auto_gain_range_max", 48.0, descriptor); // 0 - 48
        floatRange.set__from_value(0.4).set__to_value(2.4);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_gamma", 1.0, descriptor); // 0.4 - 2.4
        floatRange.set__from_value(0.0).set__to_value(2.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_saturation", 1.0, descriptor); // 0 - 2
        floatRange.set__from_value(-12.0).set__to_value(12.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_sharpness", 0.0, descriptor); // -12 - 12
        floatRange.set__from_value(-40).set__to_value(40.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_hue", 0.0, descriptor); // -40 - 40
        intRange.set__from_value(0).set__to_value(1);                         // TBD: dynamic reconfigure enumerate
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be:\n"
            "\tMinimizeNoise = 0\n"
            "\tMinimizeBlur = 1";
        this->declare_parameter("allied_narrow_camera_intensity_auto_precedence", 0, descriptor); // 0(MinimizeNoise) or 1(MinimizeBlur)
        descriptor.description = "";
        this->declare_parameter("allied_narrow_camera_auto_white_balance", false);
        intRange.set__from_value(0).set__to_value(1); // TBD: dynamic reconfigure enumerate
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be:\n"
            "\tRed = 0\n"
            "\tBlue = 1";
        this->declare_parameter("allied_narrow_camera_balance_ratio_selector", 0, descriptor); // 0(Red), 1(Blue)
        descriptor.description = "";
        floatRange.set__from_value(0.0).set__to_value(8.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_balance_ratio", 2.4, descriptor); // 0 - 8
        floatRange.set__from_value(0.0).set__to_value(100.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_balance_white_auto_rate", 100.0, descriptor); // 0 - 100
        floatRange.set__from_value(0.0).set__to_value(50.0);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_balance_white_auto_tolerance", 5.0, descriptor); // 0 - 50
        intRange.set__from_value(0).set__to_value(1544);
        descriptor.integer_range = {intRange};
        this->declare_parameter("allied_narrow_camera_auto_mode_region_height", 0, descriptor); // 0 - 1544
        intRange.set__from_value(0).set__to_value(2064);
        descriptor.integer_range = {intRange};
        this->declare_parameter("allied_narrow_camera_auto_mode_region_width", 0, descriptor); // 0 - 2064
        intRange.set__from_value(0).set__to_value(4).set__step(4);                             // TBD: dynamic reconfigure enumerate
        descriptor.integer_range = {intRange};
        descriptor.description =
            "Value must be:\n"
            "\tAutoMode = 0\n"
            "\tFullImage = 4";
        this->declare_parameter("allied_narrow_camera_intensity_controller_region", 4, descriptor); // 0(AutoMode), 4(FullImage)
        descriptor.description = "";
        floatRange.set__from_value(10).set__to_value(90);
        descriptor.floating_point_range = {floatRange};
        this->declare_parameter("allied_narrow_camera_intensity_controller_target", 50.0, descriptor); // 10 - 90
        intRange.set__from_value(0).set__to_value(4096);
        descriptor.integer_range = {intRange};
        this->declare_parameter("allied_narrow_camera_max_driver_buffers_count", 64, descriptor); // 1 - 4096
        intRange.set__from_value(0).set__to_value(1);
        descriptor.integer_range = {intRange};
        descriptor.description = 
            "Value must be:\n"
            "\tprotocol_raw_udp = 0\n"
            "\tprotocol_gstreamer = 1";
        this->declare_parameter("allied_narrow_streaming_protocol", 0, descriptor); // 0(protocol_raw_udp), 1(protocol_gstreamer)
        descriptor.description = "";
        descriptor.read_only = true;
        this->declare_parameter("allied_narrow_rtsp_pipeline", "", descriptor);
        descriptor.read_only = false;
    }

    // Initialize services
    void L3Cam::initializeServices()
    {
        srv_get_version_ = this->create_service<l3cam_interfaces::srv::GetVersion>(
            "get_version",
            std::bind(&L3Cam::getVersion, this, std::placeholders::_1, std::placeholders::_2));
        srv_initialize_ = this->create_service<l3cam_interfaces::srv::Initialize>(
            "initialize",
            std::bind(&L3Cam::initialize, this, std::placeholders::_1, std::placeholders::_2));
        srv_terminate_ = this->create_service<l3cam_interfaces::srv::Terminate>(
            "terminate",
            std::bind(&L3Cam::terminate, this, std::placeholders::_1, std::placeholders::_2));
        srv_find_devices_ = this->create_service<l3cam_interfaces::srv::FindDevices>(
            "find_devices",
            std::bind(&L3Cam::findDevices, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_local_server_address_ = this->create_service<l3cam_interfaces::srv::GetLocalServerAddress>(
            "get_local_server_address",
            std::bind(&L3Cam::getLocalServerAddress, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_device_info_ = this->create_service<l3cam_interfaces::srv::GetDeviceInfo>(
            "get_device_info",
            std::bind(&L3Cam::getDeviceInfo, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_device_status_ = this->create_service<l3cam_interfaces::srv::GetDeviceStatus>(
            "get_device_status",
            std::bind(&L3Cam::getDeviceStatus, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_sensors_available_ = this->create_service<l3cam_interfaces::srv::GetSensorsAvailable>(
            "get_sensors_available",
            std::bind(&L3Cam::getSensorsAvailable, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_streaming_protocol_ = this->create_service<l3cam_interfaces::srv::ChangeStreamingProtocol>(
            "change_streaming_protocol",
            std::bind(&L3Cam::changeStreamingProtocol, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_rtsp_pipeline_ = this->create_service<l3cam_interfaces::srv::GetRtspPipeline>(
            "get_rtsp_pipeline",
            std::bind(&L3Cam::getRtspPipeline, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_network_configuration_ = this->create_service<l3cam_interfaces::srv::GetNetworkConfiguration>(
            "get_network_configuration",
            std::bind(&L3Cam::getNetworkConfiguration, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_network_configuration_ = this->create_service<l3cam_interfaces::srv::ChangeNetworkConfiguration>(
            "change_network_configuration",
            std::bind(&L3Cam::changeNetworkConfiguration, this, std::placeholders::_1, std::placeholders::_2));
        srv_power_off_device_ = this->create_service<l3cam_interfaces::srv::PowerOffDevice>(
            "power_off_device",
            std::bind(&L3Cam::powerOffDevice, this, std::placeholders::_1, std::placeholders::_2));
        srv_start_device_ = this->create_service<l3cam_interfaces::srv::StartDevice>(
            "start_device",
            std::bind(&L3Cam::startDevice, this, std::placeholders::_1, std::placeholders::_2));
        srv_stop_device_ = this->create_service<l3cam_interfaces::srv::StopDevice>(
            "stop_device",
            std::bind(&L3Cam::stopDevice, this, std::placeholders::_1, std::placeholders::_2));
        srv_start_stream_ = this->create_service<l3cam_interfaces::srv::StartStream>(
            "start_stream",
            std::bind(&L3Cam::startStream, this, std::placeholders::_1, std::placeholders::_2));
        srv_stop_stream_ = this->create_service<l3cam_interfaces::srv::StopStream>(
            "stop_stream",
            std::bind(&L3Cam::stopStream, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_device_temperatures_ = this->create_service<l3cam_interfaces::srv::GetDeviceTemperatures>(
            "get_device_temperatures",
            std::bind(&L3Cam::getDeviceTemperatures, this, std::placeholders::_1, std::placeholders::_2));

        client_network_disconnected_ = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("network_disconnected");

        if (m_lidar_sensor != NULL && m_lidar_sensor->sensor_available) // if lidar is available
        {
            initializeLidarServices();
        }

        if (m_polarimetric_sensor != NULL && m_polarimetric_sensor->sensor_available) // if polarimetric is available
        {
            initializePolarimetricServices();
        }

        if (m_rgb_sensor != NULL && m_rgb_sensor->sensor_available) // if rgb is available
        {
            initializeRgbServices();
        }

        if (m_thermal_sensor != NULL && m_thermal_sensor->sensor_available) // if thermal is available
        {
            initializeThermalServices();
        }

        if (m_allied_wide_sensor != NULL && m_allied_wide_sensor->sensor_available) // if allied wide is available
        {
            initializeAlliedWideServices();
        }

        if (m_allied_narrow_sensor != NULL && m_allied_narrow_sensor->sensor_available) // if allied narrow is available
        {
            initializeAlliedNarrowServices();
        }

        RCLCPP_INFO(this->get_logger(), "Services ready");
    }

    void L3Cam::initializeLidarServices()
    {
        srv_change_pointcloud_color_ = this->create_service<l3cam_interfaces::srv::ChangePointcloudColor>(
            "change_pointcloud_color",
            std::bind(&L3Cam::changePointcloudColor, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_pointcloud_color_range_ = this->create_service<l3cam_interfaces::srv::ChangePointcloudColorRange>(
            "change_pointcloud_color_range",
            std::bind(&L3Cam::changePointcloudColorRange, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_distance_range_ = this->create_service<l3cam_interfaces::srv::ChangeDistanceRange>(
            "change_distance_range",
            std::bind(&L3Cam::changeDistanceRange, this, std::placeholders::_1, std::placeholders::_2));
        srv_enable_auto_bias_ = this->create_service<l3cam_interfaces::srv::EnableAutoBias>(
            "enable_auto_bias",
            std::bind(&L3Cam::enableAutoBias, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_bias_value_ = this->create_service<l3cam_interfaces::srv::ChangeBiasValue>(
            "change_bias_value",
            std::bind(&L3Cam::changeBiasValue, this, std::placeholders::_1, std::placeholders::_2));

        client_lidar_stream_disconnected_ = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("lidar_stream_disconnected");
        client_lidar_configuration_disconnected_ = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("lidar_configuration_disconnected");
    }

    void L3Cam::initializePolarimetricServices()
    {
        srv_set_polarimetric_camera_default_settings_ = this->create_service<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings>(
            "set_polarimetric_camera_default_settings",
            std::bind(&L3Cam::setPolarimetricCameraDefaultSettings, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_polarimetric_camera_brightness_ = this->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness>(
            "change_polarimetric_camera_brightness",
            std::bind(&L3Cam::changePolarimetricCameraBrightness, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_polarimetric_camera_black_level_ = this->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel>(
            "change_polarimetric_camera_black_level",
            std::bind(&L3Cam::changePolarimetricCameraBlackLevel, this, std::placeholders::_1, std::placeholders::_2));
        srv_enable_polarimetric_camera_auto_gain_ = this->create_service<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain>(
            "enable_polarimetric_camera_auto_gain",
            std::bind(&L3Cam::enablePolarimetricCameraAutoGain, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_polarimetric_camera_auto_gain_range_ = this->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange>(
            "change_polarimetric_camera_auto_gain_range",
            std::bind(&L3Cam::changePolarimetricCameraAutoGainRange, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_polarimetric_camera_gain_ = this->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraGain>(
            "change_polarimetric_camera_gain",
            std::bind(&L3Cam::changePolarimetricCameraGain, this, std::placeholders::_1, std::placeholders::_2));
        srv_enable_polarimetric_camera_auto_exposure_time_ = this->create_service<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime>(
            "enable_polarimetric_camera_auto_exposure_time",
            std::bind(&L3Cam::enablePolarimetricCameraAutoExposureTime, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_polarimetric_camera_auto_exposure_time_range_ = this->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange>(
            "change_polarimetric_camera_auto_exposure_time_range",
            std::bind(&L3Cam::changePolarimetricCameraAutoExposureTimeRange, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_polarimetric_camera_exposure_time_ = this->create_service<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime>(
            "change_polarimetric_camera_exposure_time",
            std::bind(&L3Cam::changePolarimetricCameraExposureTime, this, std::placeholders::_1, std::placeholders::_2));

        client_pol_wide_stream_disconnected_ = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("polarimetric_wide_stream_disconnected");
        client_pol_configuration_disconnected_ = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("polarimetric_configuration_disconnected");
    }

    void L3Cam::initializeRgbServices()
    {
        srv_set_rgb_camera_default_settings_ = this->create_service<l3cam_interfaces::srv::SetRgbCameraDefaultSettings>(
            "set_rgb_camera_default_settings",
            std::bind(&L3Cam::setRgbCameraDefaultSettings, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_rgb_camera_brightness_ = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraBrightness>(
            "change_rgb_camera_brightness",
            std::bind(&L3Cam::changeRgbCameraBrightness, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_rgb_camera_contrast_ = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraContrast>(
            "change_rgb_camera_contrast",
            std::bind(&L3Cam::changeRgbCameraContrast, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_rgb_camera_saturation_ = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraSaturation>(
            "change_rgb_camera_saturation",
            std::bind(&L3Cam::changeRgbCameraSaturation, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_rgb_camera_sharpness_ = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraSharpness>(
            "change_rgb_camera_sharpness",
            std::bind(&L3Cam::changeRgbCameraSharpness, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_rgb_camera_gamma_ = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraGamma>(
            "change_rgb_camera_gamma",
            std::bind(&L3Cam::changeRgbCameraGamma, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_rgb_camera_gain_ = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraGain>(
            "change_rgb_camera_gain",
            std::bind(&L3Cam::changeRgbCameraGain, this, std::placeholders::_1, std::placeholders::_2));
        srv_enable_rgb_camera_auto_white_balance_ = this->create_service<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance>(
            "enable_rgb_camera_auto_white_balance",
            std::bind(&L3Cam::enableRgbCameraAutoWhiteBalance, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_rgb_camera_white_balance_ = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance>(
            "change_rgb_camera_white_balance",
            std::bind(&L3Cam::changeRgbCameraWhiteBalance, this, std::placeholders::_1, std::placeholders::_2));
        srv_enable_rgb_camera_auto_exposure_time_ = this->create_service<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime>(
            "enable_rgb_camera_auto_exposure_time",
            std::bind(&L3Cam::enableRgbCameraAutoExposureTime, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_rgb_camera_exposure_time_ = this->create_service<l3cam_interfaces::srv::ChangeRgbCameraExposureTime>(
            "change_rgb_camera_exposure_time",
            std::bind(&L3Cam::changeRgbCameraExposureTime, this, std::placeholders::_1, std::placeholders::_2));

        client_rgb_narrow_stream_disconnected_ = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("rgb_narrow_stream_disconnected");
        client_rgb_configuration_disconnected_ = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("rgb_configuration_disconnected");
    }

    void L3Cam::initializeThermalServices()
    {
        srv_change_thermal_camera_colormap_ = this->create_service<l3cam_interfaces::srv::ChangeThermalCameraColormap>(
            "change_thermal_camera_colormap",
            std::bind(&L3Cam::changeThermalCameraColormap, this, std::placeholders::_1, std::placeholders::_2));
        srv_enable_thermal_camera_temperature_filter_ = this->create_service<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter>(
            "enable_thermal_camera_temperature_filter",
            std::bind(&L3Cam::enableThermalCameraTemperatureFilter, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_thermal_camera_temperature_filter_ = this->create_service<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter>(
            "change_thermal_camera_temperature_filter",
            std::bind(&L3Cam::changeThermalCameraTemperatureFilter, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_thermal_camera_processing_pipeline_ = this->create_service<l3cam_interfaces::srv::ChangeThermalCameraProcessingPipeline>(
            "change_thermal_camera_processing_pipeline",
            std::bind(&L3Cam::changeThermalCameraProcessingPipeline, this, std::placeholders::_1, std::placeholders::_2));
        srv_enable_thermal_camera_temperature_data_udp_ = this->create_service<l3cam_interfaces::srv::EnableThermalCameraTemperatureDataUdp>(
            "enable_thermal_camera_temperature_data_udp",
            std::bind(&L3Cam::enableThermalCameraTemperatureDataUdp, this, std::placeholders::_1, std::placeholders::_2));

        client_thermal_stream_disconnected_ = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("thermal_stream_disconnected");
        client_thermal_configuration_disconnected_ = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("thermal_configuration_disconnected");
    }

    void L3Cam::initializeAlliedWideServices()
    {
        srv_change_allied_camera_exposure_time_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime>(
            "change_allied_camera_exposure_time",
            std::bind(&L3Cam::changeAlliedCameraExposureTime, this, std::placeholders::_1, std::placeholders::_2));
        srv_enable_allied_camera_auto_exposure_time_ = this->create_service<l3cam_interfaces::srv::EnableAlliedCameraAutoExposureTime>(
            "enable_allied_camera_auto_exposure_time",
            std::bind(&L3Cam::enableAlliedCameraAutoExposureTime, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_allied_camera_auto_exposure_time_range_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraAutoExposureTimeRange>(
            "change_allied_camera_auto_exposure_time_range",
            std::bind(&L3Cam::changeAlliedCameraAutoExposureTimeRange, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_allied_camera_gain_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraGain>(
            "change_allied_camera_gain",
            std::bind(&L3Cam::changeAlliedCameraGain, this, std::placeholders::_1, std::placeholders::_2));
        srv_enable_allied_camera_auto_gain_ = this->create_service<l3cam_interfaces::srv::EnableAlliedCameraAutoGain>(
            "enable_allied_camera_auto_gain",
            std::bind(&L3Cam::enableAlliedCameraAutoGain, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_allied_camera_auto_gain_range_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraAutoGainRange>(
            "change_allied_camera_auto_gain_range",
            std::bind(&L3Cam::changeAlliedCameraAutoGainRange, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_allied_camera_gamma_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraGamma>(
            "change_allied_camera_gamma",
            std::bind(&L3Cam::changeAlliedCameraGamma, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_allied_camera_saturation_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraSaturation>(
            "change_allied_camera_saturation",
            std::bind(&L3Cam::changeAlliedCameraSaturation, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_allied_camera_hue_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraHue>(
            "change_allied_camera_hue",
            std::bind(&L3Cam::changeAlliedCameraHue, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_allied_camera_intensity_auto_precedence_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityAutoPrecedence>(
            "change_allied_camera_intensity_auto_precedence",
            std::bind(&L3Cam::changeAlliedCameraIntensityAutoPrecedence, this, std::placeholders::_1, std::placeholders::_2));
        srv_enable_allied_camera_auto_white_balance_ = this->create_service<l3cam_interfaces::srv::EnableAlliedCameraAutoWhiteBalance>(
            "enable_allied_camera_auto_white_balance",
            std::bind(&L3Cam::enableAlliedCameraAutoWhiteBalance, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_allied_camera_balance_ratio_selector_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatioSelector>(
            "change_allied_camera_balance_ratio_selector",
            std::bind(&L3Cam::changeAlliedCameraBalanceRatioSelector, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_allied_camera_balance_ratio_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceRatio>(
            "change_allied_camera_balance_ratio",
            std::bind(&L3Cam::changeAlliedCameraBalanceRatio, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_allied_camera_balance_white_auto_rate_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoRate>(
            "change_allied_camera_balance_white_auto_rate",
            std::bind(&L3Cam::changeAlliedCameraBalanceWhiteAutoRate, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_allied_camera_balance_white_auto_tolerance_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraBalanceWhiteAutoTolerance>(
            "change_allied_camera_balance_white_auto_tolerance",
            std::bind(&L3Cam::changeAlliedCameraBalanceWhiteAutoTolerance, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_allied_camera_intensity_controller_region_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerRegion>(
            "change_allied_camera_intensity_controller_region",
            std::bind(&L3Cam::changeAlliedCameraIntensityControllerRegion, this, std::placeholders::_1, std::placeholders::_2));
        srv_change_allied_camera_intensity_controller_target_ = this->create_service<l3cam_interfaces::srv::ChangeAlliedCameraIntensityControllerTarget>(
            "change_allied_camera_intensity_controller_target",
            std::bind(&L3Cam::changeAlliedCameraIntensityControllerTarget, this, std::placeholders::_1, std::placeholders::_2));

        client_pol_wide_stream_disconnected_ = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("polarimetric_wide_stream_disconnected");
        client_wide_configuration_disconnected_ = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("allied_wide_configuration_disconnected");
    }

    void L3Cam::initializeAlliedNarrowServices()
    {
        srv_get_allied_camera_black_level_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraBlackLevel>(
            "get_allied_camera_black_level",
            std::bind(&L3Cam::getAlliedCameraBlackLevel, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_exposure_time_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraExposureTime>(
            "get_allied_camera_exposure_time",
            std::bind(&L3Cam::getAlliedCameraExposureTime, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_auto_exposure_time_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTime>(
            "get_allied_camera_auto_exposure_time",
            std::bind(&L3Cam::getAlliedCameraAutoExposureTime, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_auto_exposure_time_range_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoExposureTimeRange>(
            "get_allied_camera_auto_exposure_time_range",
            std::bind(&L3Cam::getAlliedCameraAutoExposureTimeRange, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_gain_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraGain>(
            "get_allied_camera_gain",
            std::bind(&L3Cam::getAlliedCameraGain, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_auto_gain_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoGain>(
            "get_allied_camera_auto_gain",
            std::bind(&L3Cam::getAlliedCameraAutoGain, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_auto_gain_range_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoGainRange>(
            "get_allied_camera_auto_gain_range",
            std::bind(&L3Cam::getAlliedCameraAutoGainRange, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_gamma_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraGamma>(
            "get_allied_camera_gamma",
            std::bind(&L3Cam::getAlliedCameraGamma, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_saturation_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraSaturation>(
            "get_allied_camera_saturation",
            std::bind(&L3Cam::getAlliedCameraSaturation, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_sharpness_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraSharpness>(
            "get_allied_camera_sharpness",
            std::bind(&L3Cam::getAlliedCameraSharpness, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_hue_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraHue>(
            "get_allied_camera_hue",
            std::bind(&L3Cam::getAlliedCameraHue, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_intensity_auto_precedence_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraIntensityAutoPrecedence>(
            "get_allied_camera_intensity_auto_precedence",
            std::bind(&L3Cam::getAlliedCameraIntensityAutoPrecedence, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_auto_white_balance_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoWhiteBalance>(
            "get_allied_camera_auto_white_balance",
            std::bind(&L3Cam::getAlliedCameraAutoWhiteBalance, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_balance_ratio_selector_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraBalanceRatioSelector>(
            "get_allied_camera_balance_ratio_selector",
            std::bind(&L3Cam::getAlliedCameraBalanceRatioSelector, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_balance_ratio_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraBalanceRatio>(
            "get_allied_camera_balance_ratio",
            std::bind(&L3Cam::getAlliedCameraBalanceRatio, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_balance_white_auto_rate_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoRate>(
            "get_allied_camera_balance_white_auto_rate",
            std::bind(&L3Cam::getAlliedCameraBalanceWhiteAutoRate, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_balance_white_auto_tolerance_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraBalanceWhiteAutoTolerance>(
            "get_allied_camera_balance_white_auto_tolerance",
            std::bind(&L3Cam::getAlliedCameraBalanceWhiteAutoTolerance, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_auto_mode_region_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraAutoModeRegion>(
            "get_allied_camera_auto_mode_region",
            std::bind(&L3Cam::getAlliedCameraAutoModeRegion, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_intensity_controller_region_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerRegion>(
            "get_allied_camera_intensity_controller_region",
            std::bind(&L3Cam::getAlliedCameraIntensityControllerRegion, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_intensity_controller_target_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraIntensityControllerTarget>(
            "get_allied_camera_intensity_controller_target",
            std::bind(&L3Cam::getAlliedCameraIntensityControllerTarget, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_allied_camera_max_driver_buffers_count_ = this->create_service<l3cam_interfaces::srv::GetAlliedCameraMaxDriverBuffersCount>(
            "get_allied_camera_max_driver_buffers_count",
            std::bind(&L3Cam::getAlliedCameraMaxDriverBuffersCount, this, std::placeholders::_1, std::placeholders::_2));

        client_rgb_narrow_stream_disconnected_ = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("rgb_narrow_stream_disconnected");
        client_narrow_configuration_disconnected_ = this->create_client<l3cam_interfaces::srv::SensorDisconnected>("allied_narrow_configuration_disconnected");
    }

    inline void L3Cam::printDefaultError(int error, std::string param)
    {
        if (error != L3CAM_OK)
        {
            RCLCPP_WARN_STREAM(this->get_logger(),
                                "ERROR " << error << " while setting default parameter " << param << ": "
                                         << getErrorDescription(error));
        }
    }

    // Load default params
    void L3Cam::loadDefaultParams()
    {
        loadNetworkDefaultParams();
        if (m_lidar_sensor != NULL) // if lidar should be available in the L3Cam
        {
            if (m_lidar_sensor->sensor_available) // if lidar is available
            {
                loadLidarDefaultParams();
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: LiDAR not available.");
            }
        }
        if (m_polarimetric_sensor != NULL) // if polarimetric should be available in the L3Cam
        {
            if (m_polarimetric_sensor->sensor_available) // if polarimetric is available
            {
                loadPolarimetricDefaultParams();
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: Polarimetric camera not available.");
            }
        }
        if (m_rgb_sensor != NULL) // if rgb should be available in the L3Cam
        {
            if (m_rgb_sensor->sensor_available) // if rgb is available
            {
                loadRgbDefaultParams();
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: RGB camera not available.");
            }
        }
        if (m_thermal_sensor != NULL) // if thermal should be available in the L3Cam
        {
            if (m_thermal_sensor->sensor_available) // if thermal is available
            {
                loadThermalDefaultParams();
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: Thermal camera not available.");
            }
        }
        if (m_allied_wide_sensor != NULL) // if allied wide should be available in the L3Cam
        {
            if (m_allied_wide_sensor->sensor_available) // if allied wide is available
            {
                loadAlliedWideDefaultParams();
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: Allied Wide camera not available.");
            }
        }
        if (m_allied_narrow_sensor != NULL) // if allied narrow should be available in the L3Cam
        {
            if (m_allied_narrow_sensor->sensor_available) // if allied narrow is available
            {
                loadAlliedNarrowDefaultParams();
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: Allied Narrow camera not available.");
            }
        }

        RCLCPP_INFO(this->get_logger(), "Default parameters loaded");
    }

    void L3Cam::loadNetworkDefaultParams()
    {
        char *ip_address = NULL;
        char *netmask = NULL;
        char *gateway = NULL;
        int error = GET_NETWORK_CONFIGURATION(m_devices[0], &ip_address, &netmask, &gateway);
        if (!error)
        {
            this->set_parameter(rclcpp::Parameter("ip_address", std::string(ip_address)));
            this->set_parameter(rclcpp::Parameter("netmask", std::string(netmask)));
            this->set_parameter(rclcpp::Parameter("gateway", std::string(gateway)));
        }
    }

    void L3Cam::loadLidarDefaultParams()
    {
        printDefaultError(CHANGE_POINT_CLOUD_COLOR(m_devices[0],
                                                   this->get_parameter("pointcloud_color").as_int()),
                          "pointcloud_color");
        printDefaultError(CHANGE_POINT_CLOUD_COLOR_RANGE(m_devices[0],
                                                         this->get_parameter("pointcloud_color_range_minimum").as_int(),
                                                         this->get_parameter("pointcloud_color_range_maximum").as_int()),
                          "pointcloud_color_range");
        printDefaultError(CHANGE_DISTANCE_RANGE(m_devices[0],
                                                this->get_parameter("distance_range_minimum").as_int(),
                                                this->get_parameter("distance_range_maximum").as_int()),
                          "distance_range");
        ENABLE_AUTO_BIAS(m_devices[0], this->get_parameter("auto_bias").as_bool());
        if (!this->get_parameter("auto_bias").as_bool())
        { //! Values might not match after disabling auto_bias
            CHANGE_BIAS_VALUE(m_devices[0], 1, this->get_parameter("bias_value_right").as_int());
            CHANGE_BIAS_VALUE(m_devices[0], 2, this->get_parameter("bias_value_left").as_int());
        }
        if (this->get_parameter("lidar_streaming_protocol").as_int() == 1)
        {
            m_lidar_sensor->protocol = protocol_gstreamer;
            printDefaultError(CHANGE_STREAMING_PROTOCOL(m_devices[0], m_lidar_sensor), "lidar_streaming_protocol");
        }
        if (this->get_parameter("lidar_rtsp_pipeline").as_string() != "")
        {
            char *pipeline = &std::string(this->get_parameter("lidar_rtsp_pipeline").as_string())[0];
            printDefaultError(CHANGE_RTSP_PIPELINE(m_devices[0], *m_lidar_sensor, pipeline), "lidar_rtsp_pipeline");
        }
    }

    void L3Cam::loadPolarimetricDefaultParams()
    {
        printDefaultError(CHANGE_POLARIMETRIC_CAMERA_BRIGHTNESS(m_devices[0],
                                                                this->get_parameter("polarimetric_camera_brightness").as_int()),
                          "polarimetric_camera_brightness");
        printDefaultError(CHANGE_POLARIMETRIC_CAMERA_BLACK_LEVEL(m_devices[0],
                                                                 this->get_parameter("polarimetric_camera_black_level").as_double()),
                          "polarimetric_camera_black_level");
        printDefaultError(ENABLE_POLARIMETRIC_CAMERA_AUTO_GAIN(m_devices[0],
                                                               this->get_parameter("polarimetric_camera_auto_gain").as_bool()),
                          "polarimetric_camera_auto_gain");
        if (this->get_parameter("polarimetric_camera_auto_gain").as_bool())
        { //! Values might not match after enabling polarimetric_camera_auto_gain
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_AUTO_GAIN_RANGE(m_devices[0],
                                                                         this->get_parameter("polarimetric_camera_auto_gain_range_minimum").as_double(),
                                                                         this->get_parameter("polarimetric_camera_auto_gain_range_maximum").as_double()),
                              "polarimetric_camera_auto_gain_range");
        }
        else
        { //! Values might not match after disabling polarimetric_camera_auto_gain
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_GAIN(m_devices[0],
                                                              this->get_parameter("polarimetric_camera_gain").as_double()),
                              "polarimetric_camera_gain");
        }
        printDefaultError(ENABLE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0],
                                                                        this->get_parameter("polarimetric_camera_auto_exposure_time").as_bool()),
                          "polarimetric_camera_auto_exposure_time");
        if (this->get_parameter("polarimetric_camera_auto_exposure_time").as_bool())
        { //! Values might not match after enabling polarimetric_camera_auto_exposure_time
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0],
                                                                                  this->get_parameter("polarimetric_camera_auto_exposure_time_range_minimum").as_double(),
                                                                                  this->get_parameter("polarimetric_camera_auto_exposure_time_range_maximum").as_double()),
                              "polarimetric_camera_auto_exposure_time_range");
        }
        else
        { //! Values might not match after disabling polarimetric_camera_auto_exposure_time
            printDefaultError(CHANGE_POLARIMETRIC_CAMERA_EXPOSURE_TIME(m_devices[0],
                                                                       this->get_parameter("polarimetric_camera_exposure_time").as_double()),
                              "polarimetric_camera_exposure_time");
        }
        if (this->get_parameter("polarimetric_streaming_protocol").as_int() == 1)
        {
            m_polarimetric_sensor->protocol = protocol_gstreamer;
            printDefaultError(CHANGE_STREAMING_PROTOCOL(m_devices[0], m_polarimetric_sensor), "polarimetric_streaming_protocol");
        }
        if (this->get_parameter("polarimetric_rtsp_pipeline").as_string() != "")
        {
            char *pipeline = &std::string(this->get_parameter("polarimetric_rtsp_pipeline").as_string())[0];
            printDefaultError(CHANGE_RTSP_PIPELINE(m_devices[0], *m_polarimetric_sensor, pipeline), "polarimetric_rtsp_pipeline");
        }
    }

    void L3Cam::loadRgbDefaultParams()
    {
        printDefaultError(CHANGE_RGB_CAMERA_BRIGHTNESS(m_devices[0],
                                                       this->get_parameter("rgb_camera_brightness").as_int()),
                          "rgb_camera_brightness");
        printDefaultError(CHANGE_RGB_CAMERA_CONTRAST(m_devices[0],
                                                     this->get_parameter("rgb_camera_contrast").as_int()),
                          "rgb_camera_contrast");
        printDefaultError(CHANGE_RGB_CAMERA_SATURATION(m_devices[0],
                                                       this->get_parameter("rgb_camera_saturation").as_int()),
                          "rgb_camera_saturation");
        printDefaultError(CHANGE_RGB_CAMERA_SHARPNESS(m_devices[0],
                                                      this->get_parameter("rgb_camera_sharpness").as_int()),
                          "rgb_camera_sharpness");
        printDefaultError(CHANGE_RGB_CAMERA_GAMMA(m_devices[0],
                                                  this->get_parameter("rgb_camera_gamma").as_int()),
                          "rgb_camera_gamma");
        printDefaultError(CHANGE_RGB_CAMERA_GAIN(m_devices[0],
                                                 this->get_parameter("rgb_camera_gain").as_int()),
                          "rgb_camera_gain");
        printDefaultError(ENABLE_RGB_CAMERA_AUTO_WHITE_BALANCE(m_devices[0],
                                                               this->get_parameter("rgb_camera_auto_white_balance").as_bool()),
                          "rgb_camera_auto_white_balance");
        if (!this->get_parameter("rgb_camera_auto_white_balance").as_bool())
        { //! Values might not match after disabling rgb_camera_auto_white_balance
            printDefaultError(CHANGE_RGB_CAMERA_WHITE_BALANCE(m_devices[0],
                                                              this->get_parameter("rgb_camera_white_balance").as_int()),
                              "rgb_camera_white_balance");
        }
        printDefaultError(ENABLE_RGB_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0],
                                                               this->get_parameter("rgb_camera_auto_exposure_time").as_bool()),
                          "rgb_camera_auto_exposure_time");
        if (!this->get_parameter("rgb_camera_auto_exposure_time").as_bool())
        { //! Values might not match after disabling rgb_camera_auto_exposure_time
            printDefaultError(CHANGE_RGB_CAMERA_EXPOSURE_TIME(m_devices[0],
                                                              this->get_parameter("rgb_camera_exposure_time").as_int()),
                              "rgb_camera_exposure_time");
        }
        printDefaultError(CHANGE_RGB_CAMERA_RESOLUTION(m_devices[0],
                                                       (econResolutions)this->get_parameter("rgb_camera_resolution").as_int()),
                          "rgb_camera_resolution");
        printDefaultError(CHANGE_RGB_CAMERA_FRAMERATE(m_devices[0],
                                                      this->get_parameter("rgb_camera_framerate").as_int()),
                          "rgb_camera_framerate");
        if (this->get_parameter("rgb_streaming_protocol").as_int() == 1)
        {
            m_rgb_sensor->protocol = protocol_gstreamer;
            printDefaultError(CHANGE_STREAMING_PROTOCOL(m_devices[0], m_rgb_sensor), "rgb_streaming_protocol");
        }
        if (this->get_parameter("rgb_rtsp_pipeline").as_string() != "")
        {
            char *pipeline = &std::string(this->get_parameter("rgb_rtsp_pipeline").as_string())[0];
            printDefaultError(CHANGE_RTSP_PIPELINE(m_devices[0], *m_rgb_sensor, pipeline), "rgb_rtsp_pipeline");
        }
    }

    void L3Cam::loadThermalDefaultParams()
    {
        printDefaultError(CHANGE_THERMAL_CAMERA_COLORMAP(m_devices[0],
                                                         this->get_parameter("thermal_camera_colormap").as_int()),
                          "thermal_camera_colormap");
        printDefaultError(ENABLE_THERMAL_CAMERA_TEMPERATURE_FILTER(m_devices[0],
                                                                   this->get_parameter("thermal_camera_temperature_filter").as_bool()),
                          "thermal_camera_temperature_filter");
        printDefaultError(CHANGE_THERMAL_CAMERA_TEMPERATURE_FILTER(m_devices[0],
                                                                   this->get_parameter("thermal_camera_temperature_filter_min").as_int(),
                                                                   this->get_parameter("thermal_camera_temperature_filter_max").as_int()),
                          "thermal_camera_temperature_filter_range");
        printDefaultError(CHANGE_THERMAL_CAMERA_PROCESSING_PIPELINE(m_devices[0],
                                                  this->get_parameter("thermal_camera_processing_pipeline").as_int()),
                          "thermal_camera_processing_pipeline");
        printDefaultError(ENABLE_THERMAL_CAMERA_TEMPERATURE_DATA_UDP(m_devices[0],
                                                              this->get_parameter("thermal_camera_temperature_data_udp").as_bool()),
                          "thermal_camera_temperature_data_udp");
        if (this->get_parameter("thermal_streaming_protocol").as_int() == 1)
        {
            m_thermal_sensor->protocol = protocol_gstreamer;
            printDefaultError(CHANGE_STREAMING_PROTOCOL(m_devices[0], m_thermal_sensor), "thermal_streaming_protocol");
        }
        if (this->get_parameter("thermal_rtsp_pipeline").as_string() != "")
        {
            char *pipeline = &std::string(this->get_parameter("thermal_rtsp_pipeline").as_string())[0];
            printDefaultError(CHANGE_RTSP_PIPELINE(m_devices[0], *m_thermal_sensor, pipeline), "thermal_rtsp_pipeline");
        }
    }

    void L3Cam::loadAlliedWideDefaultParams()
    {
        printDefaultError(CHANGE_ALLIED_CAMERA_BLACK_LEVEL(m_devices[0], *m_allied_wide_sensor,
                                                           this->get_parameter("allied_wide_camera_black_level").as_double()),
                          "allied_wide_camera_black_level");
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], *m_allied_wide_sensor,
                                                                  this->get_parameter("allied_wide_camera_auto_exposure_time").as_bool()),
                          "allied_wide_camera_auto_exposure_time");
        printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], *m_allied_wide_sensor,
                                                                        this->get_parameter("allied_wide_camera_auto_exposure_time_range_min").as_double(),
                                                                        this->get_parameter("allied_wide_camera_auto_exposure_time_range_max").as_double()),
                          "allied_wide_camera_auto_exposure_time_range");
        if (!this->get_parameter("allied_wide_camera_auto_exposure_time").as_bool())
        {
            printDefaultError(CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(m_devices[0], *m_allied_wide_sensor,
                                                                    this->get_parameter("allied_wide_camera_exposure_time").as_double()),
                              "allied_wide_camera_exposure_time");
        }
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_GAIN(m_devices[0], *m_allied_wide_sensor,
                                                         this->get_parameter("allied_wide_camera_auto_gain").as_bool()),
                          "allied_wide_camera_auto_gain");
        printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(m_devices[0], *m_allied_wide_sensor,
                                                               (float)this->get_parameter("allied_wide_camera_auto_gain_range_min").as_double(),
                                                               (float)this->get_parameter("allied_wide_camera_auto_gain_range_max").as_double()),
                          "allied_wide_camera_auto_gain_range");
        if (!this->get_parameter("allied_wide_camera_auto_gain").as_bool())
        {
            printDefaultError(CHANGE_ALLIED_CAMERA_GAIN(m_devices[0], *m_allied_wide_sensor,
                                                        this->get_parameter("allied_wide_camera_gain").as_double()),
                              "allied_wide_camera_gain");
        }
        printDefaultError(CHANGE_ALLIED_CAMERA_GAMMA(m_devices[0], *m_allied_wide_sensor,
                                                     this->get_parameter("allied_wide_camera_gamma").as_double()),
                          "allied_wide_camera_gamma");
        printDefaultError(CHANGE_ALLIED_CAMERA_SATURATION(m_devices[0], *m_allied_wide_sensor,
                                                          this->get_parameter("allied_wide_camera_saturation").as_double()),
                          "allied_wide_camera_saturation");
        printDefaultError(CHANGE_ALLIED_CAMERA_SHARPNESS(m_devices[0], *m_allied_wide_sensor,
                                                         this->get_parameter("allied_wide_camera_sharpness").as_double()),
                          "allied_wide_camera_sharpness");
        printDefaultError(CHANGE_ALLIED_CAMERA_HUE(m_devices[0], *m_allied_wide_sensor,
                                                   this->get_parameter("allied_wide_camera_hue").as_double()),
                          "allied_wide_camera_hue");
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(m_devices[0], *m_allied_wide_sensor,
                                                                         this->get_parameter("allied_wide_camera_intensity_auto_precedence").as_int()),
                          "allied_wide_camera_intensity_auto_precedence");
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], *m_allied_wide_sensor,
                                                                  this->get_parameter("allied_wide_camera_auto_white_balance").as_bool()),
                          "allied_wide_camera_auto_white_balance");
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(m_devices[0], *m_allied_wide_sensor,
                                                                      this->get_parameter("allied_wide_camera_balance_ratio_selector").as_int()),
                          "allied_wide_camera_balance_ratio_selector");
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO(m_devices[0], *m_allied_wide_sensor,
                                                             this->get_parameter("allied_wide_camera_balance_ratio").as_double()),
                          "allied_wide_camera_balance_ratio");
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(m_devices[0], *m_allied_wide_sensor,
                                                                       this->get_parameter("allied_wide_camera_balance_white_auto_rate").as_double()),
                          "allied_wide_camera_balance_white_auto_rate");
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(m_devices[0], *m_allied_wide_sensor,
                                                                            this->get_parameter("allied_wide_camera_balance_white_auto_tolerance").as_double()),
                          "allied_wide_camera_balance_white_auto_tolerance");
        printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_MODE_REGION(m_devices[0], *m_allied_wide_sensor,
                                                                this->get_parameter("allied_wide_camera_auto_mode_region_height").as_int(),
                                                                this->get_parameter("allied_wide_camera_auto_mode_region_width").as_int()),
                          "allied_wide_camera_auto_mode_region");
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(m_devices[0], *m_allied_wide_sensor,
                                                                           this->get_parameter("allied_wide_camera_intensity_controller_region").as_int()),
                          "allied_wide_camera_intensity_controller_region");
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(m_devices[0], *m_allied_wide_sensor,
                                                                           this->get_parameter("allied_wide_camera_intensity_controller_target").as_double()),
                          "allied_wide_camera_intensity_controller_target");
        printDefaultError(CHANGE_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(m_devices[0], *m_allied_wide_sensor,
                                                                        this->get_parameter("allied_wide_camera_max_driver_buffers_count").as_int()),
                          "allied_wide_camera_max_driver_buffers_count");
        if (this->get_parameter("allied_wide_streaming_protocol").as_int() == 1)
        {
            m_allied_wide_sensor->protocol = protocol_gstreamer;
            printDefaultError(CHANGE_STREAMING_PROTOCOL(m_devices[0], m_allied_wide_sensor), "allied_wide_streaming_protocol");
        }
        if (this->get_parameter("allied_wide_rtsp_pipeline").as_string() != "")
        {
            char *pipeline = &std::string(this->get_parameter("allied_wide_rtsp_pipeline").as_string())[0];
            printDefaultError(CHANGE_RTSP_PIPELINE(m_devices[0], *m_allied_wide_sensor, pipeline), "allied_wide_rtsp_pipeline");
        }
    }

    void L3Cam::loadAlliedNarrowDefaultParams()
    {
        printDefaultError(CHANGE_ALLIED_CAMERA_BLACK_LEVEL(m_devices[0], *m_allied_narrow_sensor,
                                                           this->get_parameter("allied_narrow_camera_black_level").as_double()),
                          "allied_narrow_camera_black_level");
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], *m_allied_narrow_sensor,
                                                                  this->get_parameter("allied_narrow_camera_auto_exposure_time").as_bool()),
                          "allied_narrow_camera_auto_exposure_time");
        printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], *m_allied_narrow_sensor,
                                                                        this->get_parameter("allied_narrow_camera_auto_exposure_time_range_min").as_double(),
                                                                        this->get_parameter("allied_narrow_camera_auto_exposure_time_range_max").as_double()),
                          "allied_narrow_camera_auto_exposure_time_range");
        if (!this->get_parameter("allied_narrow_camera_auto_exposure_time").as_bool())
        {
            printDefaultError(CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(m_devices[0], *m_allied_narrow_sensor,
                                                                    this->get_parameter("allied_narrow_camera_exposure_time").as_double()),
                              "allied_narrow_camera_exposure_time");
        }
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_GAIN(m_devices[0], *m_allied_narrow_sensor,
                                                         this->get_parameter("allied_narrow_camera_auto_gain").as_bool()),
                          "allied_narrow_camera_auto_gain");
        printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(m_devices[0], *m_allied_narrow_sensor,
                                                               (float)this->get_parameter("allied_narrow_camera_auto_gain_range_min").as_double(),
                                                               (float)this->get_parameter("allied_narrow_camera_auto_gain_range_max").as_double()),
                          "allied_narrow_camera_auto_gain_range");
        if (!this->get_parameter("allied_narrow_camera_auto_gain").as_bool())
        {
            printDefaultError(CHANGE_ALLIED_CAMERA_GAIN(m_devices[0], *m_allied_narrow_sensor,
                                                        this->get_parameter("allied_narrow_camera_gain").as_double()),
                              "allied_narrow_camera_gain");
        }
        printDefaultError(CHANGE_ALLIED_CAMERA_GAMMA(m_devices[0], *m_allied_narrow_sensor,
                                                     this->get_parameter("allied_narrow_camera_gamma").as_double()),
                          "allied_narrow_camera_gamma");
        printDefaultError(CHANGE_ALLIED_CAMERA_SATURATION(m_devices[0], *m_allied_narrow_sensor,
                                                          this->get_parameter("allied_narrow_camera_saturation").as_double()),
                          "allied_narrow_camera_saturation");
        printDefaultError(CHANGE_ALLIED_CAMERA_SHARPNESS(m_devices[0], *m_allied_narrow_sensor,
                                                         this->get_parameter("allied_narrow_camera_sharpness").as_double()),
                          "allied_narrow_camera_sharpness");
        printDefaultError(CHANGE_ALLIED_CAMERA_HUE(m_devices[0], *m_allied_narrow_sensor,
                                                   this->get_parameter("allied_narrow_camera_hue").as_double()),
                          "allied_narrow_camera_hue");
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(m_devices[0], *m_allied_narrow_sensor,
                                                                         this->get_parameter("allied_narrow_camera_intensity_auto_precedence").as_int()),
                          "allied_narrow_camera_intensity_auto_precedence");
        printDefaultError(ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], *m_allied_narrow_sensor,
                                                                  this->get_parameter("allied_narrow_camera_auto_white_balance").as_bool()),
                          "allied_narrow_camera_auto_white_balance");
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(m_devices[0], *m_allied_narrow_sensor,
                                                                      this->get_parameter("allied_narrow_camera_balance_ratio_selector").as_int()),
                          "allied_narrow_camera_balance_ratio_selector");
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_RATIO(m_devices[0], *m_allied_narrow_sensor,
                                                             this->get_parameter("allied_narrow_camera_balance_ratio").as_double()),
                          "allied_narrow_camera_balance_ratio");
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(m_devices[0], *m_allied_narrow_sensor,
                                                                       this->get_parameter("allied_narrow_camera_balance_white_auto_rate").as_double()),
                          "allied_narrow_camera_balance_white_auto_rate");
        printDefaultError(CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(m_devices[0], *m_allied_narrow_sensor,
                                                                            this->get_parameter("allied_narrow_camera_balance_white_auto_tolerance").as_double()),
                          "allied_narrow_camera_balance_white_auto_tolerance");
        printDefaultError(CHANGE_ALLIED_CAMERA_AUTO_MODE_REGION(m_devices[0], *m_allied_narrow_sensor,
                                                                this->get_parameter("allied_narrow_camera_auto_mode_region_height").as_int(),
                                                                this->get_parameter("allied_narrow_camera_auto_mode_region_width").as_int()),
                          "allied_narrow_camera_auto_mode_region");
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(m_devices[0], *m_allied_narrow_sensor,
                                                                           this->get_parameter("allied_narrow_camera_intensity_controller_region").as_int()),
                          "allied_narrow_camera_intensity_controller_region");
        printDefaultError(CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(m_devices[0], *m_allied_narrow_sensor,
                                                                           this->get_parameter("allied_narrow_camera_intensity_controller_target").as_double()),
                          "allied_narrow_camera_intensity_controller_target");
        printDefaultError(CHANGE_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(m_devices[0], *m_allied_narrow_sensor,
                                                                        this->get_parameter("allied_narrow_camera_max_driver_buffers_count").as_int()),
                          "allied_narrow_camera_max_driver_buffers_count");
        if (this->get_parameter("allied_narrow_streaming_protocol").as_int() == 1)
        {
            m_allied_narrow_sensor->protocol = protocol_gstreamer;
            printDefaultError(CHANGE_STREAMING_PROTOCOL(m_devices[0], m_allied_narrow_sensor), "allied_narrow_streaming_protocol");
        }
        if (this->get_parameter("allied_narrow_rtsp_pipeline").as_string() != "")
        {
            char *pipeline = &std::string(this->get_parameter("allied_narrow_rtsp_pipeline").as_string())[0];
            printDefaultError(CHANGE_RTSP_PIPELINE(m_devices[0], *m_allied_narrow_sensor, pipeline), "allied_narrow_rtsp_pipeline");
        }
    }

    // Service callbacks
    void L3Cam::libL3camStatus(const std::shared_ptr<l3cam_interfaces::srv::LibL3camStatus::Request> req,
                               std::shared_ptr<l3cam_interfaces::srv::LibL3camStatus::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->status = (int)m_status;
    }

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
        res->error = STOP_STREAM(m_devices[0]);
        if (!res->error)
        {
            m_status = LibL3CamStatus::started;
            res->error = STOP_DEVICE(m_devices[0]);
        }
        if (!res->error)
        {
            m_status = LibL3CamStatus::connected;
            res->error = TERMINATE(m_devices[0]);
            if (!res->error)
            {
                m_status = LibL3CamStatus::terminated;
                disconnectAll(0);
                rclcpp::shutdown();
            }
        }
    }

    void L3Cam::findDevices(const std::shared_ptr<l3cam_interfaces::srv::FindDevices::Request> req,
                            std::shared_ptr<l3cam_interfaces::srv::FindDevices::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = FIND_DEVICES(&m_devices[0], &res->num_devices);
    }

    void L3Cam::getLocalServerAddress(const std::shared_ptr<l3cam_interfaces::srv::GetLocalServerAddress::Request> req,
                                      std::shared_ptr<l3cam_interfaces::srv::GetLocalServerAddress::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->local_ip_address = GET_LOCAL_SERVER_ADDRESS(m_devices[0]);
    }

    void L3Cam::getDeviceInfo(const std::shared_ptr<l3cam_interfaces::srv::GetDeviceInfo::Request> req,
                              std::shared_ptr<l3cam_interfaces::srv::GetDeviceInfo::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->ip_address = std::string(m_devices[0].ip_address);
        res->model = m_devices[0].model;
        res->serial_number = std::string(m_devices[0].serial_number);
        res->app_version = std::string(m_devices[0].app_version);
    }

    void L3Cam::getDeviceStatus(const std::shared_ptr<l3cam_interfaces::srv::GetDeviceStatus::Request> req,
                                std::shared_ptr<l3cam_interfaces::srv::GetDeviceStatus::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = GET_DEVICE_STATUS(m_devices[0], &res->system_status);
    }

    void L3Cam::getSensorsAvailable(const std::shared_ptr<l3cam_interfaces::srv::GetSensorsAvailable::Request> req,
                                    std::shared_ptr<l3cam_interfaces::srv::GetSensorsAvailable::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = GET_SENSORS_AVAILABLE(m_devices[0], m_av_sensors, &res->num_sensors);
        res->sensors.resize(res->num_sensors);
        for (int i = 0; i < res->num_sensors; ++i)
        {
            res->sensors[i].protocol = m_av_sensors[i].protocol;
            res->sensors[i].sensor_type = m_av_sensors[i].sensor_type;
            res->sensors[i].sensor_status = m_av_sensors[i].sensor_status;
            res->sensors[i].image_type = m_av_sensors[i].image_type;
            res->sensors[i].perception_enabled = m_av_sensors[i].perception_enabled;
            res->sensors[i].sensor_available = m_av_sensors[i].sensor_available;
        }
    }

    void L3Cam::changeStreamingProtocol(const std::shared_ptr<l3cam_interfaces::srv::ChangeStreamingProtocol::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::ChangeStreamingProtocol::Response> res)
    {
        STOP_STREAM(m_devices[0]);
        m_status = LibL3CamStatus::started;

        streamingProtocols protocol;
        switch (req->protocol)
        {
        case 0:
            protocol = protocol_raw_udp;
            break;
        case 1:
            protocol = protocol_gstreamer;
            break;
        default:
            protocol = protocol_raw_udp;
            break;
        }

        switch (req->sensor_type)
        {
        case ((int)sensorTypes::sensor_lidar):
            m_lidar_sensor->protocol = protocol;
            res->error = CHANGE_STREAMING_PROTOCOL(m_devices[0], m_lidar_sensor);
            break;
        case ((int)sensorTypes::sensor_pol):
            m_polarimetric_sensor->protocol = protocol;
            res->error = CHANGE_STREAMING_PROTOCOL(m_devices[0], m_polarimetric_sensor);
            break;
        case ((int)sensorTypes::sensor_econ_rgb):
            m_rgb_sensor->protocol = protocol;
            res->error = CHANGE_STREAMING_PROTOCOL(m_devices[0], m_rgb_sensor);
            break;
        case ((int)sensorTypes::sensor_thermal):
            m_thermal_sensor->protocol = protocol;
            res->error = CHANGE_STREAMING_PROTOCOL(m_devices[0], m_thermal_sensor);
            break;
        case ((int)sensorTypes::sensor_allied_wide):
            m_allied_wide_sensor->protocol = protocol;
            res->error = CHANGE_STREAMING_PROTOCOL(m_devices[0], m_allied_wide_sensor);
            break;
        case ((int)sensorTypes::sensor_allied_narrow):
            m_allied_narrow_sensor->protocol = protocol;
            res->error = CHANGE_STREAMING_PROTOCOL(m_devices[0], m_allied_narrow_sensor);
            break;
        }

        START_STREAM(m_devices[0]);
        m_status = LibL3CamStatus::streaming;
    }

    void L3Cam::getRtspPipeline(const std::shared_ptr<l3cam_interfaces::srv::GetRtspPipeline::Request> req,
                                std::shared_ptr<l3cam_interfaces::srv::GetRtspPipeline::Response> res)
    {
        char *pipeline = NULL;
        switch (req->sensor_type)
        {
        case (int)sensorTypes::sensor_lidar:
            res->error = GET_RTSP_PIPELINE(m_devices[0], *m_lidar_sensor, &pipeline);
            res->pipeline = std::string(pipeline);
            break;
        case (int)sensorTypes::sensor_pol:
            res->error = GET_RTSP_PIPELINE(m_devices[0], *m_polarimetric_sensor, &pipeline);
            res->pipeline = std::string(pipeline);
            break;
        case (int)sensorTypes::sensor_econ_rgb:
            res->error = GET_RTSP_PIPELINE(m_devices[0], *m_rgb_sensor, &pipeline);
            res->pipeline = std::string(pipeline);
            break;
        case (int)sensorTypes::sensor_thermal:
            res->error = GET_RTSP_PIPELINE(m_devices[0], *m_thermal_sensor, &pipeline);
            res->pipeline = std::string(pipeline);
            break;
        case (int)sensorTypes::sensor_allied_wide:
            res->error = GET_RTSP_PIPELINE(m_devices[0], *m_allied_wide_sensor, &pipeline);
            res->pipeline = std::string(pipeline);
            break;
        case (int)sensorTypes::sensor_allied_narrow:
            res->error = GET_RTSP_PIPELINE(m_devices[0], *m_allied_narrow_sensor, &pipeline);
            res->pipeline = std::string(pipeline);
            break;
        }
    }

    void L3Cam::getNetworkConfiguration(const std::shared_ptr<l3cam_interfaces::srv::GetNetworkConfiguration::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::GetNetworkConfiguration::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        char *ip_address = NULL;
        char *netmask = NULL;
        char *gateway = NULL;
        res->error = GET_NETWORK_CONFIGURATION(m_devices[0], &ip_address, &netmask, &gateway);
        res->ip_address = std::string(ip_address);
        res->netmask = std::string(netmask);
        res->gateway = std::string(gateway);
    }

    void L3Cam::changeNetworkConfiguration(const std::shared_ptr<l3cam_interfaces::srv::ChangeNetworkConfiguration::Request> req,
                                           std::shared_ptr<l3cam_interfaces::srv::ChangeNetworkConfiguration::Response> res)
    {
        if (req->enable_dhcp)
            res->error = CHANGE_NETWORK_CONFIGURATION(m_devices[0], NULL, NULL, NULL, true);
        else
        {
            std::string ip_address = req->ip_address;
            std::string netmask = req->netmask;
            std::string gateway = req->gateway;
            res->error = CHANGE_NETWORK_CONFIGURATION(m_devices[0], (char *)ip_address.data(), (char *)netmask.data(), (char *)gateway.data(), false);
        }
    }

    void L3Cam::powerOffDevice(const std::shared_ptr<l3cam_interfaces::srv::PowerOffDevice::Request> req,
                               std::shared_ptr<l3cam_interfaces::srv::PowerOffDevice::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        POWER_OFF_DEVICE(m_devices[0]);
        res->error = 0;
    }

    void L3Cam::startDevice(const std::shared_ptr<l3cam_interfaces::srv::StartDevice::Request> req,
                            std::shared_ptr<l3cam_interfaces::srv::StartDevice::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = START_DEVICE(m_devices[0]);
        if (!res->error)
        {
            m_status = LibL3CamStatus::started;
        }
    }

    void L3Cam::stopDevice(const std::shared_ptr<l3cam_interfaces::srv::StopDevice::Request> req,
                           std::shared_ptr<l3cam_interfaces::srv::StopDevice::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = STOP_STREAM(m_devices[0]);
        if (!res->error)
        {
            m_status = LibL3CamStatus::started;
            res->error = STOP_DEVICE(m_devices[0]);
            if (!res->error)
            {
                m_status = LibL3CamStatus::connected;
            }
        }
    }

    void L3Cam::startStream(const std::shared_ptr<l3cam_interfaces::srv::StartStream::Request> req,
                            std::shared_ptr<l3cam_interfaces::srv::StartStream::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = START_STREAM(m_devices[0]);
        if (!res->error)
        {
            m_status = LibL3CamStatus::streaming;
        }
    }

    void L3Cam::stopStream(const std::shared_ptr<l3cam_interfaces::srv::StopStream::Request> req,
                           std::shared_ptr<l3cam_interfaces::srv::StopStream::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = STOP_STREAM(m_devices[0]);
        if (!res->error)
        {
            m_status = LibL3CamStatus::started;
        }
    }

    void L3Cam::getDeviceTemperatures(const std::shared_ptr<l3cam_interfaces::srv::GetDeviceTemperatures::Request> req,
                                      std::shared_ptr<l3cam_interfaces::srv::GetDeviceTemperatures::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        int32_t *temperatures = (int32_t *)malloc(sizeof(int32_t) * 11);
        int error = GET_DEVICE_TEMPERATURES(m_devices[0], temperatures);
        res->error = error;
        if (error != L3CAM_OK)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Error " << error << " in temperatures error: " << getErrorDescription(error));
            res->bcpu_temp = 0;
            res->mcpu_temp = 0;
            res->gpu_temp = 0;
            res->pll_temp = 0;
            res->board_temp = 0;
            res->diode_temp = 0;
            res->pmic_temp = 0;
            res->fan_temp = 0;
            res->inter_temp = 0;
            res->allied_wide_temp = 0;
            res->allied_narrow_temp = 0;
        }
        else
        {
            res->bcpu_temp = temperatures[0] / 1000.0;
            res->mcpu_temp = temperatures[1] / 1000.0;
            res->gpu_temp = temperatures[2] / 1000.0;
            res->pll_temp = temperatures[3] / 1000.0;
            res->board_temp = temperatures[4] / 1000.0;
            res->diode_temp = temperatures[5] / 1000.0;
            res->pmic_temp = temperatures[6] / 1000.0;
            res->fan_temp = temperatures[7] / 1000.0;
            res->inter_temp = temperatures[8] / 1000.0;
            res->allied_wide_temp = temperatures[9] / 1000.0;
            res->allied_narrow_temp = temperatures[10] / 1000.0;
        }
        free(temperatures);
    }

    // Point Cloud
    void L3Cam::changePointcloudColor(const std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColor::Request> req,
                                      std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColor::Response> res)
    {
        res->error = CHANGE_POINT_CLOUD_COLOR(m_devices[0], req->visualization_color);
    }

    void L3Cam::changePointcloudColorRange(const std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColorRange::Request> req,
                                           std::shared_ptr<l3cam_interfaces::srv::ChangePointcloudColorRange::Response> res)
    {
        res->error = CHANGE_POINT_CLOUD_COLOR_RANGE(m_devices[0], req->min_value, req->max_value);
    }

    void L3Cam::changeDistanceRange(const std::shared_ptr<l3cam_interfaces::srv::ChangeDistanceRange::Request> req,
                                    std::shared_ptr<l3cam_interfaces::srv::ChangeDistanceRange::Response> res)
    {
        res->error = CHANGE_DISTANCE_RANGE(m_devices[0], req->min_value, req->max_value);
    }

    void L3Cam::enableAutoBias(const std::shared_ptr<l3cam_interfaces::srv::EnableAutoBias::Request> req,
                               std::shared_ptr<l3cam_interfaces::srv::EnableAutoBias::Response> res)
    {
        ROS2_BMG_UNUSED(res);
        ENABLE_AUTO_BIAS(m_devices[0], req->enabled);
    }

    void L3Cam::changeBiasValue(const std::shared_ptr<l3cam_interfaces::srv::ChangeBiasValue::Request> req,
                                std::shared_ptr<l3cam_interfaces::srv::ChangeBiasValue::Response> res)
    {
        ROS2_BMG_UNUSED(res);
        CHANGE_BIAS_VALUE(m_devices[0], req->index, req->bias);
    }

    // Polarimetric
    void L3Cam::setPolarimetricCameraDefaultSettings(const std::shared_ptr<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings::Request> req,
                                                     std::shared_ptr<l3cam_interfaces::srv::SetPolarimetricCameraDefaultSettings::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = SET_POLARIMETRIC_CAMERA_DEFAULT_SETTINGS(m_devices[0]);
    }

    void L3Cam::changePolarimetricCameraBrightness(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness::Request> req,
                                                   std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBrightness::Response> res)
    {
        res->error = CHANGE_POLARIMETRIC_CAMERA_BRIGHTNESS(m_devices[0], req->brightness);
    }

    void L3Cam::changePolarimetricCameraBlackLevel(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel::Request> req,
                                                   std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraBlackLevel::Response> res)
    {
        res->error = CHANGE_POLARIMETRIC_CAMERA_BLACK_LEVEL(m_devices[0], req->black_level);
    }

    void L3Cam::enablePolarimetricCameraAutoGain(const std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain::Request> req,
                                                 std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoGain::Response> res)
    {
        res->error = ENABLE_POLARIMETRIC_CAMERA_AUTO_GAIN(m_devices[0], req->enabled);
    }

    void L3Cam::changePolarimetricCameraAutoGainRange(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange::Request> req,
                                                      std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoGainRange::Response> res)
    {
        res->error = CHANGE_POLARIMETRIC_CAMERA_AUTO_GAIN_RANGE(m_devices[0], req->min_gain, req->max_gain);
    }

    void L3Cam::changePolarimetricCameraGain(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraGain::Request> req,
                                             std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraGain::Response> res)
    {
        res->error = CHANGE_POLARIMETRIC_CAMERA_GAIN(m_devices[0], req->gain);
    }

    void L3Cam::enablePolarimetricCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime::Request> req,
                                                         std::shared_ptr<l3cam_interfaces::srv::EnablePolarimetricCameraAutoExposureTime::Response> res)
    {
        res->error = ENABLE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], req->enabled);
    }

    void L3Cam::changePolarimetricCameraAutoExposureTimeRange(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange::Request> req,
                                                              std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraAutoExposureTimeRange::Response> res)
    {
        res->error = CHANGE_POLARIMETRIC_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], req->min_exposure, req->max_exposure);
    }

    void L3Cam::changePolarimetricCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime::Request> req,
                                                     std::shared_ptr<l3cam_interfaces::srv::ChangePolarimetricCameraExposureTime::Response> res)
    {
        res->error = CHANGE_POLARIMETRIC_CAMERA_EXPOSURE_TIME(m_devices[0], req->exposure_time);
    }

    // RGB
    void L3Cam::setRgbCameraDefaultSettings(const std::shared_ptr<l3cam_interfaces::srv::SetRgbCameraDefaultSettings::Request> req,
                                            std::shared_ptr<l3cam_interfaces::srv::SetRgbCameraDefaultSettings::Response> res)
    {
        ROS2_BMG_UNUSED(req);
        res->error = SET_RGB_CAMERA_DEFAULT_SETTINGS(m_devices[0]);
    }

    void L3Cam::changeRgbCameraBrightness(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraBrightness::Request> req,
                                          std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraBrightness::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_BRIGHTNESS(m_devices[0], req->brightness);
    }

    void L3Cam::changeRgbCameraContrast(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraContrast::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraContrast::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_CONTRAST(m_devices[0], req->contrast);
    }

    void L3Cam::changeRgbCameraSaturation(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSaturation::Request> req,
                                          std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSaturation::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_SATURATION(m_devices[0], req->saturation);
    }

    void L3Cam::changeRgbCameraSharpness(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSharpness::Request> req,
                                         std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraSharpness::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_SHARPNESS(m_devices[0], req->sharpness);
    }

    void L3Cam::changeRgbCameraGamma(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGamma::Request> req,
                                     std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGamma::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_GAMMA(m_devices[0], req->gamma);
    }

    void L3Cam::changeRgbCameraGain(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGain::Request> req,
                                    std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraGain::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_GAIN(m_devices[0], req->gain);
    }

    void L3Cam::enableRgbCameraAutoWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance::Request> req,
                                                std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoWhiteBalance::Response> res)
    {
        res->error = ENABLE_RGB_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], req->enabled);
    }

    void L3Cam::changeRgbCameraWhiteBalance(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance::Request> req,
                                            std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraWhiteBalance::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_WHITE_BALANCE(m_devices[0], req->white_balance);
    }

    void L3Cam::enableRgbCameraAutoExposureTime(const std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime::Request> req,
                                                std::shared_ptr<l3cam_interfaces::srv::EnableRgbCameraAutoExposureTime::Response> res)
    {
        res->error = ENABLE_RGB_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], req->enabled);
    }

    void L3Cam::changeRgbCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraExposureTime::Request> req,
                                            std::shared_ptr<l3cam_interfaces::srv::ChangeRgbCameraExposureTime::Response> res)
    {
        res->error = CHANGE_RGB_CAMERA_EXPOSURE_TIME(m_devices[0], req->exposure_time);
    }

    // Thermal
    void L3Cam::changeThermalCameraColormap(const std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraColormap::Request> req,
                                            std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraColormap::Response> res)
    {
        res->error = CHANGE_THERMAL_CAMERA_COLORMAP(m_devices[0], req->colormap);
    }

    void L3Cam::enableThermalCameraTemperatureFilter(const std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter::Request> req,
                                                     std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter::Response> res)
    {
        res->error = ENABLE_THERMAL_CAMERA_TEMPERATURE_FILTER(m_devices[0], req->enabled);
    }

    void L3Cam::changeThermalCameraTemperatureFilter(const std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter::Request> req,
                                                     std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter::Response> res)
    {
        res->error = CHANGE_THERMAL_CAMERA_TEMPERATURE_FILTER(m_devices[0], req->min_temperature, req->max_temperature);
    }

    void L3Cam::changeThermalCameraProcessingPipeline(const std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraProcessingPipeline::Request> req,
                                                     std::shared_ptr<l3cam_interfaces::srv::ChangeThermalCameraProcessingPipeline::Response> res)
    {
        res->error = CHANGE_THERMAL_CAMERA_PROCESSING_PIPELINE(m_devices[0], req->pipeline);
    }

    void L3Cam::enableThermalCameraTemperatureDataUdp(const std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureDataUdp::Request> req,
                                                     std::shared_ptr<l3cam_interfaces::srv::EnableThermalCameraTemperatureDataUdp::Response> res)
    {
        res->error = ENABLE_THERMAL_CAMERA_TEMPERATURE_DATA_UDP(m_devices[0], req->enabled);
    }

    // Allied
    void L3Cam::changeAlliedCameraExposureTime(const std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime::Request> req,
                                               std::shared_ptr<l3cam_interfaces::srv::ChangeAlliedCameraExposureTime::Response> res)
    {
        switch (req->allied_type)
        {
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(m_devices[0], *m_allied_wide_sensor, req->exposure_time);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_EXPOSURE_TIME_US(m_devices[0], *m_allied_narrow_sensor, req->exposure_time);
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
        case alliedCamerasIds::wide_camera:
            res->error = ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], *m_allied_wide_sensor, req->enabled);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = ENABLE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], *m_allied_narrow_sensor, req->enabled);
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
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], *m_allied_wide_sensor, req->auto_exposure_time_range_min, req->auto_exposure_time_range_max);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], *m_allied_narrow_sensor, req->auto_exposure_time_range_min, req->auto_exposure_time_range_max);
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
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_GAIN(m_devices[0], *m_allied_wide_sensor, req->gain);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_GAIN(m_devices[0], *m_allied_narrow_sensor, req->gain);
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
        case alliedCamerasIds::wide_camera:
            res->error = ENABLE_ALLIED_CAMERA_AUTO_GAIN(m_devices[0], *m_allied_wide_sensor, req->enabled);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = ENABLE_ALLIED_CAMERA_AUTO_GAIN(m_devices[0], *m_allied_narrow_sensor, req->enabled);
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
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(m_devices[0], *m_allied_wide_sensor, (float)req->auto_gain_range_min, (float)req->auto_gain_range_max);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_AUTO_GAIN_RANGE(m_devices[0], *m_allied_narrow_sensor, (float)req->auto_gain_range_min, (float)req->auto_gain_range_max);
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
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_GAMMA(m_devices[0], *m_allied_wide_sensor, req->gamma);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_GAMMA(m_devices[0], *m_allied_narrow_sensor, req->gamma);
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
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_SATURATION(m_devices[0], *m_allied_wide_sensor, req->saturation);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_SATURATION(m_devices[0], *m_allied_narrow_sensor, req->saturation);
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
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_HUE(m_devices[0], *m_allied_wide_sensor, req->hue);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_HUE(m_devices[0], *m_allied_narrow_sensor, req->hue);
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
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(m_devices[0], *m_allied_wide_sensor, req->intensity_auto_precedence);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(m_devices[0], *m_allied_narrow_sensor, req->intensity_auto_precedence);
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
        case alliedCamerasIds::wide_camera:
            res->error = ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], *m_allied_wide_sensor, req->enabled);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = ENABLE_ALLIED_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], *m_allied_narrow_sensor, req->enabled);
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
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(m_devices[0], *m_allied_wide_sensor, req->white_balance_ratio_selector);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(m_devices[0], *m_allied_narrow_sensor, req->white_balance_ratio_selector);
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
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO(m_devices[0], *m_allied_wide_sensor, req->balance_ratio);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_BALANCE_RATIO(m_devices[0], *m_allied_narrow_sensor, req->balance_ratio);
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
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(m_devices[0], *m_allied_wide_sensor, req->white_balance_auto_rate);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(m_devices[0], *m_allied_narrow_sensor, req->white_balance_auto_rate);
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
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(m_devices[0], *m_allied_wide_sensor, req->white_balance_auto_tolerance);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(m_devices[0], *m_allied_narrow_sensor, req->white_balance_auto_tolerance);
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
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(m_devices[0], *m_allied_wide_sensor, req->intensity_controller_region);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(m_devices[0], *m_allied_narrow_sensor, req->intensity_controller_region);
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
        case alliedCamerasIds::wide_camera:
            res->error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(m_devices[0], *m_allied_wide_sensor, req->intensity_controller_target);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = CHANGE_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(m_devices[0], *m_allied_narrow_sensor, req->intensity_controller_target);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_BLACK_LEVEL(m_devices[0], *m_allied_wide_sensor, &res->black_level);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_BLACK_LEVEL(m_devices[0], *m_allied_narrow_sensor, &res->black_level);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_EXPOSURE_TIME_US(m_devices[0], *m_allied_wide_sensor, &res->exposure_time);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_EXPOSURE_TIME_US(m_devices[0], *m_allied_narrow_sensor, &res->exposure_time);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], *m_allied_wide_sensor, &res->enabled);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_AUTO_EXPOSURE_TIME(m_devices[0], *m_allied_narrow_sensor, &res->enabled);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], *m_allied_wide_sensor, &res->auto_exposure_time_range_min, &res->auto_exposure_time_range_max);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_AUTO_EXPOSURE_TIME_RANGE(m_devices[0], *m_allied_narrow_sensor, &res->auto_exposure_time_range_min, &res->auto_exposure_time_range_max);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_GAIN(m_devices[0], *m_allied_wide_sensor, &res->gain);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_GAIN(m_devices[0], *m_allied_narrow_sensor, &res->gain);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_AUTO_GAIN(m_devices[0], *m_allied_wide_sensor, &res->enabled);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_AUTO_GAIN(m_devices[0], *m_allied_narrow_sensor, &res->enabled);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_AUTO_GAIN_RANGE(m_devices[0], *m_allied_wide_sensor, &res->auto_gain_range_min, &res->auto_gain_range_max);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_AUTO_GAIN_RANGE(m_devices[0], *m_allied_narrow_sensor, &res->auto_gain_range_min, &res->auto_gain_range_max);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_GAMMA(m_devices[0], *m_allied_wide_sensor, &res->gamma);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_GAMMA(m_devices[0], *m_allied_narrow_sensor, &res->gamma);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_SATURATION(m_devices[0], *m_allied_wide_sensor, &res->saturation);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_SATURATION(m_devices[0], *m_allied_narrow_sensor, &res->saturation);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_SHARPNESS(m_devices[0], *m_allied_wide_sensor, &res->sharpness);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_SHARPNESS(m_devices[0], *m_allied_narrow_sensor, &res->sharpness);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_HUE(m_devices[0], *m_allied_wide_sensor, &res->hue);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_HUE(m_devices[0], *m_allied_narrow_sensor, &res->hue);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(m_devices[0], *m_allied_wide_sensor, &res->intensity_auto_precedence);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_INTENSITY_AUTO_PRECEDENCE(m_devices[0], *m_allied_narrow_sensor, &res->intensity_auto_precedence);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], *m_allied_wide_sensor, &res->enabled);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_AUTO_WHITE_BALANCE(m_devices[0], *m_allied_narrow_sensor, &res->enabled);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(m_devices[0], *m_allied_wide_sensor, &res->white_balance_ratio_selector);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_BALANCE_RATIO_SELECTOR(m_devices[0], *m_allied_narrow_sensor, &res->white_balance_ratio_selector);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_BALANCE_RATIO(m_devices[0], *m_allied_wide_sensor, &res->balance_ratio);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_BALANCE_RATIO(m_devices[0], *m_allied_narrow_sensor, &res->balance_ratio);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(m_devices[0], *m_allied_wide_sensor, &res->white_balance_auto_rate);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_BALANCE_WHITE_AUTO_RATE(m_devices[0], *m_allied_narrow_sensor, &res->white_balance_auto_rate);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(m_devices[0], *m_allied_wide_sensor, &res->white_balance_auto_tolerance);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_BALANCE_WHITE_AUTO_TOLERANCE(m_devices[0], *m_allied_narrow_sensor, &res->white_balance_auto_tolerance);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_AUTO_MODE_REGION(m_devices[0], *m_allied_wide_sensor, &res->height, &res->width);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_AUTO_MODE_REGION(m_devices[0], *m_allied_narrow_sensor, &res->height, &res->width);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(m_devices[0], *m_allied_wide_sensor, &res->intensity_controller_region);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_INTENSITY_CONTROLLER_REGION(m_devices[0], *m_allied_narrow_sensor, &res->intensity_controller_region);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(m_devices[0], *m_allied_wide_sensor, &res->intensity_controller_target);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_INTENSITY_CONTROLLER_TARGET(m_devices[0], *m_allied_narrow_sensor, &res->intensity_controller_target);
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
        case alliedCamerasIds::wide_camera:
            res->error = GET_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(m_devices[0], *m_allied_wide_sensor, &res->max_driver_buffers_count);
            break;
        case alliedCamerasIds::narrow_camera:
            res->error = GET_ALLIED_CAMERA_MAX_DRIVER_BUFFERS_COUNT(m_devices[0], *m_allied_narrow_sensor, &res->max_driver_buffers_count);
            break;
        default:
            res->error = L3CAM_VALUE_OUT_OF_RANGE;
        }
    }

    // Sensors Disconnection
    void L3Cam::networkDisconnected(int code)
    {
        while (!client_network_disconnected_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                break;
            }
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto requestNetworkDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestNetworkDisconnected->code = code;

        auto resultNetworkDisconnected = client_network_disconnected_->async_send_request(requestNetworkDisconnected);
    }

    void L3Cam::lidarDisconnected(int code)
    {
        // Stream
        while (!client_lidar_stream_disconnected_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                break;
            }
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto requestLidarStreamDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestLidarStreamDisconnected->code = code;

        auto resultLidarStreamDisconnected = client_lidar_stream_disconnected_->async_send_request(requestLidarStreamDisconnected);

        // Configuration
        while (!client_lidar_configuration_disconnected_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                break;
            }
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto requestLidarConfigurationDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestLidarConfigurationDisconnected->code = code;

        auto resultLidarConfigurationDisconnected = client_lidar_configuration_disconnected_->async_send_request(requestLidarConfigurationDisconnected);
    }

    void L3Cam::polDisconnected(int code)
    {
        // Stream
        while (!client_pol_wide_stream_disconnected_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                break;
            }
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto requestPolWideStreamDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestPolWideStreamDisconnected->code = code;

        auto resultPolWideStreamDisconnected = client_pol_wide_stream_disconnected_->async_send_request(requestPolWideStreamDisconnected);

        // Configuration
        while (!client_pol_configuration_disconnected_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                break;
            }
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto requestPolConfigurationDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestPolConfigurationDisconnected->code = code;

        auto resultPolConfigurationDisconnected = client_pol_configuration_disconnected_->async_send_request(requestPolConfigurationDisconnected);
    }

    void L3Cam::rgbDisconnected(int code)
    {
        // Stream
        while (!client_rgb_narrow_stream_disconnected_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                break;
            }
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto requestRgbNarrowStreamDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestRgbNarrowStreamDisconnected->code = code;

        auto resultRgbNarrowStreamDisconnected = client_rgb_narrow_stream_disconnected_->async_send_request(requestRgbNarrowStreamDisconnected);

        // Configuration
        while (!client_rgb_configuration_disconnected_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                break;
            }
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto requestRgbConfigurationDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestRgbConfigurationDisconnected->code = code;

        auto resultRgbConfigurationDisconnected = client_rgb_configuration_disconnected_->async_send_request(requestRgbConfigurationDisconnected);
    }

    void L3Cam::thermalDisconnected(int code)
    {
        // Stream
        while (!client_thermal_stream_disconnected_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                break;
            }
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto requestThermalStreamDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestThermalStreamDisconnected->code = code;

        auto resultThermalStreamDisconnected = client_thermal_stream_disconnected_->async_send_request(requestThermalStreamDisconnected);

        // Configuration
        while (!client_thermal_configuration_disconnected_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                break;
            }
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto requestThermalConfigurationDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestThermalConfigurationDisconnected->code = code;

        auto resultThermalConfigurationDisconnected = client_thermal_configuration_disconnected_->async_send_request(requestThermalConfigurationDisconnected);
    }

    void L3Cam::alliedwideDisconnected(int code)
    {
        // Stream
        while (!client_pol_wide_stream_disconnected_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                break;
            }
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto requestPolWideStreamDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestPolWideStreamDisconnected->code = code;

        auto resultPolWideStreamDisconnected = client_pol_wide_stream_disconnected_->async_send_request(requestPolWideStreamDisconnected);

        // Configuration
        while (!client_wide_configuration_disconnected_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                break;
            }
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto requestWideConfigurationDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestWideConfigurationDisconnected->code = code;

        auto resultWideConfigurationDisconnected = client_wide_configuration_disconnected_->async_send_request(requestWideConfigurationDisconnected);
    }

    void L3Cam::alliedNarrowDisconnect(int code)
    {
        // Stream
        while (!client_rgb_narrow_stream_disconnected_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                break;
            }
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto requestRgbStreamDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestRgbStreamDisconnected->code = code;

        auto resultRgbStreamDisconnected = client_rgb_narrow_stream_disconnected_->async_send_request(requestRgbStreamDisconnected);

        // Configuration
        while (!client_narrow_configuration_disconnected_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                break;
            }
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto requestNarrowConfigurationDisconnected = std::make_shared<l3cam_interfaces::srv::SensorDisconnected::Request>();
        requestNarrowConfigurationDisconnected->code = code;

        auto resultNarrowConfigurationDisconnected = client_narrow_configuration_disconnected_->async_send_request(requestNarrowConfigurationDisconnected);
    }

    void L3Cam::errorNotification(const int32_t *error)
    {
        // RCLCPP_INFO(this->get_logger(), "Error notification received");
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

    node = std::make_shared<l3cam_ros2::L3Cam>();

    int error = L3CAM_OK;
    error = node->initializeDevice();
    if (error)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR " << error << " while initializing device: " << getErrorDescription(error));
        node->disconnectAll(error);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Terminating...");
        TERMINATE(node->m_devices[0]);
        node->m_status = LibL3CamStatus::terminated;
        node = NULL; //! Without this, the node becomes zombie
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Terminated.");
        return error;
    }

    error = node->startDeviceStream();
    if (error)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR " << error << " while starting device and stream: " << getErrorDescription(error));
        if (error == L3CAM_TIMEOUT_ERROR)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Device is not " << (node->m_status == connected ? "started." : "streaming."));
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Terminating...");
            node->disconnectAll(error);
            STOP_STREAM(node->m_devices[0]);
            STOP_DEVICE(node->m_devices[0]);
            TERMINATE(node->m_devices[0]);
            node->m_status = LibL3CamStatus::terminated;
            node = NULL; //! Without this, the node becomes zombie
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Terminated.");
            return error;
        }
    }

    rclcpp::spin(node);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Terminating...");
    // Before exiting stop stream, device and terminate
    STOP_STREAM(node->m_devices[0]);
    STOP_DEVICE(node->m_devices[0]);
    TERMINATE(node->m_devices[0]);
    node->m_status = LibL3CamStatus::terminated;
    node = NULL; //! Without this, the node becomes zombie
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Terminated.");

    rclcpp::shutdown();
    return 0;
}
