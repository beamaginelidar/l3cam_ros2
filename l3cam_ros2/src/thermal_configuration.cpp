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

#include "l3cam_interfaces/srv/change_thermal_camera_colormap.hpp"
#include "l3cam_interfaces/srv/enable_thermal_camera_temperature_filter.hpp"
#include "l3cam_interfaces/srv/change_thermal_camera_temperature_filter.hpp"

#include "l3cam_interfaces/srv/sensor_disconnected.hpp"

#include "l3cam_ros2_node.hpp" // for ROS2_BMG_UNUSED

using namespace std::chrono_literals;

namespace l3cam_ros2
{
    class ThermalConfiguration : public rclcpp::Node
    {
    public:
        ThermalConfiguration() : Node("thermal_configuration")
        {
            declarGetParameters();

            // Create service clients
            clientGetSensors = this->create_client<l3cam_interfaces::srv::GetSensorsAvailable>("get_sensors_available");
            clientColormap = this->create_client<l3cam_interfaces::srv::ChangeThermalCameraColormap>("change_thermal_camera_colormap");
            clientTemperatureFilter = this->create_client<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter>("enable_thermal_camera_temperature_filter");
            clientTemperatureFilterRange = this->create_client<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter>("change_thermal_camera_temperature_filter");

            // Create service server
            srvSensorDisconnected = this->create_service<l3cam_interfaces::srv::SensorDisconnected>(
                "thermal_configuration_disconnected", std::bind(&ThermalConfiguration::sensorDisconnectedCallback, this, std::placeholders::_1, std::placeholders::_2));

            // Callback on parameters changed
            callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&ThermalConfiguration::parametersCallback, this, std::placeholders::_1));
        }

        rclcpp::Client<l3cam_interfaces::srv::GetSensorsAvailable>::SharedPtr clientGetSensors;

    private:
        void declarGetParameters()
        {
            // Declare parameters with range
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            rcl_interfaces::msg::IntegerRange range;
            range.set__from_value(1).set__to_value(108).set__step(1); // TODO: dropdown menu thermalTypes
            descriptor.integer_range = {range};
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
            range.set__from_value(-40).set__to_value(200).set__step(1);
            descriptor.integer_range = {range};
            this->declare_parameter("thermal_camera_temperature_filter_min", 0, descriptor); // -40 - 200
            range.set__from_value(-40).set__to_value(200).set__step(1);
            descriptor.integer_range = {range};
            this->declare_parameter("thermal_camera_temperature_filter_max", 50, descriptor); // -40 - 200

            // Get and save parameters
            thermal_camera_colormap = this->get_parameter("thermal_camera_colormap").as_int();
            thermal_camera_temperature_filter = this->get_parameter("thermal_camera_temperature_filter").as_bool();
            thermal_camera_temperature_filter_min = this->get_parameter("thermal_camera_temperature_filter_min").as_int();
            thermal_camera_temperature_filter_max = this->get_parameter("thermal_camera_temperature_filter_max").as_int();
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
                if (param_name == "thermal_camera_colormap")
                {
                    while (!clientColormap->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestColormap = std::make_shared<l3cam_interfaces::srv::ChangeThermalCameraColormap::Request>();
                    requestColormap->colormap = param.as_int();

                    auto resultColormap = clientColormap->async_send_request(
                        requestColormap, std::bind(&ThermalConfiguration::colormapResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "thermal_camera_temperature_filter")
                {
                    while (!clientTemperatureFilter->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestTemperatureFilter = std::make_shared<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter::Request>();
                    requestTemperatureFilter->enabled = param.as_bool();

                    auto resultTemperatureFilter = clientTemperatureFilter->async_send_request(
                        requestTemperatureFilter, std::bind(&ThermalConfiguration::temperatureFilterResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "thermal_camera_temperature_filter_min")
                {
                    while (!clientTemperatureFilterRange->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestTemperatureFilterRange = std::make_shared<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter::Request>();
                    requestTemperatureFilterRange->min_temperature = param.as_int();
                    requestTemperatureFilterRange->max_temperature = thermal_camera_temperature_filter_max;

                    auto resultTemperatureFilterRange = clientTemperatureFilterRange->async_send_request(
                        requestTemperatureFilterRange, std::bind(&ThermalConfiguration::temperatureFilterRangeResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "thermal_camera_temperature_filter_max")
                {
                    while (!clientTemperatureFilterRange->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestTemperatureFilterRange = std::make_shared<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter::Request>();
                    requestTemperatureFilterRange->min_temperature = thermal_camera_temperature_filter_min;
                    requestTemperatureFilterRange->max_temperature = param.as_int();

                    auto resultTemperatureFilterRange = clientTemperatureFilterRange->async_send_request(
                        requestTemperatureFilterRange, std::bind(&ThermalConfiguration::temperatureFilterRangeResponseCallback, this, std::placeholders::_1));
                }
            }

            return result;
        }

        void colormapResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeThermalCameraColormap>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    thermal_camera_colormap = this->get_parameter("thermal_camera_colormap").as_int();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("thermal_camera_colormap", thermal_camera_colormap));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_thermal_camera_colormap");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("thermal_camera_colormap", thermal_camera_colormap));
            }
        }

        void temperatureFilterResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    thermal_camera_temperature_filter = this->get_parameter("thermal_camera_temperature_filter").as_bool();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    // this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_filter", thermal_camera_temperature_filter));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service enable_thermal_camera_temperature_filter");
                // Service could not be called, reset parameter to value before change
                // this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_filter", thermal_camera_temperature_filter));
            }
        }

        void temperatureFilterRangeResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameters changed successfully, save value
                    thermal_camera_temperature_filter_min = this->get_parameter("thermal_camera_temperature_filter_min").as_int();
                    thermal_camera_temperature_filter_max = this->get_parameter("thermal_camera_temperature_filter_max").as_int();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while changing parameter: " << getBeamErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    // this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_filter_min", thermal_camera_temperature_filter_min));
                    // this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_filter_max", thermal_camera_temperature_filter_max));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_thermal_camera_temperature_filter_min");
                // Service could not be called, reset parameters to value before change
                // this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_filter_min", thermal_camera_temperature_filter_min));
                // this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_filter_max", thermal_camera_temperature_filter_max));
            }
        }

        void sensorDisconnectedCallback(const std::shared_ptr<l3cam_interfaces::srv::SensorDisconnected::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::SensorDisconnected::Response> res)
        {
            ROS2_BMG_UNUSED(res);
            if (req->code == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Exiting cleanly.");
            }
            else
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Exiting. Sensor got disconnected with error " << req->code << ": " << getBeamErrorDescription(req->code));
            }

            rclcpp::shutdown();
        }

        int thermal_camera_colormap;
        bool thermal_camera_temperature_filter;
        int thermal_camera_temperature_filter_min;
        int thermal_camera_temperature_filter_max;

        rclcpp::Client<l3cam_interfaces::srv::ChangeThermalCameraColormap>::SharedPtr clientColormap;
        rclcpp::Client<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter>::SharedPtr clientTemperatureFilter;
        rclcpp::Client<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter>::SharedPtr clientTemperatureFilterRange;

        rclcpp::Service<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr srvSensorDisconnected;

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    }; // class ThermalConfiguration

} // namespace l3cam_ros2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::ThermalConfiguration> node = std::make_shared<l3cam_ros2::ThermalConfiguration>();

    // Check if Thermal is available
    while (!node->clientGetSensors->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
            return 0;
        }
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto requestGetSensors = std::make_shared<l3cam_interfaces::srv::GetSensorsAvailable::Request>();
    auto resultGetSensors = node->clientGetSensors->async_send_request(requestGetSensors);

    int error = L3CAM_OK;
    bool sensor_is_available = false;
    // Shutdown if sensor is not available or if error returned
    if (rclcpp::spin_until_future_complete(node, resultGetSensors) == rclcpp::FutureReturnCode::SUCCESS)
    {
        error = resultGetSensors.get()->error;

        if (!error)
        {
            for (int i = 0; i < resultGetSensors.get()->num_sensors; ++i)
            {
                if (resultGetSensors.get()->sensors[i].sensor_type == sensor_thermal)
                    sensor_is_available = true;
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while checking sensor availability: " << getBeamErrorDescription(error));
            return 1;
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_sensors_available");
        return 1;
    }

    if (sensor_is_available)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Thermal camera configuration is available");
    }
    else
    {
        return 0;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
