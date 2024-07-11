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
#include "l3cam_interfaces/srv/change_thermal_camera_processing_pipeline.hpp"
#include "l3cam_interfaces/srv/enable_thermal_camera_temperature_data_udp.hpp"
#include "l3cam_interfaces/srv/change_streaming_protocol.hpp"
#include "l3cam_interfaces/srv/get_rtsp_pipeline.hpp"

#include "l3cam_interfaces/srv/sensor_disconnected.hpp"

#include "l3cam_ros2_utils.hpp"

using namespace std::chrono_literals;

namespace l3cam_ros2
{
    class ThermalConfiguration : public rclcpp::Node
    {
    public:
        explicit ThermalConfiguration() : Node("thermal_configuration")
        {
            // Create service clients
            client_get_sensors_ = this->create_client<l3cam_interfaces::srv::GetSensorsAvailable>("get_sensors_available");
            client_colormap_ = this->create_client<l3cam_interfaces::srv::ChangeThermalCameraColormap>("change_thermal_camera_colormap");
            client_temperature_filter_ = this->create_client<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter>("enable_thermal_camera_temperature_filter");
            client_temperature_filter_range_ = this->create_client<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter>("change_thermal_camera_temperature_filter");
            client_processing_pipeline_ = this->create_client<l3cam_interfaces::srv::ChangeThermalCameraProcessingPipeline>("change_thermal_camera_processing_pipeline");
            client_temperature_data_udp_ = this->create_client<l3cam_interfaces::srv::EnableThermalCameraTemperatureDataUdp>("enable_thermal_camera_temperature_data_udp");
            client_streaming_protocol_ = this->create_client<l3cam_interfaces::srv::ChangeStreamingProtocol>("change_streaming_protocol");
            client_get_rtsp_pipeline_ = this->create_client<l3cam_interfaces::srv::GetRtspPipeline>("get_rtsp_pipeline");

            declareParams();
            loadDefaultParams();

            // Create service server
            srv_sensor_disconnected_ = this->create_service<l3cam_interfaces::srv::SensorDisconnected>(
                "thermal_configuration_disconnected", std::bind(&ThermalConfiguration::sensorDisconnectedCallback, this, std::placeholders::_1, std::placeholders::_2));

            // Callback on parameters changed
            callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&ThermalConfiguration::parametersCallback, this, std::placeholders::_1));
        }

        rclcpp::Client<l3cam_interfaces::srv::GetSensorsAvailable>::SharedPtr client_get_sensors_;
        rclcpp::Client<l3cam_interfaces::srv::GetRtspPipeline>::SharedPtr client_get_rtsp_pipeline_;

    private:
        void declareParams()
        {
            // Declare parameters with range
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            rcl_interfaces::msg::IntegerRange range;
            this->declare_parameter("timeout_secs", 60);
            range.set__from_value(0).set__to_value(8); // TBD: dynamic reconfigure dropdown menu newThermalTypes
            descriptor.integer_range = {range};
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
            range.set__from_value(-40).set__to_value(200);
            descriptor.integer_range = {range};
            this->declare_parameter("thermal_camera_temperature_filter_min", 0, descriptor); // -40 - 200
            range.set__from_value(-40).set__to_value(200);
            descriptor.integer_range = {range};
            this->declare_parameter("thermal_camera_temperature_filter_max", 50, descriptor); // -40 - 200
            range.set__from_value(0).set__to_value(2); // TBD: dynamic reconfigure enumerate thermalPipelines
            descriptor.integer_range = {range};
            descriptor.description = 
                "Value must be: (thermalPipelines)\n"
                "\tthermal_LITE = 0\n"
                "\tthermal_LEGACY = 1\n"
                "\tthermal_SEEK = 2";
            this->declare_parameter("thermal_camera_processing_pipeline", 1, descriptor); // 0 - 2
            descriptor.description = "";
            this->declare_parameter("thermal_camera_temperature_data_udp", false);
            range.set__from_value(0).set__to_value(1);
            descriptor.integer_range = {range};
            descriptor.description = 
                "Value must be:\n"
                "\tprotocol_raw_udp = 0\n"
                "\tprotocol_gstreamer = 1";
            this->declare_parameter("thermal_streaming_protocol", 0, descriptor); // 0(protocol_raw_udp), 1(protocol_gstreamer)
        }

        void loadDefaultParams()
        {
            // Get and save parameters
            thermal_camera_colormap_ = this->get_parameter("thermal_camera_colormap").as_int();
            thermal_camera_temperature_filter_ = this->get_parameter("thermal_camera_temperature_filter").as_bool();
            thermal_camera_temperature_filter_min_ = this->get_parameter("thermal_camera_temperature_filter_min").as_int();
            thermal_camera_temperature_filter_max_ = this->get_parameter("thermal_camera_temperature_filter_max").as_int();
            thermal_camera_processing_pipeline_ = this->get_parameter("thermal_camera_processing_pipeline").as_int();
            thermal_camera_temperature_data_udp_ = this->get_parameter("thermal_camera_temperature_data_udp").as_bool();
            streaming_protocol_ = this->get_parameter("thermal_streaming_protocol").as_int();
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
                if (param_name == "thermal_camera_colormap" && param.as_int() != thermal_camera_colormap_)
                {
                    callColormap(param.as_int());
                }
                if (param_name == "thermal_camera_temperature_filter" && param.as_bool() != thermal_camera_temperature_filter_)
                {
                    callTemperatureFilter(param.as_bool());
                }
                if (param_name == "thermal_camera_temperature_filter_min" && param.as_int() != thermal_camera_temperature_filter_min_)
                {
                    callTemperatureFilterRange(param.as_int(), thermal_camera_temperature_filter_max_);
                }
                if (param_name == "thermal_camera_temperature_filter_max" && param.as_int() != thermal_camera_temperature_filter_max_)
                {
                    callTemperatureFilterRange(thermal_camera_temperature_filter_min_, param.as_int());
                }
                if (param_name == "thermal_camera_processing_pipeline" && param.as_int() != thermal_camera_processing_pipeline_)
                {
                    callProcessingPipeline(param.as_int());
                }
                if (param_name == "thermal_camera_temperature_data_udp" && param.as_bool() != thermal_camera_temperature_data_udp_)
                {
                    callTemperatureDataUdp(param.as_bool());
                }
                if (param_name == "thermal_streaming_protocol" && param.as_int() != streaming_protocol_)
                {
                    callStreamingProtocol(param.as_int());
                }
            }

            return result;
        }

        // Service calls
        void callColormap(int colormap)
        {
            while (!client_colormap_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestColormap = std::make_shared<l3cam_interfaces::srv::ChangeThermalCameraColormap::Request>();
            requestColormap->colormap = colormap;

            auto resultColormap = client_colormap_->async_send_request(
                requestColormap, std::bind(&ThermalConfiguration::colormapResponseCallback, this, std::placeholders::_1));
        }

        void callTemperatureFilter(bool enabled)
        {
            while (!client_temperature_filter_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestTemperatureFilter = std::make_shared<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter::Request>();
            requestTemperatureFilter->enabled = enabled;

            auto resultTemperatureFilter = client_temperature_filter_->async_send_request(
                requestTemperatureFilter, std::bind(&ThermalConfiguration::temperatureFilterResponseCallback, this, std::placeholders::_1));
        }

        void callTemperatureFilterRange(int range_min, int range_max)
        {
            while (!client_temperature_filter_range_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestTemperatureFilterRange = std::make_shared<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter::Request>();
            requestTemperatureFilterRange->min_temperature = range_min;
            requestTemperatureFilterRange->max_temperature = range_max;

            auto resultTemperatureFilterRange = client_temperature_filter_range_->async_send_request(
                requestTemperatureFilterRange, std::bind(&ThermalConfiguration::temperatureFilterRangeResponseCallback, this, std::placeholders::_1));
        }

        void callProcessingPipeline(int pipeline)
        {
            while (!client_processing_pipeline_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestProcessingPipeline = std::make_shared<l3cam_interfaces::srv::ChangeThermalCameraProcessingPipeline::Request>();
            requestProcessingPipeline->pipeline = pipeline;

            auto resultProcessingPipeline = client_processing_pipeline_->async_send_request(
                requestProcessingPipeline, std::bind(&ThermalConfiguration::processingPipelineResponseCallback, this, std::placeholders::_1));
        }

        void callTemperatureDataUdp(bool enabled)
        {
            while (!client_temperature_data_udp_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestTemperatureDataUdp = std::make_shared<l3cam_interfaces::srv::EnableThermalCameraTemperatureDataUdp::Request>();
            requestTemperatureDataUdp->enabled = enabled;

            auto resultTemperatureDataUdp = client_temperature_data_udp_->async_send_request(
                requestTemperatureDataUdp, std::bind(&ThermalConfiguration::temperatureDataUdpResponseCallback, this, std::placeholders::_1));
        }

        void callStreamingProtocol(int protocol)
        {
            while (!client_streaming_protocol_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestStreamingProtocol = std::make_shared<l3cam_interfaces::srv::ChangeStreamingProtocol::Request>();
            requestStreamingProtocol->sensor_type = (int)sensorTypes::sensor_thermal;
            requestStreamingProtocol->protocol = protocol;

            auto resultStreamingProtocol = client_streaming_protocol_->async_send_request(
                requestStreamingProtocol, std::bind(&ThermalConfiguration::streamingProtocolResponseCallback, this, std::placeholders::_1));
        }

        // Service callbacks
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
                    thermal_camera_colormap_ = this->get_parameter("thermal_camera_colormap").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("thermal_camera_colormap", thermal_camera_colormap_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_thermal_camera_colormap");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("thermal_camera_colormap", thermal_camera_colormap_));
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
                    thermal_camera_temperature_filter_ = this->get_parameter("thermal_camera_temperature_filter").as_bool();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_filter", thermal_camera_temperature_filter_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service enable_thermal_camera_temperature_filter");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_filter", thermal_camera_temperature_filter_));
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
                    thermal_camera_temperature_filter_min_ = this->get_parameter("thermal_camera_temperature_filter_min").as_int();
                    thermal_camera_temperature_filter_max_ = this->get_parameter("thermal_camera_temperature_filter_max").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_filter_min", thermal_camera_temperature_filter_min_));
                    this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_filter_max", thermal_camera_temperature_filter_max_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_thermal_camera_temperature_filter_min");
                // Service could not be called, reset parameters to value before change
                this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_filter_min", thermal_camera_temperature_filter_min_));
                this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_filter_max", thermal_camera_temperature_filter_max_));
            }
        }

        void processingPipelineResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeThermalCameraProcessingPipeline>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    thermal_camera_processing_pipeline_ = this->get_parameter("thermal_camera_processing_pipeline").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("thermal_camera_processing_pipeline", thermal_camera_processing_pipeline_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_thermal_camera_processing_pipeline");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("thermal_camera_processing_pipeline", thermal_camera_processing_pipeline_));
            }
        }

        void temperatureDataUdpResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::EnableThermalCameraTemperatureDataUdp>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    thermal_camera_temperature_data_udp_ = this->get_parameter("thermal_camera_temperature_data_udp").as_bool();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_data_udp", thermal_camera_temperature_data_udp_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service enable_thermal_camera_temperature_data_udp");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("thermal_camera_temperature_data_udp", thermal_camera_temperature_data_udp_));
            }
        }

        void streamingProtocolResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeStreamingProtocol>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameter changed successfully, save value
                    streaming_protocol_ = this->get_parameter("thermal_streaming_protocol").as_int();
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing parameter in " << __func__ << ": " << getErrorDescription(error));
                    // Parameter could not be changed, reset parameter to value before change
                    this->set_parameter(rclcpp::Parameter("thermal_streaming_protocol", streaming_protocol_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_streaming_protocol");
                // Service could not be called, reset parameter to value before change
                this->set_parameter(rclcpp::Parameter("thermal_streaming_protocol", streaming_protocol_));
            }
        }

        void sensorDisconnectedCallback(const std::shared_ptr<l3cam_interfaces::srv::SensorDisconnected::Request> req,
                                        std::shared_ptr<l3cam_interfaces::srv::SensorDisconnected::Response> res)
        {
            ROS2_BMG_UNUSED(res);
            if (req->code == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Exiting cleanly.");
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Exiting. Sensor got disconnected with error " << req->code << ": " << getErrorDescription(req->code));
            }

            rclcpp::shutdown();
        }

        rclcpp::Client<l3cam_interfaces::srv::ChangeThermalCameraColormap>::SharedPtr client_colormap_;
        rclcpp::Client<l3cam_interfaces::srv::EnableThermalCameraTemperatureFilter>::SharedPtr client_temperature_filter_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeThermalCameraTemperatureFilter>::SharedPtr client_temperature_filter_range_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeThermalCameraProcessingPipeline>::SharedPtr client_processing_pipeline_;
        rclcpp::Client<l3cam_interfaces::srv::EnableThermalCameraTemperatureDataUdp>::SharedPtr client_temperature_data_udp_;
        rclcpp::Client<l3cam_interfaces::srv::ChangeStreamingProtocol>::SharedPtr client_streaming_protocol_;

        rclcpp::Service<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr srv_sensor_disconnected_;

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

        int thermal_camera_colormap_;
        bool thermal_camera_temperature_filter_;
        int thermal_camera_temperature_filter_min_;
        int thermal_camera_temperature_filter_max_;
        int thermal_camera_processing_pipeline_;
        bool thermal_camera_temperature_data_udp_;
        int streaming_protocol_;

    }; // class ThermalConfiguration

} // namespace l3cam_ros2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::ThermalConfiguration> node = std::make_shared<l3cam_ros2::ThermalConfiguration>();

    // Check if Thermal is available
    int i = 0;
    while (!node->client_get_sensors_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
            return 0;
        }

        if (i >= node->get_parameter("timeout_secs").as_int())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error: " << getErrorDescription(L3CAM_ROS2_SERVICE_AVAILABILITY_TIMEOUT_ERROR));
            return L3CAM_ROS2_SERVICE_AVAILABILITY_TIMEOUT_ERROR;
        }
        ++i;
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }
    node->undeclare_parameter("timeout_secs");

    auto requestGetSensors = std::make_shared<l3cam_interfaces::srv::GetSensorsAvailable::Request>();
    auto resultGetSensors = node->client_get_sensors_->async_send_request(requestGetSensors);

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
                if (resultGetSensors.get()->sensors[i].sensor_type == sensor_thermal && resultGetSensors.get()->sensors[i].sensor_available)
                    sensor_is_available = true;
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR " << error << " while checking sensor availability in " << __func__ << ": " << getErrorDescription(error));
            return error;
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service get_sensors_available");
        return L3CAM_ROS2_FAILED_TO_CALL_SERVICE;
    }

    if (sensor_is_available)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Thermal camera configuration is available");
    }
    else
    {
        return 0;
    }

    // Get pipeline
    while (!node->client_get_rtsp_pipeline_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
            return 0;
        }
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto requestGetRtspPipeline = std::make_shared<l3cam_interfaces::srv::GetRtspPipeline::Request>();
    requestGetRtspPipeline.get()->sensor_type = (int)sensorTypes::sensor_thermal;
    auto resultGetRtspPipeline = node->client_get_rtsp_pipeline_->async_send_request(requestGetRtspPipeline);

    if (rclcpp::spin_until_future_complete(node, resultGetRtspPipeline) == rclcpp::FutureReturnCode::SUCCESS)
    {
        error = resultGetRtspPipeline.get()->error;

        if (!error)
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.read_only = true;
            node->declare_parameter("thermal_rtsp_pipeline", resultGetRtspPipeline.get()->pipeline, descriptor);
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR " << error << " while getting pipeline in " << __func__ << ": " << getErrorDescription(error));
            return error;
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service get_rtsp_pipeline");
        return L3CAM_ROS2_FAILED_TO_CALL_SERVICE;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
