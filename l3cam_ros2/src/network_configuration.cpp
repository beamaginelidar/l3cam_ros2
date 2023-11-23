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

#include "l3cam_interfaces/srv/get_network_configuration.hpp"
#include "l3cam_interfaces/srv/change_network_configuration.hpp"

#include "l3cam_interfaces/srv/sensor_disconnected.hpp"

#include "l3cam_ros2_utils.hpp"

using namespace std::chrono_literals;

namespace l3cam_ros2
{
    class NetworkConfiguration : public rclcpp::Node
    {
    public:
        explicit NetworkConfiguration() : Node("network_configuration")
        {
            // Declare parameters
            this->declare_parameter("timeout_secs", 60);
            this->declare_parameter("ip_address", "192.168.1.250");
            this->declare_parameter("netmask", "255.255.255.0");
            this->declare_parameter("gateway", "0.0.0.0");
            this->declare_parameter("dhcp", false);

            // Create service clients
            client_get_network_ = this->create_client<l3cam_interfaces::srv::GetNetworkConfiguration>("get_network_configuration");
            client_network_ = this->create_client<l3cam_interfaces::srv::ChangeNetworkConfiguration>("change_network_configuration");

            srv_network_disconnected_ = this->create_service<l3cam_interfaces::srv::SensorDisconnected>(
                "network_disconnected", std::bind(&NetworkConfiguration::networkDisconnectedCallback, this, std::placeholders::_1, std::placeholders::_2));
        }

        void handleParamsCallback()
        {
            // Callback on parameters changed
            callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&NetworkConfiguration::parametersCallback, this, std::placeholders::_1));
        }

        rclcpp::Client<l3cam_interfaces::srv::GetNetworkConfiguration>::SharedPtr client_get_network_;

    private:
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
                if (param_name == "ip_address" && param.as_string() != ip_address_)
                {
                    callNetwork(param.as_string(), netmask_, gateway_, dhcp_);
                }
                if (param_name == "netmask" && param.as_string() != netmask_)
                {
                    callNetwork(ip_address_, param.as_string(), gateway_, dhcp_);
                }
                if (param_name == "gateway" && param.as_string() != gateway_)
                {
                    callNetwork(ip_address_, netmask_, param.as_string(), dhcp_);
                }
                if (param_name == "dhcp" && param.as_bool() != dhcp_)
                {
                    callNetwork(ip_address_, netmask_, gateway_, param.as_bool());
                }
            }

            return result;
        }

        // Service calls
        void callNetwork(std::string ip_address_val, std::string netmask_val, std::string gateway_val, bool dhcp_val)
        {
            while (!client_network_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto requestNetwork = std::make_shared<l3cam_interfaces::srv::ChangeNetworkConfiguration::Request>();
            requestNetwork->ip_address = ip_address_val;
            requestNetwork->netmask = netmask_val;
            requestNetwork->gateway = gateway_val;
            requestNetwork->enable_dhcp = dhcp_val;

            auto resultBrightness = client_network_->async_send_request(
                requestNetwork, std::bind(&NetworkConfiguration::networkResponseCallback, this, std::placeholders::_1));
        }

        // Service callbacks
        void networkResponseCallback(
            rclcpp::Client<l3cam_interfaces::srv::ChangeNetworkConfiguration>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                int error = future.get()->error;
                if (!error)
                {
                    // Parameters changed successfully, save values
                    ip_address_ = this->get_parameter("ip_address").as_string();
                    netmask_ = this->get_parameter("netmask").as_string();
                    gateway_ = this->get_parameter("gateway").as_string();
                    dhcp_ = this->get_parameter("dhcp").as_bool();

                    RCLCPP_INFO(this->get_logger(), "The network configuration changes will take effect once the device is restarted.");
                }
                else
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR " << error << " while changing network configuration in " << __func__ << ": " << getErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    this->set_parameter(rclcpp::Parameter("ip_address", ip_address_));
                    this->set_parameter(rclcpp::Parameter("netmask", netmask_));
                    this->set_parameter(rclcpp::Parameter("gateway", gateway_));
                    this->set_parameter(rclcpp::Parameter("dhcp", dhcp_));
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service change_network_configuration");
                // Service could not be called, reset parameters to value before change
                this->set_parameter(rclcpp::Parameter("ip_address", ip_address_));
                this->set_parameter(rclcpp::Parameter("netmask", netmask_));
                this->set_parameter(rclcpp::Parameter("gateway", gateway_));
                this->set_parameter(rclcpp::Parameter("dhcp", dhcp_));
            }
        }

        void networkDisconnectedCallback(const std::shared_ptr<l3cam_interfaces::srv::SensorDisconnected::Request> req,
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

        std::string ip_address_;
        std::string netmask_;
        std::string gateway_;
        bool dhcp_;

        rclcpp::Client<l3cam_interfaces::srv::ChangeNetworkConfiguration>::SharedPtr client_network_;
        rclcpp::Service<l3cam_interfaces::srv::SensorDisconnected>::SharedPtr srv_network_disconnected_;

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    }; // class NetworkConfiguration

} // namespace l3cam_ros2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::NetworkConfiguration> node = std::make_shared<l3cam_ros2::NetworkConfiguration>();

    // Get actual network configuration
    int i = 0;
    while (!node->client_get_network_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service in " << __func__ << ". Exiting.");
            return 0;
        }

        if (i >= node->get_parameter("timeout_secs").as_int())
            return 0;
        ++i;
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }
    node->undeclare_parameter("timeout_secs");

    auto requestGetNetwork = std::make_shared<l3cam_interfaces::srv::GetNetworkConfiguration::Request>();
    auto resultGetNetwork = node->client_get_network_->async_send_request(requestGetNetwork);

    int error = L3CAM_OK;
    // Shutdown if error returned
    if (rclcpp::spin_until_future_complete(node, resultGetNetwork) == rclcpp::FutureReturnCode::SUCCESS)
    {
        error = resultGetNetwork.get()->error;

        if (!error)
        {
            node->set_parameter(rclcpp::Parameter("ip_address", resultGetNetwork.get()->ip_address));
            node->set_parameter(rclcpp::Parameter("netmask", resultGetNetwork.get()->netmask));
            node->set_parameter(rclcpp::Parameter("gateway", resultGetNetwork.get()->gateway));
            node->handleParamsCallback();
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR " << error << " while getting network configuration in " << __func__ << ": " << getErrorDescription(error));
            return error;
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service get_network_configuration");
        return L3CAM_ROS2_FAILED_TO_CALL_SERVICE;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Network configuration is available");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
