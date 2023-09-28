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

using namespace std::chrono_literals;

namespace l3cam_ros2
{
    class NetworkConfiguration : public rclcpp::Node
    {
    public:
        NetworkConfiguration() : Node("network_configuration")
        {
            // Declare parameters
            this->declare_parameter("ip_address", "192.168.1.250");
            this->declare_parameter("netmask", "255.255.255.0");
            this->declare_parameter("gateway", "0.0.0.0");
            this->declare_parameter("dhcp", false);

            // Get and save parameters
            dhcp = this->get_parameter("dhcp").as_bool();

            // Create service clients
            clientGetNetwork = this->create_client<l3cam_interfaces::srv::GetNetworkConfiguration>("get_network_configuration");
            clientNetwork = this->create_client<l3cam_interfaces::srv::ChangeNetworkConfiguration>("change_network_configuration");

            // Callback on parameters changed
            callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&NetworkConfiguration::parametersCallback, this, std::placeholders::_1));
        }

        rclcpp::Client<l3cam_interfaces::srv::GetNetworkConfiguration>::SharedPtr clientGetNetwork;

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
                if (param_name == "ip_address")
                {
                    while (!clientNetwork->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestNetwork = std::make_shared<l3cam_interfaces::srv::ChangeNetworkConfiguration::Request>();
                    requestNetwork->ip_address = param.as_string();
                    requestNetwork->netmask = netmask;
                    requestNetwork->gateway = gateway;
                    requestNetwork->enable_dhcp = dhcp;

                    auto resultBrightness = clientNetwork->async_send_request(
                        requestNetwork, std::bind(&NetworkConfiguration::networkResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "netmask")
                {
                    while (!clientNetwork->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestNetwork = std::make_shared<l3cam_interfaces::srv::ChangeNetworkConfiguration::Request>();
                    requestNetwork->ip_address = ip_address;
                    requestNetwork->netmask = param.as_string();
                    requestNetwork->gateway = gateway;
                    requestNetwork->enable_dhcp = dhcp;

                    auto resultBrightness = clientNetwork->async_send_request(
                        requestNetwork, std::bind(&NetworkConfiguration::networkResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "gateway")
                {
                    while (!clientNetwork->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestNetwork = std::make_shared<l3cam_interfaces::srv::ChangeNetworkConfiguration::Request>();
                    requestNetwork->ip_address = ip_address;
                    requestNetwork->netmask = netmask;
                    requestNetwork->gateway = param.as_string();
                    requestNetwork->enable_dhcp = dhcp;

                    auto resultBrightness = clientNetwork->async_send_request(
                        requestNetwork, std::bind(&NetworkConfiguration::networkResponseCallback, this, std::placeholders::_1));
                }
                if (param_name == "dhcp")
                {
                    while (!clientNetwork->wait_for_service(1s))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
                            break;
                        }
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                    }

                    auto requestNetwork = std::make_shared<l3cam_interfaces::srv::ChangeNetworkConfiguration::Request>();
                    requestNetwork->ip_address = ip_address;
                    requestNetwork->netmask = netmask;
                    requestNetwork->gateway = gateway;
                    requestNetwork->enable_dhcp = param.as_bool();

                    auto resultBrightness = clientNetwork->async_send_request(
                        requestNetwork, std::bind(&NetworkConfiguration::networkResponseCallback, this, std::placeholders::_1));
                }
            }

            return result;
        }

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
                    ip_address = this->get_parameter("ip_address").as_string();
                    netmask = this->get_parameter("netmask").as_string();
                    gateway = this->get_parameter("gateway").as_string();
                    dhcp = this->get_parameter("dhcp").as_bool();
                }
                else
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), '(' << error << ") " << getBeamErrorDescription(error));
                    // Parameters could not be changed, reset parameters to value before change
                    // this->set_parameter(rclcpp::Parameter("ip_address", ip_address));
                    // this->set_parameter(rclcpp::Parameter("netmask", netmask));
                    // this->set_parameter(rclcpp::Parameter("gateway", gateway));
                    // this->set_parameter(rclcpp::Parameter("dhcp", dhcp));
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_network_configuration");
                // Service could not be called, reset parameters to value before change
                // this->set_parameter(rclcpp::Parameter("ip_address", ip_address));
                // this->set_parameter(rclcpp::Parameter("netmask", netmask));
                // this->set_parameter(rclcpp::Parameter("gateway", gateway));
                // this->set_parameter(rclcpp::Parameter("dhcp", dhcp));
            }
        }

        std::string ip_address;
        std::string netmask;
        std::string gateway;
        bool dhcp;

        rclcpp::Client<l3cam_interfaces::srv::ChangeNetworkConfiguration>::SharedPtr clientNetwork;

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    }; // class NetworkConfiguration

} // namespace l3cam_ros2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<l3cam_ros2::NetworkConfiguration> node = std::make_shared<l3cam_ros2::NetworkConfiguration>();

    while (!node->clientGetNetwork->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
            break;
        }
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto requestGetNetwork = std::make_shared<l3cam_interfaces::srv::GetNetworkConfiguration::Request>();
    auto resultGetNetwork = node->clientGetNetwork->async_send_request(requestGetNetwork);

    int error = L3CAM_OK;
    bool sensor_is_available = false;
    // Shutdown if error returned
    if (rclcpp::spin_until_future_complete(node, resultGetNetwork) == rclcpp::FutureReturnCode::SUCCESS)
    {
        error = resultGetNetwork.get()->error;

        if (!error)
        {
            node->set_parameter(rclcpp::Parameter("ip_address", resultGetNetwork.get()->ip_address));
            node->set_parameter(rclcpp::Parameter("netmask", resultGetNetwork.get()->netmask));
            node->set_parameter(rclcpp::Parameter("gateway", resultGetNetwork.get()->gateway));
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error " << error << " while getting network configuration: " << getBeamErrorDescription(error));
            return 1;
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_network_configuration");
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Network configuration is available");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
