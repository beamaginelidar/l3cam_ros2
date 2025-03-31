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

#include "l3cam_ros2_errors.hpp"
#include <libL3Cam.h>

#define ROS2_BMG_UNUSED(x) (void)x;

typedef enum LibL3CamStatus
{
    undefined_status = 0, // default status
    error_status,         // error status
    connected_status,     // after FIND_DEVICE
    disconnected_status,  //! (TBD) after notification
    started_status,       // after START_DEVICE
    streaming_status,     // after START_STREAM
    terminated_status     // after TERMINATE
} LibL3CamStatus;

typedef enum polAngle
{
    angle_0 = 0,
    angle_45,
    angle_90,
    angle_135,
    no_angle
} polAngle;

static const char *bmg_ros2_error_find_devices_timeout = "Timeout error while finding devices\0";
static const char *bmg_ros2_error_failed_to_call_service = "Failed to call service\0";
static const char *bmg_ros2_error_interrupted = "RCLCPP interrupted\0";
static const char *bmg_ros2_error_service_availability_timeout = "Timeout error while looking for service availability\0";
static const char *bmg_ros_error_invalid_polarimetric_process_type = "Invalid polarimetric process type\0";
static const char *bmg_ros2_error_undefined_error = "UNDEFINED L3CAM ERROR\0";

static const char *getBeamRos2ErrorDescription(int error_code)
{
    switch (error_code)
    {
    case L3CAM_ROS2_FIND_DEVICES_TIMEOUT_ERROR:
        return bmg_ros2_error_find_devices_timeout;
        break;
    case L3CAM_ROS2_FAILED_TO_CALL_SERVICE:
        return bmg_ros2_error_failed_to_call_service;
        break;
    case L3CAM_ROS2_RCLCPP_INTERRUPTED:
        return bmg_ros2_error_interrupted;
        break;
    case L3CAM_ROS2_SERVICE_AVAILABILITY_TIMEOUT_ERROR:
        return bmg_ros2_error_service_availability_timeout;
        break;
    case L3CAM_ROS2_INVALID_POLARIMETRIC_PROCESS_TYPE:
        return bmg_ros_error_invalid_polarimetric_process_type;
        break;
    default:
        return bmg_ros2_error_undefined_error;
    }
}

static std::string getErrorDescription(int error_code)
{
    if (error_code < 0)
    {
        return getBeamRos2ErrorDescription(error_code);
    }
    else
    {
        return getBeamErrorDescription(error_code);
    }
}
