/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2016-2024, qbrobotics®
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef QB_SOFTHAND_INDUSTRY_COMMUNICATION_HANDLER_H
#define QB_SOFTHAND_INDUSTRY_COMMUNICATION_HANDLER_H

// Standard libraries
#include <mutex>
#include <regex>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// ROS libraries
#include "rclcpp/rclcpp.hpp"

// internal libraries
#include <qb_softhand_industry_msgs/msg/connection_state.hpp>
#include <qb_softhand_industry_srvs/srv/get_measurements.hpp>
#include <qb_softhand_industry_srvs/srv/set_command.hpp>
#include <qb_softhand_industry_srvs/srv/set_commands.hpp>
#include <qb_softhand_industry_srvs/srv/trigger.hpp>
#include "qbsofthand_industry_api.h"


// qbSoftHand Industry macros
#define QBSOFTHAND_INDUSTRY_ERROR_BADPARAMS -1
#define QBSOFTHAND_INDUSTRY_ERROR_RECVAGAIN -2
#define QBSOFTHAND_INDUSTRY_ERROR_NOTRESPONDING -3
#define QBSOFTHAND_INDUSTRY_ERROR_NOTCOMPATIBLE -4

#define QBSOFTHAND_INDUSTRY_GET_CURRENT_CMD "IQ"
#define QBSOFTHAND_INDUSTRY_GET_POSITION_CMD "PU"
#define QBSOFTHAND_INDUSTRY_GET_VELOCITY_CMD "VU"

#define QBSOFTHAND_INDUSTRY_POS_MIN 0
#define QBSOFTHAND_INDUSTRY_POS_MAX 3800
#define QBSOFTHAND_INDUSTRY_VEL_MIN 400
#define QBSOFTHAND_INDUSTRY_VEL_MAX 3200
#define QBSOFTHAND_INDUSTRY_CURR_MIN 250
#define QBSOFTHAND_INDUSTRY_CURR_MAX 450

namespace qb_softhand_industry_communication_handler {
/**
 * The Communication Handler class is aimed to instantiate a ROS node which provides several ROS services to
 * communicate with one - or many - qbrobotics SoftHand Industry connected to the ROS ecosystem.
 */
class qbSoftHandIndustryCommunicationHandler {
  public:
    qbSoftHandIndustryCommunicationHandler(rclcpp::Node::SharedPtr& node);

  protected:
    rclcpp::Node::SharedPtr node_handle_;
    rclcpp::Parameter ip_;
    rclcpp::Parameter serial_port_name_;
    rclcpp::Parameter use_only_ethernet;
    
    const rclcpp::NodeOptions & options_ = (
      rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true)
      );

    std::unique_ptr<qbsofthand_industry_api::qbSoftHandIndustryAPI> api_handler_;

    /**
     * Activate the motors of the device. Do nothing if the device is not connected in the Communication Handler.
     * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
     * \sa activateCallback(), activate(const int &, const bool &, const int &), isActive()
     */
    virtual int activate(const bool &command, const int &max_repeats);

    /**
     * Activate the motors of the SoftHand Industry relative to the node requesting the service.
     * \param request The request of the given service (see qb_softhand_industry_srvs::srv::Trigger for details).
     * \param response The response of the given service (see qb_softhand_industry_srvs::srv::Trigger for details).
     * \return \p true if the call succeed (actually \p response.success may be false).
     * \sa activate(const int &, const int &)
     */
    bool activateCallback(const std::shared_ptr<qb_softhand_industry_srvs::srv::Trigger::Request> request, std::shared_ptr<qb_softhand_industry_srvs::srv::Trigger::Response> response);

    /**
     * Deactivate the motors of the SoftHand Industry relative to the node requesting the service.
     * \param request The request of the given service (see qb_softhand_industry_srvs::srv::Trigger for details).
     * \param response The response of the given service (see qb_softhand_industry_srvs::srv::Trigger for details).
     * \return \p true if the call succeed (actually \p response.success may be false).
     * \sa activate(const int &, const int &)
     */
    bool deactivateCallback(const std::shared_ptr<qb_softhand_industry_srvs::srv::Trigger::Request> request, std::shared_ptr<qb_softhand_industry_srvs::srv::Trigger::Response> response);

    /**
     * Get measurements from SoftHand Industry relative to the node requesting the service.
     * \param request The request of the given service (see qb_softhand_industry_srvs::srv::GetMeasurements for details).
     * \param response The response of the given service (see qb_softhand_industry_srvs::srv::GetMeasurements for details).
     * \return \p true if the call succeed (actually \p response.success may be false).
     */
    bool getMeasurementsCallback(const std::shared_ptr<qb_softhand_industry_srvs::srv::GetMeasurements::Request> request, std::shared_ptr<qb_softhand_industry_srvs::srv::GetMeasurements::Response> response);

    /**
     * @brief Validate current
     * @param current parameter to validate
     * 
     * @return true if valid, false otherwise
     */
    bool isValidCurrent(const int &current);  

    /**
     * @brief Validate position
     * @param position parameter to validate
     * 
     * @return true if valid, false otherwise
     */
    bool isValidPosition(const int &position);

    /**
     * @brief Validate velocity
     * @param velocity parameter to validate
     * 
     * @return true if valid, false otherwise
     */
    bool isValidVelocity(const int &velocity);

    /**
     * @brief Send a movement command to SoftHand Industry
     * 
     * @param command the position reference command [ticks]
     * @return the response status  
     */
    int setCommand(const int &command, const int &max_repeats);

    /**
     * @brief Send a movement command to SoftHand Industry
     * 
     * @param command the command [ticks]
     * @param velocity [ticks/s]
     * @param current [mA]
     * @return the response status  
     */
    int setCommands(const int &position, const int &velocity, const int &current, const int &max_repeats);
    /**
     * Send a command to SoftHand Industry.
     * \param request The request of the given service (see qb_softhand_industry_srvs::setCommand for details).
     * \param response The response of the given service (see qb_softhand_industry_srvs::setCommand for details).
     * \return \p true if the call succeed (actually \p response.success may be false).
     */
    bool setCommandCallback(const std::shared_ptr<qb_softhand_industry_srvs::srv::SetCommand::Request> request, std::shared_ptr<qb_softhand_industry_srvs::srv::SetCommand::Response> response);

    /**
     * Send a command to SoftHand Industry.
     * \param request The request of the given service (see qb_softhand_industry_srvs::setCommands for details).
     * \param response The response of the given service (see qb_softhand_industry_srvs::setCommands for details).
     * \return \p true if the call succeed (actually \p response.success may be false).
     */
    bool setCommandsCallback(const std::shared_ptr <qb_softhand_industry_srvs::srv::SetCommands::Request> request, std::shared_ptr<qb_softhand_industry_srvs::srv::SetCommands::Response> response);

  private:
    rclcpp::Service<qb_softhand_industry_srvs::srv::Trigger>::SharedPtr activate_motors_;
    rclcpp::Service<qb_softhand_industry_srvs::srv::Trigger>::SharedPtr deactivate_motors_;
    rclcpp::Service<qb_softhand_industry_srvs::srv::SetCommand>::SharedPtr set_command_;
    rclcpp::Service<qb_softhand_industry_srvs::srv::SetCommands>::SharedPtr set_commands_;
    rclcpp::Service<qb_softhand_industry_srvs::srv::GetMeasurements>::SharedPtr get_measurements_;
};
}

#endif //QB_SOFTHAND_INDUSTRY_COMMUNICATION_HANDLER_H