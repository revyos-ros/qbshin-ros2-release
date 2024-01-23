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

#include <qb_softhand_industry_driver/qb_softhand_industry_communication_handler.h>

#define BIND_CLS_CB(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2)

using namespace qb_softhand_industry_communication_handler;

qbSoftHandIndustryCommunicationHandler::qbSoftHandIndustryCommunicationHandler(rclcpp::Node::SharedPtr& node)
    : node_handle_(node),
    activate_motors_(node_handle_->create_service<qb_softhand_industry_srvs::srv::Trigger>("/qb_softhand_industry_communication_handler/activate_motors", BIND_CLS_CB(&qbSoftHandIndustryCommunicationHandler::activateCallback))),
    deactivate_motors_(node_handle_->create_service<qb_softhand_industry_srvs::srv::Trigger>("/qb_softhand_industry_communication_handler/deactivate_motors", BIND_CLS_CB(&qbSoftHandIndustryCommunicationHandler::deactivateCallback))),
    set_commands_(node_handle_->create_service<qb_softhand_industry_srvs::srv::SetCommands>("/qb_softhand_industry_communication_handler/set_commands", BIND_CLS_CB(&qbSoftHandIndustryCommunicationHandler::setCommandsCallback))),
    set_command_(node_handle_->create_service<qb_softhand_industry_srvs::srv::SetCommand>("/qb_softhand_industry_communication_handler/set_command", BIND_CLS_CB(&qbSoftHandIndustryCommunicationHandler::setCommandCallback))),
    get_measurements_(node_handle_->create_service<qb_softhand_industry_srvs::srv::GetMeasurements>("/qb_softhand_industry_communication_handler/get_measurements", BIND_CLS_CB(&qbSoftHandIndustryCommunicationHandler::getMeasurementsCallback))),
    ip_(node_handle_->get_parameter("ip_address")),
    serial_port_name_(node_handle_->get_parameter("serial_port_name")),
    use_only_ethernet(node_handle_->get_parameter("use_only_ethernet")){
      if (!use_only_ethernet.as_bool())
      { 
        if(serial_port_name_.as_string() == ""){
          api_handler_.reset(new qbsofthand_industry_api::qbSoftHandIndustryAPI());
          RCLCPP_INFO_STREAM(node_handle_->get_logger(), "Looking for device on serial or on ip " << ip_.as_string() << "...");
        } else {
          api_handler_.reset(new qbsofthand_industry_api::qbSoftHandIndustryAPI(serial_port_name_.as_string()));
          RCLCPP_INFO_STREAM(node_handle_->get_logger(), "Trying to connect to device serial at " << serial_port_name_.as_string() << " or on ip " << ip_.as_string() <<"...");
        }
      } else {
        RCLCPP_INFO_STREAM(node_handle_->get_logger(), "Trying to connect to device ip " << ip_.as_string() << "...");
        api_handler_.reset(new qbsofthand_industry_api::qbSoftHandIndustryAPI(ip_.as_string(),10));
      } 
      
      while(!(api_handler_->isInitialized())){
        rclcpp::sleep_for((std::chrono::nanoseconds)(100000000));
      }
      RCLCPP_INFO_STREAM(node_handle_->get_logger(), "Connected to device.");
    }

int qbSoftHandIndustryCommunicationHandler::activate(const bool &command, const int &max_repeats) {
  std::string command_prefix = command ? "" : "de";
  bool status = false;
  if(command) {
    api_handler_->sendRawCommand("MO=1",max_repeats); // activation command
  } else {
    api_handler_->sendRawCommand("MO=0",max_repeats); // deactivation command
  }
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "device [" << ip_.as_string() << "] motors have been " << command_prefix << "activated!");
  return 0;
}

bool qbSoftHandIndustryCommunicationHandler::activateCallback(const std::shared_ptr<qb_softhand_industry_srvs::srv::Trigger::Request> request, std::shared_ptr<qb_softhand_industry_srvs::srv::Trigger::Response> response) {
  RCLCPP_ERROR_STREAM_EXPRESSION(node_handle_->get_logger(),request->max_repeats < 0, "Device [" << ip_.as_string() << "] has request service with non-valid 'max_request' [" << request->max_repeats << "].");
  activate(true, request->max_repeats);
  response->message = "Device [" + ip_.as_string() + "] activation.";
  return true;
}

bool qbSoftHandIndustryCommunicationHandler::deactivateCallback(const std::shared_ptr<qb_softhand_industry_srvs::srv::Trigger::Request> request, std::shared_ptr<qb_softhand_industry_srvs::srv::Trigger::Response> response) {
  RCLCPP_ERROR_STREAM_EXPRESSION(node_handle_->get_logger(),request->max_repeats < 0, "Device [" << ip_.as_string() << "] has request service with non-valid 'max_request' [" << request->max_repeats << "].");
  activate(false, request->max_repeats);
  response->message = "Device [" + ip_.as_string() + "] deactivation.";
  return true;
}

bool qbSoftHandIndustryCommunicationHandler::getMeasurementsCallback(const std::shared_ptr<qb_softhand_industry_srvs::srv::GetMeasurements::Request> request, std::shared_ptr<qb_softhand_industry_srvs::srv::GetMeasurements::Response> response) {
  RCLCPP_ERROR_STREAM_EXPRESSION(node_handle_->get_logger(),request->max_repeats < 0, "Device [" << ip_.as_string() << "] has request service with non-valid 'max_request' [" << request->max_repeats << "].");
  if(request->get_position) {
    response->position = QBSOFTHAND_INDUSTRY_POS_MIN + api_handler_->getPosition() * QBSOFTHAND_INDUSTRY_POS_MAX * 0.01;
  }
  if(request->get_current) {
    response->current = QBSOFTHAND_INDUSTRY_CURR_MIN + api_handler_->getCurrent() * QBSOFTHAND_INDUSTRY_CURR_MAX * 0.01; 
  }
  if(request->get_velocity) {
    response->velocity = QBSOFTHAND_INDUSTRY_VEL_MIN +api_handler_->getVelocity() * QBSOFTHAND_INDUSTRY_VEL_MAX * 0.01;
  }

  response->stamp = rclcpp::Clock{}.now();
  response->success = true;
  return true;
}

bool qbSoftHandIndustryCommunicationHandler::isValidCurrent(const int &current){
  return current >= QBSOFTHAND_INDUSTRY_CURR_MIN && current <= QBSOFTHAND_INDUSTRY_CURR_MAX;
}

bool qbSoftHandIndustryCommunicationHandler::isValidPosition(const int &position){
  return position >= QBSOFTHAND_INDUSTRY_POS_MIN && position <= QBSOFTHAND_INDUSTRY_POS_MAX;
}

bool qbSoftHandIndustryCommunicationHandler::isValidVelocity(const int &velocity){
  return velocity >= QBSOFTHAND_INDUSTRY_VEL_MIN && velocity <= QBSOFTHAND_INDUSTRY_VEL_MAX;
}

int qbSoftHandIndustryCommunicationHandler::setCommand(const int &command, const int &max_repeats) {
  if(api_handler_->sendRawCommand("SR", max_repeats) != 0) {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Device not responding");
    return QBSOFTHAND_INDUSTRY_ERROR_NOTRESPONDING;
  }
  if(!isValidPosition(command)){
    RCLCPP_WARN_STREAM(node_handle_->get_logger(),"Not valid command.");
    return QBSOFTHAND_INDUSTRY_ERROR_BADPARAMS;
  }

  api_handler_->sendRawCommand("UI[1]=" + std::to_string(command), max_repeats);
  return 0;
}

int qbSoftHandIndustryCommunicationHandler::setCommands(const int &position, const int &velocity, const int &current, const int &max_repeats) {
  if(api_handler_->sendRawCommand("SR", max_repeats) != 0) {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(),"Device not responding");
    return QBSOFTHAND_INDUSTRY_ERROR_NOTRESPONDING;
  }

  if(!isValidPosition(position) || !isValidVelocity(velocity) || !isValidCurrent(current)){
    RCLCPP_WARN_STREAM(node_handle_->get_logger(),"Not valid commands.");
    return QBSOFTHAND_INDUSTRY_ERROR_BADPARAMS;
  }

  api_handler_->sendRawCommand("UI[2]=" + std::to_string(velocity), max_repeats);
  api_handler_->sendRawCommand("UI[3]=" + std::to_string(current), max_repeats);
  api_handler_->sendRawCommand("UI[1]=" + std::to_string(position), max_repeats);
  return 0;
}

bool qbSoftHandIndustryCommunicationHandler::setCommandCallback(const std::shared_ptr<qb_softhand_industry_srvs::srv::SetCommand::Request> request, std::shared_ptr<qb_softhand_industry_srvs::srv::SetCommand::Response> response) {
  RCLCPP_ERROR_STREAM_EXPRESSION(node_handle_->get_logger(),request->max_repeats < 0, "Device [" << ip_.as_string() << "] has request service with non-valid 'max_request' [" << request->max_repeats << "].");

  if (request->set_commands) {
    response->success = (setCommand(request->position_command, request->max_repeats) == 0);
  }
  return true;
}

bool qbSoftHandIndustryCommunicationHandler::setCommandsCallback(const std::shared_ptr <qb_softhand_industry_srvs::srv::SetCommands::Request> request, std::shared_ptr<qb_softhand_industry_srvs::srv::SetCommands::Response> response) {
  RCLCPP_ERROR_STREAM_EXPRESSION(node_handle_->get_logger(),request->max_repeats < 0, "Device [" << ip_.as_string() << "] has request service with non-valid 'max_request' [" << request->max_repeats << "].");
  if (request->set_commands) {
    response->success = (setCommands(request->position_command, request->velocity_command, request->current_command, request->max_repeats) == 0);
  }
  return true;
}
