// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
// 
// All rights reserved.
// 
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
// 
// You can contact the author at <martin.scheiber@aau.at>

#ifndef AUTONOMY_LOGGER_H_
#define AUTONOMY_LOGGER_H_

#include <ros/node_handle.h>
#include <ros/console.h>

#include <autonomy/LogMessage.h>

#define BKG_DEBUG_STREAM_NAMED(name, args) ROS_DEBUG_STREAM_NAMED(name, "* AUT_BKG: " << args)
#define BKG_INFO_STREAM_NAMED(name, args) ROS_INFO_STREAM_NAMED(name, "* AUT_BKG: " << args)
#define BKG_WARN_STREAM_NAMED(name, args) ROS_WARN_STREAM_NAMED(name, "* AUT_BKG: " << args)
#define BKG_ERROR_STREAM_NAMED(name, args) ROS_ERROR_STREAM_NAMED(name, "* AUT_BKG: " << args)
#define BKG_FATAL_STREAM_NAMED(name, args) ROS_FATAL_STREAM_NAMED(name, "* AUT_BKG: " << args)

#define AUTONOMY_UI_STREAM(args) do{std::cerr << args; ROS_INFO_STREAM(args);} while (false)

namespace autonomy {

class Logger {
private:
  const std::string C_LOGGER_NAME_ { "background_logger" };

  // Ros Nodehandle
  ros::NodeHandle nh_;

  // Publishers
  ros::Publisher pub_log_;
  uint pub_seq_ {0};

  // publish functions
  void publishLog(const autonomy::LogMessage::_type_type &type, const std::string &msg, const std::string &state, const std::string &next_state = "undef")
  {
    // setup pub msg
    autonomy::LogMessage pub_msg;
    pub_msg.header.stamp = ros::Time::now();
    pub_msg.header.seq = pub_seq_++;
    pub_msg.header.frame_id = "global";

    pub_msg.type = type;
    pub_msg.msg = msg;
    pub_msg.state = state;
    pub_msg.next_state = next_state;

    // log to file and publish ROS message
    BKG_INFO_STREAM_NAMED(C_LOGGER_NAME_, msg);
    pub_log_.publish(pub_msg);
  }

public:
  Logger(ros::NodeHandle &nh) : nh_(nh) {

    // setup logger level to FATAL for the background logger (except for DEBUG builds)
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME + std::string(".") + C_LOGGER_NAME_,
#ifdef NDEBUG
                                       ros::console::levels::Info) ) {
#else // DEBUG
                                       ros::console::levels::Debug) ) {
#endif // NDEBUG
      ros::console::notifyLoggerLevelsChanged();
    }

    BKG_INFO_STREAM_NAMED(C_LOGGER_NAME_, "started background logging");

    // setup publishers
    pub_log_ = nh_.advertise<autonomy::LogMessage>("logger", 1);
  }

  void logStateChange(const std::string &cur_state, const std::string &next_state) {
    const std::string msg = "state transition: " + cur_state + "->" + next_state;
    publishLog(autonomy::LogMessage::STATE_CHANGE, msg, cur_state, next_state);
  } // void logStateChange()

  void logServiceCall(const std::string &cur_state, const std::string &service){
    const std::string msg = "called service: " + service;
    publishLog(autonomy::LogMessage::SERVICE_CALLED, msg, cur_state);
  }

  void logServiceAnswer(const std::string &cur_state, const std::string &service, const std::string &answer){
    const std::string msg = "service answered: [" + service + "] -- " + answer;
    publishLog(autonomy::LogMessage::SERVICE_ANSWERED, msg, cur_state);
  }

  void logMessage(const std::string &cur_state, const std::string &topic, const bool &received=false, const std::string &contents="_emtpy_"){
    if (received)
    {
      const std::string msg = "message received on [" + topic + "] -- " + contents;
      publishLog(autonomy::LogMessage::MESSAGE_RECEIVED, msg, cur_state);
    }
    else
    {
      const std::string msg = "message sent on [" + topic + "] -- " + contents;
      publishLog(autonomy::LogMessage::MESSAGE_SENT, msg, cur_state);
    }
  }

  void logUserInput(const std::string &cur_state, const std::string &input){
    const std::string msg = "user input: " + input;
    publishLog(autonomy::LogMessage::USER_INPUT, msg, cur_state);
  }

  void logInfo(const std::string &cur_state, const std::string &info){
    publishLog(autonomy::LogMessage::INFO, info, cur_state);
  }

}; // class Logger

} // namespace autonomy

#endif // AUTONOMY_LOGGER_H_
