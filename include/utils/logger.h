// Copyright (C) 2021 Martin Scheiber and Alessandro Fornasier.
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@aau.at>
// and <alessandro.fornasier@aau.at>

#ifndef AUTONOMY_LOGGER_H_
#define AUTONOMY_LOGGER_H_

#define PAD 1

#include <autonomy_engine/LogMessage.h>
#include <ros/console.h>
#include <ros/node_handle.h>

#include <regex>

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include "utils/colors.h"
#include "utils/format.h"

namespace autonomy
{

/**
 * @brief Log display level
 */
enum LogDisplayLevel
{
  BASIC = 0,     //!< display basic log
  WAYPOINT = 1,  //!< display also waypoints
  SYSTEM = 2,    //!< display system info such as battery level and flight time
  ALL = 3        //!< display every information received also by nodes
};

class Logger
{
private:
  // Loggers
  std::shared_ptr<spdlog::logger> console_ui_logger_ = nullptr;
  std::shared_ptr<spdlog::logger> file_logger_ = nullptr;

  // Ros Nodehandle
  ros::NodeHandle nh_;

  // Publishers
  ros::Publisher pub_log_;
  uint pub_seq_{ 0 };

  // Settings
  LogDisplayLevel log_display_level_{ LogDisplayLevel::BASIC };

  /**
   * @brief Publish log message as a ros message and log to file if file logger is initialized
   */
  inline void publishLog(const autonomy_engine::LogMessage::_type_type& type, const std::string& msg,
                         const std::string& state, const std::string& next_state)
  {
    // setup pub msg
    autonomy_engine::LogMessage pub_msg;
    pub_msg.header.stamp = ros::Time::now();
    pub_msg.header.seq = pub_seq_++;
    pub_msg.header.frame_id = "global";

    pub_msg.type = type;
    pub_msg.msg = msg;
    pub_msg.state = state;
    pub_msg.next_state = next_state;

    // publish ROS message and try to log to file
    if (file_logger_ != nullptr)
    {
      file_logger_->trace(msg + '\n');
      file_logger_->flush();
    }
    pub_log_.publish(pub_msg);
  }

public:
  /**
   * @brief Autonomy constructor
   * @param Ros NodeHandle
   */
  Logger(ros::NodeHandle& nh) : nh_(nh)
  {
    // Loggers
    try
    {
      console_ui_logger_ = spdlog::stdout_color_mt("console_ui_logger");
      console_ui_logger_->set_level(spdlog::level::trace);
    }
    catch (const spdlog::spdlog_ex&)
    {
      std::cout << BOLD(RED("Autonomy console_ui_logger initialization failed")) << std::endl;
      exit(EXIT_FAILURE);
    }

    // Format loggers
    console_ui_logger_->set_pattern("%v");

    // setup publishers
    pub_log_ = nh_.advertise<autonomy_engine::LogMessage>("logger", 1);
  }

  /**
   * @brief Autonomy constructor
   * @param Ros NodeHandle
   * @param Filepath for file logger
   */
  Logger(ros::NodeHandle& nh, const std::string& filepath) : nh_(nh)
  {
    // Loggers
    try
    {
      console_ui_logger_ = spdlog::stdout_color_mt("console_ui_logger");
      console_ui_logger_->set_level(spdlog::level::trace);
      file_logger_ = spdlog::basic_logger_mt("file_logger", filepath);
      file_logger_->set_level(spdlog::level::trace);
    }
    catch (const spdlog::spdlog_ex&)
    {
      std::cout << BOLD(RED("Autonomy loggers initialization failed")) << std::endl;
      exit(EXIT_FAILURE);
    }

    // Format loggers
    console_ui_logger_->set_pattern("%v");
    file_logger_->set_pattern("[%Y-%m-%d %T.%e] | %-0v");

    // setup publishers
    pub_log_ = nh_.advertise<autonomy_engine::LogMessage>("logger", 1);
  }

  /**
   * @brief Initialize file for file logger
   */
  bool initFileLogger(const std::string& filepath)
  {
    try
    {
      file_logger_ = spdlog::basic_logger_mt("file_logger", filepath);
      file_logger_->set_level(spdlog::level::trace);
    }
    catch (const spdlog::spdlog_ex&)
    {
      std::cout << BOLD(RED("Autonomy file_logger initialization failed")) << std::endl;
      exit(EXIT_FAILURE);
    }

    // Format logger
    file_logger_->set_pattern("[%Y-%m-%d %T.%e] | %-0v");

    return (file_logger_ != nullptr);
  }

  /**
   * @brief Format a state change and publish log
   */
  inline void logStateChange(const std::string& cur_state, const std::string& next_state)
  {
    const std::string msg = padRight("[STATE TRANSITION] ", PAD) + "[" + cur_state + "] -> [" + next_state + "]";
    publishLog(autonomy_engine::LogMessage::STATE_CHANGE, msg, cur_state, next_state);
  }

  /**
   * @brief Format a service call and publish log
   */
  inline void logServiceCall(const std::string& cur_state, const std::string& service,
                             const std::string& next_state = "")
  {
    const std::string msg = padRight("[CALLED SERVICE] ", PAD) + service;
    publishLog(autonomy_engine::LogMessage::SERVICE_CALLED, msg, cur_state, next_state);
  }

  /**
   * @brief Format a service response and publish log
   */
  inline void logServiceResponse(const std::string& cur_state, const std::string& service, const std::string& response,
                                 const std::string& next_state = "")
  {
    const std::string msg = padRight("[SERVICE: " + service + " RESPONSE] ", PAD) + response;
    publishLog(autonomy_engine::LogMessage::SERVICE_RESPONSE, msg, cur_state, next_state);
  }

  /**
   * @brief Format a message and publish log
   */
  inline void logMessage(const std::string& cur_state, const std::string& topic, const bool& received = false,
                         const std::string& contents = "_emtpy_", const std::string& next_state = "")
  {
    if (received)
    {
      const std::string msg = padRight("[MESSAGE RECEIVED ON: " + topic + "] ", PAD) + contents;
      publishLog(autonomy_engine::LogMessage::MESSAGE_RECEIVED, msg, cur_state, next_state);
    }
    else
    {
      const std::string msg = padRight("[MESSAGE SENT ON " + topic + "] ", PAD) + contents;
      publishLog(autonomy_engine::LogMessage::MESSAGE_SENT, msg, cur_state, next_state);
    }
  }

  /**
   * @brief Format a user input and publish log
   */
  inline void logUserInput(const std::string& cur_state, const std::string& input, const std::string& next_state = "")
  {
    const std::string msg = padRight("[USER INPUT] ", PAD) + input;
    publishLog(autonomy_engine::LogMessage::USER_INPUT, msg, cur_state, next_state);
  }

  /**
   * @brief Format user interface and publish log
   */
  inline void logUserInterface(const std::string& cur_state, const std::string& ui, const std::string& next_state = "")
  {
    const std::string msg = padRight("[USER INTERFACE] ", PAD) + ui;
    publishLog(autonomy_engine::LogMessage::USER_INTERFACE, msg, cur_state, next_state);
  }

  /**
   * @brief Format info and publish log
   */
  inline void logInfo(const std::string& cur_state, const std::string& info, const std::string& next_state = "")
  {
    const std::string msg = padRight("[INFO] ", PAD) + info;
    publishLog(autonomy_engine::LogMessage::INFO, msg, cur_state, next_state);
  }

  /**
   * @brief Print UI and log user interface
   */
  inline void logUI(const std::string& cur_state, const std::string& escape_str, const std::string& ui,
                    const LogDisplayLevel& display_level = LogDisplayLevel::BASIC)
  {
    // Try to log to console (UI)
    if (console_ui_logger_ != nullptr && display_level <= log_display_level_)
    {
      console_ui_logger_->trace(escape_str + ui + RESET);
      console_ui_logger_->flush();
    }

    // Log to file and publish
    logUserInterface(cur_state, removeFormat(ui));
  }

  void setLogDisplayLevel(const int& level)
  {
    log_display_level_ = static_cast<LogDisplayLevel>(level);
  }

  void setLogDisplayLevel(const LogDisplayLevel& level)
  {
    log_display_level_ = level;
  }

};  // class Logger

}  // namespace autonomy

#endif  // AUTONOMY_LOGGER_H_
