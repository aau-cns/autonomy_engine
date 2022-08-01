// Copyright (C) 2021 Alessandro Fornasier,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <alessandro.fornasier@ieee.org>

#ifndef FORMAT_H
#define FORMAT_H

#include <regex>
#include <sstream>
#include <string>

/**
 * @brief Function to format a standard message for the console UI
 * @param msg string
 * @param number of backslash
 */
inline const std::string formatMsg(const std::string& msg, const size_t& n = 1)
{
  return " >>> " + msg + '.' + std::string(n, '\n');
}

/**
 * @brief Function to format a message reporting that a parameter has not been found for the console UI
 * @param param name string
 * @param additional message
 */
inline const std::string formatParamNotFound(const std::string& name, const std::string& msg = "")
{
  return "[" + name + "] parameter not defined. " + msg + ".\n";
}

/**
 * @brief Function to format a message reporting that a parameter is wrongly defined
 * @param param name string
 * @param additional message
 */
inline const std::string formatParamWrong(const std::string& name, const std::string& msg = "")
{
  return "[" + name + "] " + msg + ".\n";
}

/**
 * @brief Function to format a failure for the console UI
 * @param error_code string
 * @param entity string
 * @param type string
 */
inline const std::string formatFailure(const std::string& error_code, const std::string& entity,
                                       const std::string& type)
{
  std::stringstream ss;
  ss << "-------------------------------------------------\n"
     << " >>> Sensor failure reported by the watchdog\n"
     << " >>> Error code: " << error_code << '\n'
     << " >>> Entity:     " << entity + '\n'
     << " >>> Type:       " << type + '\n'
     << "-------------------------------------------------\n\n";
  return ss.str();
}

/**
 * @brief Function to format a failure fix for the console UI
 * @param entity string
 * @param type string
 */
inline const std::string formatFix(const std::string& entity, const std::string& type)
{
  std::stringstream ss;
  ss << "-------------------------------------------------\n"
     << " >>> Sensor fix reported by the watchdog\n"
     << " >>> Entity: " << entity + '\n'
     << " >>> Type:   " << type + '\n'
     << "-------------------------------------------------\n\n";
  return ss.str();
}

/**
 * @brief Function to format a state entry for the console UI
 * @param string
 */
inline const std::string formatStateEntry(const std::string& state)
{
  std::stringstream ss;
  ss << "-------------------------------------------------\n"
     << " >>> System state: " << state << '\n'
     << "-------------------------------------------------\n\n";
  return ss.str();
}

inline const std::string formatInitMsg()
{
  std::stringstream ss;
  ss << '\n'
     << " ____  _   _  ____          _____  _      ___  ____  _   _  _____   \n"
     << "/ ___|| \\ | |/ ___|        |  ___|| |    |_ _|/ ___|| | | ||_   _| \n"
     << "| |   |  \\| |\\___ \\  _____ | |_   | |     | || |  _ | |_| |  | | \n"
     << "| |___| |\\  | ___) ||_____||  _|  | |___  | || |_| ||  _  |  | |   \n"
     << "\\____||_| \\_||____/        |_|    |_____||___|\\____||_| |_|  |_| \n"
     << "        _    _   _  _____  ___   _   _   ___   __  __ __   __       \n"
     << "       / \\  | | | ||_   _|/ _ \\ | \\ | | / _ \\ |  \\/  |\\ \\ / /\n"
     << "      / _ \\ | | | |  | | | | | ||  \\| || | | || |\\/| | \\ V /    \n"
     << "     / ___ \\| |_| |  | | | |_| || |\\  || |_| || |  | |  | |       \n"
     << "    /_/   \\_\\\\___/   |_|  \\___/ |_| \\_| \\___/ |_|  |_|  |_| \n\n";
  return ss.str();
}

/**
 * @brief Remove formatting including: \n, \t, >>>, -------------------------------------------------, multiple spaces
 * @param string
 */
inline const std::string removeFormat(const std::string& formatted)
{
  std::string formatless;
  std::regex regex("\\n|-------------------------------------------------|\\t|>>>");
  formatless = std::regex_replace(formatted, regex, "");
  formatless = std::regex_replace(formatless, std::regex("( +)"), " ");
  return std::regex_replace(formatless, std::regex("(^ +)"), "");
}

inline const std::string padRight(std::string str, const size_t& width, const char& padding_char = ' ')
{
  if (width > str.size())
  {
    str.insert(str.size(), width - str.size(), padding_char);
  }
  return str;
}

#endif  // FORMAT_H
