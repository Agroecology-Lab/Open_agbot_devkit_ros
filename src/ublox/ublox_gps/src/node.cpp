//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.
//==============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <ublox_msgs/msg/aid_alm.hpp>
#include <ublox_msgs/msg/aid_eph.hpp>
#include <ublox_msgs/msg/aid_hui.hpp>
#include <ublox_msgs/msg/cfg_inf.hpp>
#include <ublox_msgs/msg/cfg_inf_block.hpp>
#include <ublox_msgs/msg/cfg_nav5.hpp>
#include <ublox_msgs/msg/cfg_prt.hpp>
#include <ublox_msgs/msg/inf.hpp>
#include <ublox_msgs/msg/mon_ver.hpp>
#include <ublox_msgs/msg/nav_clock.hpp>
#include <ublox_msgs/msg/nav_cov.hpp>
#include <ublox_msgs/msg/nav_posecef.hpp>
#include <ublox_msgs/msg/nav_status.hpp>

#include <nmea_msgs/msg/sentence.hpp>

#include <ublox_gps/adr_udr_product.hpp>
#include <ublox_gps/fix_diagnostic.hpp>
#include <ublox_gps/fts_product.hpp>
#include <ublox_gps/gnss.hpp>
#include <ublox_gps/hp_pos_rec_product.hpp>
#include <ublox_gps/hpg_ref_product.hpp>
#include <ublox_gps/hpg_rov_product.hpp>
#include <ublox_gps/node.hpp>
#include <ublox_gps/raw_data_product.hpp>
#include <ublox_gps/tim_product.hpp>
#include <ublox_gps/ublox_firmware6.hpp>
#include <ublox_gps/ublox_firmware7.hpp>
#include <ublox_gps/ublox_firmware8.hpp>
#include <ublox_gps/ublox_firmware9.hpp>

namespace ublox_node {

uint8_t modelFromString(const std::string& model) {
  std::string lower = model;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  if (lower == "portable") return ublox_msgs::msg::CfgNAV5::DYN_MODEL_PORTABLE;
  if (lower == "stationary") return ublox_msgs::msg::CfgNAV5::DYN_MODEL_STATIONARY;
  if (lower == "pedestrian") return ublox_msgs::msg::CfgNAV5::DYN_MODEL_PEDESTRIAN;
  if (lower == "automotive") return ublox_msgs::msg::CfgNAV5::DYN_MODEL_AUTOMOTIVE;
  if (lower == "sea") return ublox_msgs::msg::CfgNAV5::DYN_MODEL_SEA;
  if (lower == "airborne1") return ublox_msgs::msg::CfgNAV5::DYN_MODEL_AIRBORNE_1G;
  if (lower == "airborne2") return ublox_msgs::msg::CfgNAV5::DYN_MODEL_AIRBORNE_2G;
  if (lower == "airborne4") return ublox_msgs::msg::CfgNAV5::DYN_MODEL_AIRBORNE_4G;
  if (lower == "wristwatch") return ublox_msgs::msg::CfgNAV5::DYN_MODEL_WRIST_WATCH;

  throw std::runtime_error("Invalid settings: " + lower + " is not a valid dynamic model.");
}

uint8_t fixModeFromString(const std::string& mode) {
  std::string lower = mode;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  if (lower == "2d") return ublox_msgs::msg::CfgNAV5::FIX_MODE_2D_ONLY;
  if (lower == "3d") return ublox_msgs::msg::CfgNAV5::FIX_MODE_3D_ONLY;
  if (lower == "auto") return ublox_msgs::msg::CfgNAV5::FIX_MODE_AUTO;

  throw std::runtime_error("Invalid settings: " + mode + " is not a valid fix mode.");
}

std::vector<std::string> stringSplit(const std::string &str, const std::string &splitter) {
  std::vector<std::string> ret;
  size_t next = 0;
  size_t current = next;
  if (splitter.empty()) {
    ret.push_back(str);
    return ret;
  }
  while (next != std::string::npos) {
    next = str.find(splitter, current);
    ret.push_back(str.substr(current, next - current));
    current = next + splitter.length();
  }
  return ret;
}

UbloxNode::UbloxNode(const rclcpp::NodeOptions & options) : rclcpp::Node("ublox_gps_node", options) {
  int debug = this->declare_parameter("debug", 1);
  if (debug) {
    rcutils_logging_set_logger_level("ublox_gps_node", RCUTILS_LOG_SEVERITY_DEBUG);
  }

  gps_ = std::make_shared<ublox_gps::Gps>(debug, this->get_logger());
  gnss_ = std::make_shared<Gnss>();
  updater_ = std::make_shared<diagnostic_updater::Updater>(this);
  updater_->setHardwareID("ublox");

  initialize();
}

void UbloxNode::rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg) {
  gps_->sendRtcm(msg->message);
}

void UbloxNode::addFirmwareInterface() {
  int ublox_version;
  if (protocol_version_ < 14.0) {
    components_.push_back(std::make_shared<UbloxFirmware6>(frame_id_, updater_, freq_diag_, gnss_, this));
    ublox_version = 6;
  } else if (protocol_version_ >= 14.0 && protocol_version_ <= 15.0) {
    components_.push_back(std::make_shared<UbloxFirmware7>(frame_id_, updater_, freq_diag_, gnss_, this));
    ublox_version = 7;
  } else if (protocol_version_ > 15.0 && protocol_version_ <= 23.0) {
    components_.push_back(std::make_shared<UbloxFirmware8>(frame_id_, updater_, freq_diag_, gnss_, this));
    ublox_version = 8;
  } else {
    components_.push_back(std::make_shared<UbloxFirmware9>(frame_id_, updater_, freq_diag_, gnss_, this));
    ublox_version = 9;
  }
  RCLCPP_INFO(this->get_logger(), "U-Blox Firmware Version: %d", ublox_version);
}

void UbloxNode::addProductInterface(const std::string & product_category, const std::string & ref_rov) {
  if ((product_category ==
