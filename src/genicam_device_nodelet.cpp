/*
 * Copyright (c) 2020 Roboception GmbH
 * All rights reserved
 *
 * Author: Heiko Hirschmueller
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "genicam_device_nodelet.h"
#include "publishers/camera_info_publisher.h"
#include "publishers/camera_param_publisher.h"
#include "publishers/image_publisher.h"
#include "publishers/disparity_publisher.h"
#include "publishers/disparity_color_publisher.h"
#include "publishers/depth_publisher.h"
#include "publishers/confidence_publisher.h"
#include "publishers/error_disparity_publisher.h"
#include "publishers/error_depth_publisher.h"
#include "publishers/points2_publisher.h"

#include <rc_genicam_api/device.h>
#include <rc_genicam_api/stream.h>
#include <rc_genicam_api/buffer.h>
#include <rc_genicam_api/config.h>
#include <rc_genicam_api/pixel_formats.h>

#include <pluginlib/class_list_macros.h>
#include <exception>

#include <sstream>
#include <stdexcept>

#include <ros/ros.h>
#include <rc_common_msgs/ReturnCodeConstants.h>
#include <rc_common_msgs/CameraParam.h>

namespace rc
{
GenICamDeviceNodelet::GenICamDeviceNodelet()
{
  scomponents = 0;
  scolor = 0;

  running = false;

  gev_packet_size = 0;
  connection_loss_total = 0;
  complete_buffers_total = 0;
  incomplete_buffers_total = 0;
  image_receive_timeouts_total = 0;
  current_reconnect_trial = 0;
  streaming = false;
}

GenICamDeviceNodelet::~GenICamDeviceNodelet()
{
  NODELET_INFO("Shutting down");

  // signal running threads and wait until they finish

  running = false;
  if (grab_thread.joinable())
  {
    grab_thread.join();
  }

  rcg::System::clearSystems();
}

void GenICamDeviceNodelet::onInit()
{
  NODELET_INFO("Initialization started");

  std::string ns = ros::this_node::getNamespace();

  if (ns.size() > 0 && ns[0] == '/')
  {
    ns = ns.substr(1);
  }

  if (ns.size() > 0)
  {
    frame_id = ns + "_camera";
  }
  else
  {
    frame_id = "camera";
  }

  // get parameter configuration

  ros::NodeHandle pnh(getPrivateNodeHandle());
  ros::NodeHandle nh(getNodeHandle(), "");

  std::string id = "*";
  std::string access = "control";

  pnh.param("device", id, id);
  pnh.param("gev_access", access, access);

  rcg::Device::ACCESS access_id;
  if (access == "exclusive")
  {
    access_id = rcg::Device::EXCLUSIVE;
  }
  else if (access == "control")
  {
    access_id = rcg::Device::CONTROL;
  }
  else
  {
    NODELET_FATAL_STREAM("Access must be 'control' or 'exclusive': " << access);
    return;
  }

  // setup services

  trigger_service =
      pnh.advertiseService("depth_acquisition_trigger", &GenICamDeviceNodelet::depthAcquisitionTrigger, this);

  // add callbacks for diagnostics publishing

  updater.add("Connection", this, &GenICamDeviceNodelet::publishConnectionDiagnostics);
  updater.add("Device", this, &GenICamDeviceNodelet::publishDeviceDiagnostics);

  // start grabbing thread

  running = true;
  grab_thread = std::thread(&GenICamDeviceNodelet::grab, this, id, access_id);

  NODELET_INFO("Initialization done");
}

bool GenICamDeviceNodelet::depthAcquisitionTrigger(rc_common_msgs::Trigger::Request& req,
                                                   rc_common_msgs::Trigger::Response& res)
{
  std::lock_guard<std::recursive_mutex> lock(device_mtx);

  if (nodemap)
  {
    if (config.depth_acquisition_mode != "Continuous")
    {
      try
      {
        NODELET_DEBUG("Triggering stereo matching");

        rcg::callCommand(nodemap, "DepthAcquisitionTrigger", true);

        res.return_code.value = rc_common_msgs::ReturnCodeConstants::SUCCESS;
        res.return_code.message = "Stereo matching was triggered.";
      }
      catch (const std::exception& ex)
      {
        res.return_code.value = rc_common_msgs::ReturnCodeConstants::INTERNAL_ERROR;
        res.return_code.message = ex.what();
        NODELET_ERROR_STREAM(ex.what());
      }
    }
    else
    {
      res.return_code.value = rc_common_msgs::ReturnCodeConstants::NOT_APPLICABLE;
      res.return_code.message = "Triggering stereo matching is only possible if acquisition_mode is set to SingleFrame "
                                "or SingleFrameOut1!";
      NODELET_DEBUG_STREAM("" << res.return_code.message);
    }
  }
  else
  {
    res.return_code.value = rc_common_msgs::ReturnCodeConstants::NOT_APPLICABLE;
    res.return_code.message = "Not connected";
  }

  return true;
}

void GenICamDeviceNodelet::initConfiguration()
{
  std::lock_guard<std::recursive_mutex> lock(device_mtx);

  ros::NodeHandle pnh(getPrivateNodeHandle());

  // get current camera configuration

  config.camera_fps = rcg::getFloat(nodemap, "AcquisitionFrameRate", 0, 0, true);

  std::string v = rcg::getEnum(nodemap, "ExposureAuto", true);

  if (v == "Off")
  {
    config.camera_exp_control = "Manual";
    config.camera_exp_auto_mode = "Normal";
  }
  else if (v == "HDR")
  {
    config.camera_exp_control = "HDR";
    config.camera_exp_auto_mode = "Normal";
  }
  else
  {
    config.camera_exp_control = "Auto";

    if (v == "Continuous")
    {
      config.camera_exp_auto_mode = "Normal";
    }
    else
    {
      config.camera_exp_auto_mode = v;
    }
  }

  config.camera_exp_max = rcg::getFloat(nodemap, "ExposureTimeAutoMax", 0, 0, true) / 1000000;

  try
  {
    config.camera_exp_auto_average_max = rcg::getFloat(nodemap, "RcExposureAutoAverageMax", 0, 0, true);
    config.camera_exp_auto_average_min = rcg::getFloat(nodemap, "RcExposureAutoAverageMin", 0, 0, true);
  }
  catch (const std::exception&)
  {
    config.camera_exp_auto_average_max = 0.75f;
    config.camera_exp_auto_average_min = 0.25f;
  }

  config.camera_exp_width = rcg::getInteger(nodemap, "ExposureRegionWidth", 0, 0, true);
  config.camera_exp_height = rcg::getInteger(nodemap, "ExposureRegionHeight", 0, 0, true);
  config.camera_exp_offset_x = rcg::getInteger(nodemap, "ExposureRegionOffsetX", 0, 0, true);
  config.camera_exp_offset_y = rcg::getInteger(nodemap, "ExposureRegionOffsetY", 0, 0, true);
  config.camera_exp_value = rcg::getFloat(nodemap, "ExposureTime", 0, 0, true) / 1000000;

  rcg::setEnum(nodemap, "GainSelector", "All", false);
  config.camera_gain_value = rcg::getFloat(nodemap, "Gain", 0, 0, true);

  config.camera_gamma = rcg::getFloat(nodemap, "Gamma", 0, 0, false);
  if (config.camera_gamma == 0)
  {
    config.camera_gamma=1.0;
  }

  try
  {
    std::string v = rcg::getEnum(nodemap, "BalanceWhiteAuto", true);

    config.camera_wb_auto = (v != "Off");
    rcg::setEnum(nodemap, "BalanceRatioSelector", "Red", true);
    config.camera_wb_ratio_red = rcg::getFloat(nodemap, "BalanceRatio", 0, 0, true);
    rcg::setEnum(nodemap, "BalanceRatioSelector", "Blue", true);
    config.camera_wb_ratio_blue = rcg::getFloat(nodemap, "BalanceRatio", 0, 0, true);
  }
  catch (const std::exception&)
  {
    config.camera_wb_auto = true;
    config.camera_wb_ratio_red = 1.2;
    config.camera_wb_ratio_blue = 2.4;
  }

  // get depth image configuration

  config.depth_acquisition_mode = rcg::getEnum(nodemap, "DepthAcquisitionMode", true);
  config.depth_quality = rcg::getEnum(nodemap, "DepthQuality", true);
  config.depth_static_scene = rcg::getBoolean(nodemap, "DepthStaticScene", true);
  config.depth_double_shot = rcg::getBoolean(nodemap, "DepthDoubleShot", false);
  config.depth_seg = rcg::getInteger(nodemap, "DepthSeg", 0, 0, true);
  config.depth_smooth = rcg::getBoolean(nodemap, "DepthSmooth", true);
  config.depth_fill = rcg::getInteger(nodemap, "DepthFill", 0, 0, true);
  config.depth_minconf = rcg::getFloat(nodemap, "DepthMinConf", 0, 0, true);
  config.depth_mindepth = rcg::getFloat(nodemap, "DepthMinDepth", 0, 0, true);
  config.depth_maxdepth = rcg::getFloat(nodemap, "DepthMaxDepth", 0, 0, true);
  config.depth_maxdeptherr = rcg::getFloat(nodemap, "DepthMaxDepthErr", 0, 0, true);

  config.ptp_enabled = rcg::getBoolean(nodemap, "PtpEnable", false);

  rcg::setEnum(nodemap, "LineSelector", "Out1", true);
  config.out1_mode = rcg::getEnum(nodemap, "LineSource", true);

  rcg::setEnum(nodemap, "LineSelector", "Out2", false);
  config.out2_mode = rcg::getEnum(nodemap, "LineSource", true);

  try
  {
    config.depth_exposure_adapt_timeout = rcg::getFloat(nodemap, "DepthExposureAdaptTimeout", 0, 0, true);
  }
  catch (const std::exception&)
  {
    NODELET_WARN("rc_visard_driver: rc_visard has an older firmware, depth_exposure_adapt_timeout is not available.");
  }

  // try to get ROS parameters: if parameter is not set in parameter server
  // default to current sensor configuration

  pnh.param("camera_fps", config.camera_fps, config.camera_fps);
  pnh.param("camera_exp_control", config.camera_exp_control, config.camera_exp_control);
  pnh.param("camera_exp_auto_mode", config.camera_exp_auto_mode, config.camera_exp_auto_mode);
  pnh.param("camera_exp_max", config.camera_exp_max, config.camera_exp_max);
  pnh.param("camera_exp_auto_average_max", config.camera_exp_auto_average_max, config.camera_exp_auto_average_max);
  pnh.param("camera_exp_auto_average_min", config.camera_exp_auto_average_min, config.camera_exp_auto_average_min);
  pnh.param("camera_exp_value", config.camera_exp_value, config.camera_exp_value);
  pnh.param("camera_gain_value", config.camera_gain_value, config.camera_gain_value);
  pnh.param("camera_gamma", config.camera_gamma, config.camera_gamma);
  pnh.param("camera_exp_offset_x", config.camera_exp_offset_x, config.camera_exp_offset_x);
  pnh.param("camera_exp_offset_y", config.camera_exp_offset_y, config.camera_exp_offset_y);
  pnh.param("camera_exp_width", config.camera_exp_width, config.camera_exp_width);
  pnh.param("camera_exp_height", config.camera_exp_height, config.camera_exp_height);
  pnh.param("camera_wb_auto", config.camera_wb_auto, config.camera_wb_auto);
  pnh.param("camera_wb_ratio_red", config.camera_wb_ratio_red, config.camera_wb_ratio_red);
  pnh.param("camera_wb_ratio_blue", config.camera_wb_ratio_blue, config.camera_wb_ratio_blue);
  pnh.param("depth_acquisition_mode", config.depth_acquisition_mode, config.depth_acquisition_mode);
  pnh.param("depth_quality", config.depth_quality, config.depth_quality);
  pnh.param("depth_static_scene", config.depth_static_scene, config.depth_static_scene);
  pnh.param("depth_double_shot", config.depth_double_shot, config.depth_double_shot);
  pnh.param("depth_seg", config.depth_seg, config.depth_seg);
  pnh.param("depth_smooth", config.depth_smooth, config.depth_smooth);
  pnh.param("depth_fill", config.depth_fill, config.depth_fill);
  pnh.param("depth_minconf", config.depth_minconf, config.depth_minconf);
  pnh.param("depth_mindepth", config.depth_mindepth, config.depth_mindepth);
  pnh.param("depth_maxdepth", config.depth_maxdepth, config.depth_maxdepth);
  pnh.param("depth_maxdeptherr", config.depth_maxdeptherr, config.depth_maxdeptherr);
  pnh.param("depth_exposure_adapt_timeout", config.depth_exposure_adapt_timeout, config.depth_exposure_adapt_timeout);
  pnh.param("ptp_enabled", config.ptp_enabled, config.ptp_enabled);
  pnh.param("out1_mode", config.out1_mode, config.out1_mode);
  pnh.param("out2_mode", config.out2_mode, config.out2_mode);

  // set parameters on parameter server so that dynamic reconfigure picks them up

  pnh.setParam("camera_fps", config.camera_fps);
  pnh.setParam("camera_exp_control", config.camera_exp_control);
  pnh.setParam("camera_exp_auto_mode", config.camera_exp_auto_mode);
  pnh.setParam("camera_exp_max", config.camera_exp_max);
  pnh.setParam("camera_exp_auto_average_max", config.camera_exp_auto_average_max);
  pnh.setParam("camera_exp_auto_average_min", config.camera_exp_auto_average_min);
  pnh.setParam("camera_exp_value", config.camera_exp_value);
  pnh.setParam("camera_gain_value", config.camera_gain_value);
  pnh.setParam("camera_gamma", config.camera_gamma);
  pnh.setParam("camera_exp_offset_x", config.camera_exp_offset_x);
  pnh.setParam("camera_exp_offset_y", config.camera_exp_offset_y);
  pnh.setParam("camera_exp_width", config.camera_exp_width);
  pnh.setParam("camera_exp_height", config.camera_exp_height);
  pnh.setParam("camera_wb_auto", config.camera_wb_auto);
  pnh.setParam("camera_wb_ratio_red", config.camera_wb_ratio_red);
  pnh.setParam("camera_wb_ratio_blue", config.camera_wb_ratio_blue);
  pnh.setParam("depth_acquisition_mode", config.depth_acquisition_mode);
  pnh.setParam("depth_quality", config.depth_quality);
  pnh.setParam("depth_static_scene", config.depth_static_scene);
  pnh.setParam("depth_double_shot", config.depth_double_shot);
  pnh.setParam("depth_seg", config.depth_seg);
  pnh.setParam("depth_smooth", config.depth_smooth);
  pnh.setParam("depth_fill", config.depth_fill);
  pnh.setParam("depth_minconf", config.depth_minconf);
  pnh.setParam("depth_mindepth", config.depth_mindepth);
  pnh.setParam("depth_maxdepth", config.depth_maxdepth);
  pnh.setParam("depth_maxdeptherr", config.depth_maxdeptherr);
  pnh.setParam("depth_exposure_adapt_timeout", config.depth_exposure_adapt_timeout);
  pnh.setParam("ptp_enabled", config.ptp_enabled);
  pnh.setParam("out1_mode", config.out1_mode);
  pnh.setParam("out2_mode", config.out2_mode);
}

void GenICamDeviceNodelet::reconfigure(rc_genicam_driver::rc_genicam_driverConfig& c, uint32_t level)
{
  std::lock_guard<std::recursive_mutex> lock(device_mtx);

  try
  {
    if (nodemap)
    {
      if (level & 1)
      {
        rcg::setFloat(nodemap, "AcquisitionFrameRate", c.camera_fps, true);
      }

      if (level & 2)
      {
        if (c.camera_exp_control == "HDR")
        {
          try
          {
            rcg::setEnum(nodemap, "ExposureAuto", "HDR", true);
          }
          catch (const std::exception &)
          {
            NODELET_WARN_STREAM("Sensor does not support HDR. Please update firmware.");
            c.camera_exp_control = "Auto";
          }
        }

        if (c.camera_exp_control == "Auto")
        {
          // Normal means continuous and off must be controlled with exp_auto

          if (c.camera_exp_auto_mode == "Off" || c.camera_exp_auto_mode == "Normal")
          {
            c.camera_exp_auto_mode = "Continuous";
          }

          // find user requested auto exposure mode in the list of enums

          std::vector<std::string> list;
          rcg::getEnum(nodemap, "ExposureAuto", list, false);

          std::string mode = "Continuous";
          for (size_t i = 0; i < list.size(); i++)
          {
            if (c.camera_exp_auto_mode == list[i])
            {
              mode = list[i];
            }
          }

          // set auto exposure mode

          rcg::setEnum(nodemap, "ExposureAuto", mode.c_str(), true);

          // Continuous means normal

          if (mode == "Continuous")
          {
            mode = "Normal";
          }

          c.camera_exp_auto_mode = mode;
        }
        else if (c.camera_exp_control != "HDR")
        {
          c.camera_exp_control = "Manual";

          rcg::setEnum(nodemap, "ExposureAuto", "Off", true);

          usleep(100 * 1000);

          c.camera_exp_value = rcg::getFloat(nodemap, "ExposureTime", 0, 0, true, true) / 1000000;
          c.camera_gain_value = rcg::getFloat(nodemap, "Gain", 0, 0, true, true);
        }
      }

      if (level & 4)
      {
        rcg::setFloat(nodemap, "ExposureTimeAutoMax", 1000000 * c.camera_exp_max, true);
      }

      if (level & 8)
      {
        if (!rcg::setFloat(nodemap, "RcExposureAutoAverageMax", c.camera_exp_auto_average_max, false))
        {
          NODELET_WARN("rc_visard does not support parameter 'exp_auto_average_max'");
          c.camera_exp_auto_average_max = 0.75f;
        }
      }

      if (level & 16)
      {
        if (!rcg::setFloat(nodemap, "RcExposureAutoAverageMin", c.camera_exp_auto_average_min, false))
        {
          NODELET_WARN("rc_visard does not support parameter 'exp_auto_average_min'");
          c.camera_exp_auto_average_min = 0.25f;
        }
      }

      if (level & 32)
      {
        rcg::setFloat(nodemap, "ExposureTime", 1000000 * c.camera_exp_value, true);
      }

      c.camera_gain_value = round(c.camera_gain_value / 6) * 6;

      if (level & 64)
      {
        rcg::setFloat(nodemap, "Gain", c.camera_gain_value, true);
      }

      if (level & 536870912)
      {
        try
        {
          rcg::setFloat(nodemap, "Gamma", c.camera_gamma, true);
        }
        catch (const std::exception &)
        {
          if (c.camera_gamma != 1.0)
          {
            NODELET_WARN_STREAM("Sensor does not support gamma. Please update firmware.");
            c.camera_gamma = 1.0;
          }
        }
      }

      if (level & 128)
      {
        rcg::setInteger(nodemap, "ExposureRegionOffsetX", c.camera_exp_offset_x, true);
      }

      if (level & 256)
      {
        rcg::setInteger(nodemap, "ExposureRegionOffsetY", c.camera_exp_offset_y, true);
      }

      if (level & 512)
      {
        rcg::setInteger(nodemap, "ExposureRegionWidth", c.camera_exp_width, true);
      }

      if (level & 1024)
      {
        rcg::setInteger(nodemap, "ExposureRegionHeight", c.camera_exp_height, true);
      }

      bool color_ok = true;

      if (level & 2048)
      {
        if (c.camera_wb_auto)
        {
          color_ok = rcg::setEnum(nodemap, "BalanceWhiteAuto", "Continuous", false);
        }
        else
        {
          color_ok = rcg::setEnum(nodemap, "BalanceWhiteAuto", "Off", false);

          usleep(100 * 1000);

          rcg::setEnum(nodemap, "BalanceRatioSelector", "Red", false);
          c.camera_wb_ratio_red = rcg::getFloat(nodemap, "BalanceRatio", 0, 0, false, true);

          rcg::setEnum(nodemap, "BalanceRatioSelector", "Blue", false);
          c.camera_wb_ratio_blue = rcg::getFloat(nodemap, "BalanceRatio", 0, 0, false, true);
        }
      }

      if (level & 4096)
      {
        rcg::setEnum(nodemap, "BalanceRatioSelector", "Red", false);
        color_ok = rcg::setFloat(nodemap, "BalanceRatio", c.camera_wb_ratio_red, false);
      }

      if (level & 8192)
      {
        rcg::setEnum(nodemap, "BalanceRatioSelector", "Blue", false);
        color_ok = rcg::setFloat(nodemap, "BalanceRatio", c.camera_wb_ratio_blue, false);
      }

      if (!color_ok)
      {
        c.camera_wb_auto = true;
        c.camera_wb_ratio_red = 1.2;
        c.camera_wb_ratio_blue = 2.4;
      }

      if (level & 16384)
      {
        // correct configuration strings if needed

        if (c.depth_acquisition_mode == "S" || c.depth_acquisition_mode == "SingleFrame")
        {
          c.depth_acquisition_mode = "SingleFrame";
        }
        else if (c.depth_acquisition_mode == "O" || c.depth_acquisition_mode == "SingleFrameOut1")
        {
          c.depth_acquisition_mode = "SingleFrameOut1";
        }
        else if (c.depth_acquisition_mode == "C" || c.depth_acquisition_mode == "Continuous")
        {
          c.depth_acquisition_mode = "Continuous";
        }
        else
        {
          c.depth_acquisition_mode = "Continuous";
        }

        rcg::setEnum(nodemap, "DepthAcquisitionMode", c.depth_acquisition_mode.c_str(), true);
      }

      if (level & 32768)
      {
        if (c.depth_quality == "Full" || c.depth_quality == "F")
        {
          c.depth_quality = "Full";
        }
        else if (c.depth_quality == "High" || c.depth_quality == "H")
        {
          c.depth_quality = "High";
        }
        else if (c.depth_quality == "Medium" || c.depth_quality == "M")
        {
          c.depth_quality = "Medium";
        }
        else if (c.depth_quality == "Low" || c.depth_quality == "L")
        {
          c.depth_quality = "Low";
        }
        else
        {
          c.depth_quality = "High";
        }

        try
        {
          rcg::setEnum(nodemap, "DepthQuality", c.depth_quality.c_str(), true);
        }
        catch (const std::exception&)
        {
          c.depth_quality = "High";
          rcg::setEnum(nodemap, "DepthQuality", c.depth_quality.c_str(), false);

          NODELET_ERROR("Cannot set full quality. Sensor may have no 'stereo_plus' license!");
        }
      }

      if (level & 65536)
      {
        rcg::setBoolean(nodemap, "DepthStaticScene", c.depth_static_scene, true);
      }

      if (level & 131072)
      {
        try
        {
          rcg::setBoolean(nodemap, "DepthDoubleShot", c.depth_double_shot, true);
        }
        catch (const std::exception&)
        {
          c.depth_double_shot = false;
          NODELET_ERROR("Cannot set double shot mode. Please update the sensor to version >= 20.11.0!");
        }
      }

      if (level & 262144)
      {
        rcg::setInteger(nodemap, "DepthSeg", c.depth_seg, true);
      }

      if (level & 524288 && c.depth_smooth != config.depth_smooth)
      {
        try
        {
          rcg::setBoolean(nodemap, "DepthSmooth", c.depth_smooth, true);
        }
        catch (const std::exception&)
        {
          c.depth_smooth = false;
          rcg::setBoolean(nodemap, "DepthSmooth", c.depth_smooth, false);

          NODELET_ERROR("Cannot switch on smoothing. Sensor may have no 'stereo_plus' license!");
        }
      }

      if (level & 1048576)
      {
        rcg::setInteger(nodemap, "DepthFill", c.depth_fill, true);
      }

      if (level & 2097152)
      {
        rcg::setFloat(nodemap, "DepthMinConf", c.depth_minconf, true);
      }

      if (level & 4194304)
      {
        rcg::setFloat(nodemap, "DepthMinDepth", c.depth_mindepth, true);
      }

      if (level & 8388608)
      {
        rcg::setFloat(nodemap, "DepthMaxDepth", c.depth_maxdepth, true);
      }

      if (level & 16777216)
      {
        rcg::setFloat(nodemap, "DepthMaxDepthErr", c.depth_maxdeptherr, true);
      }

      if ((level & 33554432) && c.ptp_enabled != config.ptp_enabled)
      {
        if (!rcg::setBoolean(nodemap, "PtpEnable", c.ptp_enabled, false))
        {
          NODELET_ERROR("Cannot change PTP.");
          c.ptp_enabled = false;
        }
      }

      if ((level & 67108864) && c.out1_mode != config.out1_mode)
      {
        if (c.out1_mode != "Low" && c.out1_mode != "High" && c.out1_mode != "ExposureActive" &&
            c.out1_mode != "ExposureAlternateActive")
        {
          c.out1_mode = "Low";
        }

        rcg::setEnum(nodemap, "LineSelector", "Out1", true);

        if (!rcg::setEnum(nodemap, "LineSource", c.out1_mode.c_str(), false))
        {
          c.out1_mode = "Low";
          NODELET_ERROR("Cannot change out1 mode. Sensor may have no 'iocontrol' license!");
        }
      }

      if (level & 134217728 && c.out2_mode != config.out2_mode)
      {
        if (c.out2_mode != "Low" && c.out2_mode != "High" && c.out2_mode != "ExposureActive" &&
            c.out2_mode != "ExposureAlternateActive")
        {
          c.out2_mode = "Low";
        }

        if (rcg::setEnum(nodemap, "LineSelector", "Out2", false))
        {
          if (!rcg::setEnum(nodemap, "LineSource", c.out2_mode.c_str(), false))
          {
            c.out2_mode = "Low";
            NODELET_ERROR("Cannot change out2 mode. Sensor may have no 'iocontrol' license!");
          }
        }
      }
      if (level & 268435456)
      {
        try
        {
          rcg::setFloat(nodemap, "DepthExposureAdaptTimeout", c.depth_exposure_adapt_timeout, true);
        }
        catch (const std::exception&)
        {
          c.depth_exposure_adapt_timeout = 0.0;
          NODELET_ERROR("Cannot set depth_exposure_adapt_timeout. Please update the sensor to version >= 21.10!");
        }
      }
    }
  }
  catch (const std::exception& ex)
  {
    NODELET_ERROR_STREAM(ex.what());
  }

  config = c;
}

void GenICamDeviceNodelet::updateSubscriptions(bool force)
{
  std::lock_guard<std::recursive_mutex> lock(device_mtx);

  // collect required components and color

  int rcomponents = 0;
  bool rcolor = false;

  for (auto&& p : pub)
  {
    p->requiresComponents(rcomponents, rcolor);
  }

  // Intensity is contained in IntensityCombined

  if (rcomponents & GenICam2RosPublisher::ComponentIntensityCombined)
  {
    rcomponents &= ~GenICam2RosPublisher::ComponentIntensity;
  }

  // enable or disable components as required

  const static struct
  {
    const char* name;
    int flag;
  } comp[] = { { "Intensity", GenICam2RosPublisher::ComponentIntensity },
               { "IntensityCombined", GenICam2RosPublisher::ComponentIntensityCombined },
               { "Disparity", GenICam2RosPublisher::ComponentDisparity },
               { "Confidence", GenICam2RosPublisher::ComponentConfidence },
               { "Error", GenICam2RosPublisher::ComponentError },
               { 0, 0 } };

  for (size_t i = 0; comp[i].name != 0; i++)
  {
    if (((rcomponents ^ scomponents) & comp[i].flag) || force)
    {
      rcg::setEnum(nodemap, "ComponentSelector", comp[i].name, true);
      rcg::setBoolean(nodemap, "ComponentEnable", (rcomponents & comp[i].flag), true);

      const char* status = "disabled";
      if (rcomponents & comp[i].flag)
        status = "enabled";

      if (!force)
      {
        NODELET_INFO_STREAM("Component '" << comp[i].name << "' " << status);
      }
    }
  }

  // enable or disable color

  if (rcolor != scolor || force)
  {
    std::string format = "Mono8";
    if (rcolor)
    {
      format = color_format;
    }

    rcg::setEnum(nodemap, "ComponentSelector", "Intensity", true);
    rcg::setEnum(nodemap, "PixelFormat", format.c_str(), false);
    rcg::setEnum(nodemap, "ComponentSelector", "IntensityCombined", true);
    rcg::setEnum(nodemap, "PixelFormat", format.c_str(), false);
  }

  // store current settings

  scomponents = rcomponents;
  scolor = rcolor;
}

void GenICamDeviceNodelet::subChanged()
{
  updateSubscriptions(false);
}

void GenICamDeviceNodelet::publishConnectionDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.add("connection_loss_total", connection_loss_total);
  stat.add("complete_buffers_total", complete_buffers_total);
  stat.add("incomplete_buffers_total", incomplete_buffers_total);
  stat.add("image_receive_timeouts_total", image_receive_timeouts_total);
  stat.add("current_reconnect_trial", current_reconnect_trial);

  // general connection status

  if (device_serial.empty())
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Disconnected");
    return;
  }

  // at least we are connected to gev server

  stat.add("ip_interface", device_interface);
  stat.add("ip_address", device_ip);
  stat.add("gev_packet_size", gev_packet_size);

  if (scomponents)
  {
    if (streaming)
    {
      // someone subscribed to images, and we actually receive data via GigE vision
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Streaming");
    }
    else
    {
      // someone subscribed to images, but we do not receive any data via GigE vision (yet)
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No data");
    }
  }
  else
  {
    // no one requested images -> node is ok but stale
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Idle");
  }
}

void GenICamDeviceNodelet::publishDeviceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (device_serial.empty())
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Unknown");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Info");
    stat.add("model", device_model);
    stat.add("image_version", device_version);
    stat.add("serial", device_serial);
    stat.add("mac", device_mac);
    stat.add("user_id", device_name);
  }
}

namespace
{
std::vector<std::shared_ptr<rcg::Device> > getSupportedDevices(const std::string& devid,
                                                               const std::vector<std::string>& iname)
{
  std::vector<std::shared_ptr<rcg::System> > system = rcg::System::getSystems();
  std::vector<std::shared_ptr<rcg::Device> > ret;

  for (size_t i = 0; i < system.size(); i++)
  {
    system[i]->open();

    std::vector<std::shared_ptr<rcg::Interface> > interf = system[i]->getInterfaces();

    for (size_t k = 0; k < interf.size(); k++)
    {
      if (interf[k]->getTLType() == "GEV" &&
          (iname.size() == 0 || std::find(iname.begin(), iname.end(), interf[k]->getID()) != iname.end()))
      {
        interf[k]->open();

        std::vector<std::shared_ptr<rcg::Device> > device = interf[k]->getDevices();

        for (size_t j = 0; j < device.size(); j++)
        {
          if ((device[j]->getVendor() == "Roboception GmbH" ||
               device[j]->getModel().substr(0, 9) == "rc_visard" || device[j]->getModel().substr(0, 7) == "rc_cube") &&
              (devid == "*" || device[j]->getID() == devid || device[j]->getSerialNumber() == devid ||
               device[j]->getDisplayName() == devid))
          {
            ret.push_back(device[j]);
          }
        }

        interf[k]->close();
      }
    }

    system[i]->close();
  }

  return ret;
}

class NoDeviceException : public std::invalid_argument
{
public:
  NoDeviceException(const char* msg) : std::invalid_argument(msg)
  {
  }
};

void split(std::vector<std::string>& list, const std::string& s, char delim, bool skip_empty = true)
{
  std::stringstream in(s);
  std::string elem;

  while (getline(in, elem, delim))
  {
    if (!skip_empty || elem.size() > 0)
    {
      list.push_back(elem);
    }
  }
}

}  // namespace

void GenICamDeviceNodelet::grab(std::string id, rcg::Device::ACCESS access)
{
  try
  {
    device_model = "";
    device_version = "";
    device_serial = "";
    device_mac = "";
    device_name = "";
    device_interface = "";
    device_ip = "";
    gev_packet_size = 0;
    current_reconnect_trial = 1;

    NODELET_INFO_STREAM("Grabbing thread started for device '" << id << "'");

    // loop until nodelet is killed

    while (running)
    {
      streaming = false;

      // report standard exceptions and try again

      try
      {
        std::shared_ptr<GenApi::CChunkAdapter> chunkadapter;

        {
          std::lock_guard<std::recursive_mutex> lock(device_mtx);

          // open device and get nodemap

          std::vector<std::string> iname;  // empty
          std::string dname = id;

          {
            size_t i = dname.find(':');
            if (i != std::string::npos)
            {
              if (i > 0)
              {
                iname.push_back(id.substr(0, i));
              }

              dname = dname.substr(i + 1);
            }
          }

          std::vector<std::shared_ptr<rcg::Device> > devices = getSupportedDevices(dname, iname);

          if (devices.size() == 0)
          {
            throw NoDeviceException(("Cannot find device '" + id + "'").c_str());
          }

          if (devices.size() > 1)
          {
            throw std::invalid_argument("Too many devices, please specify unique ID");
          }

          dev = devices[0];
          dev->open(access);
          nodemap = dev->getRemoteNodeMap();

          // ensure that device version >= 20.04

          device_version = rcg::getString(nodemap, "DeviceVersion");

          std::vector<std::string> list;
          split(list, device_version, '.');

          if (list.size() < 3 || std::stoi(list[0]) < 20 || (std::stoi(list[0]) == 20 && std::stoi(list[1]) < 4))
          {
            running = false;
            throw std::invalid_argument("Device version must be 20.04 or higher: " + device_version);
          }

          // check if device is ready

          if (!rcg::getBoolean(nodemap, "RcSystemReady", true, true))
          {
            throw std::invalid_argument("Device is not yet ready");
          }

          // get serial number and IP

          device_interface = dev->getParent()->getID();
          device_serial = dev->getSerialNumber();
          device_mac = rcg::getString(nodemap, "GevMACAddress", false);
          device_name = rcg::getString(nodemap, "DeviceUserID", true);
          device_ip = rcg::getString(nodemap, "GevCurrentIPAddress", false);

          NODELET_INFO_STREAM(""
                          << "Connecting to sensor '" << device_interface << ":" << device_serial << "' alias "
                          << dev->getDisplayName());

          updater.setHardwareID(device_serial);

          // get model type of the device

          device_model = rcg::getString(nodemap, "DeviceModelName");

          // initialise configuration and start dynamic reconfigure server

          if (!reconfig)
          {
            initConfiguration();
            reconfig = new dynamic_reconfigure::Server<rc_genicam_driver::rc_genicam_driverConfig>(
                reconfig_mtx, getPrivateNodeHandle());
          }

          // initialize some values as the old values are checked in the
          // reconfigure callback

          config.depth_smooth = rcg::getBoolean(nodemap, "DepthSmooth", true);

          rcg::setEnum(nodemap, "LineSelector", "Out1", true);
          config.out1_mode = rcg::getEnum(nodemap, "LineSource", true);
          rcg::setEnum(nodemap, "LineSelector", "Out2", false);
          config.out2_mode = rcg::getEnum(nodemap, "LineSource", true);

          config.ptp_enabled = rcg::getBoolean(nodemap, "PtpEnable", false);

          // assign callback each time to trigger resetting all values

          dynamic_reconfigure::Server<rc_genicam_driver::rc_genicam_driverConfig>::CallbackType cb;
          cb = boost::bind(&GenICamDeviceNodelet::reconfigure, this, _1, _2);
          reconfig->setCallback(cb);
        }

        // enable chunk data and multipart

        rcg::setEnum(nodemap, "AcquisitionAlternateFilter", "Off", false);
        rcg::setEnum(nodemap, "AcquisitionMultiPartMode", "SingleComponent", true);
        rcg::setBoolean(nodemap, "ChunkModeActive", true, true);

        // set up chunk adapter

        chunkadapter = rcg::getChunkAdapter(nodemap, dev->getTLType());

        // check for color and iocontrol

        bool color = false;

        {
          std::vector<std::string> formats;
          rcg::setEnum(nodemap, "ComponentSelector", "Intensity", true);
          rcg::getEnum(nodemap, "PixelFormat", formats, true);
          for (auto&& format : formats)
          {
            if (format == "YCbCr411_8")
            {
              color_format = "YCbCr411_8";
              color = true;
              break;
            }

            if (format == "RGB8")
            {
              color_format = "RGB8";
              color = true;
              break;
            }
          }
        }

        bool iocontrol_avail = nodemap->_GetNode("LineSource")->GetAccessMode() == GenApi::RW;

        if (!color)
        {
          NODELET_INFO("Not a color camera. wb_auto, wb_ratio_red and wb_ratio_blue are without "
                   "function.");
        }

        if (nodemap->_GetNode("DepthSmooth")->GetAccessMode() != GenApi::RW)
        {
          NODELET_WARN("No stereo_plus license on device. quality=full and smoothing is not available.");
        }

        if (!iocontrol_avail)
        {
          NODELET_WARN("No iocontrol license on device. out1_mode and out2_mode are without function.");
        }

        // advertise publishers

        ros::NodeHandle nh(getNodeHandle(), "stereo");
        image_transport::ImageTransport it(nh);
        std::function<void()> callback = std::bind(&GenICamDeviceNodelet::subChanged, this);

        pub.clear();
        scomponents = 0;
        scolor = false;

        pub.push_back(std::make_shared<CameraInfoPublisher>(nh, frame_id, true, callback));
        pub.push_back(std::make_shared<CameraInfoPublisher>(nh, frame_id, false, callback));
        pub.push_back(std::make_shared<CameraParamPublisher>(nh, frame_id, true, callback));
        pub.push_back(std::make_shared<CameraParamPublisher>(nh, frame_id, false, callback));

        pub.push_back(std::make_shared<ImagePublisher>(it, frame_id, true, false, iocontrol_avail, callback));
        pub.push_back(std::make_shared<ImagePublisher>(it, frame_id, false, false, iocontrol_avail, callback));

        if (color)
        {
          pub.push_back(std::make_shared<ImagePublisher>(it, frame_id, true, true, iocontrol_avail, callback));
          pub.push_back(std::make_shared<ImagePublisher>(it, frame_id, false, true, iocontrol_avail, callback));
        }

        pub.push_back(std::make_shared<DisparityPublisher>(nh, frame_id, callback));
        pub.push_back(std::make_shared<DisparityColorPublisher>(it, frame_id, callback));
        pub.push_back(std::make_shared<DepthPublisher>(nh, frame_id, callback));

        pub.push_back(std::make_shared<ConfidencePublisher>(nh, frame_id, callback));
        pub.push_back(std::make_shared<ErrorDisparityPublisher>(nh, frame_id, callback));
        pub.push_back(std::make_shared<ErrorDepthPublisher>(nh, frame_id, callback));

        pub.push_back(std::make_shared<Points2Publisher>(nh, frame_id, callback));

        // make nodemap available to publshers

        for (auto&& p : pub)
        {
          p->setNodemap(nodemap);
        }

        // update subscriptions

        updateSubscriptions(true);

        // start streaming

        std::vector<std::shared_ptr<rcg::Stream> > stream = dev->getStreams();

        if (stream.size() == 0)
        {
          throw std::invalid_argument("Device does not offer streams");
        }

        stream[0]->open();
        stream[0]->startStreaming();

        current_reconnect_trial = 1;

        updater.force_update();

        NODELET_INFO_STREAM("Start streaming images");

        // grabbing and publishing

        while (running)
        {
          // grab next buffer

          const rcg::Buffer* buffer = stream[0]->grab(500);
          std::string out1_mode_on_sensor;

          // process buffer

          if (buffer)
          {
            streaming = true;

            if (buffer->getIsIncomplete())
            {
              incomplete_buffers_total++;
              out1_mode_on_sensor = "";
            }
            else
            {
              complete_buffers_total++;

              std::lock_guard<std::recursive_mutex> lock(device_mtx);

              if (gev_packet_size == 0)
              {
                gev_packet_size = rcg::getInteger(nodemap, "GevSCPSPacketSize", 0, 0, false, false);
              }

              // attach buffer to nodemap to access chunk data

              chunkadapter->AttachBuffer(reinterpret_cast<std::uint8_t*>(buffer->getGlobalBase()),
                                         buffer->getSizeFilled());

              // get out1 mode on device, which may have changed

              rcg::setEnum(nodemap, "ChunkLineSelector", "Out1", true);
              out1_mode_on_sensor = rcg::getEnum(nodemap, "ChunkLineSource", true);

              // publish all parts of buffer

              uint32_t npart = buffer->getNumberOfParts();
              for (uint32_t part = 0; part < npart; part++)
              {
                if (buffer->getImagePresent(part))
                {
                  uint64_t pixelformat = buffer->getPixelFormat(part);
                  for (auto&& p : pub)
                  {
                    p->publish(buffer, part, pixelformat);
                  }
                }
              }

              // detach buffer from nodemap

              chunkadapter->DetachBuffer();
            }
          }
          else
          {
            image_receive_timeouts_total++;
            streaming = false;

            // get out1 mode from sensor (this is also used to check if the
            // connection is still valid)

            std::lock_guard<std::recursive_mutex> lock(device_mtx);
            rcg::setEnum(nodemap, "LineSelector", "Out1", true);
            out1_mode_on_sensor = rcg::getString(nodemap, "LineSource", true, true);
          }

          {
            std::lock_guard<std::recursive_mutex> lock(device_mtx);

            // update out1 mode, if it is different to current settings on sensor
            // (which is the only GEV parameter which could have changed outside this code,
            //  i.e. on the rc_visard by the stereomatching module)

            if (out1_mode_on_sensor.size() == 0)
            {
              // use current settings if the value on the sensor cannot be determined
              out1_mode_on_sensor = config.out1_mode;
            }

            if (out1_mode_on_sensor != config.out1_mode)
            {
              config.out1_mode = out1_mode_on_sensor;
              reconfig->updateConfig(config);
            }
          }

          updater.update();
        }

        pub.clear();

        // stop streaming

        stream[0]->stopStreaming();
        stream[0]->close();

        // stop publishing

        device_model = "";
        device_version = "";
        device_serial = "";
        device_mac = "";
        device_name = "";
        device_interface = "";
        device_ip = "";
        gev_packet_size = 0;
        streaming = false;

        updater.force_update();
      }
      catch (const NoDeviceException& ex)
      {
        // report error, wait and retry

        NODELET_WARN_STREAM(ex.what());

        current_reconnect_trial++;
        streaming = false;
        pub.clear();

        updater.force_update();

        sleep(3);
      }
      catch (const std::exception& ex)
      {
        // close everything and report error

        if (device_ip.size() > 0)
        {
          connection_loss_total++;
        }

        device_model = "";
        device_version = "";
        device_serial = "";
        device_mac = "";
        device_name = "";
        device_interface = "";
        device_ip = "";
        gev_packet_size = 0;
        streaming = false;

        current_reconnect_trial++;
        pub.clear();

        NODELET_ERROR_STREAM(ex.what());

        updater.force_update();

        sleep(3);
      }

      // close device

      {
        std::lock_guard<std::recursive_mutex> lock(device_mtx);

        if (dev)
          dev->close();

        dev.reset();
        nodemap.reset();
      }
    }
  }
  catch (const std::exception& ex)
  {
    NODELET_FATAL_STREAM(ex.what());
  }
  catch (...)
  {
    NODELET_FATAL("Unknown exception");
  }

  device_model = "";
  device_version = "";
  device_serial = "";
  device_mac = "";
  device_name = "";
  device_interface = "";
  device_ip = "";
  gev_packet_size = 0;
  streaming = false;
  updater.force_update();

  running = false;
  NODELET_INFO("Grabbing thread stopped");
}

}  // namespace rc

PLUGINLIB_EXPORT_CLASS(rc::GenICamDeviceNodelet, nodelet::Nodelet)
