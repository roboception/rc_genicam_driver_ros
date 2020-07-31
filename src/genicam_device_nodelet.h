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

#ifndef RC_GENICAM_DRIVERNODELET_H
#define RC_GENICAM_DRIVERNODELET_H

#include "publishers/genicam2ros_publisher.h"

#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <rc_genicam_driver/rc_genicam_driverConfig.h>
#include <rc_common_msgs/Trigger.h>

#include <GenApi/GenApi.h>
#include <rc_genicam_api/device.h>

#include <thread>
#include <mutex>
#include <atomic>

#include <diagnostic_updater/diagnostic_updater.h>

namespace rc
{

class GenICamDeviceNodelet : public nodelet::Nodelet
{
  public:

    GenICamDeviceNodelet();
    virtual ~GenICamDeviceNodelet();

    virtual void onInit();

    bool depthAcquisitionTrigger(rc_common_msgs::Trigger::Request& req,
                                 rc_common_msgs::Trigger::Response& resp);

  private:

    void initConfiguration();

    void reconfigure(rc_genicam_driver::rc_genicam_driverConfig &c, uint32_t level);

    void updateSubscriptions(bool force=false);

    void subChanged();

    void publishConnectionDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void publishDeviceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void grab(std::string id, rcg::Device::ACCESS access);

    std::string frame_id;
    ros::ServiceServer trigger_service;

    boost::recursive_mutex reconfig_mtx;
    dynamic_reconfigure::Server<rc_genicam_driver::rc_genicam_driverConfig> *reconfig;
    rc_genicam_driver::rc_genicam_driverConfig config;

    std::recursive_mutex device_mtx;
    std::shared_ptr<rcg::Device> dev;
    std::shared_ptr<GenApi::CNodeMapRef> nodemap;

    int scomponents;
    bool scolor;

    std::thread grab_thread;
    std::atomic_bool running;

    std::vector<std::shared_ptr<GenICam2RosPublisher> > pub;

    std::string device_model;
    std::string device_version;
    std::string device_serial;
    std::string device_mac;
    std::string device_name;
    std::string device_interface;
    std::string device_ip;
    int gev_packet_size;
    int connection_loss_total;
    int complete_buffers_total;
    int incomplete_buffers_total;
    int image_receive_timeouts_total;
    int current_reconnect_trial;
    bool streaming;

    diagnostic_updater::Updater updater;
};

}

#endif
