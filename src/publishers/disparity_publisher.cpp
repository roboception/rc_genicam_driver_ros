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

#include "disparity_publisher.h"

#include <rc_genicam_api/pixel_formats.h>
#include <rc_genicam_api/config.h>

#include <sensor_msgs/image_encodings.h>

namespace rc
{
DisparityPublisher::DisparityPublisher(ros::NodeHandle& nh, const std::string& frame_id,
                                       std::function<void()>& sub_changed)
  : GenICam2RosPublisher(frame_id)
{
  sub_callback = sub_changed;
  pub = nh.advertise<stereo_msgs::DisparityImage>("disparity", 1,
                                                  boost::bind(&GenICam2RosPublisher::subChanged, this, _1),
                                                  boost::bind(&GenICam2RosPublisher::subChanged, this, _1));
}

bool DisparityPublisher::used()
{
  return pub.getNumSubscribers() > 0;
}

void DisparityPublisher::requiresComponents(int& components, bool& color)
{
  if (pub.getNumSubscribers() > 0)
  {
    components |= ComponentDisparity;
  }
}

void DisparityPublisher::publish(const rcg::Buffer* buffer, uint32_t part, uint64_t pixelformat)
{
  if (nodemap && pub.getNumSubscribers() > 0 && pixelformat == Coord3D_C16)
  {
    // allocate new image message and set meta information

    stereo_msgs::DisparityImagePtr p = boost::make_shared<stereo_msgs::DisparityImage>();

    uint64_t time = buffer->getTimestampNS();

    p->header.seq = 0;
    p->header.stamp.sec = time / 1000000000ul;
    p->header.stamp.nsec = time % 1000000000ul;
    p->header.frame_id = frame_id;

    // get some information from the buffer

    rcg::setEnum(nodemap, "ChunkComponentSelector", "Disparity", true);
    double f = rcg::getFloat(nodemap, "ChunkScan3dFocalLength", 0, 0, true);
    double t = rcg::getFloat(nodemap, "ChunkScan3dBaseline", 0, 0, true);
    float scale = rcg::getFloat(nodemap, "ChunkScan3dCoordinateScale", 0, 0, true);
    int disprange = rcg::getInteger(nodemap, "DepthDispRange", 0, 0, true);
    std::string quality = rcg::getString(nodemap, "DepthQuality", true);

    if (quality == "Full")
    {
      disprange *= 2;
    }
    else if (quality == "Medium")
    {
      disprange /= 2;
    }
    else if (quality == "Low")
    {
      disprange /= 3;
    }

    // prepare size and format of outgoing image

    p->image.header = p->header;
    p->image.width = static_cast<uint32_t>(buffer->getWidth(part));
    p->image.height = static_cast<uint32_t>(buffer->getHeight(part));
    p->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    p->image.is_bigendian = rcg::isHostBigEndian();
    p->image.step = p->image.width * sizeof(float);

    size_t px = buffer->getXPadding(part);
    const uint8_t* ps = static_cast<const uint8_t*>(buffer->getBase(part));

    // convert image information

    p->image.data.resize(p->image.step * p->image.height);

    float* pt = reinterpret_cast<float*>(&p->image.data[0]);
    float dmax = 0;

    bool bigendian = buffer->isBigEndian();

    for (uint32_t k = 0; k < p->image.height; k++)
    {
      if (bigendian)
      {
        for (uint32_t i = 0; i < p->image.width; i++)
        {
          uint16_t d = (ps[0] << 8) | ps[1];

          *pt = -1.0f;

          if (d != 0)
          {
            *pt = scale * d;
            dmax = std::max(dmax, *pt);
          }

          ps += 2;
          pt++;
        }
      }
      else
      {
        for (uint32_t i = 0; i < p->image.width; i++)
        {
          uint16_t d = (ps[1] << 8) | ps[0];

          *pt = -1.0f;

          if (d != 0)
          {
            *pt = scale * d;
            dmax = std::max(dmax, *pt);
          }

          ps += 2;
          pt++;
        }
      }

      ps += px;
    }

    p->f = f;
    p->T = t;
    p->valid_window.x_offset = 0;
    p->valid_window.y_offset = 0;
    p->valid_window.width = p->image.width;
    p->valid_window.height = p->image.height;
    p->min_disparity = 0;
    p->max_disparity = std::max(dmax, static_cast<float>(disprange));
    p->delta_d = scale;

    // publish message

    pub.publish(p);
  }
}
}  // namespace rc
