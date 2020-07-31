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

#include "disparity_color_publisher.h"

#include <rc_genicam_api/pixel_formats.h>
#include <rc_genicam_api/config.h>

#include <sensor_msgs/image_encodings.h>

namespace rc
{
DisparityColorPublisher::DisparityColorPublisher(image_transport::ImageTransport& it, const std::string& frame_id,
                                                 std::function<void()>& sub_changed)
  : GenICam2RosPublisher(frame_id)
{
  sub_callback = sub_changed;
  pub = it.advertise("disparity_color", 1, boost::bind(&GenICam2RosPublisher::subChangedIt, this, _1),
                     boost::bind(&GenICam2RosPublisher::subChangedIt, this, _1));
}

bool DisparityColorPublisher::used()
{
  return pub.getNumSubscribers() > 0;
}

void DisparityColorPublisher::requiresComponents(int& components, bool& color)
{
  if (pub.getNumSubscribers() > 0)
  {
    components |= ComponentDisparity;
  }
}

void DisparityColorPublisher::publish(const rcg::Buffer* buffer, uint32_t part, uint64_t pixelformat)
{
  if (nodemap && pub.getNumSubscribers() > 0 && pixelformat == Coord3D_C16)
  {
    // create image and initialize header

    sensor_msgs::ImagePtr im = boost::make_shared<sensor_msgs::Image>();

    uint64_t time = buffer->getTimestampNS();

    im->header.seq = 0;
    im->header.stamp.sec = time / 1000000000ul;
    im->header.stamp.nsec = time % 1000000000ul;
    im->header.frame_id = frame_id;

    // set image size

    im->width = static_cast<uint32_t>(buffer->getWidth(part));
    im->height = static_cast<uint32_t>(buffer->getHeight(part));
    im->is_bigendian = rcg::isHostBigEndian();

    // get pointer to image data in buffer

    size_t px = buffer->getXPadding(part);
    const uint8_t* ps = static_cast<const uint8_t*>(buffer->getBase(part));

    // ensure that current disprange setting is sufficient

    bool bigendian = buffer->isBigEndian();

    rcg::setEnum(nodemap, "ComponentSelector", "Disparity", true);
    rcg::setEnum(nodemap, "ChunkComponentSelector", "Disparity", true);

    int drange = rcg::getInteger(nodemap, "DepthDispRange", 0, 0, true);
    std::string quality = rcg::getString(nodemap, "DepthQuality", true);

    if (quality == "Full")
    {
      drange *= 2;
    }
    else if (quality == "Medium")
    {
      drange /= 2;
    }
    else if (quality == "Low")
    {
      drange /= 3;
    }

    float scale = rcg::getFloat(nodemap, "ChunkScan3dCoordinateScale", 0, 0, true);

    // convert image data

    im->encoding = sensor_msgs::image_encodings::RGB8;
    im->step = 3 * im->width * sizeof(uint8_t);

    im->data.resize(im->step * im->height);
    uint8_t* pt = reinterpret_cast<uint8_t*>(&im->data[0]);

    for (uint32_t k = 0; k < im->height; k++)
    {
      for (uint32_t i = 0; i < im->width; i++)
      {
        uint16_t d;

        if (bigendian)
        {
          d = (ps[0] << 8) | ps[1];
        }
        else
        {
          d = (ps[1] << 8) | ps[0];
        }

        ps += 2;

        if (d != 0)
        {
          double v = scale * d / drange;
          v = v / 1.15 + 0.1;

          double r = std::max(0.0, std::min(1.0, (1.5 - 4 * fabs(v - 0.75))));
          double g = std::max(0.0, std::min(1.0, (1.5 - 4 * fabs(v - 0.5))));
          double b = std::max(0.0, std::min(1.0, (1.5 - 4 * fabs(v - 0.25))));

          *pt++ = 255 * r + 0.5;
          *pt++ = 255 * g + 0.5;
          *pt++ = 255 * b + 0.5;
        }
        else
        {
          *pt++ = 0;
          *pt++ = 0;
          *pt++ = 0;
        }
      }

      ps += px;
    }

    // publish message

    pub.publish(im);
  }
}

}  // namespace rc
