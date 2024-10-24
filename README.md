
rc_genicam_driver
-----------------

Nodelet/node providing a ROS interface to configure a [Roboception](https://roboception.com)
[rc_visard](https://roboception.com/rc_visard)
or [rc_cube](https://roboception.com/product/rc_cube-s/) and receive images.

Please also consult the manuals for more details:

* https://doc.rc-visard.com
* https://doc.rc-cube.com

Installation
------------

On Debian/Ubuntu add the ROS sources and

```bash
sudo apt-get install ros-${ROS_DISTRO}-rc-genicam-driver
```

### From Source

This rc_genicam_driver depends on

* [rc_genicam_api](https://github.com/roboception/rc_genicam_api)

The dependencies can also be installed via rosdep.

```
rosdep install --from-paths rc_genicam_driver --ignore-src rc_genicam_driver -r -y
```

Building and installing the package follows the typical ROS catkin workflow.

As an alternative, the cmake build-flow would be something like

```bash
mkdir build && cd build
cmake -DCATKIN_BUILD_BINARY_PACKAGE="1" -DCMAKE_INSTALL_PREFIX="/opt/ros/$ROS_DISTRO" -DCMAKE_PREFIX_PATH="/opt/ros/$ROS_DISTRO" -DCMAKE_BUILD_TYPE=Release ..
make
make install
```

Alternatively, instead of the final `make install`, you can also use
`make package` and `sudo dpkg -i install ros-melodic-rc-genicam-driver_*.deb`.

### GenICam GenTL Transport Layer

The rc_genicam_driver uses [rc_genicam_api](https://github.com/roboception/rc_genicam_api)
for interfacing with the rc_visard or rc_cube via GenICam/GigE Vision and requires a
transport layer called a GenTL producer (shared library with the suffix `.cti`).
For convenience rc_genicam_api comes with producers from Baumer for common
architectures.

The path to the producer can be set with the `GENICAM_GENTL64_PATH`
environment variable (or `GENICAM_GENTL32_PATH` for 32 bit systems).
If not set, rc_genicam_driver will fall back to searching for the Baumer
producer where rc_genicam_api is installed.

If the producer .cti can't be found and you will get an error message like
```
[ERROR] [1512568083.512790905]: No transport layers found in path /opt/ros/melodic/lib/rc_genicam_api
```

In this case you need either need to actually install rc_genicam_api properly or
set the environment variable when running it. E.g. export:

```bash
GENICAM_GENTL64_PATH=/path/to/rc_genicam_api/baumer/Ubuntu-14.04/x86_64
```

Configuration
-------------

#### Parameters

Parameters to be set to the ROS param server before run-time.

* `device`: The ID of the device, e.g. Roboception rc_visard sensor or rc_cube.
  This can be either the

  * serial number, e.g. `02912345`

    IMPORTANT: preceed with a colon (`:02912345`) when passing this on the commandline,
    setting it via rosparam or in the launch file (see https://github.com/ros/ros_comm/issues/1339).

  * user defined name (factory default is the name of the rc_visard's model), must be unique among all
    reachable sensors

  * internal ID, which is generated by the used GenTL producer. Often, this ID
    contains the MAC address in some way. This ID can change with the
    implementation of the transport layer.

  See https://github.com/roboception/rc_genicam_api#device-id for more details.
  By default this parameter is set to `*`, which works with if only one
  compatible device can be found on the network.

* `gev_access`:  The gev_access mode, i.e.:
  * 'control'   Configuration and streaming with the possibility of other
                clients to read GenICam parameters. This is the default.
  * 'exclusive' Exclusive access to the sensor. This prevents other clients to
                read GenICam parameters.

#### Dynamic-reconfigure Parameters

These parameters can be changed during runtime via dynamic reconfigure:

* `ptp_enabled`: Enable PTP slave (PrecisionTimeProtocol, IEEE1588)

* `camera_fps`: Frames per second that are published by this nodelet.
  Publishing frames will be slowed down depending on this setting. Setting
  it higher than the real framerate of the specific device has no effect.

* `camera_exp_auto`: This parameter has been removed. Please use
  camera_exp_control instead.

* `camera_exp_control`: Expose control mode which can be "Manual" for setting
  exposure time and gain via camera_exp_value and camera_gain_value, "Auto"
  for auto exposure or "HDR" for high dynamic range mode. Default: Auto.

* `camera_exp_auto_mode`
  Auto-exposure mode which can be "Normal", "Out1High" or "AdaptiveOut1".
  Default: Normal.

* `camera_exp_max`: Maximum exposure time in seconds if exp_auto is true.

* `camera_exp_auto_average_max`: The auto exposure tries to set the exposure
  time and gain factor such that the average image intensity is between an
  upper and a lower bound. This parameter defines the upper bound. It can be
  reached if there is no saturation (e.g. due to reflections).

* `camera_exp_auto_average_min`: See `camera_exp_auto_average_max`. This
  parameter defines the lower bound. The average image intensity can be
  reduced to this value to reduce or avoid saturation (e.g. due to
  reflections).

* `camera_exp_value`: Exposure time in seconds if exp_auto is false.

* `camera_gain_value`: Gain factor in decibel if exp_auto is false.

* `camera_gamma`: Gamma factor. Default: 1.0.

* Auto exposure region: Definition of a region in the left image,
  if the region has zero size or is outside the image,
  then the full left and right image is used to determine the auto exposure.
  * `camera_exp_width`: Width of auto exposure region. 0 for whole image.
  * `camera_exp_height`: Height of auto exposure region. 0 for whole image.
  * `camera_exp_offset_x`: First column of auto exposure region
  * `camera_exp_offset_y`: First row of auto exposure region

* `depth_acquisition_mode`: Can be either `SingleFrame` or `Continuous`. Only
  the first letter will be checked, thus giving `S` or `C` is sufficient.

* `depth_quality`: Quality can be "Low", "Medium", "High" and "Full". Only the first
  letter will be checked, thus specification of "L", "M", "H" or "F" is
  sufficient. The quality setting effectively downscales the image after
  the downscale factor as given above:

  + Full does not downscale the image, i.e. factor is 1 (e.g. 1280x960).
    NOTE: This mode requires the 'stereo_plus' license on the rc_visard.
  + High downscales by factor 2 (e.g. 640x480).
  + Medium downscales by factor 4 (e.g. 320x240).
  + Low downscales by factor 6 (e.g. 214x160).

* `depth_static_scene`: This parameter can be set to true if the scene and
  camera is static. It only has an effect if quality is either High or Full.
  If active, input images are accumulated and averaged for 300 ms to reduce
  noise. This limits the frame rate to a maximum of 3 Hz. The timestamp of
  the disparity image is taken from the first image that was used for
  accumulation.

* `depth_fill`: Higher numbers fill gaps with measurments with potentielly
  higher errors.

* `depth_seg`: Maximum size of isolated disparity regions that will be
  invalidated, related to full resolution.

* `depth_smooth`: Switching smoothing of disparities on or off.
  NOTE: Smoothing requires the 'stereo_plus' license on the rc_visard.

* `depth_minconf`: Minimal confidence. All disparities with lower confidence
  will be set to invalid.

* `depth_mindepth`: Minimum depth in meter. All disparities with lower depth
  will be set to invalid.

* `depth_maxdepth`: Maximum depth in meter. All disparities with higher depth
  will be set to invalid.

* `depth_maxdeptherr`: Maximum depth error in meter. All disparities with a
  higher depth error will be set to invalid.

* `depth_exposure_adapt_timeout`: Maximum time in seconds to wait after triggering
  in SingleFrame modes until auto exposure has finished adjustments.

* `out1_mode`: Mode for the digital GPIO out1. Possible values are:

  * `Low` for switching out1 permanently off.
  * `High` for switching out1 permanently on.
  * `ExposureActive` for switching out1 on for the exposure time of every
    image.
  * `ExposureAlternateActive` for switching out1 on for the exposure time of
    every second image.

  The value can only be changed if the rc_visard has an `IO Control` license.
  The default is `Low`.

* `out2_mode`: Mode for the digital GPIO out2. The functionality is the same
  as for `out1_mode`. The default is `Low`.

For color sensors, the following dynamic-reconfigure parameters are additionally
available:

* `camera_wb_auto`: If true, then white balancing is done automatically. If
  false, then the red and blue to green ratios can be chosen manually.

* `camera_wb_ratio_red`: Red to green ratio for color balancing if
  `camera_wb_auto` is false.

* `camera_wb_ratio_blue`: Blue to green ratio for color balancing if
  `camera_wb_auto` is false.

Provided Topics
---------------

The following topics are provided. The nodelet tries to request only
data (e.g., images, poses) from the sensor if there is subscriber
to the corresponding topic.

#### Images, Stereo Data, Point Clouds

* /stereo/left/camera_info (sensor_msgs::CameraInfo)
* /stereo/right/camera_info (sensor_msgs::CameraInfo)
* /stereo/left/camera_param (rc_common_msgs::CameraParam)
* /stereo/right/camera_param (rc_common_msgs::CameraParam)
* /stereo/left/image_rect (sensor_msgs::Image, MONO8)
* /stereo/right/image_rect (sensor_msgs::Image, MONO8)
* /stereo/disparity (stereo_msgs::DisparityImage)
* /stereo/disparity_color (sensor_msgs::Image, RGB8, visually pleasing)
* /stereo/depth (sensor_msgs::Image, TYPE_32FC1)
* /stereo/confidence (sensor_msgs::Image, TYPE_32FC1, values between 0 and 1)
* /stereo/error_disparity (sensor_msgs::Image, TYPE_32FC1)
* /stereo/error_depth (sensor_msgs::Image, TYPE_32FC1)
* /stereo/points2 (sensor_msgs::PointCloud2)

The proprietary CameraParam messages are sent for every image and contain
information like the exposure time, gain and values of digital inputs and
outputs at the time of image capture.

For color sensors, the following topics are additionally available:

* /stereo/left/image_rect_color (sensor_msgs::Image, format: RGB8)
* /stereo/right/image_rect_color (sensor_msgs::Image, format: RGB8)

If the connected rc_visard has an `IO Control` license, then the following
topics are additionally provided for images where the GPIO out1 is either low
or high. These topics only useful if `out1_mode` is set to the special mode
`ExposureAlternateActive`.

* /stereo/left/image_rect_out1_low (sensor_msgs::Image, MONO8)
* /stereo/left/image_rect_out1_high (sensor_msgs::Image, MONO8)
* /stereo/right/image_rect_out1_low (sensor_msgs::Image, MONO8)
* /stereo/right/image_rect_out1_high (sensor_msgs::Image, MONO8)

For color sensors with an `IO Control` license, the following topics are
additionally available:

* /stereo/left/image_rect_color_out1_low (sensor_msgs::Image, format: RGB8)
* /stereo/left/image_rect_color_out1_high (sensor_msgs::Image, format: RGB8)
* /stereo/right/image_rect_color_out1_low (sensor_msgs::Image, format: RGB8)
* /stereo/right/image_rect_color_out1_high (sensor_msgs::Image, format: RGB8)

#### Running multiple rc_visard's in one ros environment

For operating multiple rc_visard's in one ros environment, each ros node must
be started in separate namespaces, e.g., `my_visard`. As a result, all
frame_ids in all ros messages will be prefixed, e.g., to `my_visard_world` or
`my_visard_camera`.

Services
--------

The following service is offered to trigger stereo matching in `SingleFrame`
mode. It returns an error if the `depth_acquisition_mode` is `Continuous`.

* `depth_acquisition_trigger`

Diagnostics
-----------

The rc_genicam_driver uses the `diagnostics_updater` class from the
[ROS diagnostics stack](https://wiki.ros.org/diagnostics) to regularly publish a
[DiagnosticStatus Message](http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html).

The regular publishing rate can be set via the `~diagnostic_period` parameter and defaults to 1 second.

Currently two status are published:

- `Device`: Information about the device that the driver is connected to. It covers
  the device serial number, mac address, user-defined GeV ID, and the firmware image
  version.

- `Connection`: Status of the current connection between rc_genicam_driver and device.
It publishes 4 different messages:

  - `Disconnected` (Error): The driver is currently not (yet) connected to the sensor and
  might try to reconnect several times according to the `max_reconnects` parameter.
  - `Idle` (Ok): The driver is connected but not publishing any data because
  no one is subscribed to any.
  - `No data` (Warning): The driver is connected and required to publish data but
  itself does not receive any data from the sensor.
  - `Streaming` (Ok): The driver is connected and properly streaming data.

  The published status values are `connection_loss_total`,
  `incomplete_buffers_total`, `image_receive_timeouts_total`, and
  `current_reconnect_trial`. If not `Disconnected`, additionally the current
  `ip_address` and `gev_packet_size` are published.

Launching
---------

IMPORTANT: Prepending numerical serial numbers with ':' is important. Otherwise,
the parameter will be ignored (see https://github.com/ros/ros_comm/issues/1339).

* Using command line parameters:
  ```bash
  rosrun rc_genicam_driver rc_genicam_driver _device:=:02912345
  ```
* As a nodelet, and in a separate **namespace**:
  ```bash
  ROS_NAMESPACE=my_visard rosrun nodelet nodelet standalone rc_genicam_driver _device:=:02912345
  ```
  Note that in this setup all frame_ids in all ros messages will be prefixed
  with `my_visard`, e.g., the frame_id of the published camera images will
  be `my_visard_camera`.
