^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rc_genicam_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2021-08-02)
------------------

For supporting the rc_sgm_producer:

* Adding support for RGB color format

Other fixes:

* Fixed rounding when converting color to monochrome
* Increased tolerance for alternate exposure mode to 1 second
* Fixed warnings during compilation

0.5.2 (2021-05-26)
------------------

* Relaxed some non-critical parameters to be optional

0.5.1 (2021-02-11)
------------------

* Accept devices of vendor Roboception or if model name that starts with rc_visard or rc_cube

0.5.0 (2020-11-20)
------------------

* Publish images without filtering on image_rect(_color) topics

0.4.0 (2020-11-20)
------------------

* Added new parameter depth_double_shot

0.3.0 (2020-11-13)
------------------

* Renaming of adaptive_out1_reduction to out1_reduction
* Added new auto exposure mode Out1High
* Added reporting test flag, out1 reduction and brightness in camera parameter messages
* Update README

0.2.0 (2020-09-23)
------------------

* Removed parameter disprange and controlling disparity range via mindepth and maxdepth
* Changed log output from `ROS_` to `NODELET_`

0.1.0 (2020-07-31)
------------------

* Initial stable release, based on rc_visard_driver
