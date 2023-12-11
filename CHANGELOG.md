# Change log for l3cam_ros2

All notable changes to the l3cam_ros2 package will be documented in this file.

## [1.0.0] - 2023-12-11

### Added

### Changed

### Fixed

### Removed

### Known Bugs

### Issues

- When changing a parameter from a configuration node, if the parameter could not be changed, it will be set to its previous value. This might not directly take effect on the `rqt_reconfigure` node and might lead to misunderstandings. Please check for any RCLCPP messages, any parameter that could not be changed will be informed. You can refresh the actual values of a configuration node in `rqt_reconfigure` by hiding and showing again the node parameters.

- The following parameters might not match the real value as they depend on another parameter to be able to be set, and getters for sensors parameters, except allied cameras, are not supported yet.

  - `bias_value_right` depends on `auto_bias`.
  - `bias_value_left` depends on `auto_bias`.
  - `polarimetric_camera_auto_gain_range_minimum` depends on `polarimetric_camera_auto_gain`.
  - `polarimetric_camera_auto_gain_range_maximum` depends on `polarimetric_camera_auto_gain`.
  - `polarimetric_camera_gain` depends on `polarimetric_camera_auto_gain`.
  - `polarimetric_camera_auto_exposure_time_range_minimum` depends on `polarimetric_camera_auto_exposure_time`.
  - `polarimetric_camera_auto_exposure_time_range_maximum` depends on `polarimetric_camera_auto_exposure_time`.
  - `polarimetric_camera_exposure_time` depends on `polarimetric_camera_auto_exposure_time`.
  - `rgb_camera_white_balance` depends on `rgb_camera_auto_white_balance`.
  - `rgb_camera_exposure_time` depends on `rgb_camera_auto_exposure_time`.

- The following parameters will change when the parameter they depend on changes. These changes might not be shown directly in `rqt_reconfigure`, so you might have to hide and show the parameter's node.

  - `allied_wide_camera_exposure_time` changes when `allied_wide_camera_auto_exposure_time` is set to false.
  - `allied_wide_camera_gain` changes when `allied_wide_camera_auto_gain` is set to false.
  - `allied_narrow_camera_exposure_time` changes when `allied_narrow_camera_auto_exposure_time` is set to false.
  - `allied_narrow_camera_gain` changes when `allied_narrow_camera_auto_gain` is set to false.

- Some error codes returned from the library are returned as uint interpreted as int.

- l3cam_ros2_node exits with -11 (SIGSEGV) when exiting if executed from ros2 launch.
