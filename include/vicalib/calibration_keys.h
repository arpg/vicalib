#pragma once

#define CALIBRATION_VERSION 10

#define KEY_VERSION "calibration_version"
#define KEY_DEVICE_TYPE "device_type"
#define KEY_DEVICE_ID "device_id"
#define KEY_CAMERAS "cameras"
#define KEY_DEPTHS "depths"
#define KEY_IMUS "imus"

// applies to cameras, depths, imus
#define KEY_SENSOR_ID "id"
#define KEY_SENSOR_NAME "name"
#define KEY_EXTRINSICS "extrinsics"
#define KEY_EXTRINSICS_T "T"
#define KEY_EXTRINSICS_W "W"
#define KEY_EXTRINSICS_T_VARIANCE "T_variance"
#define KEY_EXTRINSICS_W_VARIANCE "W_variance"

// applies to imus
#define KEY_IMU_ACCELEROMETER "accelerometer"
#define KEY_IMU_GYROSCOPE "gyroscope"
#define KEY_IMU_SCALE_AND_ALIGNMENT "scale_and_alignment"
#define KEY_IMU_BIAS "bias"
#define KEY_IMU_BIAS_VARIANCE "bias_variance"
#define KEY_IMU_NOISE_VARIANCE "noise_variance"
#define KEY_IMU_DECIMATE_BY "decimate_by"

// applies to cameras, depths
#define KEY_CAMERA_IMAGE_SIZE "size_px"
#define KEY_CAMERA_CENTER "center_px"
#define KEY_CAMERA_FOCAL_LENGTH "focal_length_px"
#define KEY_CAMERA_DISTORTION "distortion"
#define KEY_CAMERA_DISTORTION_TYPE "type"
#define KEY_CAMERA_DISTORTION_W "w"
#define KEY_CAMERA_DISTORTION_K "k"
