import depthai as dai

# Don't need to create a pipeline for extracting camera info

# Extract calibration interface to read camera extrinsics/intrinsics
calib = dai.CalibrationHandler()

# Get the extrinsics between IMU and RGB camera
imu_T_rgb = calib.getCameraToImuExtrinsics(dai.CameraBoardSocket.CAM_A)
# imu_T_rgb = calib.getCameraExtrinsics(dai.CameraBoardSocket.CAM_A, dai.CameraBoardSocket.CAM_C)
# imu_T_rgb = calib.getCameraTranslationVector(dai.CameraBoardSocket.CAM_A)

# Display results
print(imu_T_rgb)  # 4x4 numpy array