
#!/usr/bin/env python3
from pathlib import Path
import depthai as dai

# Sets the path to the .blob file
blob_file_path = str((Path(__file__).parent / Path('best_aug_openvino_2022.1_6shave.blob')).resolve().absolute())

# Create pipeline
pipeline = dai.Pipeline()

# Define nodes
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
imu = pipeline.create(dai.node.IMU)
script = pipeline.create(dai.node.Script)

# Properties
camRgb.setPreviewSize(416, 416)  # Must match the trained model
camRgb.setPreviewKeepAspectRatio(False)  # Force stretching (no letterbox)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(15)  # Maintain 12 FPS for camera input, but send at 50 Hz

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)

stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

spatialDetectionNetwork.setBlobPath(blob_file_path)
spatialDetectionNetwork.setConfidenceThreshold(0.8)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(4000) #ignore 4m+
spatialDetectionNetwork.setNumClasses(2)
spatialDetectionNetwork.setCoordinateSize(4)
spatialDetectionNetwork.setAnchors([10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326])
spatialDetectionNetwork.setAnchorMasks({"side8": [0, 1, 2], "side16": [3, 4, 5], "side32": [6, 7, 8]})
spatialDetectionNetwork.setIouThreshold(0.8)
spatialDetectionNetwork.setNumInferenceThreads(2)  # Keep at 2 for balanced performance

#50hz for antiswing 100hz for pl shis
imu.enableIMUSensor(dai.IMUSensor.ARVR_STABILIZED_GAME_ROTATION_VECTOR, 100)
#imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)
imu.setBatchReportThreshold(1)  # Keep batch threshold low to minimize buffering
imu.setMaxBatchReports(5)  # Increase slightly to handle bursts, but keep low

script.setProcessor(dai.ProcessorType.LEON_CSS)

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.depth.link(spatialDetectionNetwork.inputDepth)
camRgb.preview.link(spatialDetectionNetwork.input)
spatialDetectionNetwork.out.link(script.inputs['detection'])
imu.out.link(script.inputs['imu'])

script.inputs['detection'].setBlocking(False)
script.inputs['imu'].setBlocking(False)
script.inputs['detection'].setQueueSize(2)  # Increase slightly to prevent data loss
script.inputs['imu'].setQueueSize(1)  # Increase slightly to handle 500 Hz bursts

script.setScript("""
import time
import socket
import fcntl
import struct
import lpb

# Define constants
label_map = ["pl", "zone"]
packet_format = "<hhhhhhffff"  # Struct format: 6 shorts (pl: x y z) (zone: x y z), 4 floats (imu: quaternion)
start_time_det = time.time()
start_time_udp = time.time()
#PORT = 5007 # base camera
PORT = 5008 # top camera
pl_position = [32767, 32767, 32767]  # Default "not detected"
zone_position = [32767, 32767, 32767]
imu_rotation = [0.0, 0.0, 0.0, 0.0]
float32_max = 3.4028235e+38
float32_min = -3.4028235e+38

#hz_det = 15
#hz_udp = 100

hz_det = 200
hz_udp = 200

# Multicast settings
MULTICAST_GROUP = "192.168.80.15"
TTL = 1  # Time-to-live, keep it local

while True:
    current_time = time.time()
    if current_time - start_time_udp > 1/hz_udp:
        imuData = node.io['imu'].tryGet()
        if imuData:
            imuPackets = imuData.packets
            # Take the most recent IMU rotation to avoid processing all packets
            latest_rotation = None
            for imuPacket in imuPackets:
                rVvalues = imuPacket.rotationVector
                latest_rotation = [rVvalues.i, rVvalues.j, rVvalues.k, rVvalues.real]
            if latest_rotation:
                imu_rotation = latest_rotation
            message = struct.pack(packet_format, *pl_position, *zone_position, *imu_rotation)
            try:
                udpServer.sendto(message, (MULTICAST_GROUP, PORT))
            except Exception as e:
                #node.warn(f"Error in send to udp loop: {e}. Restarting udp server.")
                try:
                    udpServer.close()
                except:
                    pass
                time.sleep(0.01)  # Minimal delay to prevent overwhelming logs/network
                udpServer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)     
        start_time_udp = time.time()

    # Non-blocking detection data retrieval (slower, runs on another module)

    if current_time - start_time_det > 1/hz_det:
        inDet = node.io['detection'].tryGet()
        if inDet:
            detections = inDet.detections
            detected_labels = {label_map[detection.label] for detection in detections}

            # Reset positions if objects not detected
            if "pl" not in detected_labels:
                pl_position =   [32767, 32767, 32767]
            if "zone" not in detected_labels:
                zone_position = [32767, 32767, 32767]
            if not detections:
                pl_position =   [32767, 32767, 32767]
                zone_position = [32767, 32767, 32767]

            # Process detections
            for detection in detections:
                try:
                    label = label_map[detection.label]
                except:
                    label = detection.label
                if label == "pl":
                    pl_position = [int(detection.spatialCoordinates.x), 
                                   int(-detection.spatialCoordinates.y), # convert from left to right coordinate system
                                   int(detection.spatialCoordinates.z)]
        else:
            pass
        start_time_det = time.time()

    # Ensure data stays within limits
    pl_position = tuple(max(min(x, 32767), -32768) for x in pl_position)
    zone_position = tuple(max(min(x, 32767), -32768) for x in zone_position)
    imu_rotation = tuple(max(min(float(x), float32_max), float32_min) for x in imu_rotation)
    time.sleep(0.004)
""")

# Flash the pipeline
#mxid = "14442C10515CF0D600" # MXID of base camera
mxid = "14442C101102F4D600" # MXID of top camera
device_info = dai.DeviceInfo(mxid)
bootloader = dai.DeviceBootloader(devInfo=device_info)
#if not f:
    #print('No devices found, exiting...')
    #exit(-1)
#bootloader = dai.DeviceBootloader(bl)
progress = lambda p: print(f'Flashing progress: {p * 100:.1f}%\033[1A')
bootloader.flash(progress, pipeline)

# Connect camera to host (optional, for debugging)
#device = dai.Device(pipeline, dai.DeviceInfo("192.168.0.199"))

#while True:
#    pass  # Keep script running for testing