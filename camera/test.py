# first, import all necessary modules
from pathlib import Path

import blobconverter
import cv2
import depthai
import numpy as np


pipeline = depthai.Pipeline()

# First, we want the Color camera as the output
cam_rgb = pipeline.createColorCamera()
cam_rgb.setPreviewSize(300, 300)  # 300x300 will be the preview frame size, available as 'preview' output of the node
cam_rgb.setInterleaved(False)
# setup left and right cameras
cam_left = pipeline.create(depthai.node.MonoCamera)
cam_right = pipeline.create(depthai.node.MonoCamera)
cam_left.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
cam_left.setCamera("left")
cam_right.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
cam_right.setCamera("right")

# processing nodes
detection_nn = pipeline.createMobileNetDetectionNetwork()
# Blob is the Neural Network file, compiled for MyriadX. It contains both the definition and weights of the model
# We're using a blobconverter tool to retreive the MobileNetSSD blob automatically from OpenVINO Model Zoo
detection_nn.setBlobPath(blobconverter.from_zoo(name='mobilenet-ssd', shaves=6))
# Next, we filter out the detections that are below a confidence threshold. Confidence can be anywhere between <0..1>
detection_nn.setConfidenceThreshold(0.5)

# MobilenetSSD label texts

labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

# testing stereo depth
# options:
extended_dispariy = False # closer-in minimum depth
subpixel = False # better longer distance res
lr_check = True # filter out occulsions

depth = pipeline.createStereoDepth()
# depth options
depth.setDefaultProfilePreset(depthai.node.StereoDepth.PresetMode.DEFAULT)
depth.initialConfig.setMedianFilter(depthai.MedianFilter.KERNEL_7x7)
depth.setDepthAlign(depthai.CameraBoardSocket.CAM_A)
depth.setLeftRightCheck(lr_check)
depth.setExtendedDisparity(extended_dispariy)
depth.setSubpixel(subpixel)

#spatial
bbox_scale = 0.5
nn_spacial = pipeline.createMobileNetSpatialDetectionNetwork()
nn_spacial.setBlobPath(blobconverter.from_zoo(name='mobilenet-ssd', shaves=6))
nn_spacial.setConfidenceThreshold(0.5)
nn_spacial.input.setBlocking(False)
nn_spacial.setBoundingBoxScaleFactor(bbox_scale)
nn_spacial.setDepthLowerThreshold(100)
nn_spacial.setDepthUpperThreshold(5000)

imu = pipeline.createIMU()
xout_imu = pipeline.createXLinkOut()
xout_imu.setStreamName("imu")
imu.enableIMUSensor(depthai.IMUSensor.ACCELEROMETER_RAW, 500)
imu.enableIMUSensor(depthai.IMUSensor.GYROSCOPE_RAW, 400)
imu.enableIMUSensor(depthai.IMUSensor.MAGNETOMETER_RAW, 100)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)

# XLinkOut is a "way out" from the device. Any data you want to transfer to host need to be send via XLink
xout_rgb = pipeline.createXLinkOut()
xout_rgb.setStreamName("rgb")

xout_nn = pipeline.createXLinkOut()
xout_nn.setStreamName("nn")

# debug mono
xout_mono_r = pipeline.createXLinkOut()
xout_mono_r.setStreamName("mono_r")
#xout_depth = pipeline.createXLinkOut()
#xout_depth.setStreamName("disparity")

xout_spatial = pipeline.createXLinkOut()
xout_spatial.setStreamName("spatial")

# link cameras to stereo depth
cam_left.out.link(depth.left)
cam_right.out.link(depth.right)
#depth.disparity.link(xout_depth.input)
depth.depth.link(nn_spacial.inputDepth)
# link others
cam_right.out.link(xout_mono_r.input)
cam_rgb.preview.link(xout_rgb.input)
cam_rgb.preview.link(detection_nn.input)
cam_rgb.preview.link(nn_spacial.input)
detection_nn.out.link(xout_nn.input)
nn_spacial.out.link(xout_spatial.input)
imu.out.link(xout_imu.input)

# Pipeline is now finished, and we need to find an available device to run our pipeline
# we are using context manager here that will dispose the device after we stop using it
with depthai.Device(pipeline) as device:
    # From this point, the Device will be in "running" mode and will start sending data via XLink

    # To consume the device results, we get two output queues from the device, with stream names we assigned earlier
    device.setIrLaserDotProjectorIntensity(0.5) # enable IR dot projector (visible with mono camera)
    device.setIrFloodLightIntensity(0.0)
    q_rgb = device.getOutputQueue("rgb")
    q_mono_r = device.getOutputQueue("mono_r")
    q_nn = device.getOutputQueue("nn")
    #q_depth = device.getOutputQueue("disparity", maxSize=4, blocking=False)
    q_spatial = device.getOutputQueue("spatial", maxSize=4, blocking=False)
    q_imu = device.getOutputQueue("imu", maxSize=50, blocking=False)

    # Here, some of the default values are defined. Frame will be an image from "rgb" stream, detections will contain nn results
    frame = None
    #frame_depth = None
    detections = []
    det_dists = []
    # temp while testing depth
    scale = 720

    # Since the detections returned by nn have values from <0..1> range, they need to be multiplied by frame width/height to
    # receive the actual position of the bounding box on the image
    def frameNorm(frame, bbox,scale=None):
        if scale is not None:
            bbox = bbox*scale #TODO: fix
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    def detCenter(frame,det):
        center_x = det.xmin + (det.xmax - det.xmin)/2
        center_y = det.ymin + (det.ymax - det.ymin)/2
        #normalize
        center_x = np.clip(center_x * frame.shape[0]).astype(int)
        center_y = np.clip(center_y * frame.shape[1]).astype(int)
        return center_x,center_y
    
    imuf = "{:.06f}"


    while True:
        # we try to fetch the data from nn/rgb queues. tryGet will return either the data packet or None if there isn't any
        in_rgb = q_rgb.tryGet()
        in_nn = q_nn.tryGet()
        #in_disparity = q_depth.tryGet() # blocking call
        in_spacial = q_spatial.tryGet()
        in_mono_r = q_mono_r.tryGet()
        in_imu = q_imu.tryGet()

        if in_mono_r is not None:
            # If the packet from RGB camera is present, we're retrieving the frame in OpenCV format using getCvFrame
            frame_r = in_mono_r.getCvFrame()
            cv2.imshow("Right", frame_r)

        if in_rgb is not None:
            # If the packet from RGB camera is present, we're retrieving the frame in OpenCV format using getCvFrame
            frame = in_rgb.getCvFrame()

        #if in_disparity is not None:
        #    frame_depth = in_disparity.getCvFrame()

        if in_nn is not None:
            # when data from nn is received, we take the detections array that contains mobilenet-ssd results
            detections = in_nn.detections

        if in_spacial is not None:
            # If the packet from RGB camera is present, we're retrieving the frame in OpenCV format using getCvFrame
            det_dists = in_spacial.detections

        if frame is not None:
            for detection in det_dists:
                # for each bounding box, we first normalize it to match the frame size
                bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
                # and then draw a rectangle on the frame to show the actual result
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), 2)
                cv2.putText(frame,labelMap[detection.label],(bbox[0],bbox[1]+40), cv2.FONT_HERSHEY_TRIPLEX,0.5,(255,255,255))
                #center and get distance from map
                c_x,c_y = detCenter(frame,detection)
                cv2.circle(frame,(c_x,c_y),radius=1,color=(0,0,255))
                cv2.putText(frame,("z [mm]: " + str(detection.spatialCoordinates.z)),(bbox[0],bbox[1]+20), cv2.FONT_HERSHEY_TRIPLEX,0.5,(0,255,0))
                cv2.putText(frame,("confidence: " + str(detection.confidence*100) + "%"),(bbox[0],bbox[1]+60), cv2.FONT_HERSHEY_TRIPLEX,0.5,(0,255,0))
            # After all the drawing is finished, we show the frame on the screen
            cv2.imshow("preview", frame)

        if in_imu is not None:
            imuPackets = in_imu.packets
            for imuPacket in imuPackets:
                acceleroValues = imuPacket.acceleroMeter
                gyroValues = imuPacket.gyroscope
                magnetometerValues = imuPacket.magneticField

                print("Accelerometer x:" + imuf.format(acceleroValues.x) + " y:" + imuf.format(acceleroValues.y) + " z:" + imuf.format(acceleroValues.z))
                print("Gyroscope x:" + imuf.format(gyroValues.x) + " y:" + imuf.format(gyroValues.y) + " z:" + imuf.format(gyroValues.z))
                print("Magnetometer x:" + imuf.format(magnetometerValues.x) + " y:" + imuf.format(magnetometerValues.y) + " z:" + imuf.format(magnetometerValues.z))
                print('\033[4A') # move cursor up 3 lines (4-1 due to newline)

        #if frame_depth is not None:
        #    #normalize
        #    frame_depth = (frame_depth * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)
        #    # apply colormap
        #    frame_depth = cv2.applyColorMap(frame_depth, cv2.COLORMAP_JET)
        #    # draw circles for detected center dot
        #    for detection in det_dists:
        #        bbox = frameNorm(frame_depth, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
        #        cv2.rectangle(frame_depth, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), 2)
        #        cv2.putText(frame_depth,labelMap[detection.label],(bbox[0],bbox[1]+40), cv2.FONT_HERSHEY_TRIPLEX,0.5,(255,255,255))
        #        #center and get distance from map
        #        c_x,c_y = detCenter(frame_depth,detection)
        #        cv2.circle(frame_depth,(c_x,c_y),radius=1,color=(0,0,255))
        #        cv2.putText(frame_depth,("z [mm]: " + str(detection.spatialCoordinates.z)),(bbox[0],bbox[1]+20), cv2.FONT_HERSHEY_TRIPLEX,0.5,(255,255,255))
        #    cv2.imshow("disparity", frame_depth)

        # at any time, you can press "q" and exit the main loop, therefore exiting the program itself
        if cv2.waitKey(1) == ord('q'):
            break


