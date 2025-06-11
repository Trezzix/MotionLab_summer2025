from pathlib import Path

import cv2
import math
import depthai as dai
import os
import contextlib
from datetime import timedelta

fps = 30
cwd = Path.cwd()
mindevices = 2 # set if connecting to both at once is annoying

def create_pipeline(cam_list):
    pipeline = dai.Pipeline()
    cam = {}
    xout = {}
    for c in cam_list:
        xout[c] = pipeline.create(dai.node.XLinkOut)
        xout[c].setStreamName(c)
        if c == 'rgb':
            cam[c] = pipeline.create(dai.node.ColorCamera)
            cam[c].setPreviewSize(416,416)
            cam[c].preview.link(xout[c].input)
        cam[c].setFps(fps)
    return pipeline

# https://docs.python.org/3/library/contextlib.html#contextlib.ExitStack
with contextlib.ExitStack() as stack:
    device_infos = dai.Device.getAllAvailableDevices()

    if len(device_infos) == 0: raise RuntimeError("No devices found!")
    if len(device_infos) < mindevices: raise RuntimeError("Did not connect to all devices")
    else: print("Found", len(device_infos), "devices")
    queues = []


    for device_info in device_infos:
        # Note: the pipeline isn't set here, as we don't know yet what device it is.
        # The extra arguments passed are required by the existing overload variants
        openvino_version = dai.OpenVINO.Version.VERSION_2021_4
        usb2_mode = False
        device = stack.enter_context(dai.Device(openvino_version, device_info, usb2_mode))

        cam_list = {'rgb'} # only interested in rgb camera
        # make folders for storing images
        newdir = os.path.join(cwd, device_info.getMxId())
        os.makedirs(newdir, exist_ok=True)

        # Get a customized pipeline based on identified device type
        device.startPipeline(create_pipeline(cam_list))

        # Output queue will be used to get the rgb frames from the output defined above
        for cam in cam_list:
            queues.append({
                'queue': device.getOutputQueue(name=cam, maxSize=4, blocking=False),
                'msgs': [], # Frame msgs
                'mx': device.getMxId(),
                'cam': cam,
                'cnt' : len(os.listdir(newdir)) # for unique folder filename 
            })
        # enable same lighting conditions as expected in NN
        device.setIrLaserDotProjectorIntensity(0.5)
        device.setIrFloodLightIntensity(0.0)

    def check_sync(queues, timestamp):
        matching_frames = []
        for q in queues:
            for i, msg in enumerate(q['msgs']):
                time_diff = abs(msg.getTimestamp() - timestamp)
                # So below 17ms @ 30 FPS => frames are in sync
                if time_diff <= timedelta(milliseconds=math.ceil(500 / fps)):
                    matching_frames.append(i)
                    break

        if len(matching_frames) == len(queues):
            # We have all frames synced. Remove the excess ones
            for i, q in enumerate(queues):
                q['msgs'] = q['msgs'][matching_frames[i]:]
            return True
        else:
            return False

    save_frame = 0
    while True:
        for q in queues:
            new_msg = q['queue'].tryGet()
            if new_msg is not None:
                q['msgs'].append(new_msg)
                if check_sync(queues, new_msg.getTimestamp()):
                    # frames are synchronised
                    for q in queues:
                        frame = q['msgs'].pop(0).getCvFrame()
                        cv2.imshow(f"{q['cam']} - {q['mx']}", frame)
                        if save_frame != 0:
                            q['cnt'] += 1
                            frame_file = os.path.join(cwd, str(q['mx']) + "/" + str(q['cnt']) + ".png")
                            cv2.imwrite(frame_file,frame)
                            save_frame -= 1
                                
        if cv2.waitKey(1) == ord('q'):
            break
        if cv2.waitKey(1) == ord("s"):
            save_frame = len(device_infos)