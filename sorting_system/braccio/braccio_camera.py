import depthai as dai
import cv2
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def get_frame():
    pipeline = dai.Pipeline()
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(300, 300)
    cam_rgb.setInterleaved(False)
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("video")
    cam_rgb.preview.link(xout.input)

    with dai.Device(pipeline) as device:
        q = device.getOutputQueue(name="video", maxSize=1, blocking=False)
        frame = q.get().getCvFrame()
        return frame
