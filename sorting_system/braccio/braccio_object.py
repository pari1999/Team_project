import torch
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

model = torch.hub.load('ultralytics/yolov5', 'custom', path='models/yolov5n.pt')

def object_detected(frame):
    results = model(frame)
    return len(results.xyxy[0]) > 0
