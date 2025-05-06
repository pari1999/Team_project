import time
from robomaster import camera


def start_new_stream(ep_camera):
    print("Startng streammmmmmmmmmmmmmmmmmm")
    ep_camera.start_video_stream(display=True,resolution=camera.STREAM_360P)
    time.sleep(30)
    ep_camera.stop_video_stream()