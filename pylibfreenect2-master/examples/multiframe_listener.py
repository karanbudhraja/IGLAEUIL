# coding: utf-8

# libraries used
import numpy as np
import cv2
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import OpenGLPacketPipeline

# open connection to kinect
def openKinect():
    fn = Freenect2()
    pipeline = OpenGLPacketPipeline()
    device = fn.openDefaultDevice(pipeline=pipeline)

    listener = SyncMultiFrameListener(
        FrameType.Color | FrameType.Ir | FrameType.Depth)

    # Register listeners
    device.setColorFrameListener(listener)
    device.setIrAndDepthFrameListener(listener)

    device.start()

    # NOTE: must be called after device.start()
    registration = Registration(device.getIrCameraParams(),
                                device.getColorCameraParams())

    undistorted = Frame(512, 424, 4)
    registered = Frame(512, 424, 4)

    return [fn, pipeline, device, listener, registration, undistorted, registered]

# read frame data from kinect
def readKinect(fn, pipeline, device, listener, registration, undistorted, registered):
    frames = listener.waitForNewFrame()

    color = frames["color"]
    ir = frames["ir"]
    depth = frames["depth"]

    registration.apply(color, depth, undistorted, registered)

    cv2.imshow("ir", ir.asarray() / 65535.)
    cv2.imshow("depth", depth.asarray() / 4500.)
    cv2.imshow("color", cv2.resize(color.asarray(), (int(1920 / 3), int(1080 / 3))))
    cv2.imshow("registered", registered.astype(np.uint8))

    listener.release(frames)

    key = cv2.waitKey(delay=1)

    return [fn, pipeline, device, listener, registration, undistorted, registered, key]

# close connection to kinect
def closeKinect(device):
    device.stop()
    device.close()

def main():

    [fn, pipeline, device, listener, registration, undistorted, registered] = openKinect()
    
    while True:
        [fn, pipeline, device, listener, registration, undistorted, registered, key] = readKinect(fn, pipeline, device, listener, registration, undistorted, registered)

        if key == ord('q'):
            break

    closeKinect(device)

main()
