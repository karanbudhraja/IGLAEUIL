'''
@Description: This code attempts to model the joint learning architecture presented in A Joint Model of Language and Perception for Grounded Attribute Learning
			  Referer to http://cynthia.matuszek.org/Pubs/MatuszekICML2012.pdf, http://cynthia.matuszek.org/Slides/MatuszekFutureOfHRIWorkshopICRA2012.pdf
'''

__author__	= "Karan K. Budhraja, Nisha Pillai"
__email__	= "karanb1@umbc.edu, npillai1@umbc.edu"

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
    kinectConfiguration = [fn, pipeline, device, listener, registration, undistorted, registered]
    
    # kinect configuration information corresponding to this connection
    return kinectConfiguration

# read frame data from kinect
def readKinect(kinectConfiguration):
    # read kinect configuration
    [fn, pipeline, device, listener, registration, undistorted, registered] = tuple(kinectConfiguration)

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
    kinectConfiguration = [fn, pipeline, device, listener, registration, undistorted, registered]

    # return configuration and image data
    return [kinectConfiguration, key, frames]

# close connection to kinect
def closeKinect(kinectConfiguration):
    # read kinect configuration
    [fn, pipeline, device, listener, registration, undistorted, registered] = tuple(kinectConfiguration)

    # stop device
    device.stop()
    device.close()

# training phase
def train():
    # TODO: initialize learning model

    # open kinect connection
    kinectConfiguration = openKinect()
    
    while True:
        # get text description
        description = input("Please describe the scene (or press q to quit): ")

        # get image
        [kinectConfiguration, key, frames] = readKinect(kinectConfiguration)

        # TODO: extract keywords from description
        # TODO: train model on keywords and image

        # continue while quit not requested
        if(key == ord('q')):
            break
    
    # close kinect connection
    closeKinect(device)

# testing phase
def test():
    pass

# main
def main():

    train()

main()
