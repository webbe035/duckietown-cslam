#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
import cPickle as pickle
import os
import Queue

class acquisitionProcessor():
    """
    Processes the data coming from a remote device (Duckiebot or watchtower).
    """
    def __init__(self, logger, mode='live'):

        self.mode = mode
        if self.mode != 'live' and self.mode != 'postprocessing':
            raise Exception("The argument mode should be 'live' or 'postprocessing'. Received %s instead." % self.mode)

        # Get the environment variables
        self.ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', 'watchtower33')
        self.ACQ_TOPIC_RAW = os.getenv('ACQ_TOPIC_RAW', 'camera_node/image/compressed')
        self.ACQ_POSES_UPDATE_RATE = float(os.getenv('ACQ_POSES_UPDATE_RATE', 10)) #Hz

        # Initialize ROS nodes and subscribe to topics
        rospy.init_node('acquisition_processor', anonymous=True, disable_signals=True)
        self.subscriberRawImage = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_RAW, CompressedImage,
                                                    self.camera_image_process,  queue_size = 1)

        self.logger = logger
        self.lastCameraImage = None
        self.lastImageProcessed = False
        self.timeLastPub_poses = 0
        self.logger.info('Acquisition processor is set up.')


    def camera_image_process(self, currRawImage):
            self.lastCameraImage = currRawImage
            self.lastImageProcessed = False


    def liveUpdate(self, outputDictQueue, quitEvent):
        """
        Runs constantly and processes new data as it comes.
        """

        while not quitEvent.is_set():
            # Check if the last image data was not yet processed and if it's time to process it (in order to sustain the deisred update rate)
            if rospy.get_time() - self.timeLastPub_poses >= 1.0/self.ACQ_POSES_UPDATE_RATE and not self.lastImageProcessed:
                self.timeLastPub_poses = rospy.get_time()
                if self.lastCameraImage is not None:
                    # Collect latest ros_data
                    outputDict = dict()
                    outputDict['image_stream']=self.lastCameraImage
                    if outputDict is not None:
                        outputDictQueue.put(obj=pickle.dumps(outputDict, protocol=-1),
                                            block=True,
                                            timeout=None)
                        self.lastImageProcessed = True
