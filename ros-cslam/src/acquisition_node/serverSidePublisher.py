#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
import cPickle as pickle
import os
import Queue
import collections
import yaml


def publishOnServer(outputDictQueue, quitEvent, logger, mode='live'):
    """
    Publishes the processed data on the ROS Master that the graph optimizer uses.
    """
    logger.info("Setting up the server side process")

    # Get the environment variables
    ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', "watchtower33")

    seq_stamper = 0
    counts = collections.Counter()

    publisherImages = rospy.Publisher("/"+ACQ_DEVICE_NAME+"/image/compressed", CompressedImage, queue_size=1)

    # Init the node (live mode only)
    rospy.init_node('acquisition_node_'+ACQ_DEVICE_NAME)

    logger.info("Setting up the server side process completed. Waiting for messages...")

    # Run continuously, check for new data arriving from the acquisitionProcessor and processed it when it arrives
    while not quitEvent.is_set():
        try:
            newQueueData = outputDictQueue.get(block=True, timeout=5)
            incomingData = pickle.loads(newQueueData)
            if "image_stream" in incomingData:
                imgMsg = incomingData["image_stream"]
                imgMsg.header.seq = seq_stamper
                publisherImages.publish(imgMsg)
            seq_stamper+=1

        except KeyboardInterrupt:
            raise( Exception("Exiting") )
        except Queue.Empty:
            if os.getenv('ACQ_DEVICE_MODE', 'live') == 'live':
                logger.warning("No messages received in the last 5 seconds!")
        except Exception as e:
            logger.warning("Exception: %s" % str(e))
            pass
