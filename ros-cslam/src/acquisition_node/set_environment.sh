#!/bin/bash

export ACQ_POSES_UPDATE_RATE=30
export ACQ_TOPIC_RAW="camera_node/image/compressed"
export ACQ_DEVICE_MODE="live"
export ACQ_SERVER_MODE="live"
HOST=$(hostname)
export ACQ_DEVICE_NAME=$HOST
IP=$(ifconfig | awk '/inet addr/{print substr($2,6)}' | grep "192")
export ACQ_ROS_MASTER_URI_DEVICE_IP=$IP
export ACQ_ROS_MASTER_URI_DEVICE_PORT=11311
export ACQ_ROS_MASTER_URI_SERVER_IP="192.168.1.89"
export ACQ_ROS_MASTER_URI_SERVER_PORT=11311
