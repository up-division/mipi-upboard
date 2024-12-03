#!/bin/bash

export GST_PLUGIN_PATH=/usr/lib/gstreamer-1.0
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib
export LIBVA_DRIVER_NAME=iHD
export GST_GL_PLATFORM=egl

# video_nr -- assign camera device id
sudo modprobe v4l2loopback video_nr=201,202

gst-launch-1.0 icamerasrc device-name=ev2m_gom1 ! video/x-raw,format=UYVY,width=1920,height=1080 ! v4l2sink device=/dev/video201 &
gst-launch-1.0 icamerasrc device-name=ev2m_gom1-2 ! video/x-raw,format=UYVY,width=1920,height=1080 ! v4l2sink device=/dev/video202

sudo pkill -2 gst-launch-1.0

sudo ./end.sh
