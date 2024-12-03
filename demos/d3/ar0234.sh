#!/bin/bash
#export DISPLAY=:0
#xhost +
#sudo bash 

export GST_PLUGIN_PATH=/usr/lib/gstreamer-1.0/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib:/usr/lib/ipu_adl
export LIBVA_DRIVER_NAME=iHD
export GST_GL_PLATFORM=egl

rm -r ~/.cache/gstreamer-1.0

# kernel 5.19+
echo 400 > /sys/kernel/debug/intel-ipu6/buttress/isys_freq

#echo 400 > /sys/kernel/debug/intel-ipu/buttress/isys_freq

gst-launch-1.0 icamerasrc device-name=ar0234 ! video/x-raw,format=NV12,width=1280,height=960 ! videoconvert ! xvimagesink
