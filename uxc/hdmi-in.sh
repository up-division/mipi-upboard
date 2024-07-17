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

sleep 1

if [ "$1" = "" ]; then
    echo -e "Input 4K 30Hz"
    gst-launch-1.0 icamerasrc scene-mode=normal device-name=lt6911uxc printfps=true ! video/x-raw,format=UYVY,width=3840,height=2160 ! videoconvert ! xvimagesink
else
    echo -e "Input 4K 60Hz"
    gst-launch-1.0 \
    icamerasrc scene-mode=normal device-name=lt6911uxc printfps=true ! video/x-raw,format=UYVY,width=1920,height=2160 ! videoconvert ! ximagesink \
    icamerasrc scene-mode=normal device-name=lt6911uxc-2 printfps=true ! video/x-raw,format=UYVY,width=1920,height=2160 ! videoconvert ! ximagesink
fi