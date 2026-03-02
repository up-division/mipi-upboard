#下面指令16要根據dmesg出現的des mux i2c bus決定 有可能會不是16
sudo i2cset -f -y 16 0x27 0x00 0x00 0x42 i
sudo modprobe max96717_driver
echo "max96717 0x21" | sudo tee /sys/bus/i2c/devices/i2c-1/new_device

#下面指令19+要根據dmesg出現的ser mux i2c bus決定 有可能會不是19
sudo modprobe isx031_driver
echo "isx031 0x1a" | sudo tee /sys/bus/i2c/devices/i2c-19/new_device

media-ctl -l '"Intel IPU6 CSI2 1":1 -> "Intel IPU6 ISYS Capture 8":0[1]'
media-ctl -V '"isx031 19-001a":0 [fmt:UYVY8_1X16/1920x1536 field:none]'
media-ctl -V '"ser 1-0021":0 [fmt:UYVY8_1X16/1920x1536 field:none]'
media-ctl -V '"ser 1-0021":1 [fmt:UYVY8_1X16/1920x1536 field:none]'
media-ctl -V '"max96724 1-0027":0 [fmt:UYVY8_1X16/1920x1536 field:none]'
media-ctl -V '"max96724 1-0027":4 [fmt:UYVY8_1X16/1920x1536 field:none]'
media-ctl -V '"Intel IPU6 CSI2 1":0 [fmt:UYVY8_1X16/1920x1536 field:none]'

v4l2-ctl -d /dev/video8 --set-fmt-video=width=1920,height=1536,pixelformat=UYVY --get-fmt-video
v4l2-ctl -d /dev/video8 --stream-mmap --stream-count=5 --stream-to=camera_real.raw

media-ctl -p -d /dev/media0 | awk -v RS= '/- entity [0-9]+: (Intel IPU6 CSI2 1|max96724|ser|isx)/'