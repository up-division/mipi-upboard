#下面指令16要根據dmesg出現的des mux i2c bus決定 有可能會不是16
SER_I2C=16
sudo modprobe max96717_driver
echo "max96717 0x40" | sudo tee /sys/bus/i2c/devices/i2c-$SER_I2C/new_device

#下面指令19要根據dmesg出現的ser mux i2c bus決定 有可能會不是19
ISX_I2C=19
sudo modprobe isx031_driver
echo "isx031 0x1a" | sudo tee /sys/bus/i2c/devices/i2c-$ISX_I2C/new_device

media-ctl -l '"Intel IPU6 CSI2 1":1 -> "Intel IPU6 ISYS Capture 8":0[1]'
media-ctl -V '"Intel IPU6 CSI2 1":0 [fmt:UYVY8_1X16/1920x1536 field:none]'
media-ctl -V '"max96724 1-0027":0 [fmt:UYVY8_1X16/1920x1536 field:none]'
media-ctl -V '"max96724 1-0027":4 [fmt:UYVY8_1X16/1920x1536 field:none]'
#media-ctl -V '"ser 1-0021":0 [fmt:UYVY8_1X16/1920x1536 field:none]'
#media-ctl -V '"ser 1-0021":1 [fmt:UYVY8_1X16/1920x1536 field:none]'
media-ctl -V '"isx031 19-001a":0 [fmt:UYVY8_1X16/1920x1536 field:none]'
media-ctl -p -d /dev/media0 | awk -v RS= '/- entity [0-9]+: (Intel IPU6 CSI2 1|max96724|ser|isx)/'

v4l2-ctl -d /dev/video8 --set-fmt-video=width=1920,height=1536,pixelformat=UYVY --get-fmt-video
v4l2-ctl -d /dev/video8 --stream-mmap --stream-count=5 --stream-to=camera_real.raw



