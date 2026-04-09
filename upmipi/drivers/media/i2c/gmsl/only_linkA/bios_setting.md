System Agent (SA) Configuration -> IPU Device (B0:D5:F0)                        [Enabled]
System Agent (SA) Configuration -> MIPI Camera Configuration -> CVF Support     [Disabled]
System Agent (SA) Configuration -> MIPI Camera Configuration -> Control Logic 1 [Enabled]
Control Logic Type              [Discrete]
CRD Version                     [CRD-D]
Input Clock                     [19.2 MHz]
PCH Clock Source                [IMGCLKOUT_0]
Number of GPIO Pins             0



System Agent (SA) Configuration -> MIPI Camera Configuration -> Camera 1        [Enabled]
System Agent (SA) Configuration -> MIPI Camera Configuration -> Link options
Sensor Model                    [User Custom]
Video HID                       MAX96724
Lanes Clock division            [4 4 2 2]
GPIO control                    [Control Logic 1]
Camera position                 [Back]
Flash Support                   [Disable]
Privacy LED                     [Driver default]
Rotation                        [0]
Camera module name              max96724
MIPI port                       4
LaneUsed                        [x2]
PortSpeed                       [2]
MCLK                            19200000
EEPROM Type                     [ROM_NONE]
VCM Type                        [VCM_NONE]
Number of I2C                   1
I2C Channel                     [I2C5]
Device                          0
I2C Address                     27
Device Type                     [Sensor]

Flash Driver Selection          [Disabled]

Stream On command (It will output a .raw file names "camera_real.raw", the raw is five-seq-image): 

media-ctl -l '"Intel IPU6 CSI2 4":1 -> "Intel IPU6 ISYS Capture 32":0[1]'
media-ctl -V '"Intel IPU6 CSI2 4":0 [fmt:UYVY8_1X16/1920x1536 field:none]'
media-ctl -p -d /dev/media0 | awk -v RS= '/- entity [0-9]+: (Intel IPU6 CSI2 4|max967|ser|des|isx)/'

v4l2-ctl -d /dev/video32 --set-fmt-video=width=1920,height=1536,pixelformat=UYVY --get-fmt-video
v4l2-ctl -d /dev/video32 --stream-mmap --stream-count=5 --stream-to=camera_real.raw
