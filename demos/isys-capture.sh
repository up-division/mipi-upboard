#!/bin/bash

if  [ $# -lt 5 ] ; then
echo "usage: raw-capture.sh [DeviceName] [Port] [ISYS Capture Channel] [Resolution] [Format]"
echo -e "ar0234 example: isys-capture.sh 'ar0234 0-0010' 0 0 1280x960 SGRBG10_1X10"
echo -e "imx477 example: isys-capture.sh 'imx477 0-001a' 0 0 4056x3040 SRGGB12_1X12"
echo -e "imx708 example: isys-capture.sh 'imx708' 0 0 4608x2592 SRGGB10_1X10"
exit 1
fi

if [ ! -d mipi-up ]; then
python -m venv mipi-up
source mipi-up/bin/activate
pip install -r requirement.txt
else
source mipi-up/bin/activate
fi

DEV=$1
PORT=$2
ISYS=$3
RESOLUTION=$4
FMT=$5
VFMT=${FMT%_*}
WIDTH=${RESOLUTION%x*}
HEIGHT=${RESOLUTION#*x}
CSI2PORT="Intel IPU6 CSI2 $PORT"
ISYSCAPTURE="Intel IPU6 ISYS Capture $ISYS"

#modify imx477 capture width
if [[ "$DEV" =~ "imx477" ]]; then
WIDTH=$((WIDTH+8))
fi

#setting camera
media-ctl -r
media-ctl -V "'$DEV':0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "'$CSI2PORT':0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "'$CSI2PORT':1 [fmt:$FMT/${RESOLUTION}]"
#media-ctl -V "'$ISYSCAPTURE':0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -l "'$DEV':0 -> '$CSI2PORT':0[1]"
media-ctl -l "'$CSI2PORT':1 -> '$ISYSCAPTURE':0[1]"

#capture 1 frame
CAPTURE_DEV=$(media-ctl -e "$ISYSCAPTURE")
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
CAPTURE_FILE="$DEV-$TIMESTAMP.bin"
sudo yavta -c1 -n5 ${CAPTURE_DEV} -I -s${RESOLUTION} -F"${CAPTURE_FILE}" -f $VFMT
python grbg2jpg.py "$CAPTURE_FILE" -W $WIDTH -H $HEIGHT

deactivate
