#!/bin/bash

# 檢查參數
if  [ $# -lt 5 ] ; then
    echo "usage: ./live-stream.sh [DeviceName] [Port] [ISYS Capture Channel] [Resolution] [Format] [Mode]"
    echo -e "Options for [Mode]: live (default), picture, record, burst"
    echo -e "Examples:"
    echo -e "  Live View:  ./live-stream.sh 'ov5647' 0 0 1296x972 SBGGR10_1X10 live"
    echo -e "  Burst 100:  ./live-stream.sh 'imx708' 0 0 1536x864 SRGGB10_1X10 burst"
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
MODE=${6:-live}
BURST_COUNT=${7:-100}

# 自動補全
if [[ "$DEV" == "ov5647" ]]; then DEV="ov5647 0-0036"; echo "Auto: $DEV"; fi
if [[ "$DEV" == "imx219" ]]; then DEV="imx219 0-0010"; echo "Auto: $DEV"; fi
if [[ "$DEV" == "imx708" ]]; then echo "Auto: imx708"; fi 

# 變數解析
VFMT=${FMT%_*} 
WIDTH=${RESOLUTION%x*}
HEIGHT=${RESOLUTION#*x}
CSI2PORT="Intel IPU6 CSI2 $PORT"
ISYSCAPTURE="Intel IPU6 ISYS Capture $ISYS"

if [[ "$DEV" =~ "imx477" ]]; then WIDTH=$((WIDTH+8)); fi

# 格式翻譯
V4L2_FMT="RG10" 
if [[ "$VFMT" == "SRGGB12" ]]; then V4L2_FMT="RG12"
elif [[ "$VFMT" == "SRGGB8" ]]; then V4L2_FMT="RGGB"
elif [[ "$VFMT" == "SBGGR10" ]]; then V4L2_FMT="BG10"
elif [[ "$VFMT" == "SGRBG10" ]]; then V4L2_FMT="BA10"
elif [[ "$VFMT" == "SGBRG10" ]]; then V4L2_FMT="GB10"
elif [[ "$VFMT" == "SRGGB10" ]]; then V4L2_FMT="RG10"
fi

# 設定管線
media-ctl -r
media-ctl -V "'$DEV':0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "'$CSI2PORT':0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "'$CSI2PORT':1 [fmt:$FMT/${RESOLUTION}]"
media-ctl -l "'$DEV':0 -> '$CSI2PORT':0[1]"
media-ctl -l "'$CSI2PORT':1 -> '$ISYSCAPTURE':0[1]"

CAPTURE_DEV=$(media-ctl -e "$ISYSCAPTURE")
SUBDEV=$(media-ctl -e "$DEV")

# 曝光設定
if [[ "$DEV" == *"ov5647"* ]]; then
    SENSOR_TYPE="ov5647"
    v4l2-ctl -d $SUBDEV --set-ctrl exposure=1000 2>/dev/null
    v4l2-ctl -d $SUBDEV --set-ctrl analogue_gain=32 2>/dev/null
elif [[ "$DEV" == *"imx708"* ]]; then
    SENSOR_TYPE="imx708"
    v4l2-ctl -d $SUBDEV --set-ctrl exposure=1000 2>/dev/null
    v4l2-ctl -d $SUBDEV --set-ctrl analogue_gain=256 2>/dev/null
else
    SENSOR_TYPE="imx219"
    v4l2-ctl -d $SUBDEV --set-ctrl exposure=1000 2>/dev/null
    v4l2-ctl -d $SUBDEV --set-ctrl analogue_gain=96 2>/dev/null
fi

echo "========== Stream Config =========="
echo "Dev: $SENSOR_TYPE | Res: $WIDTH x $HEIGHT | Mode: $MODE"

# ==========================================
# 5. 執行串流
# ==========================================

# 預設抓取數量：0 = 無限
STREAM_COUNT=0
BURST_ARG=""

if [ "$MODE" == "burst" ]; then
    # [修改] 使用傳入的張數
    STREAM_COUNT=$BURST_COUNT
    BURST_ARG="--burst-count $BURST_COUNT" # 準備傳給 Python 的參數
    echo "Burst Mode: Capturing $BURST_COUNT frames..."
fi

CMD_V4L2="v4l2-ctl -d $CAPTURE_DEV --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=$V4L2_FMT --stream-mmap --stream-count=$STREAM_COUNT --stream-to=-"
CMD_PYTHON="python3 camera_app.py --sensor $SENSOR_TYPE --width $WIDTH --height $HEIGHT --mode $MODE $BURST_ARG"

if [ "$MODE" == "record" ]; then
    # 錄影模式邏輯 (略)
    if [ "$WIDTH" -gt 3000 ]; then DIV=4; elif [ "$WIDTH" -gt 1000 ]; then DIV=2; else DIV=1; fi
    REC_WIDTH=$((WIDTH / DIV))
    REC_HEIGHT=$((HEIGHT / DIV))
    OUT_FILE="video_${SENSOR_TYPE}_${WIDTH}x${HEIGHT}_$(date +%Y%m%d-%H%M%S).mp4"
    $CMD_V4L2 | $CMD_PYTHON | gst-launch-1.0 -e -v fdsrc ! videoparse format=bgr width=$REC_WIDTH height=$REC_HEIGHT framerate=30/1 ! videoconvert ! x264enc tune=zerolatency ! mp4mux ! filesink location="$OUT_FILE"

else
    # 拍照、Live、Burst 模式
    $CMD_V4L2 | $CMD_PYTHON
fi

if [ -n "$VIRTUAL_ENV" ]; then
    deactivate
fi