#!/bin/bash

# 檢查參數 (至少 5 個參數)
if  [ $# -lt 2 ]; then
    echo "usage: ./live_stream.sh [DeviceName] [Port] [ISYS Capture Channel] [Resolution] [Format]"
    echo -e "Example:"
    echo -e "./live_stream.sh 0 0"
    echo -e "./live_stream.sh 0 1"
    echo -e "./live_stream.sh 0 2"
    echo -e "./live_stream.sh 0 3"
    echo -e ""
    echo -e "./live_stream.sh 1 8 "
    echo -e "./live_stream.sh 1 9 "
    echo -e "./live_stream.sh 1 10"
    echo -e "./live_stream.sh 1 11"
    echo -e ""
    echo -e "./live_stream.sh 2 16"
    echo -e "./live_stream.sh 2 17"
    echo -e "./live_stream.sh 2 18"
    echo -e "./live_stream.sh 2 19"
    echo -e ""
    echo -e "./live_stream.sh 4 32"
    echo -e "./live_stream.sh 4 33"
    echo -e "./live_stream.sh 4 34"
    echo -e "./live_stream.sh 4 35"
    exit 1
fi

# 啟動虛擬環境 (如果有的話)
if [ ! -d mipi-up ]; then
    python3 -m venv mipi-up
    source mipi-up/bin/activate
    pip install -r requirement.txt
else
    source mipi-up/bin/activate
fi

DEV=max96724
PORT=$1
ISYS=$2
RESOLUTION=1920x1536
FMT=UYVY8_1X16



# ==========================================
# 自動尋找 Intel IPU6 對應的 media 節點
# ==========================================
IPU_MEDIA=""
for m in /dev/media*; do
    if media-ctl -d "$m" -p 2>/dev/null | grep -q "Intel IPU6"; then
        IPU_MEDIA="$m"
        break
    fi
done

if [ -z "$IPU_MEDIA" ]; then
    echo "❌ 錯誤: 找不到 Intel IPU6 的 media 節點，請確認驅動已載入！"
    exit 1
fi
echo "✅ 找到 IPU6 Media 節點: $IPU_MEDIA"

# ==========================================
# 變數解析與格式翻譯
# ==========================================
VFMT=${FMT%_*} 
WIDTH=${RESOLUTION%x*}
HEIGHT=${RESOLUTION#*x}
CSI2PORT="Intel IPU6 CSI2 $PORT"
ISYSCAPTURE="Intel IPU6 ISYS Capture $ISYS"

V4L2_FMT="RG10" 
if [[ "$VFMT" == "SRGGB12" ]]; then V4L2_FMT="RG12"
elif [[ "$VFMT" == "SRGGB8" ]]; then V4L2_FMT="RGGB"
elif [[ "$VFMT" == "SBGGR10" ]]; then V4L2_FMT="BG10"
elif [[ "$VFMT" == "SGRBG10" ]]; then V4L2_FMT="BA10"
elif [[ "$VFMT" == "SGBRG10" ]]; then V4L2_FMT="GB10"
elif [[ "$VFMT" == "SRGGB10" ]]; then V4L2_FMT="RG10"
elif [[ "$VFMT" == "UYVY8" ]]; then V4L2_FMT="UYVY" 
elif [[ "$VFMT" == "YUYV8" ]]; then V4L2_FMT="YUYV"
fi

# ==========================================
# 設定管線 (使用自動找到的 IPU_MEDIA)
# ==========================================
media-ctl -d "$IPU_MEDIA" -r
echo "flag"
media-ctl -d "$IPU_MEDIA" -V "'$DEV':0 [fmt:$FMT/${RESOLUTION} field:none]"
echo "flag"
media-ctl -d "$IPU_MEDIA" -V "'$CSI2PORT':0 [fmt:$FMT/${RESOLUTION} field:none]"
echo "flag"
media-ctl -d "$IPU_MEDIA" -V "'$CSI2PORT':1 [fmt:$FMT/${RESOLUTION} field:none]"
echo "flag"
# media-ctl -d "$IPU_MEDIA" -l "'$DEV':0 -> '$CSI2PORT':0[1]"
# media-ctl -d "$IPU_MEDIA" -l "'$CSI2PORT':1 -> '$ISYSCAPTURE':0[1]"

media-ctl -l '"Intel IPU6 CSI2 0":1 -> "Intel IPU6 ISYS Capture 0":0[1]'
media-ctl -l '"Intel IPU6 CSI2 0":2 -> "Intel IPU6 ISYS Capture 1":0[1]'
media-ctl -l '"Intel IPU6 CSI2 0":3 -> "Intel IPU6 ISYS Capture 2":0[1]'
media-ctl -l '"Intel IPU6 CSI2 0":4 -> "Intel IPU6 ISYS Capture 3":0[1]'

media-ctl -l '"Intel IPU6 CSI2 1":1 -> "Intel IPU6 ISYS Capture 8":0[1]'
media-ctl -l '"Intel IPU6 CSI2 1":2 -> "Intel IPU6 ISYS Capture 9":0[1]'
media-ctl -l '"Intel IPU6 CSI2 1":3 -> "Intel IPU6 ISYS Capture 10":0[1]'
media-ctl -l '"Intel IPU6 CSI2 1":4 -> "Intel IPU6 ISYS Capture 11":0[1]'

media-ctl -l '"Intel IPU6 CSI2 2":1 -> "Intel IPU6 ISYS Capture 16":0[1]'
media-ctl -l '"Intel IPU6 CSI2 2":2 -> "Intel IPU6 ISYS Capture 17":0[1]'
media-ctl -l '"Intel IPU6 CSI2 2":3 -> "Intel IPU6 ISYS Capture 18":0[1]'
media-ctl -l '"Intel IPU6 CSI2 2":4 -> "Intel IPU6 ISYS Capture 19":0[1]'

media-ctl -l '"Intel IPU6 CSI2 4":1 -> "Intel IPU6 ISYS Capture 32":0[1]'
media-ctl -l '"Intel IPU6 CSI2 4":2 -> "Intel IPU6 ISYS Capture 33":0[1]'
media-ctl -l '"Intel IPU6 CSI2 4":3 -> "Intel IPU6 ISYS Capture 34":0[1]'
media-ctl -l '"Intel IPU6 CSI2 4":4 -> "Intel IPU6 ISYS Capture 35":0[1]'

echo "flag"


CAPTURE_DEV=$(media-ctl -d "$IPU_MEDIA" -e "$ISYSCAPTURE")
SUBDEV=$(media-ctl -d "$IPU_MEDIA" -e "$DEV")

# ==========================================
# 判斷感測器類型並重置曝光
# ==========================================
if [[ "$DEV" == *"ov5647"* ]]; then
    SENSOR_TYPE="ov5647"
    v4l2-ctl -d $SUBDEV --set-ctrl exposure=1000 2>/dev/null
    v4l2-ctl -d $SUBDEV --set-ctrl analogue_gain=32 2>/dev/null
elif [[ "$DEV" == *"imx708"* ]]; then
    SENSOR_TYPE="imx708"
    v4l2-ctl -d $SUBDEV --set-ctrl exposure=1000 2>/dev/null
    v4l2-ctl -d $SUBDEV --set-ctrl analogue_gain=256 2>/dev/null
elif [[ "$DEV" == *"isx031"* || "$DEV" == *"max96724"* ]]; then
    SENSOR_TYPE="isx031"   # <-- 讓 Python 收到正確的 isx031 參數
    # 車載相機由 ISP 控制，此處可略過曝光/增益的 v4l2-ctl 設定
else
    SENSOR_TYPE="imx219"
    v4l2-ctl -d $SUBDEV --set-ctrl exposure=1000 2>/dev/null
    v4l2-ctl -d $SUBDEV --set-ctrl analogue_gain=96 2>/dev/null
fi

echo "========== Stream Config =========="
echo "Dev: $SENSOR_TYPE | Res: $WIDTH x $HEIGHT | Mode: Live Only"

# ==========================================
# 執行即時串流：V4L2 -> Python
# ==========================================
CMD_V4L2="v4l2-ctl -d $CAPTURE_DEV --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=$V4L2_FMT --stream-mmap --stream-count=0 --stream-to=-"
CMD_PYTHON="python3 camera_app.py --sensor $SENSOR_TYPE --width $WIDTH --height $HEIGHT"

$CMD_V4L2 | $CMD_PYTHON

if [ -n "$VIRTUAL_ENV" ]; then
    deactivate
fi