#!/bin/bash

# 檢查參數 (現在需要 6 個參數，或至少 5 個，第 6 個預設 live)
if  [ $# -lt 5 ] ; then
    echo "usage: ./live-stream.sh [DeviceName] [Port] [ISYS Capture Channel] [Resolution] [Format] [Mode]"
    echo -e "Options for [Mode]: live (default), picture, record"
    echo -e "Examples:"
    echo -e "  Live View:  ./live-stream.sh 'ov5647' 0 0 1296x972 SBGGR10_1X10 live"
    echo -e "  Take Photo: ./live-stream.sh 'imx708' 0 0 4608x2592 SRGGB10_1X10 picture"
    echo -e "  Recording:  ./live-stream.sh 'imx219' 0 0 1640x1232 SGRBG10_1X10 record"
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
MODE=${6:-live} # 預設為 live


# ==========================================
# 3. 變數解析
# ==========================================
VFMT=${FMT%_*} 
WIDTH=${RESOLUTION%x*}
HEIGHT=${RESOLUTION#*x}
CSI2PORT="Intel IPU6 CSI2 $PORT"
ISYSCAPTURE="Intel IPU6 ISYS Capture $ISYS"

if [[ "$DEV" =~ "imx477" ]]; then WIDTH=$((WIDTH+8)); fi

# ==========================================
# 4. 格式翻譯與曝光重置邏輯
# ==========================================
V4L2_FMT="RG10" 
if [[ "$VFMT" == "SRGGB12" ]]; then V4L2_FMT="RG12"
elif [[ "$VFMT" == "SRGGB8" ]]; then V4L2_FMT="RGGB"
elif [[ "$VFMT" == "SBGGR10" ]]; then V4L2_FMT="BG10"
elif [[ "$VFMT" == "SGRBG10" ]]; then V4L2_FMT="BA10"
elif [[ "$VFMT" == "SGBRG10" ]]; then V4L2_FMT="GB10"
elif [[ "$VFMT" == "SRGGB10" ]]; then V4L2_FMT="RG10"
fi

# 設定管線 (需 sudo 權限)
media-ctl -r
media-ctl -V "'$DEV':0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "'$CSI2PORT':0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "'$CSI2PORT':1 [fmt:$FMT/${RESOLUTION}]"
media-ctl -l "'$DEV':0 -> '$CSI2PORT':0[1]"
media-ctl -l "'$CSI2PORT':1 -> '$ISYSCAPTURE':0[1]"


CAPTURE_DEV=$(media-ctl -e "$ISYSCAPTURE")
SUBDEV=$(media-ctl -e "$DEV")

# 自動曝光管理
if [[ "$DEV" == *"ov5647"* ]]; then
    SENSOR_TYPE="ov5647"
    echo "Configuring OV5647 (Night Vision Mode)..."
    v4l2-ctl -d $SUBDEV --set-ctrl exposure=1000 2>/dev/null
    v4l2-ctl -d $SUBDEV --set-ctrl analogue_gain=32 2>/dev/null
elif [[ "$DEV" == *"imx708"* ]]; then
    SENSOR_TYPE="imx708"
    echo "Resetting Exposure (IMX708)..."
    v4l2-ctl -d $SUBDEV --set-ctrl exposure=1000 2>/dev/null
    v4l2-ctl -d $SUBDEV --set-ctrl analogue_gain=256 2>/dev/null
else
    SENSOR_TYPE="imx219"
    echo "Resetting Exposure (IMX219)..."
    v4l2-ctl -d $SUBDEV --set-ctrl exposure=1000 2>/dev/null
    v4l2-ctl -d $SUBDEV --set-ctrl analogue_gain=96 2>/dev/null
fi

echo "========== Stream Config =========="
echo "Dev: $SENSOR_TYPE | Res: $WIDTH x $HEIGHT | Mode: $MODE"

# ==========================================
# 5. 執行串流 (根據模式決定 pipeline)
# ==========================================

# 基礎 V4L2 指令 (抓圖)
CMD_V4L2="v4l2-ctl -d $CAPTURE_DEV --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=$V4L2_FMT --stream-mmap --stream-count=0 --stream-to=-"

# 基礎 Python 指令 (修圖)
CMD_PYTHON="python3 camera_app.py --sensor $SENSOR_TYPE --width $WIDTH --height $HEIGHT --mode $MODE"

if [ "$MODE" == "record" ]; then
    # --- 錄影模式 (需要計算縮放後的解析度給 GStreamer) ---
    
    # 這裡必須與 Python 中的 get_resize_scale 邏輯保持一致
    if [ "$WIDTH" -gt 3000 ]; then
        DIV=4 # 4K 縮 1/4
    elif [ "$WIDTH" -gt 1000 ]; then
        DIV=2 # 1080p 縮 1/2
    else
        DIV=1 # 小圖不縮
    fi
    
    REC_WIDTH=$((WIDTH / DIV))
    REC_HEIGHT=$((HEIGHT / DIV))
    TIMESTAMP=$(date +%Y%m%d-%H%M%S)
    OUT_FILE="video_${SENSOR_TYPE}_${WIDTH}x${HEIGHT}_${TIMESTAMP}.mp4"
    
    echo "Recording to $OUT_FILE (Size: ${REC_WIDTH}x${REC_HEIGHT})"
    echo "Press Ctrl+C to stop recording."

    # 執行：V4L2 -> Python -> GStreamer (Encoding)
    # -e 參數讓 GStreamer 在 Ctrl+C 時能正確存檔
    $CMD_V4L2 | $CMD_PYTHON | gst-launch-1.0 -e -v fdsrc ! \
        videoparse format=bgr width=$REC_WIDTH height=$REC_HEIGHT framerate=30/1 ! \
        videoconvert ! x264enc tune=zerolatency ! mp4mux ! filesink location="$OUT_FILE"

else
    # --- 拍照或即時觀看模式 ---
    # 直接執行 Python，它會自己決定是開視窗還是存 JPG
    $CMD_V4L2 | $CMD_PYTHON
fi

if [ -n "$VIRTUAL_ENV" ]; then
    deactivate
fi