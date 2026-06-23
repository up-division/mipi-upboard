#!/bin/bash
set -euo pipefail

# ============================================================
# AAEOE 8 Camera Grid - stable v4l2-ctl loop writer + GST compositor
#
# Stable core:
#   v4l2-ctl persistent writer loop -> FIFO -> blocking read FD
#   -> gst fdsrc blocksize=frame_bytes -> videoparse uyvy
#   -> BGRx 30fps -> compositor BGRx 30fps -> sink
#
# Camera mapping:
#   CAMERA_PROFILE=topology  : force media-ctl topology detection from max96724 -> CSI2 links
#   CAMERA_PROFILE=auto      : prefer topology, then fallback to DMI board rule
#   CAMERA_PROFILE=current   : CSI2 0 + 4  => 0-0..0-3 / 4-32..4-35
#   CAMERA_PROFILE=alderlake : CSI2 1 + 2  => 1-8..1-11 / 2-16..2-19
#
# DMI fallback rule used here:
#   UPX-MTL01 / UPX-ARL01 / i14 / MTL / ARL => current, CSI2 0 + 4
#   Other UP boards fallback => alderlake, CSI2 1 + 2
# ============================================================

if [ "${EUID}" -ne 0 ]; then
    echo "需要 sudo 權限設定 media pipeline / v4l2，正在切換 sudo..."
    exec sudo -E "$0" "$@"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

DEV="${DEV:-max96724}"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1536}"
RESOLUTION="${WIDTH}x${HEIGHT}"
FMT="${FMT:-UYVY8_1X16}"
V4L2_FMT="${V4L2_FMT:-UYVY}"
VIDEOPARSE_FMT="${VIDEOPARSE_FMT:-uyvy}"
FPS_NUM="${FPS_NUM:-30}"
FPS_DEN="${FPS_DEN:-1}"

DISPLAY_W="${DISPLAY_W:-480}"
DISPLAY_H="${DISPLAY_H:-540}"

AUTO_DETECT="${AUTO_DETECT:-1}"
ACTIVE_CAMS="${ACTIVE_CAMS:-0-0}"
CAMERA_PROFILE="${CAMERA_PROFILE:-auto}"
PROBE_TIMEOUT="${PROBE_TIMEOUT:-4}"
PROBE_COUNT="${PROBE_COUNT:-1}"
POST_PROBE_SETTLE="${POST_PROBE_SETTLE:-1.5}"

WRITER_RESTART_DELAY="${WRITER_RESTART_DELAY:-0.3}"
STREAM_START_GAP="${STREAM_START_GAP:-0.1}"
GST_LAUNCH_RETRIES="${GST_LAUNCH_RETRIES:-1}"
MIN_RUN_SECONDS="${MIN_RUN_SECONDS:-2}"

SINK="${SINK:-autovideosink}"
SYNC="${SYNC:-false}"
FULLSCREEN="${FULLSCREEN:-1}"
SHOW_LABEL="${SHOW_LABEL:-0}"
KEEP_LOG="${KEEP_LOG:-0}"

FRAME_BYTES="${FRAME_BYTES:-$((WIDTH * HEIGHT * 2))}"
QUEUE_OPTS="max-size-buffers=2 max-size-bytes=0 max-size-time=0 leaky=downstream"

CAM_NAMES=()
CSI_GROUPS=()
CAPTURE_BASES=()
DEFAULT_ACTIVE_CAMS=""

echo 1 | sudo tee /sys/module/max_gmsl_lib/parameters/hotplug_rescan

if ! declare -A __test_assoc 2>/dev/null; then
    echo "❌ 需要 Bash 4+ 支援 associative array"
    exit 1
fi
unset __test_assoc
declare -A CAP_BY_NAME
declare -A FIFO_BY_NAME
declare -A FD_BY_NAME

FIFO_DIR=""
PROBE_DIR=""
IPU_MEDIA=""
PIDS=()
OPEN_FDS=()

cleanup_attempt() {
    for pid in "${PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done

    sleep 0.3

    for pid in "${PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done

    PIDS=()

    for fd in "${OPEN_FDS[@]:-}"; do
        eval "exec ${fd}<&-" 2>/dev/null || true
    done

    OPEN_FDS=()
    FD_BY_NAME=()
}

cleanup() {
    local code=$?
    echo ""
    echo "🛑 關閉 v4l2 loop-writer + GST 8 camera grid..."
    cleanup_attempt

    if [ -n "${FIFO_DIR:-}" ] && [ -d "$FIFO_DIR" ]; then
        if [ "$KEEP_LOG" = "1" ]; then
            echo "📁 保留 FIFO/log 目錄：$FIFO_DIR"
        else
            rm -rf "$FIFO_DIR"
        fi
    fi

    if [ -n "${PROBE_DIR:-}" ] && [ -d "$PROBE_DIR" ]; then
        if [ "$KEEP_LOG" = "1" ]; then
            echo "📁 保留 probe 目錄：$PROBE_DIR"
        else
            rm -rf "$PROBE_DIR"
        fi
    fi

    echo "✅ 已結束"
    exit "$code"
}
trap cleanup EXIT INT TERM

log_section() {
    echo ""
    echo "============================================================"
    echo "$1"
    echo "============================================================"
}

need_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "❌ 找不到指令：$1"
        exit 1
    fi
}

check_runtime() {
    need_cmd media-ctl
    need_cmd v4l2-ctl
    need_cmd gst-launch-1.0
    if ! gst-inspect-1.0 compositor >/dev/null 2>&1; then
        echo "❌ 找不到 GStreamer compositor plugin"
        echo "請安裝：sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good"
        exit 1
    fi
}

find_ipu_media() {
    IPU_MEDIA=""
    for try in 1 2 3 4 5; do
        echo "搜尋 IPU6 media node，第 $try/5 次..."
        for m in /dev/media*; do
            [ -e "$m" ] || continue
            info="$(media-ctl -d "$m" -p 2>/dev/null || true)"
            if echo "$info" | grep -qE "Intel IPU6|IPU6 CSI2|IPU6 ISYS|Intel IPU6 ISYS Capture"; then
                IPU_MEDIA="$m"
                break
            fi
        done

        if [ -n "$IPU_MEDIA" ]; then
            echo "✅ 找到 IPU6 Media 節點: $IPU_MEDIA"
            return 0
        fi

        sleep 0.5
    done

    echo "❌ 找不到 Intel IPU6 media 節點"
    ls -l /dev/media* 2>/dev/null || true
    exit 1
}

apply_ipu6_tuning() {
    log_section "套用 IPU6 tuning"
    local deskew_path="/sys/module/intel_ipu6_isys/parameters/dwc_deskew_polarity_rw_override"
    if [ -w "$deskew_path" ]; then
        echo 0 > "$deskew_path"
        echo "✅ dwc_deskew_polarity_rw_override=0"
    else
        echo "⚠️ 找不到或無法寫入 $deskew_path，略過"
    fi
}

detect_topology_csi_ports() {
    # Parse max96724 entity blocks and extract actual connected Intel IPU6 CSI2 ports.
    # Example line:
    #   -> "Intel IPU6 CSI2 4":0 [ENABLED,IMMUTABLE]
    local ports

    if [ -z "${IPU_MEDIA:-}" ]; then
        echo ""
        return 0
    fi

    ports="$(
        media-ctl -p -d "$IPU_MEDIA" 2>/dev/null | \
        awk -v RS= -v dev="$DEV" '
        $0 ~ ("- entity [0-9]+: " dev "([[:space:]]|\\()") {
            while (match($0, /Intel IPU6 CSI2 [0-9]+/)) {
                s = substr($0, RSTART, RLENGTH)
                sub(/^Intel IPU6 CSI2 /, "", s)
                print s
                $0 = substr($0, RSTART + RLENGTH)
            }
        }' | sort -n -u | xargs
    )"

    echo "$ports"
}

apply_dynamic_csi_ports() {
    local ports="$1"
    local csi base i capnum

    CAM_NAMES=()
    CSI_GROUPS=()
    CAPTURE_BASES=()

    for csi in $ports; do
        base=$((csi * 8))
        CSI_GROUPS+=("$csi")
        CAPTURE_BASES+=("$base")

        for i in 0 1 2 3; do
            capnum=$((base + i))
            CAM_NAMES+=("$csi-$capnum")
        done
    done

    DEFAULT_ACTIVE_CAMS="${CAM_NAMES[*]}"
}

read_dmi_value() {
    local f="$1"
    [ -r "$f" ] && cat "$f" 2>/dev/null || true
}

detect_dmi_profile() {
    local vendor product board bios all
    vendor="$(read_dmi_value /sys/class/dmi/id/sys_vendor)"
    product="$(read_dmi_value /sys/class/dmi/id/product_name)"
    board="$(read_dmi_value /sys/class/dmi/id/board_name)"
    bios="$(read_dmi_value /sys/class/dmi/id/bios_version)"
    all="$vendor $product $board $bios"

    echo "DMI vendor : ${vendor:-unknown}" >&2
    echo "DMI product: ${product:-unknown}" >&2
    echo "DMI board  : ${board:-unknown}" >&2

    # From mipi-upboard.h:
    #   UPX-MTL01 / UPX-ARL01 use the i14/MTL/ARL path.
    # User confirmed this should use CSI2 0 + 4.
    if echo "$all" | grep -qiE "UPX-MTL01|UPX-ARL01|i14|MTL|ARL|Meteor|Arrow"; then
        echo "current"
        return 0
    fi

    # Other UP boards in your current usage should use CSI2 1 + 2 as fallback.
    if echo "$all" | grep -qiE "AAEON|UPX-ADLP01|UPS-ADLP01|UPN-ADLN01|UPN-TWL01|UPS-TWL01|UPN-ASLH01|UPN-EDGE-ASLH01|UPS-ASL01|ADL|Alder"; then
        echo "alderlake"
        return 0
    fi

    echo "alderlake"
}

detect_camera_profile() {
    local requested="$CAMERA_PROFILE"
    local ports=""
    local fallback=""

    if [ "$requested" = "auto" ] || [ "$requested" = "topology" ]; then
        ports="$(detect_topology_csi_ports)"

        if [ -n "$ports" ]; then
            CAMERA_PROFILE="topology"
            apply_dynamic_csi_ports "$ports"
            echo "✅ Camera profile: topology"
            echo "   max96724 connected CSI ports: $ports"
            echo "   CAM_NAMES: ${CAM_NAMES[*]}"
            echo "   CSI_GROUPS: ${CSI_GROUPS[*]}"
            echo "   CAPTURE_BASES: ${CAPTURE_BASES[*]}"

            if [ -z "${ACTIVE_CAMS:-}" ]; then
                ACTIVE_CAMS="$DEFAULT_ACTIVE_CAMS"
            fi
            return 0
        fi

        if [ "$requested" = "topology" ]; then
            echo "❌ CAMERA_PROFILE=topology 但 media graph 中找不到 $DEV -> Intel IPU6 CSI2 link"
            echo "請執行確認："
            echo "media-ctl -p -d $IPU_MEDIA | awk -v RS= '/- entity [0-9]+: ($DEV)/'"
            exit 1
        fi

        echo "⚠️ media topology 沒抓到 $DEV CSI port，fallback 到 DMI board 判斷"
        fallback="$(detect_dmi_profile)"
        CAMERA_PROFILE="$fallback"
    fi

    case "$CAMERA_PROFILE" in
        current|default|mtl|arl|arrowlake|meteorlake|i14)
            CAMERA_PROFILE="current"
            CAM_NAMES=("0-0" "0-1" "0-2" "0-3" "4-32" "4-33" "4-34" "4-35")
            CSI_GROUPS=(0 4)
            CAPTURE_BASES=(0 32)
            DEFAULT_ACTIVE_CAMS="4-35 0-1 0-2 0-3"
            ;;
        alderlake|adl|adl-n|other)
            CAMERA_PROFILE="alderlake"
            CAM_NAMES=("1-8" "1-9" "1-10" "1-11" "2-16" "2-17" "2-18" "2-19")
            CSI_GROUPS=(1 2)
            CAPTURE_BASES=(8 16)
            DEFAULT_ACTIVE_CAMS="1-8 1-9 1-10 1-11 2-16 2-17 2-18 2-19"
            ;;
        *)
            echo "❌ 不支援 CAMERA_PROFILE=$CAMERA_PROFILE"
            echo "可用：auto / topology / current / alderlake"
            exit 1
            ;;
    esac

    if [ -z "${ACTIVE_CAMS:-}" ]; then
        ACTIVE_CAMS="$DEFAULT_ACTIVE_CAMS"
    fi

    echo "✅ Camera profile: $CAMERA_PROFILE"
    echo "   CAM_NAMES: ${CAM_NAMES[*]}"
    echo "   CSI_GROUPS: ${CSI_GROUPS[*]}"
    echo "   CAPTURE_BASES: ${CAPTURE_BASES[*]}"
}

setup_media_pipeline() {
    log_section "設定 Media Pipeline"
    if [ -z "${IPU_MEDIA:-}" ]; then
        echo "❌ IPU_MEDIA 是空的，不能設定 pipeline"
        exit 1
    fi

    CAP_BY_NAME=()

    media-ctl -d "$IPU_MEDIA" -r || true
    media-ctl -d "$IPU_MEDIA" -V "'$DEV':0 [fmt:$FMT/${RESOLUTION} field:none]" || true

    local csi base i pad capnum name capdev gi

    for csi in "${CSI_GROUPS[@]}"; do
        for pad in 0 1 2 3 4; do
            media-ctl -d "$IPU_MEDIA" -V "'Intel IPU6 CSI2 $csi':$pad [fmt:$FMT/${RESOLUTION} field:none]" || true
        done
    done

    for gi in "${!CSI_GROUPS[@]}"; do
        csi="${CSI_GROUPS[$gi]}"
        base="${CAPTURE_BASES[$gi]}"

        for i in 0 1 2 3; do
            pad=$((i + 1))
            capnum=$((base + i))
            media-ctl -d "$IPU_MEDIA" -l "\"Intel IPU6 CSI2 $csi\":$pad -> \"Intel IPU6 ISYS Capture $capnum\":0[1]" || true
        done
    done

    for name in "${CAM_NAMES[@]}"; do
        capnum="${name#*-}"
        capdev="$(media-ctl -d "$IPU_MEDIA" -e "Intel IPU6 ISYS Capture $capnum" 2>/dev/null || true)"
        CAP_BY_NAME["$name"]="$capdev"
    done

    echo "========== Capture Devices =========="
    for name in "${CAM_NAMES[@]}"; do
        echo "$name -> ${CAP_BY_NAME[$name]:-}"
    done
}

cap_for_name() {
    echo "${CAP_BY_NAME[$1]:-}"
}

fifo_for_name() {
    echo "${FIFO_BY_NAME[$1]:-}"
}

fd_for_name() {
    echo "${FD_BY_NAME[$1]:-}"
}

is_active_cam() {
    local name="$1"
    for cam in $ACTIVE_CAMS; do
        if [ "$cam" = "$name" ]; then
            return 0
        fi
    done
    return 1
}

probe_one_cam() {
    local name="$1" cap="$2" log raw size
    log="${PROBE_DIR}/${name//-/_}_probe.log"
    raw="${PROBE_DIR}/${name//-/_}_probe.raw"

    echo "🔍 偵測 $name: $cap"

    if [ -z "$cap" ] || [ ! -e "$cap" ]; then
        echo "⏸️ $name capture device 不存在，視為不可 stream"
        return 1
    fi

    rm -f "$raw"

    if timeout "$PROBE_TIMEOUT" v4l2-ctl -d "$cap" \
        --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=$V4L2_FMT \
        --stream-mmap \
        --stream-count="$PROBE_COUNT" \
        --stream-to="$raw" \
        >"$log" 2>&1; then

        size="$(stat -c%s "$raw" 2>/dev/null || echo 0)"
        if [ "$size" -ge "$FRAME_BYTES" ]; then
            echo "✅ $name 可 stream，probe bytes=$size"
            rm -f "$raw"
            return 0
        fi

        echo "⏸️ $name probe 成功但資料不足：bytes=$size，視為不可 stream"
    else
        echo "⏸️ $name probe 失敗或 timeout，視為不可 stream"
    fi

    if [ "$KEEP_LOG" = "1" ]; then
        echo "   probe log: $log"
    fi

    rm -f "$raw"
    return 1
}

auto_detect_cams() {
    log_section "自動偵測可 stream 相機"
    PROBE_DIR="/tmp/eight_cam_probe_$$"
    mkdir -p "$PROBE_DIR"

    local detected="" name cap

    for name in "${CAM_NAMES[@]}"; do
        cap="$(cap_for_name "$name")"
        if probe_one_cam "$name" "$cap"; then
            detected+="$name "
        fi
        sleep 0.1
    done

    detected="$(echo "$detected" | xargs || true)"
    ACTIVE_CAMS="$detected"
    echo "✅ 偵測完成：ACTIVE_CAMS=\"$ACTIVE_CAMS\""
    echo "⏳ 等待 ${POST_PROBE_SETTLE}s 讓前一次 probe streamoff 完成..."
    sleep "$POST_PROBE_SETTLE"
}

start_loop_writer_if_active() {
    local name="$1" cap fifo safe_name log
    cap="$(cap_for_name "$name")"
    fifo="$(fifo_for_name "$name")"
    safe_name="${name//-/_}"
    log="${FIFO_DIR}/${safe_name}_v4l2.log"

    if is_active_cam "$name"; then
        if [ -z "$cap" ] || [ ! -e "$cap" ]; then
            echo "⚠️ active $name 但 capture device 不存在：$cap，改顯示 WAIT"
            return 0
        fi

        echo "🚀 start persistent writer $name: $cap -> $fifo"
        (
            trap 'exit 0' INT TERM
            echo "[$(date '+%F %T')] writer loop start: $name $cap" >>"$log"
            while true; do
                v4l2-ctl -d "$cap" \
                    --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=$V4L2_FMT \
                    --stream-mmap \
                    --stream-count=0 \
                    --stream-to=- \
                    2>>"$log" || true
                echo "[$(date '+%F %T')] v4l2-ctl exited; restart after ${WRITER_RESTART_DELAY}s" >>"$log"
                sleep "$WRITER_RESTART_DELAY"
            done
        ) >"$fifo" &
        PIDS+=("$!")
        sleep "$STREAM_START_GAP"
    else
        echo "⏸️ inactive $name：GST 顯示 WAIT..."
    fi
}

open_read_fd_for_active() {
    local name="$1" fifo fd cap

    if ! is_active_cam "$name"; then
        return 0
    fi

    cap="$(cap_for_name "$name")"
    if [ -z "$cap" ] || [ ! -e "$cap" ]; then
        return 0
    fi

    fifo="$(fifo_for_name "$name")"
    echo "🔌 open blocking read FD for $name: $fifo"
    exec {fd}<"$fifo"
    FD_BY_NAME["$name"]="$fd"
    OPEN_FDS+=("$fd")
    echo "   $name -> fd $fd"
}

make_branch() {
    local name="$1" idx="$2" fd

    if is_active_cam "$name"; then
        fd="$(fd_for_name "$name")"
        if [ -n "$fd" ]; then
            if [ "$SHOW_LABEL" = "1" ]; then
                echo "fdsrc fd=$fd blocksize=$FRAME_BYTES do-timestamp=true ! videoparse format=$VIDEOPARSE_FMT width=$WIDTH height=$HEIGHT framerate=$FPS_NUM/$FPS_DEN ! queue $QUEUE_OPTS ! videoconvert ! videoscale ! video/x-raw,format=BGRx,width=$DISPLAY_W,height=$DISPLAY_H,framerate=$FPS_NUM/$FPS_DEN ! textoverlay text=\"$name\" valignment=top halignment=left font-desc=\"Sans, 18\" shaded-background=true ! queue $QUEUE_OPTS ! mix.sink_$idx"
            else
                echo "fdsrc fd=$fd blocksize=$FRAME_BYTES do-timestamp=true ! videoparse format=$VIDEOPARSE_FMT width=$WIDTH height=$HEIGHT framerate=$FPS_NUM/$FPS_DEN ! queue $QUEUE_OPTS ! videoconvert ! videoscale ! video/x-raw,format=BGRx,width=$DISPLAY_W,height=$DISPLAY_H,framerate=$FPS_NUM/$FPS_DEN ! queue $QUEUE_OPTS ! mix.sink_$idx"
            fi
            return 0
        fi
    fi

    echo "videotestsrc is-live=true pattern=black ! video/x-raw,format=BGRx,width=$DISPLAY_W,height=$DISPLAY_H,framerate=$FPS_NUM/$FPS_DEN ! textoverlay text=\"$name WAIT...\" valignment=center halignment=center font-desc=\"Sans, 28\" shaded-background=true ! queue $QUEUE_OPTS ! mix.sink_$idx"
}

make_fifos_for_attempt() {
    FIFO_DIR="/tmp/eight_cam_gst_stable_${$}_attempt_${1}"
    rm -rf "$FIFO_DIR"
    mkdir -p "$FIFO_DIR"

    FIFO_BY_NAME=()
    FD_BY_NAME=()

    local name safe fifo
    for name in "${CAM_NAMES[@]}"; do
        safe="${name//-/_}"
        fifo="$FIFO_DIR/cam_${safe}.fifo"
        FIFO_BY_NAME["$name"]="$fifo"
        mkfifo "$fifo"
    done
}

print_v4l2_short_logs() {
    echo ""
    echo "========== v4l2 short logs =========="
    if [ -n "${FIFO_DIR:-}" ] && [ -d "$FIFO_DIR" ]; then
        for f in "$FIFO_DIR"/*_v4l2.log; do
            [ -f "$f" ] || continue
            echo "--- $f ---"
            tail -n 8 "$f" || true
        done
    fi
    echo "====================================="
}

run_gst_once() {
    local attempt="$1"
    log_section "啟動 GStreamer 8 Camera Grid，第 ${attempt}/${GST_LAUNCH_RETRIES} 次"
    cleanup_attempt
    make_fifos_for_attempt "$attempt"

    local total_w=$((DISPLAY_W * 4))
    local total_h=$((DISPLAY_H * 2))

    echo "Camera profile: $CAMERA_PROFILE"
    echo "Active cameras: $ACTIVE_CAMS"
    echo "Input: ${WIDTH}x${HEIGHT} $V4L2_FMT -> videoparse $VIDEOPARSE_FMT ${FPS_NUM}/${FPS_DEN} fps"
    echo "Frame bytes: $FRAME_BYTES"
    echo "Tile: ${DISPLAY_W}x${DISPLAY_H}"
    echo "Canvas: ${total_w}x${total_h}"
    echo "Sink: $SINK sync=$SYNC"
    echo "SHOW_LABEL=$SHOW_LABEL"
    echo "Writer: persistent loop, restart delay=${WRITER_RESTART_DELAY}s"
    echo ""

    local name idx
    for name in "${CAM_NAMES[@]}"; do
        start_loop_writer_if_active "$name"
    done

    for name in "${CAM_NAMES[@]}"; do
        open_read_fd_for_active "$name"
    done

    local compositor="compositor name=mix background=black "
    for idx in "${!CAM_NAMES[@]}"; do
        local x=$((DISPLAY_W * (idx % 4)))
        local y=$((DISPLAY_H * (idx / 4)))
        compositor+="sink_${idx}::xpos=${x} sink_${idx}::ypos=${y} "
    done
    compositor+="! video/x-raw,format=BGRx,width=$total_w,height=$total_h,framerate=$FPS_NUM/$FPS_DEN ! queue $QUEUE_OPTS ! videoconvert ! $SINK sync=$SYNC"

    local pipeline="$compositor "
    for idx in "${!CAM_NAMES[@]}"; do
        pipeline+="$(make_branch "${CAM_NAMES[$idx]}" "$idx") "
    done

    echo "========== GST Pipeline =========="
    echo "$pipeline"
    echo "=================================="

    if [ "$FULLSCREEN" = "1" ]; then
        (
            for i in $(seq 1 50); do
                if command -v wmctrl >/dev/null 2>&1; then
                    wid="$(wmctrl -l | awk '/gst-launch-1.0/ {print $1; exit}')"
                    if [ -n "$wid" ]; then
                        wmctrl -i -r "$wid" -b add,fullscreen
                        exit 0
                    fi
                fi
                sleep 0.2
            done
        ) &
    fi

    local start_ts end_ts elapsed status
    start_ts="$(date +%s)"

    set +e
    eval "gst-launch-1.0 -e $pipeline"
    status=$?
    set -e

    end_ts="$(date +%s)"
    elapsed=$((end_ts - start_ts))

    if [ "$status" -ne 0 ]; then
        echo "⚠️ GST 以 status=$status 結束"
        print_v4l2_short_logs
        return "$status"
    fi

    if [ "$elapsed" -lt "$MIN_RUN_SECONDS" ]; then
        echo "⚠️ GST 在 ${elapsed}s 內結束，判定為啟動閃退"
        print_v4l2_short_logs
        return 99
    fi

    return 0
}

start_gst_grid_with_retries() {
    local try status
    for try in $(seq 1 "$GST_LAUNCH_RETRIES"); do
        if run_gst_once "$try"; then
            return 0
        fi

        status=$?
        if [ "$try" = "$GST_LAUNCH_RETRIES" ]; then
            echo "❌ GST 啟動重試 ${GST_LAUNCH_RETRIES} 次仍失敗"
            return "$status"
        fi

        echo "🔁 清理目前 writer/fd，等待後重新嘗試..."
        cleanup_attempt
        sleep 1
    done
}

main() {
    log_section "AAEOE 8 Camera v4l2 loop-writer FIFO FD + GStreamer Launcher"
    echo "Project: $SCRIPT_DIR"
    echo "Resolution: $RESOLUTION"
    echo "Format: $FMT / V4L2 $V4L2_FMT / videoparse $VIDEOPARSE_FMT"
    echo "Display per camera: ${DISPLAY_W}x${DISPLAY_H}"
    echo "AUTO_DETECT=$AUTO_DETECT"
    echo "CAMERA_PROFILE=$CAMERA_PROFILE"

    check_runtime
    apply_ipu6_tuning
    find_ipu_media
    detect_camera_profile
    setup_media_pipeline

    if [ "$AUTO_DETECT" = "1" ]; then
        auto_detect_cams
        echo "🔁 probe 完成後重新整理 media pipeline..."
        sleep "$POST_PROBE_SETTLE"
        setup_media_pipeline
    fi

    start_gst_grid_with_retries
}

main "$@"
