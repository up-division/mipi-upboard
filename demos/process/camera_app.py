#!/usr/bin/env python3
import sys
import time
import argparse
import numpy as np
import cv2

# ==========================================
# 1. 感測器設定檔 (Sensor Profiles)
# ==========================================
ov_gain = 2.2
SENSOR_PROFILES = {
    "imx219": {
        "width": 3280,
        "height": 2464,
        "pattern": "BGGR",
        "black_level": 64,
        "gamma": 2.0,
        "gains": (1.28*2.78, 1.28*2.0, 1.28*2.42), # R, G, B
        "bit_depth": 10
    },
    "ov5647": {
        "width": 2592,
        "height": 1944,
        "pattern": "RGGB", 
        "black_level": 0,  
        "gamma": 1.7,      
        "gains": (ov_gain*1.2, ov_gain*1.0, ov_gain*1.2), 
        "bit_depth": 10
    },
    "imx708": {
        "width": 1536,     
        "height": 864,
        "pattern": "BGGR",
        "black_level": 64,
        "gamma": 2.2,
        "gains": (1.2*2.81, 1.2*2.0, 1.2*2.42),
        "bit_depth": 10
    }
}

# ==========================================
# 2. 通用常數與工具
# ==========================================
MODE_PICTURE = "picture"
MODE_LIVE = "live"
MODE_RECORD = "record"

OPENCV_BAYER_CODE = {
    "RGGB": cv2.COLOR_BayerRG2BGR,
    "GRBG": cv2.COLOR_BayerGR2BGR,
    "GBRG": cv2.COLOR_BayerGB2BGR,
    "BGGR": cv2.COLOR_BayerBG2BGR,
}


def get_stride(width):
    # Intel IPU6 64-byte alignment
    line_bytes = width * 2 
    if line_bytes % 64 != 0:
        aligned_bytes = (line_bytes + 63) // 64 * 64
        return aligned_bytes // 2
    return width

def get_resize_scale(w):
    if w > 3000: return 0.25 
    elif w > 1000: return 0.5 
    return 1.0

def create_luts(black_level, r_gain, g_gain, b_gain, gamma, bit_depth):
    # 輸出到 stderr
    print(f"[init] LUTs: Black={black_level}, Gamma={gamma}, Gains=({r_gain:.2f}, {g_gain:.2f}, {b_gain:.2f})", file=sys.stderr)
    
    x = np.arange(65536, dtype=np.float32)
    x = np.maximum(x - black_level, 0)
    max_val = (1 << bit_depth) - 1
    x = x / max_val
    
    # gamma計算
    if gamma > 0 and gamma != 1.0:
        x = np.power(x, 1.0 / gamma)

    lut_r = np.clip(x * r_gain * 255.0, 0, 255).astype(np.uint8)
    lut_g = np.clip(x * g_gain * 255.0, 0, 255).astype(np.uint8)
    lut_b = np.clip(x * b_gain * 255.0, 0, 255).astype(np.uint8)
    return lut_r, lut_g, lut_b

# ==========================================
# 3. 主程式
# ==========================================
def main():
    parser = argparse.ArgumentParser(description="Unified Camera App")
    
    parser.add_argument("--sensor", type=str, required=True, 
                        choices=["imx219", "ov5647", "imx708"],
                        help="Choose sensor profile")
    
    parser.add_argument("--mode", type=str, default=MODE_LIVE, 
                        choices=[MODE_PICTURE, MODE_LIVE, MODE_RECORD])
    
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--stride", type=int, default=0)
    
    # 增加一個參數讓我們可以手動調整要跳過幾幀 (預設 3)
    parser.add_argument("--skip-frames", type=int, default=3, help="Frames to skip before capture")

    args = parser.parse_args()

    # 1. 載入 Profile
    profile = SENSOR_PROFILES[args.sensor]
    
    width = args.width if args.width is not None else profile["width"]
    height = args.height if args.height is not None else profile["height"]
    pattern = profile["pattern"]
    
    r_gain = profile["gains"][0]
    g_gain = profile["gains"][1]
    b_gain = profile["gains"][2]
    
    gamma = profile["gamma"]
    black_level = profile["black_level"]
    bit_depth = profile["bit_depth"]

    stride = args.stride if args.stride > 0 else get_stride(width)
    frame_bytes = stride * height * 2
    resize_scale = get_resize_scale(width)

    # 輸出狀態
    print(f"========== [Camera App: {args.sensor}] ==========", file=sys.stderr)
    print(f"Mode: {args.mode} | Res: {width}x{height} | Pattern: {pattern}", file=sys.stderr)
    print(f"Stride: {stride} | Scale: {resize_scale}", file=sys.stderr)
    if args.mode == MODE_PICTURE:
        print(f"Skipping first {args.skip_frames} frames...", file=sys.stderr)

    # 建立 LUT
    lut_r, lut_g, lut_b = create_luts(black_level, r_gain, g_gain, b_gain, gamma, bit_depth)

    # IO Setup
    stdin_fd = sys.stdin.buffer
    stdout_fd = sys.stdout.buffer
    buf = b""
    
    fps_frames = 0
    fps_last_t = time.time()
    fps_val = 0.0

    # 用來計算目前跑到第幾幀
    frame_counter = 0
    record_start_time = None

    while True:
        try:
            while len(buf) < frame_bytes:
                chunk = stdin_fd.read(frame_bytes - len(buf))
                if not chunk: return
                buf += chunk
        except KeyboardInterrupt:
            break
        except Exception:
            return

        current_frame = buf[:frame_bytes]
        buf = buf[frame_bytes:]

        # Raw -> BGR
        raw_padded = np.frombuffer(current_frame, dtype="<u2").reshape(height, stride)
        raw16 = raw_padded[:, :width]
        bgr16 = cv2.cvtColor(raw16, OPENCV_BAYER_CODE[pattern])

        # Resize
        if resize_scale != 1.0:
            bgr16 = cv2.resize(bgr16, (0, 0), fx=resize_scale, fy=resize_scale, interpolation=cv2.INTER_NEAREST)

        # LUT Apply
        out_b = lut_b[bgr16[:,:,0]]
        out_g = lut_g[bgr16[:,:,1]]
        out_r = lut_r[bgr16[:,:,2]]
        disp = np.dstack((out_b, out_g, out_r))

        # FPS Stats
        fps_frames += 1
        now = time.time()
        if now - fps_last_t >= 1.0:
            fps_val = fps_frames / (now - fps_last_t)
            if args.mode != MODE_RECORD:
                print(f"\rFPS: {fps_val:.2f} ", end="", file=sys.stderr)
            fps_frames = 0
            fps_last_t = now

        # Modes Handling
        if args.mode == MODE_PICTURE:
            frame_counter += 1
            # 顯示進度點
            if frame_counter <= args.skip_frames:
                print(".", end="", file=sys.stderr, flush=True)
                continue # 跳過這一幀，不存檔

            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f"{args.sensor}_{width}x{height}_{timestamp}.jpg"
            cv2.imwrite(filename, disp)
            print(f"\nSaved: {filename} (Frame #{frame_counter})", file=sys.stderr)
            break

        elif args.mode == MODE_LIVE:
            cv2.putText(disp, f"FPS: {fps_val:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow(f"{args.sensor} {width}x{height} Live", disp)
            if cv2.waitKey(1) & 0xFF == 27: 
                break

        elif args.mode == MODE_RECORD:
            try:
                stdout_fd.write(disp.tobytes())
            except BrokenPipeError:
                break

    if args.mode == MODE_LIVE:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()