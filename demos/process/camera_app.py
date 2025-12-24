#!/usr/bin/env python3
import sys
import time
import argparse
import numpy as np
import cv2
import os       # [新增] 檔案路徑處理
import shutil   # [新增] 用來清空資料夾

# ==========================================
# 1. 感測器設定檔 (Sensor Profiles)
# ==========================================
ov_gain = 1.4
imx708_gain = 1.45
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
        "gamma": 2.2,
        "gains": (ov_gain*1.3, ov_gain*1.0, ov_gain*1.3),
        "bit_depth": 10
    },
    "imx708": {
        "width": 1536,
        "height": 864,
        "pattern": "BGGR",
        "black_level": 64,
        "gamma": 2.0,
        "gains": (imx708_gain*2.81, imx708_gain*2.0, imx708_gain*2.42),
        "bit_depth": 10
    }
}

# ==========================================
# 2. 通用常數與工具
# ==========================================
MODE_PICTURE = "picture"
MODE_LIVE = "live"
MODE_RECORD = "record"
MODE_BURST = "burst"

OPENCV_BAYER_CODE = {
    "RGGB": cv2.COLOR_BayerRG2BGR,
    "GRBG": cv2.COLOR_BayerGR2BGR,
    "GBRG": cv2.COLOR_BayerGB2BGR,
    "BGGR": cv2.COLOR_BayerBG2BGR,
}

def get_stride(width):
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
    print(f"[init] LUTs: Black={black_level}, Gamma={gamma}, Gains=({r_gain:.2f}, {g_gain:.2f}, {b_gain:.2f})", file=sys.stderr)
    x = np.arange(65536, dtype=np.float32)
    x = np.maximum(x - black_level, 0)
    max_val = (1 << bit_depth) - 1
    x = x / max_val
    if gamma > 0 and gamma != 1.0:
        x = np.power(x, 1.0 / gamma)
    lut_r = np.clip(x * r_gain * 255.0, 0, 255).astype(np.uint8)
    lut_g = np.clip(x * g_gain * 255.0, 0, 255).astype(np.uint8)
    lut_b = np.clip(x * b_gain * 255.0, 0, 255).astype(np.uint8)
    return lut_b, lut_g, lut_r

# ==========================================
# 3. 主程式
# ==========================================
def main():
    parser = argparse.ArgumentParser(description="Unified Camera App")
    parser.add_argument("--sensor", type=str, required=True, choices=["imx219", "ov5647", "imx708"])
    parser.add_argument("--mode", type=str, default=MODE_LIVE, choices=[MODE_PICTURE, MODE_LIVE, MODE_RECORD, MODE_BURST])
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--stride", type=int, default=0)
    parser.add_argument("--skip-frames", type=int, default=2)
    parser.add_argument("--burst-count", type=int, default=100, help="Total frames to capture in burst mode")

    args = parser.parse_args()

    # 1. 載入 Profile
    profile = SENSOR_PROFILES[args.sensor]
    width = args.width if args.width is not None else profile["width"]
    height = args.height if args.height is not None else profile["height"]
    pattern = profile["pattern"]
    stride = args.stride if args.stride > 0 else get_stride(width)
    frame_bytes = stride * height * 2
    resize_scale = get_resize_scale(width)

    # 輸出狀態
    print(f"========== [Camera App: {args.sensor}] ==========", file=sys.stderr)
    print(f"Mode: {args.mode} | Res: {width}x{height} | Scale: {resize_scale}", file=sys.stderr)

    # 建立 LUT
    lut_b, lut_g, lut_r = create_luts(profile["black_level"], profile["gains"][0], profile["gains"][1], profile["gains"][2], profile["gamma"], profile["bit_depth"])

    # ==========================================
    # [新增] Burst 模式資料夾初始化邏輯
    # ==========================================
    burst_dir = "burst"
    burst_timestamp = time.strftime("%Y%m%d-%H%M%S")
    
    if args.mode == MODE_BURST:
        if not os.path.exists(burst_dir):
            os.makedirs(burst_dir)
            print(f"[Burst] Created directory: {burst_dir}", file=sys.stderr)
        else:
            # 資料夾存在，清空內容
            print(f"[Burst] Clearing directory: {burst_dir} ...", file=sys.stderr)
            for filename in os.listdir(burst_dir):
                file_path = os.path.join(burst_dir, filename)
                try:
                    if os.path.isfile(file_path) or os.path.islink(file_path):
                        os.unlink(file_path)
                    elif os.path.isdir(file_path):
                        shutil.rmtree(file_path)
                except Exception as e:
                    print(f"Failed to delete {file_path}. Reason: {e}", file=sys.stderr)
            print(f"[Burst] Directory cleared.", file=sys.stderr)

    # IO Setup
    stdin_fd = sys.stdin.buffer
    stdout_fd = sys.stdout.buffer
    buf = b""
    fps_frames = 0
    fps_last_t = time.time()
    frame_counter = 0

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
        frame_counter += 1

        # Raw -> BGR
        raw_padded = np.frombuffer(current_frame, dtype="<u2").reshape(height, stride)
        raw16 = raw_padded[:, :width]
        bgr16 = cv2.cvtColor(raw16, OPENCV_BAYER_CODE[pattern])

        # Resize (連拍模式為了效能，建議維持縮放)
        if resize_scale != 1.0:
            bgr16 = cv2.resize(bgr16, (0, 0), fx=resize_scale, fy=resize_scale, interpolation=cv2.INTER_NEAREST)

        # LUT Apply
        out_b = lut_b[bgr16[:,:,0]]
        out_g = lut_g[bgr16[:,:,1]]
        out_r = lut_r[bgr16[:,:,2]]
        disp = np.dstack((out_b, out_g, out_r))

        # --- 模式處理 ---
        if args.mode == MODE_BURST:
            # 存入 burst/ 資料夾
            filename = os.path.join(burst_dir, f"burst{frame_counter:03d}.jpg")
            cv2.imwrite(filename, disp)
            # 使用 \r 覆蓋輸出，避免刷屏
            print(f"\r[Burst] Saved to {burst_dir}/: {frame_counter}/{args.burst_count}", end="", file=sys.stderr)

        elif args.mode == MODE_PICTURE:
            if frame_counter <= args.skip_frames:
                print(".", end="", file=sys.stderr, flush=True)
                continue
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f"{args.sensor}_{width}x{height}_{timestamp}.jpg"
            cv2.imwrite(filename, disp)
            print(f"\nSaved: {filename}", file=sys.stderr)
            break

        elif args.mode == MODE_LIVE:
            fps_frames += 1
            now = time.time()
            if now - fps_last_t >= 1.0:
                fps_val = fps_frames / (now - fps_last_t)
                fps_frames = 0
                fps_last_t = now
            cv2.putText(disp, f"FPS: {fps_val:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow(f"{args.sensor} Live", disp)
            if cv2.waitKey(1) & 0xFF == 27: 
                break

        elif args.mode == MODE_RECORD:
            try:
                stdout_fd.write(disp.tobytes())
            except BrokenPipeError:
                break

    if args.mode == MODE_LIVE:
        cv2.destroyAllWindows()
    if args.mode == MODE_BURST:
        print("\n[Burst] Capture completed.", file=sys.stderr)

if __name__ == "__main__":
    main()