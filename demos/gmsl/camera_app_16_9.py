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
    },
    "isx031": {
        "width": 1920,
        "height": 1536,
        "pattern": "YUV422",
        "black_level": 0,
        "gamma": 1.0,
        "gains": (1.0, 1.0, 1.0),
        "bit_depth": 8 
    }
}

OPENCV_BAYER_CODE = {
    "RGGB": cv2.COLOR_BayerRG2BGR,
    "GRBG": cv2.COLOR_BayerGR2BGR,
    "GBRG": cv2.COLOR_BayerGB2BGR,
    "BGGR": cv2.COLOR_BayerBG2BGR,
}

# ==========================================
# 2. 通用工具函數
# ==========================================
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

def center_crop_to_aspect(frame, target_w=16, target_h=9):
    """
    將影像置中裁切成指定比例。
    例如 1920x1536 會裁成 1920x1080，符合 16:9。
    """
    h, w = frame.shape[:2]
    target_ratio = target_w / target_h
    current_ratio = w / h

    if abs(current_ratio - target_ratio) < 1e-3:
        return frame

    if current_ratio < target_ratio:
        # 畫面太高，裁掉上下
        new_h = int(w / target_ratio)
        y0 = max((h - new_h) // 2, 0)
        return frame[y0:y0 + new_h, :]
    else:
        # 畫面太寬，裁掉左右
        new_w = int(h * target_ratio)
        x0 = max((w - new_w) // 2, 0)
        return frame[:, x0:x0 + new_w]

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
    return lut_r, lut_g, lut_b

# ==========================================
# 3. 主程式 (Live Mode Only)
# ==========================================
def main():
    parser = argparse.ArgumentParser(description="Unified Camera App (Live Only)")
    parser.add_argument("--sensor", type=str, required=True, choices=["imx219", "ov5647", "imx708", "isx031"])
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--stride", type=int, default=0)
    args = parser.parse_args()

    # 載入 Profile
    profile = SENSOR_PROFILES[args.sensor]
    width = args.width if args.width is not None else profile["width"]
    height = args.height if args.height is not None else profile["height"]
    pattern = profile["pattern"]
    r_gain, g_gain, b_gain = profile["gains"]
    gamma = profile["gamma"]
    black_level = profile["black_level"]
    bit_depth = profile["bit_depth"]

    stride = args.stride if args.stride > 0 else get_stride(width)
    frame_bytes = stride * height * 2
    resize_scale = get_resize_scale(width)

    print(f"========== [Camera App: {args.sensor}] ==========", file=sys.stderr)
    print(f"Mode: Live | Res: {width}x{height} | Pattern: {pattern}", file=sys.stderr)
    print(f"Stride: {stride} | Scale: {resize_scale}", file=sys.stderr)

    # 建立 LUT
    lut_r, lut_g, lut_b = create_luts(black_level, r_gain, g_gain, b_gain, gamma, bit_depth)

    # IO Setup
    stdin_fd = sys.stdin.buffer
    buf = b""
    
    fps_frames = 0
    fps_last_t = time.time()
    fps_val = 0.0

    print(">> Waiting for video stream...", file=sys.stderr)

    while True:
        try:
            # 確保讀滿一個 Frame 的位元組
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


        if pattern == "YUV422":
            # YUV422 (UYVY) 是 8-bit，每個像素對應 2 Bytes。
            # OpenCV 轉換 UYVY 需要 (height, width, 2) 的 3D 陣列結構
            yuv = np.frombuffer(current_frame, dtype=np.uint8).reshape(height, stride, 2)
            # 裁切掉 padding 的部分 (只取實際寬度)
            yuv = yuv[:, :width, :]
            # UYVY 轉 BGR
            disp = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_UYVY)

            # 固定輸入 1920x1536 時，顯示前置中裁切成 16:9，例如 1920x1080
            disp = center_crop_to_aspect(disp, 16, 9)
            
            # 縮放處理
            if resize_scale != 1.0:
                disp = cv2.resize(disp, (0, 0), fx=resize_scale, fy=resize_scale, interpolation=cv2.INTER_NEAREST)
                
        else:
            # === 原本的 RAW Bayer 處理邏輯 ===
            raw_padded = np.frombuffer(current_frame, dtype="<u2").reshape(height, stride)
            raw16 = raw_padded[:, :width]
            bgr16 = cv2.cvtColor(raw16, OPENCV_BAYER_CODE[pattern])

            if resize_scale != 1.0:
                bgr16 = cv2.resize(bgr16, (0, 0), fx=resize_scale, fy=resize_scale, interpolation=cv2.INTER_NEAREST)

            out_b = lut_b[bgr16[:,:,0]]
            out_g = lut_g[bgr16[:,:,1]]
            out_r = lut_r[bgr16[:,:,2]]
            disp = np.dstack((out_b, out_g, out_r))

            # 顯示前置中裁切成 16:9
            disp = center_crop_to_aspect(disp, 16, 9)

        # 計算 FPS
        fps_frames += 1
        now = time.time()
        if now - fps_last_t >= 1.0:
            fps_val = fps_frames / (now - fps_last_t)
            print(f"\rFPS: {fps_val:.2f} ", end="", file=sys.stderr)
            fps_frames = 0
            fps_last_t = now

        # 顯示畫面
        cv2.putText(disp, f"FPS: {fps_val:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow(f"{args.sensor} {1920}x{1080} Live", disp)
        
        # 按下 ESC 鍵退出
        if cv2.waitKey(1) & 0xFF == 27: 
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()