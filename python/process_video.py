# process_video.py
# Frame-by-frame video processing through Tang Nano 20K FPGA via UART
import os
import sys
import time
import cv2
import numpy as np
import serial
import serial.tools.list_ports

BAUD_RATE = 1350000
CHUNK_SIZE = 64
CHUNK_TIMEOUT = 1.0

def process_single_frame(ser, frame_gray, threshold, width, height, send_header=True):
    """Sends a single frame to FPGA and returns the edge detected frame."""
    total_pixels = width * height
    
    if send_header:
        # Flush buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.25) # Allow FPGA idle timer to reset to CFG_STATE_THR
        ser.reset_input_buffer()
        
        # 5-byte configuration header
        header = bytes([
            threshold,
            (width >> 8) & 0xFF,
            width & 0xFF,
            (height >> 8) & 0xFF,
            height & 0xFF
        ])
        ser.write(header)
        
        # Wait for echo
        echo = bytearray()
        deadline = time.time() + 1.5
        while len(echo) < 5 and time.time() < deadline:
            c = ser.read(5 - len(echo))
            if c:
                echo.extend(c)
        if len(echo) != 5 or bytes(echo) != header:
            raise RuntimeError(f"FPGA header echo failed! Expected {list(header)}, got {list(echo)}")
            
    # Send pixels in 64-byte chunks
    pixel_bytes = frame_gray.tobytes()
    received = bytearray()
    
    for offset in range(0, total_pixels, CHUNK_SIZE):
        chunk = pixel_bytes[offset:offset + CHUNK_SIZE]
        ser.write(chunk)
        
        buf = bytearray()
        deadline = time.time() + CHUNK_TIMEOUT
        while len(buf) < len(chunk) and time.time() < deadline:
            c = ser.read(len(chunk) - len(buf))
            if c:
                buf.extend(c)
        if len(buf) != len(chunk):
            raise TimeoutError(f"FPGA timeout at offset {offset}")
        received.extend(buf)
        
    raw = np.frombuffer(bytes(received[:total_pixels]), dtype=np.uint8).reshape((height, width))
    aligned = np.zeros_like(raw)
    aligned[:-2, :-2] = raw[2:, 2:]
    return aligned

def process_video_file(video_path, port, output_path="output_video.mp4", 
                       width=128, height=128, threshold=80, 
                       max_frames=60, frame_stride=1):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video file {video_path}")
        return
        
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 20.0
    total_src_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"Opening video: {video_path} ({total_src_frames} frames, {src_fps:.1f} FPS)")
    print(f"Target Resolution: {width}x{height}, Threshold: {threshold}")
    print(f"Connecting to FPGA on {port} @ {BAUD_RATE} baud...")
    
    ser = serial.Serial(port=port, baudrate=BAUD_RATE, timeout=0.05)
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, src_fps / frame_stride, (width, height), isColor=False)
    
    frame_idx = 0
    processed_count = 0
    start_total_time = time.time()
    
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret or (max_frames > 0 and processed_count >= max_frames):
                break
                
            if frame_idx % frame_stride != 0:
                frame_idx += 1
                continue
                
            frame_idx += 1
            processed_count += 1
            
            # Convert to grayscale and resize
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            resized = cv2.resize(gray, (width, height), interpolation=cv2.INTER_AREA)
            
            t0 = time.time()
            edge = process_single_frame(ser, resized, threshold, width, height, send_header=True)
            elapsed = time.time() - t0
            
            out.write(edge)
            fps_live = 1.0 / elapsed if elapsed > 0 else 0
            print(f"Frame {processed_count}/{max_frames or total_src_frames}: {elapsed:.2f}s ({fps_live:.2f} FPS)")
            
    finally:
        cap.release()
        out.release()
        ser.close()
        total_time = time.time() - start_total_time
        print(f"Video processing finished in {total_time:.1f}s. Saved to: {output_path}")

if __name__ == "__main__":
    port = "COM6"
    test_video = os.path.join(os.path.dirname(__file__), "..", "sample_video.mp4")
    if len(sys.argv) > 1:
        test_video = sys.argv[1]
    if len(sys.argv) > 2:
        port = sys.argv[2]
    process_video_file(test_video, port, max_frames=20)
