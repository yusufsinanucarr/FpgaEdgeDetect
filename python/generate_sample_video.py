# generate_sample_video.py
# Generates a clean synthetic video for testing FPGA video edge detection
import cv2
import numpy as np
import os

def create_sample_video(output_path="sample_video.mp4", duration_sec=4, fps=20, width=256, height=256):
    total_frames = duration_sec * fps
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height), isColor=False)
    
    center_x, center_y = width // 2, height // 2
    
    for i in range(total_frames):
        # Create dark background
        frame = np.zeros((height, width), dtype=np.uint8)
        frame[:] = 20 # Dark gray background
        
        # 1. Bouncing / Moving Circle
        t = i / total_frames * 2 * np.pi
        circle_x = int(center_x + (width // 3.5) * np.cos(t))
        circle_y = int(center_y + (height // 3.5) * np.sin(t * 2))
        cv2.circle(frame, (circle_x, circle_y), width // 8, 220, -1)
        cv2.circle(frame, (circle_x, circle_y), (width // 8) - 6, 80, -1)
        
        # 2. Rotating Rectangle in Center
        angle = (i * 360 / total_frames)
        rect = ((center_x, center_y), (width // 3, height // 3), angle)
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.drawContours(frame, [box], 0, 255, 2)
        
        # 3. Dynamic Text
        cv2.putText(
            frame, "FPGA VIDEO", 
            (width // 6, height - 15), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, 200, 1, cv2.LINE_AA
        )
        
        # 4. Corner ticks
        cv2.rectangle(frame, (8, 8), (width - 8, height - 8), 160, 1)

        out.write(frame)

    out.release()
    print(f"Sample video created: {output_path} ({total_frames} frames, {width}x{height} @ {fps} FPS)")

if __name__ == "__main__":
    target = os.path.join(os.path.dirname(__file__), "..", "sample_video.mp4")
    create_sample_video(target, duration_sec=3, fps=15, width=128, height=128)
