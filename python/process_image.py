# process_image.py
# Sends an image over UART to Tang Nano 20K FPGA with Gaussian Noise Reduction
# and Dynamic Runtime Thresholding (256x256).

import os
import sys
import time
import numpy as np
from PIL import Image, ImageOps
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt

IMG_WIDTH = 512
IMG_HEIGHT = 512
TOTAL_PIXELS = IMG_WIDTH * IMG_HEIGHT
BAUD_RATE = 1350000
DEFAULT_THRESHOLD = 80
CHUNK_SIZE = 64
CHUNK_TIMEOUT = 1.0
FRAME_SYNC_GAP = 0.4
MAX_RETRIES = 3


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("[!] No serial COM ports detected.")
        return []
    print("\n--- Available COM Ports ---")
    for i, p in enumerate(ports):
        print(f" [{i}] {p.device}: {p.description}")
    return ports


def prepare_image(image_path, preserve_aspect=True):
    if not os.path.exists(image_path):
        print(f"[!] Error: File '{image_path}' not found!")
        sys.exit(1)

    img = Image.open(image_path).convert("L")
    if preserve_aspect:
        img.thumbnail((IMG_WIDTH, IMG_HEIGHT), Image.Resampling.LANCZOS)
        dw = IMG_WIDTH - img.size[0]
        dh = IMG_HEIGHT - img.size[1]
        padding = (dw // 2, dh // 2, dw - (dw // 2), dh - (dh // 2))
        img_resized = ImageOps.expand(img, padding, fill=0)
    else:
        img_resized = img.resize((IMG_WIDTH, IMG_HEIGHT), Image.Resampling.LANCZOS)

    return np.array(img_resized, dtype=np.uint8)


def reference_gaussian_sobel(img, threshold):
    """Bit-exact software model: 3x3 Gaussian Blur -> 3x3 Sobel."""
    p = img.astype(np.int32)
    g_sum = (
        p[:-2, :-2] + 2 * p[:-2, 1:-1] + p[:-2, 2:] +
        2 * p[1:-1, :-2] + 4 * p[1:-1, 1:-1] + 2 * p[1:-1, 2:] +
        p[2:, :-2] + 2 * p[2:, 1:-1] + p[2:, 2:]
    )
    smooth = np.zeros_like(img)
    smooth[1:-1, 1:-1] = (g_sum // 16).astype(np.uint8)

    sp = smooth.astype(np.int32)
    gx = (sp[:-2, 2:] + 2 * sp[1:-1, 2:] + sp[2:, 2:]) - (sp[:-2, :-2] + 2 * sp[1:-1, :-2] + sp[2:, :-2])
    gy = (sp[:-2, :-2] + 2 * sp[:-2, 1:-1] + sp[:-2, 2:]) - (sp[2:, :-2] + 2 * sp[2:, 1:-1] + sp[2:, 2:])
    mag = np.abs(gx) + np.abs(gy)

    out = np.zeros_like(img)
    out[1:-1, 1:-1] = np.where(mag > threshold, 255, 0).astype(np.uint8)
    return out


def read_exact(ser, n, timeout):
    buf = bytearray()
    deadline = time.time() + timeout
    while len(buf) < n and time.time() < deadline:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


def transfer_frame(ser, pixel_bytes, threshold):
    ser.reset_input_buffer()
    time.sleep(FRAME_SYNC_GAP)
    ser.reset_input_buffer()

    # Send threshold byte and verify echo
    ser.write(bytes([threshold]))
    thr_echo = ser.read(1)
    if not thr_echo or thr_echo[0] != threshold:
        return bytearray(), False, 0

    received = bytearray()
    for offset in range(0, TOTAL_PIXELS, CHUNK_SIZE):
        chunk = pixel_bytes[offset:offset + CHUNK_SIZE]
        ser.write(chunk)
        echo = read_exact(ser, len(chunk), CHUNK_TIMEOUT)
        received.extend(echo)

        if len(echo) != len(chunk):
            return received, False, offset

        if (offset // CHUNK_SIZE) % 32 == 0:
            pct = 100 * len(received) / TOTAL_PIXELS
            sys.stdout.write(f"\r  [Progress] {len(received)}/{TOTAL_PIXELS} bytes ({pct:5.1f}%)")
            sys.stdout.flush()

    sys.stdout.write(f"\r  [Progress] {TOTAL_PIXELS}/{TOTAL_PIXELS} bytes (100.0%)\n")
    return received, True, None


def main():
    print("==================================================")
    print("  Tang Nano 20K FPGA Gaussian + Sobel (256x256)   ")
    print("==================================================")

    ports = list_serial_ports()
    if not ports:
        return

    names = [p.device for p in ports]
    default_port = "COM6" if "COM6" in names else names[0]
    port_input = input(f"\nEnter COM port (default: {default_port}): ").strip()
    selected_port = port_input if port_input else default_port

    default_img = "test_image.png"
    img_input = input(f"Enter image path (default: {default_img}): ").strip().strip('"')
    image_path = img_input if img_input else default_img

    thr_input = input(f"Enter threshold 20-200 (default: {DEFAULT_THRESHOLD}): ").strip()
    threshold = int(thr_input) if thr_input.isdigit() else DEFAULT_THRESHOLD

    pixel_array = prepare_image(image_path)
    pixel_bytes = pixel_array.tobytes()

    print(f"[+] Connecting to {selected_port}...")
    ser = serial.Serial(port=selected_port, baudrate=BAUD_RATE, timeout=0.05)

    data, ok, _ = transfer_frame(ser, pixel_bytes, threshold)
    ser.close()

    if not ok:
        print("[!] Transfer failed.")
        return

    raw = np.frombuffer(bytes(data[:TOTAL_PIXELS]), dtype=np.uint8).reshape((IMG_HEIGHT, IMG_WIDTH))
    aligned = np.zeros_like(raw)
    aligned[:-2, :-2] = raw[2:, 2:]

    ref = reference_gaussian_sobel(pixel_array, threshold)
    interior = (slice(2, -2), slice(2, -2))
    mismatches = int(np.count_nonzero(aligned[interior] != ref[interior]))
    total = (IMG_HEIGHT - 4) * (IMG_WIDTH - 4)

    print(f"[+] FPGA vs Python reference: {mismatches} mismatches ({100 * (total - mismatches) / total:.2f}% match)")
    Image.fromarray(aligned).save("output_filtered_edge.png")
    print("[+] Saved to 'output_filtered_edge.png'")


if __name__ == "__main__":
    main()
