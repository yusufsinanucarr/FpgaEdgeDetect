# gui_app.py
# Modern PyQt5 Desktop GUI for Tang Nano 20K FPGA
# Supports BOTH:
# 1) Photo Mode: Dynamic Multi-Resolution Edge Detection (128x128 up to 1024x1024 / 1 Megapixel)
# 2) Video Mode: Frame-by-frame video & webcam processing through FPGA with live playback & MP4 export
# 460,800 Baud High-Speed UART, Hardware Gaussian Blur & Dynamic Thresholding
# + Hardware Knight Rider LED Chaser during processing!

import os
import sys
import time
import numpy as np
from PIL import Image, ImageOps
import serial
import serial.tools.list_ports
import cv2

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QFileDialog, QProgressBar,
    QCheckBox, QGroupBox, QMessageBox, QSlider, QTabWidget, QSpinBox
)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, QThread, pyqtSignal

BAUD_RATE = 1350000    # Ultra High-Speed UART (1.35 Mbaud - 0.00% error on 27 MHz)
CHUNK_SIZE = 64         # 64-byte chunks (matches BL616 USB endpoint buffer)
CHUNK_TIMEOUT = 1.0
FRAME_SYNC_GAP = 0.25   # 250 ms sync gap (matches 200ms hardware idle timeout)

RESOLUTIONS_PHOTO = [
    ("128 x 128 (Çok Hızlı / ~4 sn)", 128, 128),
    ("256 x 256 (Hızlı & Dengeli / ~15 sn)", 256, 256),
    ("512 x 512 (Önerilen - Yüksek Detay / ~1 dk)", 512, 512),
    ("640 x 480 (VGA 4:3 / ~1.2 dk)", 640, 480),
    ("800 x 600 (SVGA 4:3 / ~1.9 dk)", 800, 600),
    ("1024 x 576 (16:9 Geniş Ekran / ~2.3 dk)", 1024, 576),
    ("1024 x 768 (XGA 4:3 / ~3 dk)", 1024, 768),
    ("1024 x 1024 (1 Megapiksel Ultra / ~4 dk)", 1024, 1024),
]

RESOLUTIONS_VIDEO = [
    ("64 x 64 (Ultra Hızlı / ~3.2 FPS)", 64, 64),
    ("128 x 128 (Önerilen Video / ~1.8 FPS)", 128, 128),
    ("160 x 120 (4:3 QWVGA / ~1.5 FPS)", 160, 120),
    ("256 x 256 (Yüksek Kalite / ~0.6 FPS)", 256, 256),
    ("320 x 240 (QVGA / ~0.5 FPS)", 320, 240),
]


def np_to_qpixmap(arr):
    h, w = arr.shape
    qimg = QImage(arr.data, w, h, w, QImage.Format_Grayscale8)
    return QPixmap.fromImage(qimg)


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


class FpgaPhotoWorker(QThread):
    progress_signal = pyqtSignal(int, str)
    finished_signal = pyqtSignal(np.ndarray, np.ndarray, int, float)
    error_signal = pyqtSignal(str)

    def __init__(self, port, pixel_array, threshold, width, height):
        super().__init__()
        self.port = port
        self.pixel_array = pixel_array
        self.threshold = threshold
        self.width = width
        self.height = height
        self.total_pixels = width * height

    def run(self):
        pixel_bytes = self.pixel_array.tobytes()
        try:
            ser = serial.Serial(port=self.port, baudrate=BAUD_RATE, timeout=0.05)
        except Exception as e:
            self.error_signal.emit(f"Port açılamadı ({self.port}):\n{e}")
            return

        try:
            self.progress_signal.emit(0, "FPGA senkronize ediliyor...")
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(FRAME_SYNC_GAP)
            ser.reset_input_buffer()

            # 1. Send 5-byte Configuration Header: [Threshold, W_hi, W_lo, H_hi, H_lo]
            header = bytes([
                self.threshold,
                (self.width >> 8) & 0xFF,
                self.width & 0xFF,
                (self.height >> 8) & 0xFF,
                self.height & 0xFF
            ])
            ser.write(header)

            # Wait for 5-byte echo
            echo_deadline = time.time() + 1.5
            header_echo = bytearray()
            while len(header_echo) < 5 and time.time() < echo_deadline:
                c = ser.read(5 - len(header_echo))
                if c:
                    header_echo.extend(c)

            if len(header_echo) != 5 or bytes(header_echo) != header:
                ser.close()
                self.error_signal.emit(
                    "FPGA çözünürlük başlığını onaylamadı.\n"
                    "Lütfen Tang Nano 20K bağlı ve bitstream yüklü mü kontrol edin."
                )
                return

            self.progress_signal.emit(0, f"Çözünürlük ({self.width}x{self.height}) ayarlandı. Piksel akışı başladı...")

            # 2. Stream pixels in lock-step 64-byte chunks
            received = bytearray()
            total_chunks = self.total_pixels // CHUNK_SIZE
            start_time = time.time()

            for offset in range(0, self.total_pixels, CHUNK_SIZE):
                chunk = pixel_bytes[offset:offset + CHUNK_SIZE]
                ser.write(chunk)

                buf = bytearray()
                deadline = time.time() + CHUNK_TIMEOUT
                while len(buf) < len(chunk) and time.time() < deadline:
                    c = ser.read(len(chunk) - len(buf))
                    if c:
                        buf.extend(c)

                received.extend(buf)

                if len(buf) != len(chunk):
                    ser.close()
                    self.error_signal.emit(
                        f"Zaman aşımı: FPGA'den {offset}. pikselde beklenen veri gelmedi."
                    )
                    return

                chunk_num = offset // CHUNK_SIZE
                if chunk_num % 64 == 0 or chunk_num == total_chunks - 1:
                    pct = int((len(received) / self.total_pixels) * 100)
                    self.progress_signal.emit(pct, f"İşleniyor: {len(received):,} / {self.total_pixels:,} piksel (%{pct})")

            elapsed = time.time() - start_time
            ser.close()

            # Align 2-pixel cascade latency
            raw = np.frombuffer(bytes(received[:self.total_pixels]), dtype=np.uint8).reshape((self.height, self.width))
            aligned = np.zeros_like(raw)
            aligned[:-2, :-2] = raw[2:, 2:]

            ref = reference_gaussian_sobel(self.pixel_array, self.threshold)
            interior = (slice(2, -2), slice(2, -2))
            mismatches = int(np.count_nonzero(aligned[interior] != ref[interior]))

            self.finished_signal.emit(aligned, ref, mismatches, elapsed)

        except Exception as e:
            if ser.is_open:
                ser.close()
            self.error_signal.emit(f"Haberleşme hatası:\n{e}")


class FpgaVideoWorker(QThread):
    frame_ready = pyqtSignal(np.ndarray, np.ndarray, np.ndarray, int, int, float)
    progress_signal = pyqtSignal(int, str)
    finished_signal = pyqtSignal(str, float, int)
    error_signal = pyqtSignal(str)

    def __init__(self, port, video_source, width, height, threshold, max_frames=60, stride=1, save_video=True):
        super().__init__()
        self.port = port
        self.video_source = video_source
        self.width = width
        self.height = height
        self.threshold = threshold
        self.max_frames = max_frames
        self.stride = stride
        self.save_video = save_video
        self.is_running = True

    def stop(self):
        self.is_running = False

    def run(self):
        try:
            if str(self.video_source).isdigit():
                cap = cv2.VideoCapture(int(self.video_source))
            else:
                cap = cv2.VideoCapture(self.video_source)
        except Exception as e:
            self.error_signal.emit(f"Video kaynağı açılamadı: {e}")
            return

        if not cap.isOpened():
            self.error_signal.emit(f"Video açılamadı: {self.video_source}")
            return

        src_fps = cap.get(cv2.CAP_PROP_FPS) or 20.0
        total_video_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if total_video_frames <= 0:
            total_video_frames = self.max_frames if self.max_frames > 0 else 100

        target_frames = min(total_video_frames // self.stride, self.max_frames) if self.max_frames > 0 else (total_video_frames // self.stride)

        try:
            ser = serial.Serial(port=self.port, baudrate=BAUD_RATE, timeout=0.05)
        except Exception as e:
            cap.release()
            self.error_signal.emit(f"Port açılamadı ({self.port}):\n{e}")
            return

        output_path = ""
        writer = None
        if self.save_video:
            output_dir = os.path.dirname(__file__)
            output_path = os.path.join(output_dir, "..", "output_video.mp4")
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(output_path, fourcc, max(src_fps / self.stride, 5.0), (self.width, self.height), isColor=False)

        total_pixels = self.width * self.height
        frame_idx = 0
        processed_count = 0
        overall_start = time.time()

        try:
            while self.is_running and cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break

                if frame_idx % self.stride != 0:
                    frame_idx += 1
                    continue

                frame_idx += 1
                processed_count += 1

                # Preprocess frame
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                resized = cv2.resize(gray, (self.width, self.height), interpolation=cv2.INTER_AREA)

                t_frame_start = time.time()

                # 1. Sync & Send Header for frame
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                time.sleep(0.22) # Safe 220ms sync gap (matches 200ms hardware idle timeout)
                ser.reset_input_buffer()

                header = bytes([
                    self.threshold,
                    (self.width >> 8) & 0xFF,
                    self.width & 0xFF,
                    (self.height >> 8) & 0xFF,
                    self.height & 0xFF
                ])
                ser.write(header)

                echo = bytearray()
                deadline = time.time() + 1.2
                while len(echo) < 5 and time.time() < deadline:
                    c = ser.read(5 - len(echo))
                    if c:
                        echo.extend(c)

                if len(echo) != 5 or bytes(echo) != header:
                    raise RuntimeError("FPGA başlık onayında zaman aşımı oluştu.")

                # 2. Stream pixels in 64-byte chunks
                pixel_bytes = resized.tobytes()
                received = bytearray()

                for offset in range(0, total_pixels, CHUNK_SIZE):
                    if not self.is_running:
                        break
                    chunk = pixel_bytes[offset:offset + CHUNK_SIZE]
                    ser.write(chunk)

                    buf = bytearray()
                    chk_deadline = time.time() + CHUNK_TIMEOUT
                    while len(buf) < len(chunk) and time.time() < chk_deadline:
                        c = ser.read(len(chunk) - len(buf))
                        if c:
                            buf.extend(c)
                    received.extend(buf)

                if not self.is_running:
                    break

                # 3. Align 2-pixel cascade latency
                raw = np.frombuffer(bytes(received[:total_pixels]), dtype=np.uint8).reshape((self.height, self.width))
                aligned = np.zeros_like(raw)
                aligned[:-2, :-2] = raw[2:, 2:]

                # 4. Reference
                ref = reference_gaussian_sobel(resized, self.threshold)

                frame_elapsed = time.time() - t_frame_start
                live_fps = 1.0 / frame_elapsed if frame_elapsed > 0 else 0

                if writer is not None:
                    writer.write(aligned)

                pct = int((processed_count / max(target_frames, 1)) * 100)
                self.progress_signal.emit(pct, f"Video İşleniyor: Kare {processed_count}/{target_frames} (%{pct}) - {live_fps:.1f} FPS")
                self.frame_ready.emit(resized, ref, aligned, processed_count, target_frames, live_fps)

                if self.max_frames > 0 and processed_count >= self.max_frames:
                    break

            total_elapsed = time.time() - overall_start
            if writer:
                writer.release()
            cap.release()
            ser.close()

            self.finished_signal.emit(output_path, total_elapsed, processed_count)

        except Exception as e:
            if writer:
                writer.release()
            cap.release()
            if ser.is_open:
                ser.close()
            self.error_signal.emit(f"Video işleme hatası:\n{e}")


class FpgaEdgeGui(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tang Nano 20K - FPGA Donanımsal Görüntü & Video Kenar Bulma Laboratuvarı")
        self.setMinimumSize(1080, 850)

        # Photo state
        self.current_image_path = None
        self.processed_pixel_array = None
        self.last_fpga_result = None
        self.photo_width = 512
        self.photo_height = 512

        # Video state
        self.video_worker = None
        self.current_video_path = None
        self.video_width = 128
        self.video_height = 128

        self.init_ui()
        self.refresh_ports()
        self.load_default_test_image()
        self.check_default_sample_video()

    def init_ui(self):
        self.setStyleSheet("""
            QMainWindow {
                background-color: #161b22;
            }
            QWidget {
                color: #e6edf3;
                font-family: 'Segoe UI', sans-serif;
                font-size: 13px;
            }
            QTabWidget::pane {
                border: 1px solid #30363d;
                border-radius: 8px;
                background-color: #1e1e24;
                top: -1px;
            }
            QTabBar::tab {
                background: #21262d;
                color: #8b949e;
                padding: 10px 24px;
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
                font-weight: bold;
                font-size: 14px;
                margin-right: 4px;
            }
            QTabBar::tab:selected {
                background: #1e1e24;
                color: #58a6ff;
                border: 1px solid #30363d;
                border-bottom: 2px solid #58a6ff;
            }
            QTabBar::tab:hover {
                color: #c9d1d9;
            }
            QGroupBox {
                border: 1px solid #30363d;
                border-radius: 8px;
                margin-top: 10px;
                font-weight: bold;
                padding-top: 15px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 5px;
                color: #58a6ff;
            }
            QPushButton {
                background-color: #238636;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 8px 16px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2ea043;
            }
            QPushButton:disabled {
                background-color: #21262d;
                color: #6e7681;
            }
            QPushButton#secondaryBtn {
                background-color: #30363d;
                color: #c9d1d9;
            }
            QPushButton#secondaryBtn:hover {
                background-color: #3b434d;
            }
            QPushButton#stopBtn {
                background-color: #da3633;
                color: white;
            }
            QPushButton#stopBtn:hover {
                background-color: #f85149;
            }
            QComboBox, QSpinBox {
                background-color: #0d1117;
                border: 1px solid #30363d;
                border-radius: 6px;
                padding: 6px 10px;
                color: #c9d1d9;
            }
            QSlider::groove:horizontal {
                border: 1px solid #30363d;
                height: 8px;
                background: #0d1117;
                border-radius: 4px;
            }
            QSlider::sub-page:horizontal {
                background: #58a6ff;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #ffffff;
                border: 1px solid #58a6ff;
                width: 18px;
                margin-top: -5px;
                margin-bottom: -5px;
                border-radius: 9px;
            }
            QProgressBar {
                border: 1px solid #30363d;
                border-radius: 6px;
                text-align: center;
                color: white;
                font-weight: bold;
                background-color: #0d1117;
                height: 22px;
            }
            QProgressBar::chunk {
                background-color: #238636;
                border-radius: 5px;
            }
            QLabel#imageBox {
                background-color: #0d1117;
                border: 2px dashed #30363d;
                border-radius: 8px;
            }
        """)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(10)

        # ---------------- Shared Connection Header ----------------
        top_bar = QHBoxLayout()
        top_bar.addWidget(QLabel("<b>FPGA Donanım Bağlantısı:</b>"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(180)
        top_bar.addWidget(self.port_combo)

        refresh_btn = QPushButton("Yenile")
        refresh_btn.setObjectName("secondaryBtn")
        refresh_btn.clicked.connect(self.refresh_ports)
        top_bar.addWidget(refresh_btn)

        top_bar.addSpacing(25)
        self.lbl_baud_info = QLabel("Baud Rate: <b>1,350,000 (1.35M)</b> | FPGA Saat: <b>27 MHz</b> | Hat: <b>2x BSRAM</b>")
        self.lbl_baud_info.setStyleSheet("color: #8b949e;")
        top_bar.addWidget(self.lbl_baud_info)

        top_bar.addStretch()
        main_layout.addLayout(top_bar)

        # ---------------- Tabs (Photo vs Video) ----------------
        self.tabs = QTabWidget()
        self.tab_photo = QWidget()
        self.tab_video = QWidget()

        self.setup_photo_tab()
        self.setup_video_tab()

        self.tabs.addTab(self.tab_photo, "📷 Fotoğraf Modu (Ultra Yüksek Çözünürlük)")
        self.tabs.addTab(self.tab_video, "🎥 Video & Kamera Modu (Kare Kare Akış)")
        main_layout.addWidget(self.tabs)

    # ==========================================================
    # PHOTO TAB
    # ==========================================================
    def setup_photo_tab(self):
        layout = QVBoxLayout(self.tab_photo)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(12)

        # Controls
        ctrl_group = QGroupBox("Fotoğraf Ayarları ve Eşik Değeri")
        ctrl_layout = QVBoxLayout(ctrl_group)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Çözünürlük:"))
        self.photo_res_combo = QComboBox()
        self.photo_res_combo.setMinimumWidth(240)
        for label, w, h in RESOLUTIONS_PHOTO:
            self.photo_res_combo.addItem(label, (w, h))
        self.photo_res_combo.setCurrentIndex(2)  # Default: 512x512
        self.photo_res_combo.currentIndexChanged.connect(self.on_photo_res_changed)
        row1.addWidget(self.photo_res_combo)

        row1.addSpacing(15)
        self.photo_select_btn = QPushButton("Fotoğraf Seç...")
        self.photo_select_btn.setObjectName("secondaryBtn")
        self.photo_select_btn.clicked.connect(self.choose_image)
        row1.addWidget(self.photo_select_btn)

        self.aspect_cb = QCheckBox("Oranları Koru")
        self.aspect_cb.setChecked(True)
        self.aspect_cb.toggled.connect(self.reload_current_image)
        row1.addWidget(self.aspect_cb)
        row1.addStretch()
        ctrl_layout.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Kenar Eşiği (Threshold):"))
        self.photo_thr_slider = QSlider(Qt.Horizontal)
        self.photo_thr_slider.setRange(20, 200)
        self.photo_thr_slider.setValue(80)
        self.photo_thr_slider.valueChanged.connect(self.on_photo_thr_changed)
        row2.addWidget(self.photo_thr_slider)

        self.photo_thr_val_lbl = QLabel("80")
        self.photo_thr_val_lbl.setMinimumWidth(35)
        self.photo_thr_val_lbl.setStyleSheet("color: #58a6ff; font-weight: bold; font-size: 14px;")
        row2.addWidget(self.photo_thr_val_lbl)

        row2.addSpacing(20)
        self.photo_run_btn = QPushButton("▶  FPGA'e Gönder ve Kenar Bul")
        self.photo_run_btn.setFixedHeight(36)
        self.photo_run_btn.clicked.connect(self.start_photo_processing)
        row2.addWidget(self.photo_run_btn)
        ctrl_layout.addLayout(row2)
        layout.addWidget(ctrl_group)

        # Status
        photo_status_layout = QHBoxLayout()
        self.photo_status_lbl = QLabel("Durum: Hazır")
        self.photo_status_lbl.setStyleSheet("color: #8b949e;")
        photo_status_layout.addWidget(self.photo_status_lbl)

        self.photo_progress_bar = QProgressBar()
        self.photo_progress_bar.setVisible(False)
        self.photo_progress_bar.setMinimumWidth(320)
        photo_status_layout.addWidget(self.photo_progress_bar)
        layout.addLayout(photo_status_layout)

        # 3 Panels
        panels_layout = QHBoxLayout()
        panels_layout.setSpacing(12)

        # P1
        b1 = QVBoxLayout()
        self.lbl_p1_title = QLabel(f"1. Giriş Görüntüsü ({self.photo_width}x{self.photo_height})")
        self.lbl_p1_title.setAlignment(Qt.AlignCenter)
        self.lbl_p1_title.setStyleSheet("font-weight: bold; color: #58a6ff;")
        self.lbl_photo_in = QLabel()
        self.lbl_photo_in.setObjectName("imageBox")
        self.lbl_photo_in.setFixedSize(315, 315)
        self.lbl_photo_in.setAlignment(Qt.AlignCenter)
        b1.addWidget(self.lbl_p1_title)
        b1.addWidget(self.lbl_photo_in)
        panels_layout.addLayout(b1)

        # P2
        b2 = QVBoxLayout()
        t2 = QLabel("2. Referans (Gaussian + Sobel)")
        t2.setAlignment(Qt.AlignCenter)
        t2.setStyleSheet("font-weight: bold; color: #d2a8ff;")
        self.lbl_photo_ref = QLabel()
        self.lbl_photo_ref.setObjectName("imageBox")
        self.lbl_photo_ref.setFixedSize(315, 315)
        self.lbl_photo_ref.setAlignment(Qt.AlignCenter)
        b2.addWidget(t2)
        b2.addWidget(self.lbl_photo_ref)
        panels_layout.addLayout(b2)

        # P3
        b3 = QVBoxLayout()
        t3 = QLabel("3. FPGA Çıktısı (Tang Nano 20K)")
        t3.setAlignment(Qt.AlignCenter)
        t3.setStyleSheet("font-weight: bold; color: #7ee787;")
        self.lbl_photo_out = QLabel()
        self.lbl_photo_out.setObjectName("imageBox")
        self.lbl_photo_out.setFixedSize(315, 315)
        self.lbl_photo_out.setAlignment(Qt.AlignCenter)
        b3.addWidget(t3)
        b3.addWidget(self.lbl_photo_out)
        panels_layout.addLayout(b3)

        layout.addLayout(panels_layout)

        # Footer
        footer = QHBoxLayout()
        self.photo_match_lbl = QLabel("Doğruluk: -")
        self.photo_match_lbl.setStyleSheet("font-size: 14px; font-weight: bold;")
        footer.addWidget(self.photo_match_lbl)
        footer.addStretch()

        self.photo_save_btn = QPushButton("💾  Sonucu Kaydet (.png)")
        self.photo_save_btn.setObjectName("secondaryBtn")
        self.photo_save_btn.setEnabled(False)
        self.photo_save_btn.clicked.connect(self.save_photo_result)
        footer.addWidget(self.photo_save_btn)
        layout.addLayout(footer)

    # ==========================================================
    # VIDEO TAB
    # ==========================================================
    def setup_video_tab(self):
        layout = QVBoxLayout(self.tab_video)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(12)

        # Video Settings Group
        ctrl_group = QGroupBox("Video Kaynağı ve Akış Parametreleri")
        ctrl_layout = QVBoxLayout(ctrl_group)

        # Row 1: Source & Resolution
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Kaynak:"))
        self.video_source_combo = QComboBox()
        self.video_source_combo.setMinimumWidth(220)
        self.video_source_combo.addItem("🎬 Örnek Test Videosu (sample_video.mp4)", "sample")
        self.video_source_combo.addItem("📁 Video Dosyası Seç (.mp4, .avi, .mov)", "file")
        self.video_source_combo.addItem("📷 Webcam / Kamera 0", "webcam")
        self.video_source_combo.currentIndexChanged.connect(self.on_video_source_changed)
        row1.addWidget(self.video_source_combo)

        self.video_file_btn = QPushButton("Dosya...")
        self.video_file_btn.setObjectName("secondaryBtn")
        self.video_file_btn.setEnabled(False)
        self.video_file_btn.clicked.connect(self.choose_video_file)
        row1.addWidget(self.video_file_btn)

        row1.addSpacing(15)
        row1.addWidget(QLabel("Video Çözünürlüğü:"))
        self.video_res_combo = QComboBox()
        self.video_res_combo.setMinimumWidth(220)
        for label, w, h in RESOLUTIONS_VIDEO:
            self.video_res_combo.addItem(label, (w, h))
        self.video_res_combo.setCurrentIndex(1)  # Default: 128x128
        self.video_res_combo.currentIndexChanged.connect(self.on_video_res_changed)
        row1.addWidget(self.video_res_combo)

        row1.addStretch()
        ctrl_layout.addLayout(row1)

        # Row 2: Stride, Frame limit, Save video checkbox
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Kare Atlama:"))
        self.video_stride_combo = QComboBox()
        self.video_stride_combo.addItem("Her Kare (1x)", 1)
        self.video_stride_combo.addItem("Her 2. Kare (2x Hızlı)", 2)
        self.video_stride_combo.addItem("Her 3. Kare (3x Hızlı)", 3)
        self.video_stride_combo.addItem("Her 5. Kare (5x Hızlı)", 5)
        row2.addWidget(self.video_stride_combo)

        row2.addSpacing(15)
        row2.addWidget(QLabel("Kare Limiti:"))
        self.video_limit_combo = QComboBox()
        self.video_limit_combo.addItem("İlk 15 Kare (Hızlı Test)", 15)
        self.video_limit_combo.addItem("İlk 30 Kare (Önerilen)", 30)
        self.video_limit_combo.addItem("İlk 60 Kare", 60)
        self.video_limit_combo.addItem("Tüm Video", 0)
        self.video_limit_combo.setCurrentIndex(1)
        row2.addWidget(self.video_limit_combo)

        row2.addSpacing(15)
        self.video_save_cb = QCheckBox("Sonucu Video Olarak Kaydet (.mp4)")
        self.video_save_cb.setChecked(True)
        row2.addWidget(self.video_save_cb)

        row2.addStretch()
        ctrl_layout.addLayout(row2)

        # Row 3: Threshold & Run / Stop buttons
        row3 = QHBoxLayout()
        row3.addWidget(QLabel("Kenar Eşiği (Threshold):"))
        self.video_thr_slider = QSlider(Qt.Horizontal)
        self.video_thr_slider.setRange(20, 200)
        self.video_thr_slider.setValue(80)
        self.video_thr_slider.valueChanged.connect(self.on_video_thr_changed)
        row3.addWidget(self.video_thr_slider)

        self.video_thr_val_lbl = QLabel("80")
        self.video_thr_val_lbl.setMinimumWidth(35)
        self.video_thr_val_lbl.setStyleSheet("color: #58a6ff; font-weight: bold; font-size: 14px;")
        row3.addWidget(self.video_thr_val_lbl)

        row3.addSpacing(20)
        self.video_run_btn = QPushButton("▶  Videoyu FPGA ile İşlet")
        self.video_run_btn.setFixedHeight(36)
        self.video_run_btn.clicked.connect(self.start_video_processing)
        row3.addWidget(self.video_run_btn)

        self.video_stop_btn = QPushButton("⏹ Durdur")
        self.video_stop_btn.setObjectName("stopBtn")
        self.video_stop_btn.setFixedHeight(36)
        self.video_stop_btn.setEnabled(False)
        self.video_stop_btn.clicked.connect(self.stop_video_processing)
        row3.addWidget(self.video_stop_btn)

        ctrl_layout.addLayout(row3)
        layout.addWidget(ctrl_group)

        # Status & Progress
        video_status_layout = QHBoxLayout()
        self.video_status_lbl = QLabel("Durum: Video seçildi, işleme hazır.")
        self.video_status_lbl.setStyleSheet("color: #8b949e;")
        video_status_layout.addWidget(self.video_status_lbl)

        self.video_progress_bar = QProgressBar()
        self.video_progress_bar.setVisible(False)
        self.video_progress_bar.setMinimumWidth(320)
        video_status_layout.addWidget(self.video_progress_bar)
        layout.addLayout(video_status_layout)

        # 3 Live Video Panels
        panels_layout = QHBoxLayout()
        panels_layout.setSpacing(12)

        # Video Panel 1
        vb1 = QVBoxLayout()
        self.lbl_v1_title = QLabel(f"1. Giriş Karesi ({self.video_width}x{self.video_height})")
        self.lbl_v1_title.setAlignment(Qt.AlignCenter)
        self.lbl_v1_title.setStyleSheet("font-weight: bold; color: #58a6ff;")
        self.lbl_video_in = QLabel()
        self.lbl_video_in.setObjectName("imageBox")
        self.lbl_video_in.setFixedSize(315, 315)
        self.lbl_video_in.setAlignment(Qt.AlignCenter)
        vb1.addWidget(self.lbl_v1_title)
        vb1.addWidget(self.lbl_video_in)
        panels_layout.addLayout(vb1)

        # Video Panel 2
        vb2 = QVBoxLayout()
        vt2 = QLabel("2. Referans Kenar (Yazılım)")
        vt2.setAlignment(Qt.AlignCenter)
        vt2.setStyleSheet("font-weight: bold; color: #d2a8ff;")
        self.lbl_video_ref = QLabel()
        self.lbl_video_ref.setObjectName("imageBox")
        self.lbl_video_ref.setFixedSize(315, 315)
        self.lbl_video_ref.setAlignment(Qt.AlignCenter)
        vb2.addWidget(vt2)
        vb2.addWidget(self.lbl_video_ref)
        panels_layout.addLayout(vb2)

        # Video Panel 3
        vb3 = QVBoxLayout()
        vt3 = QLabel("3. Canlı FPGA Video Akışı")
        vt3.setAlignment(Qt.AlignCenter)
        vt3.setStyleSheet("font-weight: bold; color: #7ee787;")
        self.lbl_video_out = QLabel()
        self.lbl_video_out.setObjectName("imageBox")
        self.lbl_video_out.setFixedSize(315, 315)
        self.lbl_video_out.setAlignment(Qt.AlignCenter)
        vb3.addWidget(vt3)
        vb3.addWidget(self.lbl_video_out)
        panels_layout.addLayout(vb3)

        layout.addLayout(panels_layout)

        # Footer / Info
        v_footer = QHBoxLayout()
        self.video_fps_lbl = QLabel("Akış Hızı: - | İşlenen Kare: 0")
        self.video_fps_lbl.setStyleSheet("font-size: 14px; font-weight: bold; color: #7ee787;")
        v_footer.addWidget(self.video_fps_lbl)
        v_footer.addStretch()

        self.video_open_btn = QPushButton("📂  Kayıt Klasörünü Aç")
        self.video_open_btn.setObjectName("secondaryBtn")
        self.video_open_btn.setEnabled(False)
        self.video_open_btn.clicked.connect(self.open_output_folder)
        v_footer.addWidget(self.video_open_btn)
        layout.addLayout(v_footer)

    # ==========================================================
    # COMMON METHODS
    # ==========================================================
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        default_index = 0

        for i, p in enumerate(ports):
            text = f"{p.device} ({p.description})"
            self.port_combo.addItem(text, p.device)
            if p.device == "COM6":
                default_index = i

        if ports:
            self.port_combo.setCurrentIndex(default_index)
        else:
            self.port_combo.addItem("Port Bulunamadı", "")

    # ==========================================================
    # PHOTO TAB LOGIC
    # ==========================================================
    def on_photo_res_changed(self, index):
        w, h = self.photo_res_combo.currentData()
        self.photo_width = w
        self.photo_height = h
        self.lbl_p1_title.setText(f"1. Giriş Görüntüsü ({w}x{h})")
        self.reload_current_image()

    def on_photo_thr_changed(self, value):
        self.photo_thr_val_lbl.setText(str(value))
        if self.processed_pixel_array is not None:
            ref_array = reference_gaussian_sobel(self.processed_pixel_array, value)
            ref_pixmap = np_to_qpixmap(ref_array).scaled(305, 305, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.lbl_photo_ref.setPixmap(ref_pixmap)

    def load_default_test_image(self):
        default_path = os.path.join(os.path.dirname(__file__), "..", "test_image.png")
        if os.path.exists(default_path):
            self.load_image_file(default_path)

    def choose_image(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Fotoğraf Seç", "", "Resim Dosyaları (*.png *.jpg *.jpeg *.bmp)"
        )
        if file_path:
            self.load_image_file(file_path)

    def reload_current_image(self):
        if self.current_image_path:
            self.load_image_file(self.current_image_path)

    def load_image_file(self, path):
        try:
            self.current_image_path = path
            img = Image.open(path).convert("L")

            w, h = self.photo_width, self.photo_height
            if self.aspect_cb.isChecked():
                img.thumbnail((w, h), Image.Resampling.LANCZOS)
                dw = w - img.size[0]
                dh = h - img.size[1]
                padding = (dw // 2, dh // 2, dw - (dw // 2), dh - (dh // 2))
                img_processed = ImageOps.expand(img, padding, fill=0)
            else:
                img_processed = img.resize((w, h), Image.Resampling.LANCZOS)

            self.processed_pixel_array = np.array(img_processed, dtype=np.uint8)

            pixmap = np_to_qpixmap(self.processed_pixel_array).scaled(305, 305, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.lbl_photo_in.setPixmap(pixmap)

            thr = self.photo_thr_slider.value()
            ref_array = reference_gaussian_sobel(self.processed_pixel_array, thr)
            ref_pixmap = np_to_qpixmap(ref_array).scaled(305, 305, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.lbl_photo_ref.setPixmap(ref_pixmap)

            self.lbl_photo_out.setText("FPGA İşlemi Bekleniyor...")
            total = w * h
            self.photo_status_lbl.setText(f"Fotoğraf: {os.path.basename(path)} ({w}x{h} - {total:,} piksel)")
            self.photo_match_lbl.setText("Doğruluk: Henüz test edilmedi")
            self.photo_save_btn.setEnabled(False)

        except Exception as e:
            QMessageBox.critical(self, "Hata", f"Resim yüklenirken hata oluştu:\n{e}")

    def start_photo_processing(self):
        selected_port = self.port_combo.currentData()
        if not selected_port:
            QMessageBox.warning(self, "Uyarı", "Lütfen geçerli bir COM portu seçin!")
            return

        if self.processed_pixel_array is None:
            QMessageBox.warning(self, "Uyarı", "Lütfen önce bir fotoğraf seçin!")
            return

        thr = self.photo_thr_slider.value()

        self.photo_run_btn.setEnabled(False)
        self.photo_select_btn.setEnabled(False)
        self.photo_res_combo.setEnabled(False)
        self.photo_thr_slider.setEnabled(False)
        self.photo_progress_bar.setValue(0)
        self.photo_progress_bar.setVisible(True)
        self.photo_status_lbl.setText(f"FPGA'e bağlanılıyor ({self.photo_width}x{self.photo_height})...")

        self.worker = FpgaPhotoWorker(selected_port, self.processed_pixel_array, thr, self.photo_width, self.photo_height)
        self.worker.progress_signal.connect(self.on_photo_progress)
        self.worker.finished_signal.connect(self.on_photo_finished)
        self.worker.error_signal.connect(self.on_photo_error)
        self.worker.start()

    def on_photo_progress(self, percent, text):
        self.photo_progress_bar.setValue(percent)
        self.photo_status_lbl.setText(text)

    def on_photo_finished(self, fpga_array, ref_array, mismatches, elapsed):
        self.last_fpga_result = fpga_array
        self.photo_progress_bar.setVisible(False)
        self.photo_run_btn.setEnabled(True)
        self.photo_select_btn.setEnabled(True)
        self.photo_res_combo.setEnabled(True)
        self.photo_thr_slider.setEnabled(True)
        self.photo_save_btn.setEnabled(True)

        pixmap = np_to_qpixmap(fpga_array).scaled(305, 305, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.lbl_photo_out.setPixmap(pixmap)

        total_tested = (self.photo_height - 4) * (self.photo_width - 4)
        match_pct = 100 * (total_tested - mismatches) / total_tested

        self.photo_status_lbl.setText(f"Tamamlandı ({elapsed:.2f} saniye).")
        if mismatches == 0:
            self.photo_match_lbl.setText(f"✅ FPGA Doğruluk: 0 Hata / %100 Mükemmel Eşleşme ({elapsed:.2f}s)")
            self.photo_match_lbl.setStyleSheet("color: #7ee787; font-size: 14px; font-weight: bold;")
        else:
            self.photo_match_lbl.setText(f"⚠️ FPGA Doğruluk: {mismatches} uyumsuz piksel (%{match_pct:.2f})")
            self.photo_match_lbl.setStyleSheet("color: #f0883e; font-size: 14px; font-weight: bold;")

    def on_photo_error(self, message):
        self.photo_progress_bar.setVisible(False)
        self.photo_run_btn.setEnabled(True)
        self.photo_select_btn.setEnabled(True)
        self.photo_res_combo.setEnabled(True)
        self.photo_thr_slider.setEnabled(True)
        self.photo_status_lbl.setText("Hata oluştu.")
        QMessageBox.critical(self, "Hata", message)

    def save_photo_result(self):
        if self.last_fpga_result is None:
            return
        save_path, _ = QFileDialog.getSaveFileName(
            self, "Sonucu Kaydet", f"output_{self.photo_width}x{self.photo_height}.png", "PNG Dosyası (*.png)"
        )
        if save_path:
            img = Image.fromarray(self.last_fpga_result)
            img.save(save_path)
            QMessageBox.information(self, "Başarılı", f"Sonuç kaydedildi:\n{save_path}")

    # ==========================================================
    # VIDEO TAB LOGIC
    # ==========================================================
    def check_default_sample_video(self):
        sample_path = os.path.join(os.path.dirname(__file__), "..", "sample_video.mp4")
        if os.path.exists(sample_path):
            self.current_video_path = sample_path

    def on_video_source_changed(self, index):
        mode = self.video_source_combo.currentData()
        if mode == "file":
            self.video_file_btn.setEnabled(True)
            if not self.current_video_path or "sample_video" in str(self.current_video_path):
                self.choose_video_file()
        elif mode == "sample":
            self.video_file_btn.setEnabled(False)
            self.current_video_path = os.path.join(os.path.dirname(__file__), "..", "sample_video.mp4")
            self.video_status_lbl.setText("Örnek sentetik video seçildi (sample_video.mp4)")
        elif mode == "webcam":
            self.video_file_btn.setEnabled(False)
            self.current_video_path = 0
            self.video_status_lbl.setText("Dahili Kamera (Webcam) seçildi.")

    def choose_video_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Video Dosyası Seç", "", "Video Dosyaları (*.mp4 *.avi *.mov *.mkv)"
        )
        if file_path:
            self.current_video_path = file_path
            self.video_status_lbl.setText(f"Seçilen video: {os.path.basename(file_path)}")

    def on_video_res_changed(self, index):
        w, h = self.video_res_combo.currentData()
        self.video_width = w
        self.video_height = h
        self.lbl_v1_title.setText(f"1. Giriş Karesi ({w}x{h})")

    def on_video_thr_changed(self, value):
        self.video_thr_val_lbl.setText(str(value))
        if self.video_worker and self.video_worker.isRunning():
            self.video_worker.threshold = value

    def start_video_processing(self):
        selected_port = self.port_combo.currentData()
        if not selected_port:
            QMessageBox.warning(self, "Uyarı", "Lütfen geçerli bir COM portu seçin!")
            return

        mode = self.video_source_combo.currentData()
        source = self.current_video_path
        if mode == "sample":
            source = os.path.join(os.path.dirname(__file__), "..", "sample_video.mp4")
            if not os.path.exists(source):
                from generate_sample_video import create_sample_video
                create_sample_video(source, duration_sec=3, fps=15, width=128, height=128)
        elif mode == "webcam":
            source = 0
        elif not source:
            QMessageBox.warning(self, "Uyarı", "Lütfen işlenecek bir video dosyası seçin!")
            return

        thr = self.video_thr_slider.value()
        stride = self.video_stride_combo.currentData()
        max_f = self.video_limit_combo.currentData()
        save_vid = self.video_save_cb.isChecked()

        self.video_run_btn.setEnabled(False)
        self.video_stop_btn.setEnabled(True)
        self.video_source_combo.setEnabled(False)
        self.video_file_btn.setEnabled(False)
        self.video_res_combo.setEnabled(False)
        self.video_stride_combo.setEnabled(False)
        self.video_limit_combo.setEnabled(False)
        self.video_save_cb.setEnabled(False)

        self.video_progress_bar.setValue(0)
        self.video_progress_bar.setVisible(True)
        self.video_status_lbl.setText(f"FPGA Video hattı başlatılıyor ({self.video_width}x{self.video_height})...")

        self.video_worker = FpgaVideoWorker(
            port=selected_port,
            video_source=source,
            width=self.video_width,
            height=self.video_height,
            threshold=thr,
            max_frames=max_f,
            stride=stride,
            save_video=save_vid
        )
        self.video_worker.frame_ready.connect(self.on_video_frame_ready)
        self.video_worker.progress_signal.connect(self.on_video_progress)
        self.video_worker.finished_signal.connect(self.on_video_finished)
        self.video_worker.error_signal.connect(self.on_video_error)
        self.video_worker.start()

    def stop_video_processing(self):
        if self.video_worker and self.video_worker.isRunning():
            self.video_status_lbl.setText("Durduruluyor...")
            self.video_worker.stop()

    def on_video_progress(self, pct, text):
        self.video_progress_bar.setValue(pct)
        self.video_status_lbl.setText(text)

    def on_video_frame_ready(self, in_frame, ref_frame, fpga_frame, frame_idx, total_frames, fps):
        p1 = np_to_qpixmap(in_frame).scaled(305, 305, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.lbl_video_in.setPixmap(p1)

        p2 = np_to_qpixmap(ref_frame).scaled(305, 305, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.lbl_video_ref.setPixmap(p2)

        p3 = np_to_qpixmap(fpga_frame).scaled(305, 305, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.lbl_video_out.setPixmap(p3)

        self.video_fps_lbl.setText(f"Hız: {fps:.2f} FPS | Kare: {frame_idx}/{total_frames} | LED: Ping-Pong Aktif")

    def on_video_finished(self, out_path, total_time, processed_frames):
        self.reset_video_ui()
        avg_fps = (processed_frames / total_time) if total_time > 0 else 0
        self.video_status_lbl.setText(f"Tamamlandı! {processed_frames} kare {total_time:.1f} saniyede işlendi (Ort: {avg_fps:.2f} FPS).")
        self.video_fps_lbl.setText(f"Bitti: {processed_frames} kare | Süre: {total_time:.1f}s | Ort: {avg_fps:.2f} FPS")

        if out_path and os.path.exists(out_path):
            self.video_open_btn.setEnabled(True)
            QMessageBox.information(
                self, "Video Tamamlandı",
                f"FPGA üzerinden video işleme başarıyla tamamlandı!\n\n"
                f"Toplam İşlenen Kare: {processed_frames}\n"
                f"Harcanan Süre: {total_time:.1f} sn ({avg_fps:.2f} FPS)\n"
                f"Kayıt Yolu: {os.path.abspath(out_path)}"
            )

    def on_video_error(self, message):
        self.reset_video_ui()
        self.video_status_lbl.setText("Hata oluştu.")
        QMessageBox.critical(self, "Video Hatası", message)

    def reset_video_ui(self):
        self.video_progress_bar.setVisible(False)
        self.video_run_btn.setEnabled(True)
        self.video_stop_btn.setEnabled(False)
        self.video_source_combo.setEnabled(True)
        if self.video_source_combo.currentData() == "file":
            self.video_file_btn.setEnabled(True)
        self.video_res_combo.setEnabled(True)
        self.video_stride_combo.setEnabled(True)
        self.video_limit_combo.setEnabled(True)
        self.video_save_cb.setEnabled(True)

    def open_output_folder(self):
        folder = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        os.startfile(folder)


def main():
    app = QApplication(sys.argv)
    window = FpgaEdgeGui()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
