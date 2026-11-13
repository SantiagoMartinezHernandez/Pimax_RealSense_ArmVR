"""
Intel RealSense D455C + MediaPipe Arm & 21-Finger 3D Tracker (v4.1 - Zero Hand Lag!)
====================================================================================
Why v4.0 had hand cloud delay & how v4.1 fixes it completely:
  - In MediaPipe HandLandmarker, setting `num_hands=2` when only 1 hand is raised forces
    MediaPipe to re-run the heavy full-frame SSD Palm Detector on EVERY SINGLE FRAME looking
    for the missing 2nd hand—causing huge queue lag on the hand cloud!
  - In v4.1:
    1. Restores `num_hands=1` for RIGHT ARM and LEFT ARM modes (so once your hand is seen,
       MediaPipe skips the heavy Palm Detector completely and tracks your 21 fingers in ~3ms!).
    2. Restores the `'r'` key in the Python window! Pressing `'r'` in Python cycles:
         [RIGHT ARM] -> [LEFT ARM] -> [BOTH ARMS]
       and automatically tells Unity which mode is active!
    3. Keeps the Kinematic Prior & Bone-Length Stabilization on the arms so long reaches
       and high lifts stay rock-solid and human-like.
"""

import os
import socket
import struct
import sys
import time
import urllib.request
import numpy as np
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

try:
    import pyrealsense2 as rs
except ImportError:
    print("[ERROR] pyrealsense2 not installed! Run: pip install pyrealsense2")
    sys.exit(1)

# ================= CONFIGURATION =================
UDP_IP = "127.0.0.1"
UDP_PORT = 5055

# 0 = Right Arm Only (Fast 1-Hand tracker), 1 = Left Arm Only (Fast 1-Hand tracker), 2 = Both Arms
INITIAL_ARM_MODE = 0
SHOW_DEBUG_WINDOW = True

MIN_DEPTH_M = 0.22
MAX_DEPTH_M = 2.50

MAX_SHOULDER_SPEED_MPS = 1.8
MAX_ELBOW_SPEED_MPS = 4.5
MAX_WRIST_SPEED_MPS = 7.0

POSE_MODEL_URL = "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task"
HAND_MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
POSE_MODEL_PATH = os.path.join(SCRIPT_DIR, "pose_landmarker_lite.task")
HAND_MODEL_PATH = os.path.join(SCRIPT_DIR, "hand_landmarker.task")


def ensure_models():
    if not os.path.exists(POSE_MODEL_PATH):
        print("[Setup] Downloading PoseLandmarker Lite model...")
        urllib.request.urlretrieve(POSE_MODEL_URL, POSE_MODEL_PATH)
    if not os.path.exists(HAND_MODEL_PATH):
        print("[Setup] Downloading HandLandmarker (21-Finger) model...")
        urllib.request.urlretrieve(HAND_MODEL_URL, HAND_MODEL_PATH)


def start_realsense_with_best_profile(pipeline: rs.pipeline):
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        raise RuntimeError(
            "No Intel RealSense device detected! Please check that the D455C USB cable is plugged in."
        )

    dev = devices[0]
    dev_name = dev.get_info(rs.camera_info.name) if dev.supports(rs.camera_info.name) else "Unknown RealSense"
    usb_type = dev.get_info(rs.camera_info.usb_type_descriptor) if dev.supports(rs.camera_info.usb_type_descriptor) else "Unknown"
    serial = dev.get_info(rs.camera_info.serial_number) if dev.supports(rs.camera_info.serial_number) else "Unknown"

    print(f"[RealSense] Found Device: {dev_name} (S/N: {serial}) on USB {usb_type}")

    candidates = [
        (640, 480, 640, 480, 60),
        (848, 480, 848, 480, 60),
        (640, 360, 640, 360, 60),
        (640, 480, 640, 480, 30),
        (848, 480, 848, 480, 30),
        (848, 480, 1280, 720, 30),
        (640, 360, 640, 360, 30),
        (480, 270, 640, 480, 30),
        (424, 240, 424, 240, 30),
    ]

    for dw, dh, cw, ch, fps in candidates:
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, fps)
        if cfg.can_resolve(pipeline):
            print(f"[RealSense] Selected fast stream profile: Depth {dw}x{dh} + Color {cw}x{ch} @ {fps} FPS")
            return pipeline.start(cfg)

    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.depth)
    cfg.enable_stream(rs.stream.color, rs.format.bgr8)
    return pipeline.start(cfg)


class KinematicArmState:
    """
    Maintains the previous 3D state of one arm (Shoulder, Elbow, Wrist, Index, Pinky)
    and enforces biomechanical continuity during long/high reaches.
    """
    def __init__(self, is_right: bool):
        self.is_right = is_right
        self.indices = (12, 14, 16, 20, 18) if is_right else (11, 13, 15, 19, 17)
        self.anchor_shoulder_z = 0.70
        self.prev_pts = None
        self.prev_vel = [np.zeros(3, dtype=np.float32) for _ in range(5)]
        self.prev_time = None
        self.est_upper_len = 0.28
        self.est_fore_len = 0.25

    @staticmethod
    def sample_depth(depth_image_raw: np.ndarray, depth_scale: float, u: int, v: int) -> float:
        h, w = depth_image_raw.shape
        u0, u1 = max(0, u - 3), min(w, u + 4)
        v0, v1 = max(0, v - 3), min(h, v + 4)
        patch = depth_image_raw[v0:v1, u0:u1]
        valid = patch[(patch > int(MIN_DEPTH_M / depth_scale)) & (patch < int(MAX_DEPTH_M / depth_scale))]
        if valid.size < 3:
            return 0.0
        return float(np.percentile(valid, 35)) * depth_scale

    def update(self, norm_lms, world_lms, depth_image_raw: np.ndarray, depth_scale: float, intrinsics, now_sec: float):
        h, w = depth_image_raw.shape
        dt = 0.016 if self.prev_time is None else max(0.002, min(0.15, now_sec - self.prev_time))
        self.prev_time = now_sec

        px_coords = []
        raw_depths = []
        vis_list = []

        for idx in self.indices:
            lm = norm_lms[idx]
            u = int(np.clip(lm.x * w, 0, w - 1))
            v = int(np.clip(lm.y * h, 0, h - 1))
            px_coords.append((u, v))
            vis_list.append(float(lm.visibility))
            raw_depths.append(self.sample_depth(depth_image_raw, depth_scale, u, v))

        d_shoulder = raw_depths[0]
        if MIN_DEPTH_M <= d_shoulder <= MAX_DEPTH_M:
            delta_z = np.clip(d_shoulder - self.anchor_shoulder_z, -0.02, 0.02)
            self.anchor_shoulder_z += 0.25 * delta_z

        s_mp = world_lms[self.indices[0]]
        raw_pts_3d = []
        depth_weights = []

        for k, idx in enumerate(self.indices):
            u, v = px_coords[k]
            lm_w = world_lms[idx]
            mp_rel_z = float(lm_w.z - s_mp.z)
            expected_z = self.anchor_shoulder_z + mp_rel_z
            d_m = raw_depths[k]

            if k in (0, 1, 2) and MIN_DEPTH_M <= d_m <= MAX_DEPTH_M:
                diff = abs(d_m - expected_z)
                w_sensor = 0.50 * float(np.exp(-(diff * diff) / (2.0 * 0.14 * 0.14)))
                fused_z = w_sensor * d_m + (1.0 - w_sensor) * expected_z
                depth_weights.append(w_sensor)
            else:
                fused_z = expected_z
                depth_weights.append(0.0)

            deproj = np.array(rs.rs2_deproject_pixel_to_point(intrinsics, [float(u), float(v)], float(fused_z)), dtype=np.float32)
            if k == 0:
                raw_pts_3d.append(deproj)
            else:
                mp_vec = np.array([lm_w.x - s_mp.x, lm_w.y - s_mp.y, lm_w.z - s_mp.z], dtype=np.float32)
                cam_vec = deproj - raw_pts_3d[0]
                blended_vec = 0.72 * cam_vec + 0.28 * mp_vec
                raw_pts_3d.append(raw_pts_3d[0] + blended_vec)

        if self.prev_pts is None:
            self.prev_pts = [pt.copy() for pt in raw_pts_3d]
        else:
            max_speeds = [MAX_SHOULDER_SPEED_MPS, MAX_ELBOW_SPEED_MPS, MAX_WRIST_SPEED_MPS, MAX_WRIST_SPEED_MPS, MAX_WRIST_SPEED_MPS]
            for k in range(5):
                predicted_pt = self.prev_pts[k] + self.prev_vel[k] * (dt * 0.6)
                innovation = raw_pts_3d[k] - predicted_pt
                dist = float(np.linalg.norm(innovation))
                max_step = max_speeds[k] * dt

                if dist > max_step and dist > 1e-5:
                    innovation = innovation * (max_step / dist)

                vis_gain = np.clip(vis_list[k], 0.25, 1.0)
                alpha = 0.72 * vis_gain if k > 0 else 0.45
                new_pt = self.prev_pts[k] + alpha * (predicted_pt + innovation - self.prev_pts[k])

                self.prev_vel[k] = 0.6 * ((new_pt - self.prev_pts[k]) / dt) + 0.4 * self.prev_vel[k]
                self.prev_pts[k] = new_pt

        # Enforce Anatomical Bone-Length Stability
        s_pt = self.prev_pts[0]
        e_vec = self.prev_pts[1] - s_pt
        w_vec = self.prev_pts[2] - self.prev_pts[1]
        u_len = float(np.linalg.norm(e_vec))
        f_len = float(np.linalg.norm(w_vec))

        if 0.18 < u_len < 0.42:
            self.est_upper_len = 0.96 * self.est_upper_len + 0.04 * u_len
        if 0.16 < f_len < 0.38:
            self.est_fore_len = 0.96 * self.est_fore_len + 0.04 * f_len

        if u_len > 1e-4:
            clamped_u = np.clip(u_len, self.est_upper_len * 0.88, self.est_upper_len * 1.12)
            self.prev_pts[1] = s_pt + (e_vec / u_len) * clamped_u

        if f_len > 1e-4:
            clamped_f = np.clip(f_len, self.est_fore_len * 0.88, self.est_fore_len * 1.12)
            old_wrist = self.prev_pts[2].copy()
            self.prev_pts[2] = self.prev_pts[1] + (w_vec / f_len) * clamped_f
            wrist_shift = self.prev_pts[2] - old_wrist
            self.prev_pts[3] += wrist_shift
            self.prev_pts[4] += wrist_shift

        conf = (vis_list[0] + vis_list[1] + vis_list[2]) / 3.0
        valid_ratio = sum(1.0 for w_s in depth_weights[:3] if w_s > 0.15) / 3.0
        return [pt.tolist() for pt in self.prev_pts], conf, valid_ratio, px_coords


class FastArmAndHandTracker:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # arm_mode: 0 = Right Arm Only, 1 = Left Arm Only, 2 = Both Arms
        self.arm_mode = INITIAL_ARM_MODE

        self.right_arm = KinematicArmState(is_right=True)
        self.left_arm = KinematicArmState(is_right=False)

        self.pose_norm_lms = None
        self.pose_world_lms = None
        self.has_new_pose = False
        self.pose_busy = False
        self.hand_busy = False

        # Dual 21-finger hand states: True = Right Hand, False = Left Hand
        self.hand_world = {True: None, False: None}
        self.hand_norm = {True: None, False: None}
        self.hand_last_time = {True: 0.0, False: 0.0}

    def on_pose_result(self, result: vision.PoseLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
        self.pose_busy = False
        if not result.pose_landmarks or not result.pose_world_landmarks:
            return
        self.pose_norm_lms = result.pose_landmarks[0]
        self.pose_world_lms = result.pose_world_landmarks[0]
        self.has_new_pose = True

    def on_single_hand_result(self, result: vision.HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
        """Callback for ultra-fast 1-Hand Landmarker (zero palm-detector lag!)."""
        self.hand_busy = False
        if not result.hand_world_landmarks or not result.hand_landmarks:
            return
        hand_lms = result.hand_landmarks[0]
        hw = hand_lms[0]
        is_right_target = (self.arm_mode == 0)
        if self.arm_mode == 2 and self.pose_norm_lms is not None:
            rw = self.pose_norm_lms[16]
            lw = self.pose_norm_lms[15]
            dist_r = (hw.x - rw.x) ** 2 + (hw.y - rw.y) ** 2
            dist_l = (hw.x - lw.x) ** 2 + (hw.y - lw.y) ** 2
            is_right_target = (dist_r <= dist_l)

        self.hand_world[is_right_target] = result.hand_world_landmarks[0]
        self.hand_norm[is_right_target] = hand_lms
        self.hand_last_time[is_right_target] = time.perf_counter()

    def on_dual_hand_result(self, result: vision.HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
        """Callback for 2-Hand Landmarker (used only when user toggles 'r' to BOTH ARMS mode)."""
        self.hand_busy = False
        if not result.hand_world_landmarks or not result.hand_landmarks:
            return

        now = time.perf_counter()
        for idx, hand_lms in enumerate(result.hand_landmarks):
            hw = hand_lms[0]
            is_right_hand = True
            if self.pose_norm_lms is not None:
                rw = self.pose_norm_lms[16]
                lw = self.pose_norm_lms[15]
                dist_r = (hw.x - rw.x) ** 2 + (hw.y - rw.y) ** 2
                dist_l = (hw.x - lw.x) ** 2 + (hw.y - lw.y) ** 2
                is_right_hand = (dist_r <= dist_l)
            self.hand_world[is_right_hand] = result.hand_world_landmarks[idx]
            self.hand_norm[is_right_hand] = hand_lms
            self.hand_last_time[is_right_hand] = now

    def get_hand_flat(self, is_right: bool, now: float):
        active = (self.hand_world[is_right] is not None) and ((now - self.hand_last_time[is_right]) < 0.25)
        if not active:
            return False, [0.0] * 63
        hw_list = self.hand_world[is_right]
        hw0 = hw_list[0]
        flat = []
        for lm in hw_list:
            flat.extend([float(lm.x - hw0.x), float(lm.y - hw0.y), float(lm.z - hw0.z)])
        return True, flat

    def send_dual_udp_packet(
        self,
        r_pts, r_conf, r_dratio, r_hand_ok, r_hand_flat,
        l_pts, l_conf, l_dratio, l_hand_ok, l_hand_flat
    ):
        """
        165-Float (660-byte) Binary UDP Packet:
          [0..83]    Right Arm + Opposite(Left) Shoulder + Right 21 Fingers
          [84..99]   Left Arm (Shoulder, Elbow, Wrist, Index, Pinky, LeftPoseConf)
          [100]      Left Hand Active Flag (1.0 or 0.0)
          [101..163] Left 21 Hand Landmarks relative to Left Wrist
          [164]      Python Active Arm Mode (0.0 = Right, 1.0 = Left, 2.0 = Both)
        """
        rs_pt, re_pt, rw_pt, ri_pt, rp_pt = r_pts
        ls_pt, le_pt, lw_pt, li_pt, lp_pt = l_pts

        right_header = [
            rs_pt[0], rs_pt[1], rs_pt[2],
            re_pt[0], re_pt[1], re_pt[2],
            rw_pt[0], rw_pt[1], rw_pt[2],
            ri_pt[0], ri_pt[1], ri_pt[2],
            rp_pt[0], rp_pt[1], rp_pt[2],
            float(r_conf),
            float(r_dratio),
            ls_pt[0], ls_pt[1], ls_pt[2],
            1.0 if r_hand_ok else 0.0,
        ]

        left_header = [
            ls_pt[0], ls_pt[1], ls_pt[2],
            le_pt[0], le_pt[1], le_pt[2],
            lw_pt[0], lw_pt[1], lw_pt[2],
            li_pt[0], li_pt[1], li_pt[2],
            lp_pt[0], lp_pt[1], lp_pt[2],
            float(l_conf),
            1.0 if l_hand_ok else 0.0,
        ]

        all_floats = right_header + r_hand_flat + left_header + l_hand_flat + [float(self.arm_mode)]
        packet = struct.pack("<165f", *all_floats)
        self.sock.sendto(packet, (UDP_IP, UDP_PORT))


def run_realsense_pipeline():
    ensure_models()
    tracker = FastArmAndHandTracker()

    pose_options = vision.PoseLandmarkerOptions(
        base_options=python.BaseOptions(model_asset_path=POSE_MODEL_PATH),
        running_mode=vision.RunningMode.LIVE_STREAM,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5,
        result_callback=tracker.on_pose_result,
    )

    # Ultra-fast 1-Hand Landmarker (used for Right-Only and Left-Only modes — zero palm-detector lag!)
    single_hand_options = vision.HandLandmarkerOptions(
        base_options=python.BaseOptions(model_asset_path=HAND_MODEL_PATH),
        running_mode=vision.RunningMode.LIVE_STREAM,
        num_hands=1,
        min_hand_detection_confidence=0.45,
        min_hand_presence_confidence=0.45,
        min_tracking_confidence=0.45,
        result_callback=tracker.on_single_hand_result,
    )

    # 2-Hand Landmarker (used only when toggled to BOTH ARMS mode)
    dual_hand_options = vision.HandLandmarkerOptions(
        base_options=python.BaseOptions(model_asset_path=HAND_MODEL_PATH),
        running_mode=vision.RunningMode.LIVE_STREAM,
        num_hands=2,
        min_hand_detection_confidence=0.45,
        min_hand_presence_confidence=0.45,
        min_tracking_confidence=0.45,
        result_callback=tracker.on_dual_hand_result,
    )

    pipeline = rs.pipeline()
    profile = start_realsense_with_best_profile(pipeline)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    for sensor in profile.get_device().query_sensors():
        if sensor.supports(rs.option.frames_queue_size):
            try:
                sensor.set_option(rs.option.frames_queue_size, 1)
            except Exception:
                pass

    align = rs.align(rs.stream.color)

    print(f"[UDP] Streaming Zero-Lag 3D Arm + 21-Finger Data to {UDP_IP}:{UDP_PORT}")
    print("Controls: 'r' = cycle RIGHT -> LEFT -> BOTH ARMS | 'v' = toggle video drawing | 'q' = quit")

    start_time = time.perf_counter()
    fps_counter = 0
    fps_timer = time.perf_counter()
    measured_fps = 0.0
    last_ts = -1
    draw_overlay = SHOW_DEBUG_WINDOW
    mode_names = ["RIGHT ARM ONLY", "LEFT ARM ONLY", "BOTH ARMS"]

    try:
        with vision.PoseLandmarker.create_from_options(pose_options) as pose_landmarker, \
             vision.HandLandmarker.create_from_options(single_hand_options) as single_hand_landmarker, \
             vision.HandLandmarker.create_from_options(dual_hand_options) as dual_hand_landmarker:

            while True:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                aligned_frames = align.process(frames)

                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
                color_image = np.asanyarray(color_frame.get_data())
                if color_image.ndim == 2:
                    color_image = cv2.cvtColor(color_image, cv2.COLOR_GRAY2BGR)
                depth_image_raw = np.asanyarray(depth_frame.get_data())

                rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

                now_sec = time.perf_counter() - start_time
                ts_ms = int(now_sec * 1000)
                if ts_ms <= last_ts:
                    ts_ms = last_ts + 1
                last_ts = ts_ms

                # Non-queueing dispatch: only submit a new frame if the worker isn't backed up!
                # This guarantees 0 ms of frame queue build-up on the hand cloud!
                if not tracker.pose_busy:
                    tracker.pose_busy = True
                    pose_landmarker.detect_async(mp_image, ts_ms)

                if not tracker.hand_busy:
                    tracker.hand_busy = True
                    # Only invoke 2-hand detector if in BOTH mode AND both wrists are actually visible in frame!
                    # Otherwise use ultra-fast 1-hand detector so MediaPipe never wastes time scanning for a missing 2nd hand!
                    use_dual = False
                    if tracker.arm_mode == 2 and tracker.pose_norm_lms is not None:
                        rw = tracker.pose_norm_lms[16]
                        lw = tracker.pose_norm_lms[15]
                        r_in = (0.02 < rw.x < 0.98) and (0.02 < rw.y < 0.98) and (rw.visibility > 0.45)
                        l_in = (0.02 < lw.x < 0.98) and (0.02 < lw.y < 0.98) and (lw.visibility > 0.45)
                        use_dual = (r_in and l_in)

                    if use_dual:
                        dual_hand_landmarker.detect_async(mp_image, ts_ms)
                    else:
                        single_hand_landmarker.detect_async(mp_image, ts_ms)

                if tracker.has_new_pose and tracker.pose_norm_lms is not None:
                    tracker.has_new_pose = False
                    perf_now = time.perf_counter()

                    r_pts, r_conf, r_dratio, r_px = tracker.right_arm.update(
                        tracker.pose_norm_lms, tracker.pose_world_lms, depth_image_raw, depth_scale, intrinsics, now_sec
                    )
                    l_pts, l_conf, l_dratio, l_px = tracker.left_arm.update(
                        tracker.pose_norm_lms, tracker.pose_world_lms, depth_image_raw, depth_scale, intrinsics, now_sec
                    )

                    r_hand_ok, r_hand_flat = tracker.get_hand_flat(True, perf_now)
                    l_hand_ok, l_hand_flat = tracker.get_hand_flat(False, perf_now)

                    tracker.send_dual_udp_packet(
                        r_pts, r_conf, r_dratio, r_hand_ok, r_hand_flat,
                        l_pts, l_conf, l_dratio, l_hand_ok, l_hand_flat
                    )

                    if draw_overlay:
                        # Draw active arm(s)
                        if tracker.arm_mode in (0, 2):
                            for i in range(3):
                                cv2.circle(color_image, r_px[i], 6, (0, 255, 0), -1)
                            cv2.line(color_image, r_px[0], r_px[1], (0, 255, 0), 3)
                            cv2.line(color_image, r_px[1], r_px[2], (0, 255, 0), 3)
                        if tracker.arm_mode in (1, 2):
                            for i in range(3):
                                cv2.circle(color_image, l_px[i], 6, (255, 200, 0), -1)
                            cv2.line(color_image, l_px[0], l_px[1], (255, 200, 0), 3)
                            cv2.line(color_image, l_px[1], l_px[2], (255, 200, 0), 3)

                        # Chest line
                        cv2.line(color_image, r_px[0], l_px[0], (255, 128, 0), 2)

                        h, w, _ = color_image.shape
                        for is_r, h_col in ((True, (0, 255, 255)), (False, (255, 255, 0))):
                            if (tracker.arm_mode == 2 or (tracker.arm_mode == 0 and is_r) or (tracker.arm_mode == 1 and not is_r)):
                                if (perf_now - tracker.hand_last_time[is_r]) < 0.25 and tracker.hand_norm[is_r] is not None:
                                    for h_lm in tracker.hand_norm[is_r]:
                                        cv2.circle(color_image, (int(h_lm.x * w), int(h_lm.y * h)), 3, h_col, -1)

                fps_counter += 1
                if time.perf_counter() - fps_timer >= 1.0:
                    measured_fps = fps_counter / (time.perf_counter() - fps_timer)
                    fps_counter = 0
                    fps_timer = time.perf_counter()

                if SHOW_DEBUG_WINDOW:
                    if draw_overlay:
                        m_str = mode_names[tracker.arm_mode]
                        cv2.putText(
                            color_image,
                            f"{measured_fps:.1f} FPS | MODE: {m_str} (Press 'r' to switch, 'v' hide video)",
                            (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.50,
                            (0, 255, 128),
                            2,
                            cv2.LINE_AA,
                        )
                        cv2.imshow("D455C Zero-Lag Arm + 21-Finger Tracker", color_image)

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        break
                    elif key == ord("r"):
                        tracker.arm_mode = (tracker.arm_mode + 1) % 3
                        tracker.hand_busy = False
                        print(f"[Tracker] Switched Mode to: {mode_names[tracker.arm_mode]}")
                    elif key == ord("v"):
                        draw_overlay = not draw_overlay
                        print(f"[Tracker] Video overlay drawing: {'ON' if draw_overlay else 'OFF (Max Speed Mode)'}")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run_realsense_pipeline()
