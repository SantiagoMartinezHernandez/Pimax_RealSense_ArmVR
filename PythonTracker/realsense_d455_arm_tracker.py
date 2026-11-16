"""
Intel RealSense D455C + MediaPipe Arm & 21-Finger 3D Tracker (v6.0 - Rigid-Link + Kalman)
=========================================================================================
Major Upgrades in v6.0:
  1. Rigid-Link Forward Kinematics + 1D Torso-Depth Kalman Filtering:
     - Replaces independent per-joint pixel-depth deprojection (which caused silhouette
       depth spikes during 10cm arm movements) with a heavily damped 1D Kalman Filter on
       the shoulder/torso depth (`11x11` foreground patch).
     - Continuously estimates metric arm scale via slow Kalman updates when high-visibility
       measurements are available, eliminating mandatory per-session scale calibration.
     - Extracts smooth unit bone direction vectors for Upper Arm and Forearm and reconstructs
       Elbow and Wrist strictly via Rigid-Link Forward Kinematics BEFORE updating velocities,
       eliminating limit-cycle chatter.
  2. Adaptive Single/Dual 21-Finger Hand Gate + Dual-Arm Streaming (660-byte UDP @ 60 Hz).
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


def start_realsense_with_best_profile(pipeline: rs.pipeline, prefer_low_bandwidth: bool = False):
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        raise RuntimeError(
            "No Intel RealSense device detected! Please check:\n"
            "  1. The D455C USB cable is plugged into a USB 3.0 / Thunderbolt port.\n"
            "  2. Windows Settings -> Privacy & security -> Camera -> 'Camera access' & 'Let desktop apps access your camera' are ON."
        )

    dev = devices[0]
    dev_name = dev.get_info(rs.camera_info.name) if dev.supports(rs.camera_info.name) else "Unknown RealSense"
    usb_type = dev.get_info(rs.camera_info.usb_type_descriptor) if dev.supports(rs.camera_info.usb_type_descriptor) else "Unknown"
    serial = dev.get_info(rs.camera_info.serial_number) if dev.supports(rs.camera_info.serial_number) else "Unknown"

    print(f"[RealSense] Found Device: {dev_name} (S/N: {serial}) on USB {usb_type}")

    if prefer_low_bandwidth:
        # Low USB-bandwidth profiles for mini-PCs (like ASUS ROG NUC) when sharing USB controller with Pimax VR
        candidates = [
            (640, 480, 640, 480, 30),
            (640, 360, 640, 360, 30),
            (480, 270, 640, 480, 30),
            (424, 240, 424, 240, 30),
            (640, 480, 640, 480, 15),
        ]
    else:
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
            print(f"[RealSense] Selected stream profile: Depth {dw}x{dh} + Color {cw}x{ch} @ {fps} FPS")
            return pipeline.start(cfg)

    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.depth)
    cfg.enable_stream(rs.stream.color, rs.format.bgr8)
    return pipeline.start(cfg)


def start_and_verify_first_frame(pipeline: rs.pipeline):
    """
    Starts the RealSense pipeline and waits up to 5000ms for the first frame.
    If the USB controller on a NUC/Laptop drops frames at 60 FPS (due to sharing bandwidth
    with a VR headset) or if the sensor is locked from a previous run, automatically
    performs a hardware reset / falls back to a 30 FPS low-bandwidth USB profile!
    """
    for attempt in range(3):
        low_bw = (attempt >= 1)
        profile = start_realsense_with_best_profile(pipeline, prefer_low_bandwidth=low_bw)
        try:
            print("[RealSense] Waiting for sensor warmup and first frame (up to 5s)...")
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            if frames:
                print("[RealSense] First frame received successfully!")
                return profile
        except RuntimeError as e:
            print(f"[RealSense] Warmup timeout on attempt {attempt + 1}: {e}")
            try:
                pipeline.stop()
            except Exception:
                pass

            if attempt == 1:
                print("[RealSense] Performing USB Hardware Reset on D455C sensor (waiting 4 seconds to re-enumerate)...")
                try:
                    ctx = rs.context()
                    if len(ctx.query_devices()) > 0:
                        ctx.query_devices()[0].hardware_reset()
                        time.sleep(4.5)
                except Exception as reset_err:
                    print(f"[RealSense] Hardware reset warning: {reset_err}")
            else:
                print("[RealSense] Retrying with lower USB-bandwidth 30 FPS profile (ideal when sharing USB hub with Pimax VR)...")
                time.sleep(1.0)

    raise RuntimeError(
        "RealSense D455C started but could not deliver frames over USB!\n"
        "On ASUS ROG NUC: Move the D455C to the FRONT USB 3.2 port (or Thunderbolt USB-C port) "
        "so it does not share the rear USB root hub with the Pimax Crystal headset."
    )


class KinematicArmState:
    """
    v6.0 Rigid-Link Forward Kinematics + 1D Torso-Depth Kalman Sensor Fusion.
    Decouples torso depth estimation (slow 1D Kalman Filter on an 11x11 shoulder patch)
    from rigid-link arm rotations (smooth unit bone vectors reconstructed via fixed-length
    forward kinematics BEFORE velocity updates, preventing bone-clamp limit cycles).
    """
    BASE_UPPER_LEN = 0.28
    BASE_FORE_LEN = 0.25

    def __init__(self, is_right: bool):
        self.is_right = is_right
        self.indices = (12, 14, 16, 20, 18) if is_right else (11, 13, 15, 19, 17)

        # 1D Kalman state for Torso/Shoulder depth (meters)
        self.torso_z_est = 0.70
        self.torso_z_var = 0.05
        self.torso_z_q = 0.0002
        self.torso_z_r = 0.015

        # 1D Kalman state for continuous metric arm scale (eliminates manual scale calibration)
        self.est_arm_scale = 1.0
        self.arm_scale_var = 0.04
        self.arm_scale_q = 0.00005
        self.arm_scale_r = 0.02

        # Smoothed relative depth deltas for Elbow (0) and Wrist (1)
        self.rel_z_smooth = [0.0, 0.0]

        # Low-bandwidth 3D state for Shoulder in camera space
        self.shoulder_cam_pos = None

        # Unit bone direction vectors & kinematic state
        self.prev_u_upper = None
        self.prev_u_fore = None
        self.prev_pts = None
        self.prev_vel = [np.zeros(3, dtype=np.float32) for _ in range(5)]
        self.prev_time = None

    @staticmethod
    def sample_depth(depth_image_raw: np.ndarray, depth_scale: float, u: int, v: int, patch_radius: int = 3) -> float:
        h, w = depth_image_raw.shape
        u0, u1 = max(0, u - patch_radius), min(w, u + patch_radius + 1)
        v0, v1 = max(0, v - patch_radius), min(h, v + patch_radius + 1)
        patch = depth_image_raw[v0:v1, u0:u1]
        valid = patch[(patch > int(MIN_DEPTH_M / depth_scale)) & (patch < int(MAX_DEPTH_M / depth_scale))]
        if valid.size < 3:
            return 0.0
        return float(np.percentile(valid, 35)) * depth_scale

    @staticmethod
    def _normalize_vec(v: np.ndarray, fallback: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(v))
        if norm > 1e-5:
            return (v / norm).astype(np.float32)
        return fallback.copy()

    def update(self, norm_lms, world_lms, depth_image_raw: np.ndarray, depth_scale: float, intrinsics, now_sec: float):
        h, w = depth_image_raw.shape
        dt = 0.016 if self.prev_time is None else max(0.002, min(0.15, now_sec - self.prev_time))
        self.prev_time = now_sec

        px_coords = []
        vis_list = []
        for idx in self.indices:
            lm = norm_lms[idx]
            u = int(np.clip(lm.x * w, 0, w - 1))
            v = int(np.clip(lm.y * h, 0, h - 1))
            px_coords.append((u, v))
            vis_list.append(float(lm.visibility))

        # 1. 1D Kalman Filter on Shoulder/Torso Depth using wider 11x11 patch (patch_radius=5)
        u_s, v_s = px_coords[0]
        d_shoulder = self.sample_depth(depth_image_raw, depth_scale, u_s, v_s, patch_radius=5)

        self.torso_z_var += self.torso_z_q
        if MIN_DEPTH_M <= d_shoulder <= MAX_DEPTH_M:
            # Huber-bounded innovation so sudden arm/hand pass over shoulder never pulls torso depth
            innov_z = float(np.clip(d_shoulder - self.torso_z_est, -0.05, 0.05))
            r_adapt = self.torso_z_r / max(vis_list[0] ** 2, 0.15)
            k_z = self.torso_z_var / (self.torso_z_var + r_adapt)
            self.torso_z_est += k_z * innov_z
            self.torso_z_var *= (1.0 - k_z)

        # Deproject Shoulder at Kalman-filtered torso_z_est and apply low-bandwidth 3D state filter
        s_deproj = np.array(
            rs.rs2_deproject_pixel_to_point(intrinsics, [float(u_s), float(v_s)], float(self.torso_z_est)),
            dtype=np.float32,
        )
        if self.shoulder_cam_pos is None:
            self.shoulder_cam_pos = s_deproj.copy()
        else:
            s_diff = s_deproj - self.shoulder_cam_pos
            s_dist = float(np.linalg.norm(s_diff))
            max_s_step = MAX_SHOULDER_SPEED_MPS * dt
            if s_dist > max_s_step and s_dist > 1e-5:
                s_diff = s_diff * (max_s_step / s_dist)
            self.shoulder_cam_pos = self.shoulder_cam_pos + 0.25 * s_diff

        # 2. Elbow (k=1) & Wrist (k=2) Direction Estimation with Gentle Huber Depth Innovation
        s_mp = world_lms[self.indices[0]]
        raw_joint_pos = [self.shoulder_cam_pos]
        depth_valid_flags = [MIN_DEPTH_M <= d_shoulder <= MAX_DEPTH_M, False, False]

        for k in (1, 2):
            idx = self.indices[k]
            u_k, v_k = px_coords[k]
            lm_w = world_lms[idx]
            mp_rel_vec = np.array([lm_w.x - s_mp.x, lm_w.y - s_mp.y, lm_w.z - s_mp.z], dtype=np.float32)
            mp_rel_z = float(mp_rel_vec[2])

            d_m = self.sample_depth(depth_image_raw, depth_scale, u_k, v_k, patch_radius=3)
            expected_abs_z = self.torso_z_est + mp_rel_z

            if MIN_DEPTH_M <= d_m <= MAX_DEPTH_M and abs(d_m - expected_abs_z) < 0.18:
                # Gentle Huber-bounded innovation (max +-0.20 weight) into smoothed relative depth state
                d_rel_meas = d_m - self.torso_z_est
                innov_rel = float(np.clip(d_rel_meas - mp_rel_z, -0.06, 0.06))
                target_rel_z = mp_rel_z + 0.20 * innov_rel
                self.rel_z_smooth[k - 1] = 0.20 * target_rel_z + 0.80 * self.rel_z_smooth[k - 1]
                depth_valid_flags[k] = True
            else:
                self.rel_z_smooth[k - 1] = 0.20 * mp_rel_z + 0.80 * self.rel_z_smooth[k - 1]

            fused_abs_z = max(MIN_DEPTH_M, self.torso_z_est + self.rel_z_smooth[k - 1])
            cam_deproj = np.array(
                rs.rs2_deproject_pixel_to_point(intrinsics, [float(u_k), float(v_k)], float(fused_abs_z)),
                dtype=np.float32,
            )
            cam_rel_vec = cam_deproj - self.shoulder_cam_pos
            # Blend 50% optical camera ray vector + 50% MediaPipe relative world vector
            blended_rel_vec = 0.50 * cam_rel_vec + 0.50 * mp_rel_vec
            raw_joint_pos.append(self.shoulder_cam_pos + blended_rel_vec)

        raw_upper_vec = raw_joint_pos[1] - raw_joint_pos[0]
        raw_fore_vec = raw_joint_pos[2] - raw_joint_pos[1]
        raw_upper_len = float(np.linalg.norm(raw_upper_vec))
        raw_fore_len = float(np.linalg.norm(raw_fore_vec))

        # 3. Slow Kalman Estimate of Metric Arm Scale when visibility is high (> 0.65)
        self.arm_scale_var += self.arm_scale_q
        min_vis = min(vis_list[0], vis_list[1], vis_list[2])
        if min_vis > 0.65 and 0.30 < (raw_upper_len + raw_fore_len) < 0.90:
            meas_scale = (raw_upper_len + raw_fore_len) / (self.BASE_UPPER_LEN + self.BASE_FORE_LEN)
            meas_scale = float(np.clip(meas_scale, 0.72, 1.35))
            k_s = self.arm_scale_var / (self.arm_scale_var + self.arm_scale_r)
            self.est_arm_scale += k_s * (meas_scale - self.est_arm_scale)
            self.arm_scale_var *= (1.0 - k_s)

        # 4. Extract Unit Bone Direction Vectors & Reconstruct via Rigid-Link Forward Kinematics
        default_upper = np.array([0.15 if self.is_right else -0.15, -0.85, 0.40], dtype=np.float32)
        default_upper = self._normalize_vec(default_upper, np.array([0.0, -1.0, 0.0], dtype=np.float32))
        default_fore = np.array([0.0, -0.25, 0.95], dtype=np.float32)
        default_fore = self._normalize_vec(default_fore, np.array([0.0, 0.0, 1.0], dtype=np.float32))

        u_upper = self._normalize_vec(raw_upper_vec, self.prev_u_upper if self.prev_u_upper is not None else default_upper)
        u_fore = self._normalize_vec(raw_fore_vec, self.prev_u_fore if self.prev_u_fore is not None else default_fore)

        if self.prev_u_upper is None:
            self.prev_u_upper = u_upper
            self.prev_u_fore = u_fore
        else:
            vis_u = np.clip(0.5 * (vis_list[0] + vis_list[1]), 0.30, 1.0)
            vis_f = np.clip(0.5 * (vis_list[1] + vis_list[2]), 0.30, 1.0)
            self.prev_u_upper = self._normalize_vec((1.0 - 0.68 * vis_u) * self.prev_u_upper + (0.68 * vis_u) * u_upper, u_upper)
            self.prev_u_fore = self._normalize_vec((1.0 - 0.72 * vis_f) * self.prev_u_fore + (0.72 * vis_f) * u_fore, u_fore)

        len_upper = self.BASE_UPPER_LEN * self.est_arm_scale
        len_fore = self.BASE_FORE_LEN * self.est_arm_scale

        # Strictly rigid-link forward kinematics BEFORE updating velocity states (zero limit-cycle chatter!)
        fk_shoulder = self.shoulder_cam_pos
        fk_elbow = fk_shoulder + self.prev_u_upper * len_upper
        fk_wrist = fk_elbow + self.prev_u_fore * len_fore

        # Index (k=3) and Pinky (k=4) from MediaPipe relative wrist offsets anchored to fk_wrist
        w_mp = world_lms[self.indices[2]]
        i_mp = world_lms[self.indices[3]]
        p_mp = world_lms[self.indices[4]]
        fk_index = fk_wrist + np.array([i_mp.x - w_mp.x, i_mp.y - w_mp.y, i_mp.z - w_mp.z], dtype=np.float32)
        fk_pinky = fk_wrist + np.array([p_mp.x - w_mp.x, p_mp.y - w_mp.y, p_mp.z - w_mp.z], dtype=np.float32)

        fk_pts = [fk_shoulder, fk_elbow, fk_wrist, fk_index, fk_pinky]

        if self.prev_pts is None:
            self.prev_pts = [pt.copy() for pt in fk_pts]
        else:
            for k in range(5):
                self.prev_vel[k] = 0.6 * ((fk_pts[k] - self.prev_pts[k]) / dt) + 0.4 * self.prev_vel[k]
                self.prev_pts[k] = fk_pts[k]

        conf = (vis_list[0] + vis_list[1] + vis_list[2]) / 3.0
        valid_ratio = sum(1.0 for flag in depth_valid_flags if flag) / 3.0
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
    profile = start_and_verify_first_frame(pipeline)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Keep frames_queue_size >= 4 so librealsense's Depth+Color hardware syncer never drops
    # a frame before its timestamp-matched partner arrives on Intel Core Ultra / ROG NUC USB controllers!
    for sensor in profile.get_device().query_sensors():
        if sensor.supports(rs.option.frames_queue_size):
            try:
                sensor.set_option(rs.option.frames_queue_size, 4)
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
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=3000)
                except RuntimeError as wait_err:
                    print(f"[RealSense] Frame sync hiccup ({wait_err}), continuing...")
                    continue
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
