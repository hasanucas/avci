#!/usr/bin/env python3
"""
PN Teacher v2
==============
v1'den farklar:
  1) İlk frame fix: prev_ang_x ilk geçerli obs'tan initialize
  2) pn_to_roll: 0.012 → 0.025 (daha agresif PN tepkisi)
  3) Roll gating bbox tabanlı:
     - bbox < 400px² (norm ~0.0013): gating aktif (uzakta yaw ile dön)
     - bbox > 400px²: gating KAPALI (yakında roll serbest)
"""

import math
import numpy as np


class PrivilegedState:
    def __init__(self):
        self.drone_vel_ned = np.zeros(3)
        self.drone_att_rad = np.zeros(3)
        self.altitude = 0.0
        self.valid = False
    
    def update(self, state_dict):
        if not state_dict.get('valid', True):
            return False
        self.drone_vel_ned = np.asarray(state_dict['vel_ned'], dtype=np.float64)
        self.drone_att_rad = np.asarray(state_dict['att_rad'], dtype=np.float64)
        self.altitude = float(state_dict['altitude'])
        self.valid = True
        return True
    
    def update_direct(self, vel_ned, att_rad, altitude):
        self.drone_vel_ned = np.asarray(vel_ned, dtype=np.float64)
        self.drone_att_rad = np.asarray(att_rad, dtype=np.float64)
        self.altitude = float(altitude)
        self.valid = True
    
    def get_body_velocity(self):
        if not self.valid:
            return 0.0, 0.0, 0.0
        yaw = self.drone_att_rad[2]
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        vx, vy, vz = self.drone_vel_ned
        return cos_y*vx + sin_y*vy, -sin_y*vx + cos_y*vy, vz
    
    def get_horizontal_speed(self):
        if not self.valid:
            return 0.0
        vx, vy = self.drone_vel_ned[0], self.drone_vel_ned[1]
        return math.sqrt(vx*vx + vy*vy)


class PNTeacher:
    def __init__(self):
        # === PN ===
        self.N = 3.0
        self.V_own_default = 15.0
        
        # === LOS Rate ===
        self.prev_ang_x = None         # None = henüz initialize edilmedi
        self.prev_ang_y = None
        self.los_rate_az = 0.0
        self.los_rate_el = 0.0
        self.los_rate_alpha = 0.36
        
        # === PN → Komut ===
        self.pn_to_roll = 0.025        # v1: 0.012 → 0.025
        self.pn_to_yaw = 0.006
        self.max_roll = 0.18
        
        # === YAW ===
        self.kp_yaw = 0.027
        self.kd_yaw = 0.010
        self.max_yaw = 0.3
        
        # === ROLL ek ===
        self.kp_roll_attitude = 0.012
        self.kd_roll = 0.003
        
        # === Roll gating — bbox tabanlı ===
        # Uzakta (bbox küçük): gating aktif → büyük ang_x'te roll kapalı
        # Yakında (bbox büyük): gating devre dışı → roll her zaman serbest
        self.roll_angx_full = 3.0
        self.roll_angx_zero = 8.0
        self.roll_gating_bbox_threshold = 0.0013  # ~400px² (20x20 / 640x480)
        
        # === PITCH ===
        self.kp_pitch_down = 0.03
        self.kp_pitch_up = 0.025
        self.kd_pitch = 0.005
        self.max_pitch_down = 0.08
        self.max_pitch_up = 0.06
        self.bbox_pitch_min = 0.0001
        self.bbox_pitch_max = 0.002
        self.pitch_up_boost_mult = 5.0
        self.pitch_up_max_boost = 4.0
        self.max_pitch_down_far = 0.055
        self.max_pitch_down_near = 0.08
        self.bbox_pitch_down_far = 0.0005
        self.bbox_pitch_down_near = 0.005
        
        # === Privileged ===
        self.kv_lateral_roll = 0.015
        self.max_forward_speed = 30.0
        self.kv_speed_brake = 0.008

        # === Throttle ===
        self.hover_thrust = 0.07
        self.prev_bbox_size = 0.0
        self.bbox_rate = 0.0
        self.bbox_rate_alpha = 0.2

        # === Smoothing ===
        self.alpha = 0.35
        self.roll_rate_limit = 0.08
        self.pitch_rate_limit = 0.04
        self.yaw_rate_limit = 0.12

        self.lost_yaw_rate = 0.0
        self.prev_action = np.array([0.0, 0.0, 0.035, 0.0])
        self.step_count = 0

    def _roll_gating(self, ang_x, bbox_area):
        """
        Bbox tabanlı roll gating:
          bbox büyük (yakın) → gate=1.0 (roll serbest)
          bbox küçük (uzak) → ang_x'e göre gate (büyük ang_x'te roll kapalı)
        """
        if bbox_area >= self.roll_gating_bbox_threshold:
            return 1.0  # yakında: roll her zaman serbest
        
        # Uzakta: ang_x tabanlı gating
        abs_ax = abs(ang_x)
        if abs_ax <= self.roll_angx_full:
            return 1.0
        elif abs_ax >= self.roll_angx_zero:
            return 0.0
        return (self.roll_angx_zero - abs_ax) / (self.roll_angx_zero - self.roll_angx_full)

    def compute_action(self, obs, privileged=None):
        ang_x      = obs[4]
        ang_y      = obs[5]
        bbox_w     = obs[2]
        bbox_h     = obs[3]
        roll_deg   = obs[6]
        pitch_deg  = obs[7]
        roll_rate  = obs[9]
        pitch_rate = obs[10]
        yaw_rate   = obs[11]
        bbox_valid = obs[13]

        self.step_count += 1
        bbox_area = bbox_w * bbox_h
        bbox_size = math.sqrt(max(bbox_area, 1e-10))

        lateral_speed = 0.0
        horizontal_speed = self.V_own_default
        if privileged is not None and privileged.valid:
            _, lateral_speed, _ = privileged.get_body_velocity()
            hs = privileged.get_horizontal_speed()
            if hs > 1.0:
                horizontal_speed = hs

        # === LOS RATE — strapdown + ilk frame fix ===
        if bbox_valid > 0.5:
            # İlk frame fix: prev değerlerini ilk geçerli obs'tan al
            if self.prev_ang_x is None:
                self.prev_ang_x = ang_x
                self.prev_ang_y = ang_y
                self.prev_bbox_size = bbox_size
            
            raw_az = (ang_x - self.prev_ang_x) * (math.pi / 180.0) * 30.0
            raw_el = (ang_y - self.prev_ang_y) * (math.pi / 180.0) * 30.0
            
            yaw_rate_rad = yaw_rate * (math.pi / 180.0)
            pitch_rate_rad = pitch_rate * (math.pi / 180.0)
            
            inertial_az = raw_az + yaw_rate_rad
            inertial_el = raw_el + pitch_rate_rad
            
            self.los_rate_az = (self.los_rate_alpha * inertial_az +
                                (1 - self.los_rate_alpha) * self.los_rate_az)
            self.los_rate_el = (self.los_rate_alpha * inertial_el +
                                (1 - self.los_rate_alpha) * self.los_rate_el)
            
            self.prev_ang_x = ang_x
            self.prev_ang_y = ang_y
            
            raw_bbox_rate = (bbox_size - self.prev_bbox_size) * 30.0
            self.bbox_rate = (self.bbox_rate_alpha * raw_bbox_rate +
                              (1 - self.bbox_rate_alpha) * self.bbox_rate)
            self.prev_bbox_size = bbox_size
        else:
            self.los_rate_az *= 0.9
            self.los_rate_el *= 0.9
            self.bbox_rate *= 0.8

        if bbox_valid < 0.5:
            target_action = np.array([0.0, 0.0, self.hover_thrust, self.lost_yaw_rate])
        else:
            # === PN KOMUTU ===
            V_own = max(horizontal_speed, 5.0)
            a_lateral = self.N * V_own * self.los_rate_az

            # === YAW ===
            yaw_cmd = (self.kp_yaw * ang_x 
                       - self.kd_yaw * yaw_rate
                       + self.pn_to_yaw * a_lateral)
            yaw_cmd = np.clip(yaw_cmd, -self.max_yaw, self.max_yaw)
            self.lost_yaw_rate = np.clip(yaw_cmd, -0.05, 0.05)

            # === PITCH ===
            if ang_y >= 0:
                if bbox_area <= self.bbox_pitch_down_far:
                    pdf = 0.0
                elif bbox_area >= self.bbox_pitch_down_near:
                    pdf = 1.0
                else:
                    pdf = ((bbox_area - self.bbox_pitch_down_far) /
                           (self.bbox_pitch_down_near - self.bbox_pitch_down_far))
                max_pd = self.max_pitch_down_far + \
                         (self.max_pitch_down_near - self.max_pitch_down_far) * pdf
                pitch_cmd = -self.kp_pitch_down * ang_y - self.kd_pitch * pitch_rate
                pitch_cmd = np.clip(pitch_cmd, -max_pd, self.max_pitch_up)
            else:
                if bbox_area <= self.bbox_pitch_min:
                    bf = 0.0
                elif bbox_area >= self.bbox_pitch_max:
                    bf = 1.0
                else:
                    bf = ((bbox_area - self.bbox_pitch_min) /
                          (self.bbox_pitch_max - self.bbox_pitch_min))
                kp_eff = self.kp_pitch_up * (1.0 + self.pitch_up_boost_mult * bf)
                max_up = self.max_pitch_up * (1.0 + self.pitch_up_max_boost * bf)
                pitch_cmd = -kp_eff * ang_y - self.kd_pitch * pitch_rate
                pitch_cmd = np.clip(pitch_cmd, -self.max_pitch_down, max_up)

            if horizontal_speed > self.max_forward_speed:
                pitch_cmd += self.kv_speed_brake * (horizontal_speed - self.max_forward_speed)
                pitch_cmd = np.clip(pitch_cmd, -self.max_pitch_down, self.max_pitch_up)

            # === ROLL — PN + bbox gating ===
            gate = self._roll_gating(ang_x, bbox_area)
            
            roll_pn = self.pn_to_roll * a_lateral * gate
            roll_att = -self.kp_roll_attitude * roll_deg
            roll_damp = -self.kd_roll * roll_rate
            roll_lat = -self.kv_lateral_roll * lateral_speed
            
            roll_cmd = roll_pn + roll_att + roll_damp + roll_lat
            roll_cmd = np.clip(roll_cmd, -self.max_roll, self.max_roll)

            # === THROTTLE ===
            pitch_rad = math.radians(pitch_deg)
            roll_rad = math.radians(roll_deg)
            cos_tilt = math.cos(pitch_rad) * math.cos(roll_rad)
            tilt_comp = (1.0 / max(cos_tilt, 0.5) - 1.0) * self.hover_thrust
            
            if self.bbox_rate > 0.001:
                tau = bbox_size / self.bbox_rate
            else:
                tau = 99.0
            
            if tau > 5.0:
                base_t = self.hover_thrust
            elif tau > 2.0:
                base_t = self.hover_thrust + 0.02
            else:
                base_t = self.hover_thrust + 0.04
            
            throttle_cmd = np.clip(base_t + tilt_comp, -0.2, 0.5)

            target_action = np.array([roll_cmd, pitch_cmd, throttle_cmd, yaw_cmd],
                                     dtype=np.float32)

        # Smoothing + rate limit
        smoothed = self.alpha * target_action + (1 - self.alpha) * self.prev_action
        limits = np.array([self.roll_rate_limit, self.pitch_rate_limit,
                           self.roll_rate_limit, self.yaw_rate_limit])
        delta = np.clip(smoothed - self.prev_action, -limits, limits)
        action = np.clip(self.prev_action + delta, -1.0, 1.0)
        self.prev_action = action.copy()
        return action.astype(np.float32)

    def reset(self):
        self.prev_action = np.array([0.0, 0.0, 0.035, 0.0])
        self.lost_yaw_rate = 0.0
        self.step_count = 0
        self.prev_ang_x = None     # İlk frame'de initialize edilecek
        self.prev_ang_y = None
        self.los_rate_az = 0.0
        self.los_rate_el = 0.0
        self.prev_bbox_size = 0.0
        self.bbox_rate = 0.0


HybridTeacher = PNTeacher
PIDTeacher = PNTeacher


if __name__ == "__main__":
    t = PNTeacher()
    obs = np.zeros(14)
    obs[13] = 1.0

    print("=" * 70)
    print("PN Teacher v2 Testleri")
    print("=" * 70)
    
    print("\n--- Test 1: İlk frame fix — sahte rate yok ---")
    t.reset()
    obs[4] = 15.0; obs[5] = 10.0; obs[11] = 0.0
    obs[2] = 0.01; obs[3] = 0.01
    a = t.compute_action(obs)
    print(f"  İlk frame ang_x=15°: LOS_rate={t.los_rate_az:.4f} (SIFIR olmalı)")
    print(f"  R_cmd={a[0]:+.4f} (çok küçük olmalı — sahte rate yok)")
    
    print("\n--- Test 2: Bbox gating — uzakta roll kapalı ---")
    t.reset()
    t.prev_ang_x = 10.0; t.prev_ang_y = 5.0
    obs[4] = 10.0; obs[5] = 5.0
    obs[2] = 0.01; obs[3] = 0.01  # bbox_area=0.0001 (uzak, < 0.0013)
    t.los_rate_az = 0.1
    a = t.compute_action(obs)
    gate = t._roll_gating(10.0, 0.0001)
    print(f"  Uzak (bbox=0.0001), ang_x=10°: gate={gate:.2f} (0 olmalı)")
    
    print("\n--- Test 3: Bbox gating — yakında roll serbest ---")
    t.reset()
    t.prev_ang_x = 10.0; t.prev_ang_y = 5.0
    obs[4] = 10.0; obs[5] = 5.0
    obs[2] = 0.05; obs[3] = 0.05  # bbox_area=0.0025 (yakın, > 0.0013)
    t.los_rate_az = 0.1
    a = t.compute_action(obs)
    gate = t._roll_gating(10.0, 0.0025)
    print(f"  Yakın (bbox=0.0025), ang_x=10°: gate={gate:.2f} (1.0 olmalı!)")
    
    print("\n--- Test 4: Sola kayma — PN roll tepkisi ---")
    t.reset()
    for i in range(10):
        obs[4] = 5.0 - i * 0.5; obs[5] = 3.0; obs[11] = 0.0
        obs[2] = 0.01; obs[3] = 0.01
        a = t.compute_action(obs)
    print(f"  ang_x 5→0.5 (sola kayma): LOS_rate={t.los_rate_az:.4f}")
    print(f"  R_cmd={a[0]:+.4f} (negatif — sola roll)")
    print(f"  pn_to_roll=0.025 ile v1'den ~2x güçlü")
    
    print("\n--- Test 5: Yakında ani hareket — roll serbest ---")
    t.reset()
    t.prev_ang_x = 0.0; t.prev_ang_y = 2.0
    obs[2] = 0.1; obs[3] = 0.1  # bbox büyük (yakın)
    for i in range(5):
        obs[4] = i * 3.0  # hedef hızla sağa kayıyor
        obs[5] = 2.0; obs[11] = 0.0
        a = t.compute_action(obs)
    gate = t._roll_gating(obs[4], 0.01)
    print(f"  Yakın + ang_x=12°: gate={gate:.2f} R={a[0]:+.4f}")
    print(f"  (Uzakta olsaydı gate=0 olurdu, roll veremezdi)")
