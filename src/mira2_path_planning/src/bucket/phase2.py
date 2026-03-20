#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Point
from std_msgs.msg import String
from custom_msgs.msg import Commands

class Phase:
    INIT = -1
    SEARCH = 0
    ALIGN_XY = 1
    LOCK = 2

PHASE_NAMES = {
    Phase.INIT: "INITIALIZING",
    Phase.SEARCH: "SEARCHING",
    Phase.ALIGN_XY: "ALIGNING_XY",
    Phase.LOCK: "DROPPING"
}

SIM = 1  # 1 = Simulation (clamp to min 1540), 0 = Real (no clamping)

class BucketControls(Node):

    def __init__(self):
        super().__init__("bucket_control_node")

        self.kp_sway,  self.kd_sway  = 210.0, 20.0
        self.kp_surge, self.kd_surge = 210.0, 20.0

        self.pwm_neutral      = 1500
        self.pwm_max_effort   = 400
        self.pwm_min_move     = 1540  # minimum PWM to actually move (only used when SIM=0)
        self.search_speed     = 1550

        self.current_phase   = Phase.INIT
        self.init_counter    = 0
        self.bucket_visible  = False
        self.sees_blue       = False

        self.target_nx       = 0.0
        self.target_ny       = 0.0
        self.last_time_seen  = 0.0

        self.prev_nx_err     = 0.0
        self.prev_ny_err     = 0.0
        self.last_brain_tick = time.time()

        self.xy_hold_timer    = None
        self.blind_lock_timer = None
        self.lateral_aligned  = False

        self.cmd_pub   = self.create_publisher(Commands, "/master/commands", 10)
        self.debug_pub = self.create_publisher(String, "bucket_debug", 10)

        self.create_subscription(Point,  "bucket/p2offset", self.camera_callback, 10)
        self.create_subscription(String, "bucket/p2color",  self.color_callback,  10)

        self.create_timer(0.05, self.think_and_act)

    def camera_callback(self, msg):
        self.last_time_seen = time.time()
        self.bucket_visible = True
        self.target_nx = msg.x
        self.target_ny = msg.y

    def color_callback(self, msg):
        self.sees_blue = (msg.data == "blue")

    def change_phase(self, new_phase, reason=""):
        if self.current_phase != new_phase:
            old_name = PHASE_NAMES.get(self.current_phase, "UNKNOWN")
            new_name = PHASE_NAMES.get(new_phase, "UNKNOWN")
            self.get_logger().warn(f"\n>>> STATE CHANGE: {old_name} -> {new_name} | {reason} <<<\n")
            self.current_phase = new_phase

    def apply_min_move(self, pwm):
        """
        SIM=1: clamp to min 1540 (positive) or max 1460 (negative) for sim.
        SIM=0: no clamping, return pwm as-is for real thrusters.
        """
        if not SIM:
            return pwm
        if pwm > self.pwm_neutral:
            return max(pwm, self.pwm_min_move)
        elif pwm < self.pwm_neutral:
            return min(pwm, self.pwm_neutral - (self.pwm_min_move - self.pwm_neutral))  # mirror: 1460
        return self.pwm_neutral

    # ==========================================
    # PID
    # ==========================================
    def run_lateral_pid(self, nx, dt):
        if abs(nx) < 0.05:
            return self.pwm_neutral  # deadband: error too small, don't move
        derivative = (nx - self.prev_nx_err) / dt if dt > 0 else 0.0
        self.prev_nx_err = nx
        output = int((nx * self.kp_sway) + (derivative * self.kd_sway))
        output = max(min(output, self.pwm_max_effort), -self.pwm_max_effort)
        return self.apply_min_move(self.pwm_neutral + output)

    def run_surge_pid(self, ny, dt):
        if abs(ny) < 0.05:
            return self.pwm_neutral  # deadband: error too small, don't move
        derivative = (ny - self.prev_ny_err) / dt if dt > 0 else 0.0
        self.prev_ny_err = ny
        output = int((ny * self.kp_surge) + (derivative * self.kd_surge))
        output = max(min(output, self.pwm_max_effort), -self.pwm_max_effort)
        return self.apply_min_move(self.pwm_neutral + output)

    # ==========================================
    # MAIN LOOP
    # ==========================================
    def think_and_act(self):
        cmd = Commands()
        cmd.pitch   = self.pwm_neutral
        cmd.roll    = self.pwm_neutral
        cmd.yaw     = self.pwm_neutral
        cmd.lateral = self.pwm_neutral
        cmd.forward = self.pwm_neutral
        cmd.thrust  = self.pwm_neutral
        cmd.arm     = False
        cmd.mode    = "MANUAL"

        current_time = time.time()
        dt = current_time - self.last_brain_tick
        self.last_brain_tick = current_time

        # ==========================================
        # INIT: Disarm(2s) -> Set MANUAL mode(2s) -> Arm(2s) -> done
        # ==========================================
        if self.current_phase == Phase.INIT:
            self.init_counter += 1
            t = self.init_counter

            if t <= 40:
                cmd.arm  = False
                cmd.mode = "MANUAL"
                if t == 1:
                    self.get_logger().info("INIT 1/3 — Disarming...")

            elif t <= 80:
                cmd.arm  = False
                cmd.mode = "MANUAL"
                if t == 41:
                    self.get_logger().info("INIT 2/3 — Setting mode to MANUAL...")

            elif t <= 120:
                cmd.arm  = True
                cmd.mode = "MANUAL"
                if t == 81:
                    self.get_logger().info("INIT 3/3 — Arming...")

            else:
                cmd.arm  = True
                cmd.mode = "MANUAL"
                self.get_logger().info("INIT complete. Starting SEARCH.")
                self.change_phase(Phase.SEARCH, "Setup done.")

            self.cmd_pub.publish(cmd)
            return

        cmd.arm  = True
        cmd.mode = "MANUAL"

        # ==========================================
        # FLIGHT LOGIC
        # ==========================================
        if self.bucket_visible and (current_time - self.last_time_seen > 1.0):
            self.get_logger().warn("Target LOST from camera view! (1.0s timeout)")
            self.bucket_visible = False
            self.sees_blue      = False

        nx = self.target_nx if self.bucket_visible else 0.0
        ny = self.target_ny if self.bucket_visible else 0.0

        if self.current_phase == Phase.SEARCH:
            cmd.forward = self.search_speed
            if self.bucket_visible and self.sees_blue:
                self.lateral_aligned = False
                self.change_phase(Phase.ALIGN_XY, "Found BLUE target.")

        elif self.current_phase == Phase.ALIGN_XY:
            if not self.bucket_visible:
                self.xy_hold_timer   = None
                self.lateral_aligned = False
                self.change_phase(Phase.SEARCH, "Target lost during alignment")
                return

            if abs(nx) > 0.05:
                if self.lateral_aligned:
                    self.get_logger().info(f"Lateral drifted ({nx:+.3f})! Re-aligning lateral...")
                self.lateral_aligned = False
                self.xy_hold_timer   = None

            if not self.lateral_aligned:
                cmd.lateral = self.run_lateral_pid(nx, dt)
                cmd.forward = self.pwm_neutral
                if abs(nx) < 0.05:
                    self.lateral_aligned = True
                    self.get_logger().info("Lateral aligned! Now aligning forward...")

            else:
                cmd.lateral = self.pwm_neutral
                if abs(ny) < 0.05:
                    cmd.forward = self.pwm_neutral  # stop
                    if self.xy_hold_timer is None:
                        self.xy_hold_timer = current_time
                        self.get_logger().info("XY Aligned! Holding position briefly...")
                    elif (current_time - self.xy_hold_timer) > 2.5:
                        self.blind_lock_timer = current_time
                        self.change_phase(Phase.LOCK, "XY lock stabilized. Ready to drop ball.")
                else:
                    cmd.forward = self.run_surge_pid(ny, dt)
                    self.xy_hold_timer = None

        elif self.current_phase == Phase.LOCK:
            if self.bucket_visible:
                cmd.lateral = self.run_lateral_pid(nx, dt)
                cmd.forward = self.run_surge_pid(ny, dt)
            if current_time - self.blind_lock_timer > 3.0:
                self.get_logger().info(
                    "Maintaining position over bucket. DROPPING BALL NOW!",
                    throttle_duration_sec=2.0
                )

        self.log_motion(cmd)
        self.cmd_pub.publish(cmd)
        self.publish_debug(cmd, nx, ny)

    # ==========================================
    # LOGGING
    # ==========================================
    def log_motion(self, cmd):
        if cmd.lateral > 1530:
            self.get_logger().info(f"-> Moving RIGHT  (Lateral PWM: {cmd.lateral})", throttle_duration_sec=0.5)
        elif cmd.lateral < 1470:
            self.get_logger().info(f"<- Moving LEFT   (Lateral PWM: {cmd.lateral})", throttle_duration_sec=0.5)

        if cmd.forward > 1530:
            self.get_logger().info(f"^ Moving FWD     (Forward PWM: {cmd.forward})", throttle_duration_sec=0.5)
        elif cmd.forward < 1470:
            self.get_logger().info(f"v Moving BACK    (Forward PWM: {cmd.forward})", throttle_duration_sec=0.5)

    def publish_debug(self, cmd, nx, ny):
        state_str = PHASE_NAMES[self.current_phase]
        vis   = "YES" if self.bucket_visible else "NO"
        color = "BLUE" if self.sees_blue else "NONE"
        log_msg = (
            f"[{state_str:<13}] Vis:{vis}({color}) | "
            f"Err(nX,nY): {nx:>+6.3f}, {ny:>+6.3f} | "
            f"PWM(Lat,Fwd): {cmd.lateral:>4d}, {cmd.forward:>4d}"
        )
        self.debug_pub.publish(String(data=log_msg))
        self.get_logger().info(log_msg, throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = BucketControls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()