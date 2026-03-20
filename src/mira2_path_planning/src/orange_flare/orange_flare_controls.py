#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from custom_msgs.msg import Commands


class Phase:
    INIT     = -1
    SEARCH   =  0
    ALIGN_Y  =  1
    ALIGN_X  =  2
    APPROACH =  3
    AVOID    =  4
    DONE     =  5


PHASE_NAMES = {
    Phase.INIT:     "INIT",
    Phase.SEARCH:   "SEARCHING",
    Phase.ALIGN_Y:  "ALIGN_Y",
    Phase.ALIGN_X:  "ALIGN_X",
    Phase.APPROACH: "APPROACH",
    Phase.AVOID:    "AVOID",
    Phase.DONE:     "DONE",
}


class FlareControls(Node):

    def __init__(self):
        super().__init__("flare_control_node")

        self.kp_thrust  = 150.0
        self.kp_lateral = 150.0

        self.pwm_neutral    = 1500
        self.pwm_max_effort = 400
        self.hover_thrust   = 1500
        self.search_speed   = 1600

        # Hysteresis thresholds
        # Y: stop correcting at 0.05, restart at 0.15
        self.y_stop  = 0.05
        self.y_start = 0.15
        # X: stop correcting at 0.05, restart at 0.10
        self.x_stop  = 0.05
        self.x_start = 0.10

        self.z_approach = 2.0
        self.hold_time  = 2.5

        # Hysteresis state flags
        self.y_correcting = False
        self.x_correcting = False

        # Avoid timings
        self.avoid_lat1_time = 5
        self.avoid_fwd_time  = 10.0
        self.avoid_lat2_time = 2.0
        self.avoid_lat_pwm   = 1600
        self.avoid_fwd_pwm   = 1700

        # State
        self.current_phase  = Phase.INIT
        self.init_counter   = 0
        self.flare_visible  = False
        self.last_time_seen = 0.0

        self.tx = 0.0
        self.ty = 0.0
        self.tz = 999.0

        self.last_brain_tick = time.time()

        self.y_aligned   = False
        self.hold_timer  = None
        self.avoid_start = None

        self.cmd_pub   = self.create_publisher(Commands, "/master/commands", 10)
        self.debug_pub = self.create_publisher(String,   "/flare/ctrl_debug", 10)

        self.create_subscription(PoseStamped, "/flare/pose", self.pose_callback, 10)
        self.create_timer(0.05, self.think_and_act)

    def pose_callback(self, msg):
        if msg.header.frame_id == "none" or msg.pose.position.z < 0:
            if self.flare_visible and (time.time() - self.last_time_seen > 1.0):
                self.flare_visible = False
            return
        tx =  msg.pose.position.x
        ty = -msg.pose.position.y
        tz =  msg.pose.position.z
        if math.isnan(tx) or math.isnan(ty) or math.isnan(tz):
            return
        self.last_time_seen = time.time()
        self.flare_visible  = True
        self.tx = tx
        self.ty = ty
        self.tz = tz

    def change_phase(self, new_phase, reason=""):
        if self.current_phase != new_phase:
            self.get_logger().warn(
                f"\n>>> {PHASE_NAMES[self.current_phase]} -> {PHASE_NAMES[new_phase]} | {reason} <<<\n"
            )
            self.current_phase = new_phase

    def thrust_pid(self, ty):
        """
        Hysteresis on Y:
          - Start correcting when |ty| > y_start (0.15)
          - Stop correcting when |ty| < y_stop  (0.05)
        """
        if self.y_correcting:
            if abs(ty) < self.y_stop:
                self.y_correcting = False
                return self.hover_thrust
        else:
            if abs(ty) > self.y_start:
                self.y_correcting = True
            else:
                return self.hover_thrust

        out = int(ty * self.kp_thrust)
        out = max(min(out, self.pwm_max_effort), -self.pwm_max_effort)
        return self.pwm_neutral + out

    def lateral_pid(self, tx):
        """
        Hysteresis on X:
          - Start correcting when |tx| > x_start (0.10)
          - Stop correcting when |tx| < x_stop  (0.05)
        """
        if self.x_correcting:
            if abs(tx) < self.x_stop:
                self.x_correcting = False
                return self.pwm_neutral
        else:
            if abs(tx) > self.x_start:
                self.x_correcting = True
            else:
                return self.pwm_neutral

        out = int(tx * self.kp_lateral)
        out = max(min(out, self.pwm_max_effort), -self.pwm_max_effort)
        return self.pwm_neutral + out

    def think_and_act(self):
        cmd = Commands()
        cmd.pitch   = self.pwm_neutral
        cmd.roll    = self.pwm_neutral
        cmd.yaw     = self.pwm_neutral
        cmd.lateral = self.pwm_neutral
        cmd.forward = self.pwm_neutral
        cmd.thrust  = self.hover_thrust
        cmd.arm     = False
        cmd.mode    = "MANUAL"

        current_time = time.time()
        self.last_brain_tick = current_time

        if self.current_phase == Phase.INIT:
            self.init_counter += 1
            t = self.init_counter
            if t <= 40:
                cmd.arm    = False
                cmd.mode   = "MANUAL"
                cmd.thrust = self.pwm_neutral
                if t == 1:
                    self.get_logger().info("INIT 1/3 — Disarming...")
            elif t <= 80:
                cmd.arm    = False
                cmd.mode   = "MANUAL"
                cmd.thrust = self.pwm_neutral
                if t == 41:
                    self.get_logger().info("INIT 2/3 — Setting MANUAL mode...")
            elif t <= 120:
                cmd.arm    = True
                cmd.mode   = "MANUAL"
                cmd.thrust = self.pwm_neutral
                if t == 81:
                    self.get_logger().info("INIT 3/3 — Arming...")
            else:
                cmd.arm  = True
                cmd.mode = "MANUAL"
                self.get_logger().info("INIT done. Starting SEARCH.")
                self.change_phase(Phase.SEARCH, "Setup done.")
            self.cmd_pub.publish(cmd)
            return

        cmd.arm  = True
        cmd.mode = "MANUAL"

        # Timeout
        if self.flare_visible and (current_time - self.last_time_seen > 1.0):
            self.get_logger().warn("Flare LOST (1s timeout)")
            self.flare_visible = False

        tx = self.tx if self.flare_visible else 0.0
        ty = self.ty if self.flare_visible else 0.0
        tz = self.tz

        # ── SEARCH ────────────────────────────────────────────────────────
        if self.current_phase == Phase.SEARCH:
            cmd.forward = self.search_speed
            if self.flare_visible:
                self.y_aligned    = False
                self.y_correcting = False
                self.x_correcting = False
                self.change_phase(Phase.ALIGN_Y, "Flare detected.")

        # ── ALIGN_Y ───────────────────────────────────────────────────────
        elif self.current_phase == Phase.ALIGN_Y:
            if not self.flare_visible:
                self.y_aligned    = False
                self.hold_timer   = None
                self.y_correcting = False
                self.change_phase(Phase.SEARCH, "Lost flare during ALIGN_Y.")
                return

            cmd.thrust  = self.thrust_pid(ty)
            cmd.lateral = self.pwm_neutral
            cmd.forward = self.pwm_neutral

            # Hold timer only runs when not correcting (within dead band)
            if not self.y_correcting:
                if self.hold_timer is None:
                    self.hold_timer = current_time
                    self.get_logger().info("Y aligned! Holding...")
                elif (current_time - self.hold_timer) > self.hold_time:
                    self.y_aligned    = True
                    self.hold_timer   = None
                    self.y_correcting = False
                    self.change_phase(Phase.ALIGN_X, "Y locked. Aligning X.")
            else:
                self.hold_timer = None

        # ── ALIGN_X ───────────────────────────────────────────────────────
        elif self.current_phase == Phase.ALIGN_X:
            if not self.flare_visible:
                self.y_aligned    = False
                self.hold_timer   = None
                self.x_correcting = False
                self.change_phase(Phase.SEARCH, "Lost flare during ALIGN_X.")
                return

            # If Y drifts past y_start, go back to ALIGN_Y
            if abs(ty) > self.y_start:
                self.get_logger().info(f"Y drifted ({ty:+.3f})! Back to ALIGN_Y.")
                self.y_aligned    = False
                self.hold_timer   = None
                self.y_correcting = False
                self.x_correcting = False
                self.change_phase(Phase.ALIGN_Y, "Y drifted.")
                return

            cmd.lateral = self.lateral_pid(tx)
            cmd.thrust  = self.hover_thrust
            cmd.forward = self.pwm_neutral

            # Hold timer only runs when not correcting
            if not self.x_correcting:
                if self.hold_timer is None:
                    self.hold_timer = current_time
                    self.get_logger().info("X aligned! Holding...")
                elif (current_time - self.hold_timer) > self.hold_time:
                    self.hold_timer   = None
                    self.x_correcting = False
                    self.change_phase(Phase.APPROACH, "XY locked. Approaching.")
            else:
                self.hold_timer = None

        # ── APPROACH ──────────────────────────────────────────────────────
        elif self.current_phase == Phase.APPROACH:
            if not self.flare_visible:
                self.change_phase(Phase.SEARCH, "Lost flare during APPROACH.")
                return

            cmd.thrust  = self.thrust_pid(ty)
            cmd.lateral = self.lateral_pid(tx)
            cmd.forward = self.search_speed

            if tz < self.z_approach:
                self.avoid_start = current_time
                self.change_phase(Phase.AVOID, f"Close enough tz={tz:.2f}m. Avoiding.")

        # ── AVOID ─────────────────────────────────────────────────────────
        elif self.current_phase == Phase.AVOID:
            elapsed = current_time - self.avoid_start
            t1 = self.avoid_lat1_time
            t2 = t1 + self.avoid_fwd_time
            t3 = t2 + self.avoid_lat2_time

            if elapsed < t1:
                cmd.lateral = self.avoid_lat_pwm
                cmd.forward = self.pwm_neutral
                self.get_logger().info("AVOID: RIGHT", throttle_duration_sec=0.5)
            elif elapsed < t2:
                cmd.lateral = self.pwm_neutral
                cmd.forward = self.avoid_fwd_pwm
                self.get_logger().info("AVOID: FWD", throttle_duration_sec=0.5)
            elif elapsed < t3:
                cmd.lateral = self.pwm_neutral - (self.avoid_lat_pwm - self.pwm_neutral)
                cmd.forward = self.pwm_neutral
                self.get_logger().info("AVOID: LEFT", throttle_duration_sec=0.5)
            else:
                self.change_phase(Phase.DONE, "Avoidance complete.")

        # ── DONE ──────────────────────────────────────────────────────────
        elif self.current_phase == Phase.DONE:
            self.get_logger().info("Flare task DONE.", throttle_duration_sec=2.0)

        self.log_motion(cmd)
        self.cmd_pub.publish(cmd)
        self.publish_debug(cmd, tx, ty, tz)

    def log_motion(self, cmd):
        if cmd.lateral > 1530:
            self.get_logger().info(f"-> RIGHT  (Lat PWM: {cmd.lateral})", throttle_duration_sec=0.5)
        elif cmd.lateral < 1470:
            self.get_logger().info(f"<- LEFT   (Lat PWM: {cmd.lateral})", throttle_duration_sec=0.5)
        if cmd.forward > 1530:
            self.get_logger().info(f"^ FWD     (Fwd PWM: {cmd.forward})", throttle_duration_sec=0.5)
        elif cmd.forward < 1470:
            self.get_logger().info(f"v BACK    (Fwd PWM: {cmd.forward})", throttle_duration_sec=0.5)
        if cmd.thrust > 1510:
            self.get_logger().info(f"↑ UP      (Thr PWM: {cmd.thrust})", throttle_duration_sec=0.5)
        elif cmd.thrust < 1470:
            self.get_logger().info(f"↓ DOWN    (Thr PWM: {cmd.thrust})", throttle_duration_sec=0.5)

    def publish_debug(self, cmd, tx, ty, tz):
        phase = PHASE_NAMES[self.current_phase]
        vis   = "YES" if self.flare_visible else "NO"
        yc    = "Y" if self.y_correcting else "-"
        xc    = "X" if self.x_correcting else "-"
        msg   = (
            f"[{phase:<10}] Vis:{vis} Corr:[{yc}{xc}] | "
            f"tx:{tx:>+6.3f} ty:{ty:>+6.3f} tz:{tz:>6.3f} | "
            f"PWM(Lat,Thr,Fwd): {cmd.lateral}, {cmd.thrust}, {cmd.forward}"
        )
        self.debug_pub.publish(String(data=msg))
        self.get_logger().info(msg, throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = FlareControls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()