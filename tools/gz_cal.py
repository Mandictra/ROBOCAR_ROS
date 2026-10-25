#!/usr/bin/env python3
"""Guided gyro test: tells you when to stay still and when to drive straight, then gives the verdict.
Run: python3 gz_stats.py   (micro-ROS agent must be running; drive with the dashboard D-pad when asked)"""
import math, threading, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

STILL_S = 5.0          # still window
DRIVE_S = 5.0          # recorded time while moving
V_MIN = 0.05           # m/s: counts as "driving"
WAIT_S = 30.0          # how long to wait for you to start driving
LOAD_BIAS_LIMIT = 0.005  # rad/s


def verdict(g_still, g_drive, vx):
    d = g_drive - g_still
    lines = [f"still : mean gz = {g_still:+.4f} rad/s ({math.degrees(g_still):+.2f} deg/s)",
             f"driving: mean gz = {g_drive:+.4f} rad/s ({math.degrees(g_drive):+.2f} deg/s) at {vx:.2f} m/s",
             f"change under load = {d:+.4f} rad/s -> {math.degrees(d) * 10:+.1f} deg of yaw error per 10 s"]
    if abs(d) > LOAD_BIAS_LIMIT:
        lines.append("VERDICT: the gyro offset shifts while the motors run (vibration / supply noise). "
                     "If the robot stayed straight, this is the yaw drift. Send me these numbers for the fix.")
    else:
        lines.append("VERDICT: gyro offset is the same still and driving. If yaw still grows while the robot "
                     "tracks straight, check it again; if the robot visibly veered, the veer is real (mechanical): "
                     "compare per-wheel rpm at the same command, tyres, TICKS_PER_REV, PI gains.")
    return "\n".join(lines)


class Stats(Node):
    def __init__(self):
        super().__init__('gz_stats')
        self.rec_still = False
        self.rec_drive = False
        self.vx = 0.0
        self.gz_still, self.gz_drive, self.vx_drive = [], [], []
        self.create_subscription(Imu, '/imu_data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, qos_profile_sensor_data)

    def on_odom(self, m):
        self.vx = m.twist.twist.linear.x

    def on_imu(self, m):
        if self.rec_still:
            self.gz_still.append(m.angular_velocity.z)
        elif self.rec_drive and abs(self.vx) > V_MIN:     # only samples taken while really moving
            self.gz_drive.append(m.angular_velocity.z)
            self.vx_drive.append(abs(self.vx))


def countdown(msg, secs):
    for s in range(int(secs), 0, -1):
        print(f"  {msg} {s}...", flush=True)
        time.sleep(1)


def main():
    rclpy.init()
    n = Stats()
    spin_thread = threading.Thread(target=rclpy.spin, args=(n,), daemon=True)
    spin_thread.start()
    try:
        print("\nSTEP 1: keep the robot completely STILL (do not touch it, motors idle).")
        input("Press Enter when it is still... ")
        n.rec_still = True
        countdown("recording STILL, hold on:", STILL_S)
        n.rec_still = False
        if not n.gz_still:
            print("No /imu_data received. Is the micro-ROS agent running?")
            return

        print("\nSTEP 2: point the robot along a straight line with 3+ m free ahead.")
        print(f"        When you are ready, DRIVE FORWARD in a straight line (dashboard D-pad up, slow speed).")
        print(f"        Recording starts by itself when speed > {V_MIN} m/s. Keep driving straight for {DRIVE_S:.0f} s.")
        n.rec_drive = True
        t0 = time.time()
        while len(n.gz_drive) == 0 and time.time() - t0 < WAIT_S:
            time.sleep(0.05)
        if not n.gz_drive:
            print("The robot never moved. Check /odometry/filtered, then run again.")
            return
        print(">>> DRIVING DETECTED - KEEP GOING STRAIGHT, recording...", flush=True)
        t1 = time.time()
        while time.time() - t1 < DRIVE_S:
            time.sleep(0.05)
        n.rec_drive = False
        print(">>> DONE - RELEASE THE D-PAD / STOP THE ROBOT NOW.\n")

        gs = sum(n.gz_still) / len(n.gz_still)
        gd = sum(n.gz_drive) / len(n.gz_drive)
        vd = sum(n.vx_drive) / len(n.vx_drive)
        print(verdict(gs, gd, vd))
        print("\nNow tell me: did the robot stay on the line or drift (which side, how far over how many metres)?")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        rclpy.try_shutdown()          # stops spin()
        spin_thread.join(timeout=2)   # avoids the 'terminate called without an active exception' abort at exit


if __name__ == '__main__':
    main()