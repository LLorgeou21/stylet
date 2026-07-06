#!/usr/bin/env python3
import csv
import math
import os

import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

# Verite terrain (simulation uniquement) : la cible est statique, placee dans
# stylet.world a <pose>0.15 0.15 0 0 0 0</pose>. Le plan original prevoyait de
# requeter le service Gazebo "get_model_state" (Gazebo Classic) - ce service
# n'existe pas nativement sous Gazebo Harmonic/ros_gz (Jazzy). On reutilise donc
# la meme verite terrain codee en dur que dans surface_registration.cpp
# (voir ADR-028), a remplacer par une vraie requete Gazebo si la cible devient
# mobile (Phase 5).
GROUND_TRUTH_TRANSLATION = (0.15, 0.15, 0.0)
GROUND_TRUTH_QUATERNION = (0.0, 0.0, 0.0, 1.0)  # (x, y, z, w), identite

SUCCESS_TRANS_MM = 1.0
SUCCESS_ROT_DEG = 0.5
N_SAMPLES = 50


def translation_error_mm(position):
    dx = position.x - GROUND_TRUTH_TRANSLATION[0]
    dy = position.y - GROUND_TRUTH_TRANSLATION[1]
    dz = position.z - GROUND_TRUTH_TRANSLATION[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz) * 1000.0


def rotation_error_deg(orientation):
    q_est = (orientation.x, orientation.y, orientation.z, orientation.w)
    q_true = GROUND_TRUTH_QUATERNION
    dot = sum(a * b for a, b in zip(q_est, q_true))
    dot = max(-1.0, min(1.0, abs(dot)))
    return 2.0 * math.degrees(math.acos(dot))


class ValidateRegistration(Node):
    def __init__(self):
        super().__init__("validate_registration")
        self.samples = []
        self.subscription = self.create_subscription(
            PoseStamped, "/stylet/perception/target_pose", self.callback, 10
        )
        self.get_logger().info(
            f"En attente de {N_SAMPLES} echantillons sur /stylet/perception/target_pose..."
        )

    def callback(self, msg: PoseStamped):
        if len(self.samples) >= N_SAMPLES:
            return

        trans_err = translation_error_mm(msg.pose.position)
        rot_err = rotation_error_deg(msg.pose.orientation)
        success = trans_err < SUCCESS_TRANS_MM and rot_err < SUCCESS_ROT_DEG
        self.samples.append((trans_err, rot_err, success))

        self.get_logger().info(
            f"[{len(self.samples)}/{N_SAMPLES}] erreur translation: {trans_err:.4f} mm, "
            f"erreur rotation: {rot_err:.4f} deg, succes: {success}"
        )

        if len(self.samples) == N_SAMPLES:
            self.finish()

    def finish(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        csv_path = os.path.join(script_dir, "../config/registration_errors.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["translation_error_mm", "rotation_error_deg", "success"])
            writer.writerows(self.samples)

        trans_errs = [s[0] for s in self.samples]
        rot_errs = [s[1] for s in self.samples]
        successes = sum(1 for s in self.samples if s[2])

        self.get_logger().info(
            f"Termine : {successes}/{N_SAMPLES} succes "
            f"(translation < {SUCCESS_TRANS_MM}mm, rotation < {SUCCESS_ROT_DEG}deg). "
            f"CSV sauvegarde: {csv_path}"
        )

        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        axes[0].hist(trans_errs, bins=15)
        axes[0].set_xlabel("Erreur translation (mm)")
        axes[0].set_ylabel("Nombre d'echantillons")
        axes[0].set_title("Erreur de translation")

        axes[1].hist(rot_errs, bins=15)
        axes[1].set_xlabel("Erreur rotation (deg)")
        axes[1].set_title("Erreur de rotation")

        fig.tight_layout()
        png_path = os.path.join(script_dir, "../config/registration_errors.png")
        fig.savefig(png_path)
        self.get_logger().info(f"Histogramme sauvegarde: {png_path}")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = ValidateRegistration()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
