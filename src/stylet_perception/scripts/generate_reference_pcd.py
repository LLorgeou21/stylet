#!/usr/bin/env python3

import os
import open3d as o3d
import sys

path = os.path.dirname(os.path.abspath(__file__))
chemin_stl = os.path.join(path, "../../stylet_description/meshes/target.stl")
chemin_pcd = os.path.join(path, "../config/target_reference.pcd")

mesh = o3d.io.read_triangle_mesh(chemin_stl)
mesh.scale(
    0.005, center=(0, 0, 0)
)  # même échelle que dans model.sdf (cible réelle 250mm)
pcd = mesh.sample_points_uniformly(number_of_points=25000)
o3d.io.write_point_cloud(chemin_pcd, pcd)
