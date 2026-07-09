#!/usr/bin/env python3

import os
import open3d as o3d

path = os.path.dirname(os.path.abspath(__file__))
stl_path = os.path.join(path, "../../stylet_description/meshes/target.stl")
pcd_path = os.path.join(path, "../config/target_reference.pcd")

mesh = o3d.io.read_triangle_mesh(stl_path)
mesh.scale(
    0.005, center=(0, 0, 0)
)  # same scale as in model.sdf (real target 250mm)
pcd = mesh.sample_points_uniformly(number_of_points=25000)
o3d.io.write_point_cloud(pcd_path, pcd)
