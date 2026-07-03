#!/usr/bin/env python3

import os
import open3d as o3d
import sys

path = os.path.dirname(os.path.abspath(__file__))
chemin_stl = os.path.join(path, "../../stylet_description/meshes/target.stl")
chemin_pcd = os.path.join(path, "../config/target_reference.pcd")

mesh = o3d.io.read_triangle_mesh(chemin_stl)
pcd = mesh.sample_points_uniformly(number_of_points=50000)
o3d.io.write_point_cloud(chemin_pcd, pcd)
