#!/usr/bin/env python3

import glob
import os
from pathlib import Path
from typing import List

import click
from cv2 import sqrt
import numpy as np
import open3d as o3d
import pykitti
from tomlkit import value
from tqdm import tqdm

from poisson import run_poisson
from preprocess_cloud import preprocess_cloud
import math
from copy import deepcopy

@click.command()
@click.option(
    "--dataset",
    "-d",
    type=click.Path(exists=True),
    default=os.environ["HOME"] + "/data/kitti-odometry/ply/",
    help="Location of the KITTI-like dataset",
)
@click.option(
    "--out_dir",
    type=click.Path(exists=True),
    default="./results/",
    help="Location of the output directory",
)
@click.option(
    "--sequence", "-s", type=str, default="00", help="Sequence number"
)
@click.option(
    "--n_scans", "-n", type=int, default=-1, help="Number of scans to integrate"
)
@click.option(
    "--start", "-sp", type=int, default=0, help="Start from this scan on"
)
@click.option(
    "--depth",
    type=int,
    default=10,
    help="Depth of the tree that will be used for reconstruction",
)
@click.option(
    "--visualize",
    is_flag=True,
    default=False,
    help="Visualize all the preprocess pipeline",
)
@click.option(
    "--normals",
    type=click.Choice(["range_image", "kdtree"], case_sensitive=False),
    default="range_image",
    help="Which normal computation to use",
)
@click.option(
    "--min_density",
    "-md",
    type=float,
    default=0.1,
    help="The minimun vertex density of the final mesh",
)
def main(
    dataset,
    out_dir,
    sequence,
    n_scans,
    start,
    depth,
    visualize,
    normals,
    min_density,
):
    """This script can be used to create GT mesh-model maps using GT poses. It
    assumes you have the data in the kitti-like format and all the scans where
    already pre-converted to '.ply', for example:

    \b
    kitti/ply
    ├── poses
    │   └── 00.txt
    └── sequences
        └── 00
            ├── calib.txt
            ├── poses.txt
            ├── times.txt
            └── velodyne
                ├── 000000.ply
                ├── 000001.ply
                └── ...

    You would also need the .txt files calib.txt and times.txt, altough not
    really used in this script, for convince, we reuse pykitti functionality
    that requires these files to be created.

    How to run it and check a quick example:

    \b
    $ ./build_gt_mesh.py -d $DATASETS/kitti/ply/ -s 00 -n 200 --depth 10
    """

    mesh = o3d.io.read_triangle_mesh("./results/Map_PCNormal.ply")
    point_cloud_data = o3d.io.read_point_cloud(dataset)

    pcd_tree = o3d.geometry.KDTreeFlann(point_cloud_data)
    vertices_size = np.size(mesh.vertices, 0)
    # vertex_mask = List[vertices_size]
    vertex_dist = np.zeros(vertices_size) 
    print("start compute vertex_dist")
    if 0 :
        for i in range(0, vertices_size):
            point_temp = mesh.vertices[i]
            # find 10 points that closed to point_temp
            [k, idx, _] = pcd_tree.search_knn_vector_3d(point_temp, 10)
            # compute distance between point_temp and 10 points
            d_out = 0
            for j in range(0, 10):
                d = 0
                pt = point_cloud_data.points[idx[j]]
                d += (pt[0] - point_temp[0]) * (pt[0] - point_temp[0])
                d += (pt[1] - point_temp[1]) * (pt[1] - point_temp[1])
                d += (pt[2] - point_temp[2]) * (pt[2] - point_temp[2])
                d = math.sqrt(d)
                d_out += d

            d_mean = d_out / 10
            vertex_dist[i] = d_mean
            if (i % 100000 == 0):
                print("100,000 compute vertex_dist" + str(i / 10000))
    else:
        vertex_dist = np.load("filename.npy")




    print("end compute vertex_dist")
    # np.save("filename.npy",vertex_dist)

    mesh1 = deepcopy(mesh)
    mesh2 = deepcopy(mesh)
    mesh3 = deepcopy(mesh)


    map_name = (
        "gt_"
        + "Map"
        + "_depth_"
        + str(depth)
    )
    min_density = 0.95
    vertex_mask = vertex_dist > np.quantile(vertex_dist, min_density)
    mesh.remove_vertices_by_mask(vertex_mask)
    mesh_file = os.path.join(out_dir, map_name + "_0-05" + "_mesh.ply")
    print("Saving mesh to " + mesh_file)
    o3d.io.write_triangle_mesh(mesh_file, mesh)


    min_density = 0.9
    vertex_mask = vertex_dist > np.quantile(vertex_dist, min_density)
    mesh1.remove_vertices_by_mask(vertex_mask)
    mesh_file = os.path.join(out_dir, map_name + "_0-1" + "_mesh.ply")
    print("Saving mesh to " + mesh_file)
    o3d.io.write_triangle_mesh(mesh_file, mesh1)

    min_density = 0.85
    vertex_mask = vertex_dist > np.quantile(vertex_dist, min_density)
    mesh2.remove_vertices_by_mask(vertex_mask)
    mesh_file = os.path.join(out_dir, map_name + "_0-15" + "_mesh.ply")
    print("Saving mesh to " + mesh_file)
    o3d.io.write_triangle_mesh(mesh_file, mesh2)

    min_density = 0.8
    vertex_mask = vertex_dist > np.quantile(vertex_dist, min_density)
    mesh3.remove_vertices_by_mask(vertex_mask)
    mesh_file = os.path.join(out_dir, map_name + "_0-2" + "_mesh.ply")
    print("Saving mesh to " + mesh_file)
    o3d.io.write_triangle_mesh(mesh_file, mesh3)


if __name__ == "__main__":
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)

main()
