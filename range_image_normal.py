#!/usr/bin/env python3
# import click
import open3d as o3d
import numpy as np

import gen_normal_map
from range_image import project_to_range_image


def compute_normals(cloud, w, h):
    range_image, vertex_map = project_to_range_image(cloud, w, h)
    # normal_map = gen_normal_map.gen_normal_map(range_image, vertex_map, w, h)

    current_vertex = np.asarray(cloud.points)
    depth = np.linalg.norm(current_vertex, axis=1)

    current_vertex = current_vertex[(depth > 0.01) & (depth < 20)]
    flag = current_vertex[:, 2] > -2
    current_vertex = current_vertex[flag]
    
    # get scan components
    scan_x = current_vertex[:, 0]
    scan_y = current_vertex[:, 1]
    scan_z = current_vertex[:, 2]
    
    cloud_map = gen_normal_map.gen_normal_map(scan_x, scan_y, scan_z)


    iCount = scan_x.size
    normal_map = cloud_map[0 : iCount * 3]
    point_map = cloud_map[iCount * 3 : iCount * 6]
    cloud.points = o3d.utility.Vector3dVector(point_map.reshape(iCount, 3))
    cloud.normals = o3d.utility.Vector3dVector(normal_map.reshape(iCount, 3))
    # print(cloud.normals[0][0])
    # print(cloud.normals[0][1])
    # print(cloud.normals[0][2])


    cloud.remove_non_finite_points()
    return cloud


# @click.command()
# @click.argument("file", type=click.Path(exists=True))
# @click.option("-w", default=1024, type=int)
# @click.option("-h", default=64, type=int)
# def main(file, w, h):
#     cloud = compute_normals(o3d.io.read_point_cloud(file), w, h)
#     o3d.visualization.draw_geometries([cloud])


# if __name__ == "__main__":
#     main()
