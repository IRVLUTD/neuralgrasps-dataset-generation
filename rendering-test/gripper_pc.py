import json
import math
import os
import argparse
import pybullet as p
import numpy as np
import open3d as o3d

import render
import sdf_utils


def get_urdf_name(gripper_name: str):
    if gripper_name == "fetch_gripper":
        gripper_urdf = "test_fetch.urdf"
    elif gripper_name == "Barrett":
        gripper_urdf = "test_barrett.urdf"
    elif gripper_name == "HumanHand":
        gripper_urdf = "bkp_HumanHand20DOF-zero.urdf"
    else:
        print(f"ERROR: Incorret gripper name specified {gripper_name}")
    return gripper_urdf


def get_estimated_normals(points, camera_loc):
    tmp_pcd = o3d.geometry.PointCloud()
    tmp_pcd.points = o3d.utility.Vector3dVector(points)
    tmp_pcd.estimate_normals()
    tmp_pcd.orient_normals_towards_camera_location(camera_loc)
    return np.asarray(tmp_pcd.normals)


def get_points_from_random_viewpoint(obj_id, params):
    samp_view = render.sample_view(obj_id, params)
    depth_img = samp_view['depth']
    intrinsic = samp_view['meta']['intrinsic_matrix']
    camera_pose = samp_view['meta']['camera_pose']
    camera_location = np.asarray(samp_view['view_params']['camera_pos'])
    points = render.backproject_to_wcs(
        depth_img, intrinsic, camera_pose, factor=1)
    normals = get_estimated_normals(points, camera_location)
    return (points, normals)


parser = argparse.ArgumentParser(
    description='Point cloud for Gripper in Canonical Pose and DOFs')

parser.add_argument('-o', '--path_out', type=str, default='../../../dataset_train/',
                    help='dataset directory where the point clouds will be populated')
# Data set directory
# Assumes that the grasps will be in: datadir/refined_grasps/obj-gripper.json
# The saved point clouds will be in: datadir/object_name/point_cloud/gripper/*.ply
# The saved mesh         will be in: datadir/object_name/mesh/gripper/*.obj

parser.add_argument('--urdf_dir', type=str,
                    default='../prepare-graspit-urdf/urdf/', help="Folder for gripper URDF files")

parser.add_argument('-m', '--grippers', nargs='*',
                    default=['fetch_gripper', 'Barrett', 'HumanHand'])
parser.add_argument('-l', '--grippers_file', type=str,
                    default='')  # one gripper model entry per line
parser.add_argument('-s', '--size', type=int, default=2048,
                    help="desired number of points in the output point clouds")


parser.add_argument('--num_views', type=int, default=128,
                    help="desired number of views to sample the point cloud from")
parser.add_argument('--img_width', type=int, default=320,
                    help="depth image width")
parser.add_argument('--img_height', type=int, default=240,
                    help="depth image height")
parser.add_argument('--cam_near', type=float, default=0.01,
                    help="virtual camera near plane")
parser.add_argument('--cam_far', type=float, default=10,
                    help="virtual camera far  plane")
parser.add_argument('--cam_fov', type=float, default=45,
                    help="virtual camera fov")


def main(args):
    if not args.path_out:
        print('Output directory not specified')
        exit(0)

    if not args.grippers_file and args.grippers:
        grippers = args.grippers
    else:
        with open(args.models_file) as f:
            grippers = f.read().splitlines()

    num_views = args.num_views
    num_points = args.size    
    
    pc_folder = os.path.join(args.path_out, 'urdf_point_cloud')
    if not os.path.isdir(pc_folder):
        print('Folder does not exist for the output point clouds')
        print('Creating the point cloud folder:', pc_folder)
        os.makedirs(pc_folder)

    # Camera Params
    params = {}
    params['img_width'] = args.img_width
    params['img_height'] = args.img_height
    params['near'] = args.cam_near
    params['far'] = args.cam_far
    params['fov'] = args.cam_fov
    # factor = 1

    p.connect(p.DIRECT)
    data_grippers = {}
    # scale_global = 0.5
    scale_global = 0
    for gripper_name in grippers:
        gripper_urdf = get_urdf_name(gripper_name)
        gripper_urdf_file = os.path.join(args.urdf_dir, gripper_urdf)
        if not os.path.isfile(gripper_urdf_file):
            print("URDF file for gripper not found! Exiting...\n")
            print("Searching for {} in location: {}\n".format(
                gripper_urdf, args.urdf_dir))
            exit(0)
            # Load the robot urdf in the scene:

        robot_id = p.loadURDF(gripper_urdf_file)
        print("\n\nLoaded the Gripper URDF")
        pc_fname = f'{gripper_name}.ply'
        pc_file = os.path.join(pc_folder, pc_fname)

        points_normals_views = [get_points_from_random_viewpoint(
            robot_id, params) for i in range(num_views)]
        print(f"Obtained points and normals from {num_views} views.\n")

        points_views = [points_normals_views[i][0]
                        for i in range(num_views)]
        normls_views = [points_normals_views[i][1]
                        for i in range(num_views)]
        data_points = np.concatenate(points_views, axis=0)
        data_normal = np.concatenate(normls_views, axis=0)

        # Center at origin and Normalize to unit sphere.
        offset, scale_local = sdf_utils.get_norm_params_sphere(
            np.asarray(data_points), buffer=1)
        scale_global = max(scale_global, scale_local)
        ## DONT SCALE AND OFFSET HERE
        # data_points = sdf_utils.scale_using_params(data_points, 1.0, offset)
        
        local_data = {}
        local_data['points'] = data_points
        local_data['normals'] = data_normal
        local_data['offset'] = offset
        local_data['scale'] = scale_local # Not strictly needed but caching it anyways
        local_data['filename'] = pc_file
        data_grippers[gripper_name] = local_data

        # Remove the gripper (robot) from the world
        p.removeBody(robot_id)

    # Center the points and adjust to the global scale
    for gripper_name in data_grippers:
        local_data = data_grippers[gripper_name]
        data_points = local_data['points']
        data_normal = local_data['normals']
        offset = local_data['offset']
        pc_file = local_data['filename']
        
        normalized_points = sdf_utils.scale_using_params(data_points, scale_global, offset)
        _, idxs = sdf_utils.farthest_point_sampling(normalized_points, num_points)

        # Use the idxs to select a representative sample from the point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(normalized_points[idxs])
        pcd.normals = o3d.utility.Vector3dVector(data_normal[idxs])
        pcd = pcd.normalize_normals()
        print("\nSaving the point cloud...\n")
        o3d.io.write_point_cloud(pc_file, pcd)

        print("\nSaving as NPZ file ...\n")
        npz_fname = pc_file[:-4] + ".npz"
        np.savez(npz_fname, point_data=normalized_points[idxs])



if __name__ == '__main__':
    main(parser.parse_args())
