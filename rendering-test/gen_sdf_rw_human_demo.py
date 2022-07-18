import json
import math
import os
import argparse
import numpy as np
import open3d as o3d

import sdf_utils
from compute_sdf import PointCloudData

parser = argparse.ArgumentParser(description='Point cloud ot SDF values')
parser.add_argument('-m', '--models', nargs='*',
                    default=['003_cracker_box_google_16k_textured_scale_1000'])
parser.add_argument('-l', '--models_file', type=str,
                    default='')  # one model entry per line
parser.add_argument('-o', '--path_out', type=str, default='./../../../dataset_human_demo/',
                    help='dataset dir. assumes that the refined grasps will be here')
# Data set directory
# Assumes that the grasps will be in: datadir/refined_grasps/obj-gripper.json
# The saved point clouds will be in: datadir/object_name/point_cloud/gripper/*.ply
parser.add_argument('-ds', '--dataset', type=str,
                    default='ycb', help="dataset name")
parser.add_argument('-r', '--gripper', type=str,
                    default='RealHuman', help="Having a separate name for the real world human demo")
parser.add_argument('-n', '--num_samples', type=int, default=40000,
                    help="number of points to sample from scene")
parser.add_argument('-s', '--normal_sample_count', type=int, default=200,
                    help="number of normals to vote for sign of sdf values")


def main(args):
    if args.models_file and not os.path.isfile(args.models_file):
        print('File does not exist: "{}"'.format(args.models_file))
        exit(0)
    if not args.path_out:
        print('Output directory not specified')
        exit(0)
    if not args.models_file and args.models:
        models = args.models
    else:
        with open(args.models_file) as f:
            models = f.read().splitlines()
    dataset_dir = os.path.abspath(args.path_out)
    gripper_name = args.gripper
    normal_sample_count = args.normal_sample_count
    # To be equally divided amongst objet and gripper
    num_sample_points = args.num_samples // 2

    #################################################################################
    for model in models:
        modelobj_pc_file = os.path.join(
            dataset_dir, model, 'object_point_cloud.ply')
        pc_folder = os.path.join(dataset_dir, model,
                                 'point_cloud', gripper_name)
        sdf_folder = os.path.join(dataset_dir, model, 'sdf', gripper_name)
        if not os.path.isfile(modelobj_pc_file):
            print('Object Point cloud not in specified folder')
            print(f'Expecting file: {modelobj_pc_file}')
            print('Continuing to next object model without saving SDFs....')
            continue
        if not os.path.isdir(pc_folder):
            print('Folder does not exist for the output point clouds')
            print('Continuing to next object model without saving SDFs....')
            continue
        if not os.path.isdir(sdf_folder):
            print('Folder does not exist for the output sdf values')
            print('Creating the sdf folder:', sdf_folder)
            os.makedirs(sdf_folder)

        obj_pcd = o3d.io.read_point_cloud(modelobj_pc_file)
        # Collect offset and scale for the object point cloud
        # Have a buffer of 2 or 3 (so essentially scaling to sphere of radius 0.5).
        # This is so that gripper points can also safely fit in.
        # Also save the normalization params to an npz file
        # offset, scale = sdf_utils.get_norm_params_sphere(
        #     np.asarray(obj_pcd.points), buffer=3)
        # print(f"Normalization params for {model}")
        # print(f"Offset and Scale : {offset}, {scale}")
        # # Saving the normalization params in the base object folder
        normalization_file = os.path.join(dataset_dir, model, "norm_params.npz")
        # np.savez(normalization_file, offset=offset, scale=scale)
        data = np.load(normalization_file)
        offset = data['offset']
        scale  = data['scale']

        obj_points = sdf_utils.scale_using_params(
            np.asarray(obj_pcd.points), scale, offset)
        obj_sdfpc = PointCloudData(obj_points, np.asarray(obj_pcd.normals))
        query_points_obj = obj_sdfpc.sample_query_points(
            number_of_points=num_sample_points)
        _pc_files = os.listdir(pc_folder)
        # number of files = len(_pc_files)
        print('Object query points generated...')
        # Div by 2 since the other half of files correspond to the combined point cloud!
        for idx in range(len(_pc_files)//2):
            grp_pc_fname = f'single_graspnum_{idx}.ply'
            print(grp_pc_fname)
            grp_pc_file = os.path.join(pc_folder, grp_pc_fname)
            sdf_file = os.path.join(sdf_folder, f'sdf_graspnum_{idx}.npz')

            grp_pcd = o3d.io.read_point_cloud(grp_pc_file)
            grp_points = sdf_utils.scale_using_params(
                np.asarray(grp_pcd.points), scale, offset)
            grp_sdfpc = PointCloudData(grp_points, np.asarray(grp_pcd.normals))

            query_points_grp = grp_sdfpc.sample_query_points(
                number_of_points=num_sample_points)
            print('Gripper query points generated...')

            query_points = np.concatenate(
                [query_points_obj, query_points_grp], axis=0)

            print('Computing SDF to object...')
            sdf_obj = obj_sdfpc.get_sdf_in_batches(
                query_points, sample_count=normal_sample_count)
            sdf_obj = sdf_obj[:, np.newaxis]  # Make it (N,1) from (N,)

            print('Computing SDF to gripper...')
            sdf_grp = grp_sdfpc.get_sdf_in_batches(
                query_points, sample_count=normal_sample_count)
            sdf_grp = sdf_grp[:, np.newaxis]  # Make it (N,1) from (N,)

            print('Saving the data as .npz file...\n')
            # Data saved as N x 5 array with N query points and 1 column each for
            # obj and gripper sdf values
            data_sdf = np.concatenate([query_points, sdf_obj, sdf_grp], axis=1)
            np.savez(sdf_file, data=data_sdf)


if __name__ == '__main__':
    main(parser.parse_args())
