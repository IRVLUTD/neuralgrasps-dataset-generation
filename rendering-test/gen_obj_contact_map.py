import json
import math
import os
import argparse
import numpy as np
import open3d as o3d

import sdf_utils
from compute_sdf import PointCloudData

parser = argparse.ArgumentParser(description='Generate contact maps for object across different grasps')
parser.add_argument('-m', '--models', nargs='*',
                    default=['003_cracker_box_google_16k_textured_scale_1000'])
parser.add_argument('-l', '--models_file', type=str,
                    default='')  # one model entry per line
parser.add_argument('-o', '--path_out', type=str, default='../../../dataset_train/',
                    help='dataset dir. assumes that the refined grasps will be here')
# Data set directory
# Assumes that the grasps will be in: datadir/refined_grasps/obj-gripper.json
# The saved point clouds will be in: datadir/object_name/point_cloud/gripper/*.ply
parser.add_argument('-ds', '--dataset', type=str,
                    default='ycb', help="dataset name")

parser.add_argument('-r', '--gripper', type=str,
                    default='fetch_gripper', help="robot gripper")

parser.add_argument('--delta', type=float, default=0.05,
                    help="tolerance for the heatmap generation. See Obman paper Sec 3.3 --\
                          This corresponds to the characteristic distance of actions.\
                          Might need to tune this parameter depending on the results.")

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

    gripper_name = args.gripper    
    delta = args.delta

    #################################################################################
    for model in models:
        modelobj_pc_file = os.path.join(
            args.path_out, model, 'object_point_cloud.ply')
        pc_folder = os.path.join(args.path_out, model,
                                 'point_cloud', gripper_name)
        contactmap_folder = os.path.join(args.path_out, model, 'contactmap', gripper_name)
        if not os.path.isfile(modelobj_pc_file):
            print('Object Point cloud not in specified folder')
            print(f'Expecting file: {modelobj_pc_file}')
            print('Continuing to next object model without saving SDFs....')
            continue
        if not os.path.isdir(pc_folder):
            print('Folder does not exist for the output point clouds')
            print('Continuing to next object model without saving SDFs....')
            continue
        if not os.path.isdir(contactmap_folder):
            print('Folder does not exist for the output sdf values')
            print('Creating the sdf folder:', contactmap_folder)
            os.makedirs(contactmap_folder)

        obj_pcd = o3d.io.read_point_cloud(modelobj_pc_file)
        print(f"Loaded {len(obj_pcd.points)} points from the object point cloud\n")
        
        # Collect offset and scale for the object point cloud
        # Have a buffer of 2 (so essentially scaling to sphere of radius 0.5).
        # This is so that gripper points can also safely fit in.
        offset, scale = sdf_utils.get_norm_params_sphere(
            np.asarray(obj_pcd.points), buffer=3)
        
        obj_points = sdf_utils.scale_using_params(
            np.asarray(obj_pcd.points), scale, offset)
        
        # obj_sdfpc = PointCloudData(obj_points, np.asarray(obj_pcd.normals))


        # query_points_obj = obj_sdfpc.sample_query_points(
        #     number_of_points=num_sample_points)
        query_points_obj = obj_points

        _pc_files = os.listdir(pc_folder)
        # number of files = len(_pc_files)
        print('Object query points generated...\n')
        
        # Div by 2 since the other half of files correspond to the combined point cloud!
        for idx in range(len(_pc_files)//2):
            grp_pc_fname = f'single_graspnum_{idx}.ply'
            print(grp_pc_fname)
            grp_pc_file = os.path.join(pc_folder, grp_pc_fname)
            contactmap_file = os.path.join(contactmap_folder, f'contactmap_graspnum_{idx}.npz')

            grp_pcd = o3d.io.read_point_cloud(grp_pc_file)

            grp_points = sdf_utils.scale_using_params(
                np.asarray(grp_pcd.points), scale, offset)

            grp_sdfpc = PointCloudData(grp_points, np.asarray(grp_pcd.normals))

            print('Computing closest distances to gripper for the object query points...')
            # dist_grp = grp_sdfpc.get_sdf_in_batches(
            #     obj_points, sample_count=2)           

            # For contact map, we dont care about the sign, only the absolute value!
            dist_grp = grp_sdfpc.closest_point_dist(obj_points)
            print(f"dist shape: {dist_grp.shape}")
            print(f"min: {min(dist_grp)}, max: {max(dist_grp)}")
            
            data_heatmap = np.exp(-np.abs(dist_grp)/delta)
            print(f"heatmap shape: {data_heatmap.shape}")
            print(f"min: {min(data_heatmap)}, max: {max(data_heatmap)}")
            
            print('Saving the data as .npz file...\n')
            # data_heatmap = np.concatenate([query_points, sdf_obj, sdf_grp], axis=1)
            np.savez(contactmap_file, heatmap=data_heatmap)


if __name__ == '__main__':
    main(parser.parse_args())
