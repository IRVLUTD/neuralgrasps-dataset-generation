import os
import json
import math
import argparse

import numpy as np
import imageio
import pybullet as p
import open3d as o3d
from tqdm import tqdm

from grasp_pc import set_pose_dofs

parser = argparse.ArgumentParser(description='Grasp to Images')
parser.add_argument('-m', '--models', nargs='*',
                    default=['003_cracker_box_google_16k_textured_scale_1000'])
parser.add_argument('-l', '--models_file', type=str,
                    default='')  # one model entry per line

# Data set directory
# Assumes that the grasps will be in: datadir/refined_grasps/obj-gripper.json
# The saved point clouds will be in: datadir/object_name/point_cloud/gripper/*.ply

parser.add_argument('-o', '--path_out', type=str, default='../../../dataset_train/',
                    help='dataset dir. assumes that the refined grasps will be here')

parser.add_argument('--urdf_dir', type=str,
                    default='../prepare-graspit-urdf/urdf/', help="Folder for gripper URDF fles")

parser.add_argument('--obj_urdf_dir', type=str, default='./ycb-urdfs/',
                    help="Folder for ycb object URDF fles")

parser.add_argument('-r', '--gripper', type=str,
                    default='fetch_gripper', help="robot gripper")


def compute_view_projection(objpc, ycbid: str):
        obj_center = np.mean(objpc, axis=0)
        limit_max = np.max(objpc, axis=0)
        limit_min = np.min(objpc, axis=0)
        obj_range = abs(limit_max-limit_min)

        width = 1920
        height = 1080
        fov = 45
        aspect = width / height
        near = 0.08
        far = 5

        delta_dir = np.array(obj_center)
        if obj_range[0] < obj_range[1]:
            delta_dir -= np.array([0.8 ,0, 0])
        else:
            delta_dir -= np.array([0, 0.8, 0])
            
        target_delta = np.array([0, 0, 0.1])
        if ycbid == "010":
            target_delta[2] = -0.1
        elif ycbid == "008" or ycbid == "009":
            delta_dir = np.array([0, 0, 0.5]) # We change it completely for 008, 009
            
        # cam_eye_pos = [-0.5, -0.5, 1.0]
        cam_eye_pos = list(delta_dir)
        # cam_tar_pos = [0, 0, 0.2]
        cam_tar_pos = list(obj_center - target_delta)
        cam_up_vect = [0, 0, 1]

        view_matrix = p.computeViewMatrix(cam_eye_pos, cam_tar_pos, cam_up_vect)
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
        return view_matrix, projection_matrix, width, height


def main(args):
    if args.models_file and not os.path.isfile(args.models_file):
        print('File not exists: "{}"'.format(args.models_file))
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
    imgdir_name = "highres_imgs"
    if gripper_name == "fetch_gripper":
        gripper_urdf = "fetch_gripper/fetch_gripper.urdf"
    elif gripper_name == "Barrett":
        gripper_urdf = "Barrett/Barrett.urdf"
    elif gripper_name == "HumanHand":
        gripper_urdf = "HumanHand/HumanHand.urdf"
    elif gripper_name == "Allegro":
        gripper_urdf = "Allegro/allegro_hand_description_right.urdf"
    elif gripper_name == "panda_grasp":
        gripper_urdf = "franka_panda/panda_arm.urdf"

    gripper_urdf_file = os.path.join(args.urdf_dir, gripper_urdf)
    if not os.path.isfile(gripper_urdf_file):
        print("URDF file for gripper not found! Exiting...")
        print("Searching for {} in location: {}".format(
            gripper_urdf, args.urdf_dir))
        exit(0)

    # p.connect(p.GUI) # with GUI, useful to see final result
    p.connect(p.DIRECT) # without GUI, useful when coding and trying out pybullet functions


    # Load the robot urdf in the scene:
    robot_id = p.loadURDF(gripper_urdf_file)
    print('\n')

    for model in models:
        ycbid = model[:3]
        print(f"Model: {model}")
        object_urdf_file = os.path.join(args.obj_urdf_dir, model + ".urdf")
        print(object_urdf_file)
        obj_id = p.loadURDF(object_urdf_file)

        # Load the object PC
        objpc_f = os.path.join(args.path_out, model, "object_point_cloud.ply")
        obj_pcd = o3d.io.read_point_cloud(objpc_f)
        objpc = np.array(obj_pcd.points)
        view_matrix, projection_matrix, width, height = compute_view_projection(objpc, ycbid)


        grasps_file = 'refined_{}-{}.json'.format(model, gripper_name)
        grasp_fpath = os.path.join(
            args.path_out, 'refined_grasps', grasps_file)
        if not os.path.isfile(grasp_fpath):
            print('Refined grasps file (.json) does exist!')
            print('Check the location:', grasp_fpath)
            print('Continuing to next object model without saving images....')
            continue
        with open(grasp_fpath, "r") as gf:
            all_data = json.load(gf)
        grasp_data = all_data['grasps']
        print("\nLoaded {} grasps \n".format(len(grasp_data)))

        # images_dir = './images/' + gripper_name
        images_dir = os.path.join(args.path_out, model, imgdir_name, gripper_name)
        if not os.path.isdir(images_dir):
            print('Folder does not exist for the output images')
            print('Creating the HIGHRES images folder:', images_dir)
            os.makedirs(images_dir)

        # for i in range(len(grasp_data)):
        for i in range(100, 110):
            print(i)      
            full_pose = grasp_data[i]['pose']
            dofs = grasp_data[i]['dofs']
            set_pose_dofs(gripper_name, robot_id, full_pose, dofs)
            # Generate the images
            images = p.getCameraImage(width, height, view_matrix, projection_matrix)
            rgb_opengl = np.reshape(images[2], (height, width, 4)) # * 1. / 255.            
            img_fname = f'img_graspnum_{i}.png'
            imageio.imwrite(os.path.join(images_dir, img_fname), rgb_opengl)
        
        # REMOVE the model from the scene -- otherwise the objects are not removed!!
        p.removeBody(obj_id)
        print("\n\n")

    
if __name__ == '__main__':
    main(parser.parse_args())
