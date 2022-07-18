import os
import json
import math
import argparse

import numpy as np
import imageio
import pybullet as p

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



def main(args):
    if args.models_file and not os.path.isfile(args.models_file):
        print('File not exists: "{}"'.format(args.models_file))
        exit(0)

    if not args.path_out:
        print('Output directory not specified')
        exit(0)

    # if not os.path.isdir(os.path.join(args.path_out, 'refined_grasps')):
    #     print('Refined grasps are not in specified folder')
    #     print('Folder:', os.path.join(args.path_out, 'refined_grasps'))
    #     exit(0)

    if not args.models_file and args.models:
        models = args.models
    else:
        with open(args.models_file) as f:
            models = f.read().splitlines()

    gripper_name = args.gripper

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



    ##########################################################
    # Camera related stuff
    width = 1920
    height = 1920
    fov = 60
    aspect = width / height
    near = 0.02
    far = 1

    view_matrix = p.computeViewMatrix([0, 0, 0.5], [0, 0, 0], [1, 0, 0])
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

    cam_target_pos = [-0.97, 0.21, -0.33]
    distance = 1.80
    cam_pitch = -23.80
    cam_yaw = 74.0
    cam_roll = 0
    up_axis = 2 # 1 for Y-axis, 2 for Z-axis

    view_matrix_2 = p.computeViewMatrixFromYawPitchRoll(
        cam_target_pos,
        distance,
        cam_yaw,
        cam_pitch,
        cam_roll,
        2)
    ##########################################################

    # Load the robot urdf in the scene:
    robot_id = p.loadURDF(gripper_urdf_file)
    print('\n')

    for model in models:
        print(f"Model: {model}")
        object_urdf_file = os.path.join(args.obj_urdf_dir, model + ".urdf")
        print(object_urdf_file)
        obj_id = p.loadURDF(object_urdf_file)

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
        images_dir = os.path.join(args.path_out, model, "images", gripper_name)
        if not os.path.isdir(images_dir):
            print('Folder does not exist for the output images')
            print('Creating the images folder:', images_dir)
            os.makedirs(images_dir)

        for i in range(len(grasp_data)):
        # for i in range(100, 110):
            full_pose = grasp_data[i]['pose']
            dofs = grasp_data[i]['dofs']
            set_pose_dofs(gripper_name, robot_id, full_pose, dofs)
            w,h,img_rgb,_,_ = p.getCameraImage(width, height, view_matrix_2, projection_matrix)
            img = np.reshape(img_rgb, (w,h,4))
            img_fname = f'img_graspnum_{i}.png'
            imageio.imwrite(os.path.join(images_dir, img_fname), img)
        
        # REMOVE the model from the scene -- otherwise the objects are not removed!!
        p.removeBody(obj_id)

    
if __name__ == '__main__':
    main(parser.parse_args())
