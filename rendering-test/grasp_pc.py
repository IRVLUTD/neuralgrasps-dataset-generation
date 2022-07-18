import json
import math
import os
import argparse
import pybullet as p
import numpy as np
import open3d as o3d

import render


def set_pose_dofs(gripper, robot_id, full_pose, dofs):
    pos = full_pose[:3]
    orn = full_pose[3:]
    p.resetBasePositionAndOrientation(robot_id, pos, orn)

    if gripper == "fetch_gripper":
        dofvalue = (dofs[0] / 1000.0) * 10.0
        p.resetJointState(robot_id, 1, dofvalue)
        p.resetJointState(robot_id, 3, dofvalue)
    elif gripper == "Barrett":
        counter = 0
        joint_idxs = []
        for i in range(p.getNumJoints(robot_id)):
            info = p.getJointInfo(robot_id, i)
            joint_type = info[2]
            if joint_type == 0:  # revolute joint
                counter += 1
                jid = info[0]
                joint_idxs.append(jid)
        j_states = [dofs[0], dofs[1], dofs[1]/3.0, dofs[0],
                    dofs[2], dofs[2]/3.0, dofs[3], dofs[3]/3.0]
        for i, jid in enumerate(joint_idxs):
            p.resetJointState(robot_id, jid, j_states[i])
    elif gripper == "HumanHand":
        counter = 0
        joint_idxs = []
        for i in range(p.getNumJoints(robot_id)):
            info = p.getJointInfo(robot_id, i)
            joint_type = info[2]
            if joint_type == 0:  # revolute joint
                counter += 1
                jid = info[0]
                joint_idxs.append(jid)
        for i, jid in enumerate(joint_idxs):
            p.resetJointState(robot_id, jid, dofs[i])
    elif gripper == "Allegro":
        counter = 0
        joint_idxs = []
        for i in range(p.getNumJoints(robot_id)):
            info = p.getJointInfo(robot_id, i)
            joint_type = info[2]
            if joint_type == 0:  # revolute joint
                counter += 1
                jid = info[0]
                joint_idxs.append(jid)
        for i, jid in enumerate(joint_idxs):
            p.resetJointState(robot_id, jid, dofs[i])
    elif gripper == "panda_grasp":
        # dofvalue = (dofs[0] / 1000.0) * 10.0
        p.resetJointState(robot_id, 0, dofs[0]/1000)
        p.resetJointState(robot_id, 1, dofs[1]/1000)

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


parser = argparse.ArgumentParser(description='Grasp to Point cloud')
parser.add_argument('-m', '--models', nargs='*',
                    default=['003_cracker_box_google_16k_textured_scale_1000'])
parser.add_argument('-l', '--models_file', type=str,
                    default='')  # one model entry per line
# Data set directory
# Assumes that the grasps will be in: datadir/refined_grasps/obj-gripper.json
# The saved point clouds will be in: datadir/object_name/point_cloud/gripper/*.ply
# The saved mesh         will be in: datadir/object_name/mesh/gripper/*.obj

parser.add_argument('-o', '--path_out', type=str, default='../../../dataset_train/',
                    help='dataset dir. assumes that the refined grasps will be here')
parser.add_argument('--urdf_dir', type=str,
                    default='../prepare-graspit-urdf/urdf/', help="Folder for gripper URDF fles")
parser.add_argument('--obj_urdf_dir', type=str, default='./ycb-urdfs/',
                    help="Folder for ycb object URDF fles")
parser.add_argument('-ds', '--dataset', type=str,
                    default='ycb', help="dataset name")
parser.add_argument('-r', '--gripper', type=str,
                    default='fetch_gripper', help="robot gripper")

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
    num_views = args.num_views

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

    # Camera Params
    params = {}
    params['img_width'] = args.img_width
    params['img_height'] = args.img_height
    params['near'] = args.cam_near
    params['far'] = args.cam_far
    params['fov'] = args.cam_fov
    # factor = 1

    p.connect(p.DIRECT)
    # Load the robot urdf in the scene:
    robot_id = p.loadURDF(gripper_urdf_file)
    print('\n')

    #################################################################################
    # Only the gripper
    for model in models:
        print(f"Model: {model}")
        pc_folder = os.path.join(args.path_out, model,
                                 'point_cloud', gripper_name)
        # mesh_folder = os.path.join(args.path_out, model, 'mesh', gripper_name)
        grasps_file = 'refined_{}-{}.json'.format(model, gripper_name)
        grasp_fpath = os.path.join(
            args.path_out, 'refined_grasps', grasps_file)
        if not os.path.isfile(grasp_fpath):
            print('Refined grasps file (.json) does exist!')
            print('Check the location:', grasp_fpath)
            print('Continuing to next object model without saving point clouds....')
            continue
        if not os.path.isdir(pc_folder):
            print('Folder does not exist for the output point clouds')
            print('Creating the point cloud folder:', pc_folder)
            os.makedirs(pc_folder)
        # if not os.path.isdir(mesh_folder):
        #     print('Folder does not exist for the output meshes')
        #     print('Creating the mesh folder:', mesh_folder)
        #     os.makedirs(mesh_folder)

        with open(grasp_fpath, "r") as gf:
            all_data = json.load(gf)
        grasp_data = all_data['grasps']
        print("\nLoaded {} grasps \n".format(len(grasp_data)))
        for idx in range(len(grasp_data)):
            pc_fname = 'single_graspnum_{}.ply'.format(idx)
            pc_file = os.path.join(pc_folder, pc_fname)
            # mesh_fname = 'single_graspnum_{}.obj'.format(idx)
            # mesh_file = os.path.join(mesh_folder, mesh_fname)

            print("Generating point cloud for grasp num:", idx)
            full_pose = grasp_data[idx]['pose']
            dofs = grasp_data[idx]['dofs']
            set_pose_dofs(gripper_name, robot_id, full_pose, dofs)
            # Render the grasp!
            points_normals_views = [get_points_from_random_viewpoint(
                robot_id, params) for i in range(num_views)]
            points_views = [points_normals_views[i][0]
                            for i in range(num_views)]
            normls_views = [points_normals_views[i][1]
                            for i in range(num_views)]
            data_points = np.concatenate(points_views, axis=0)
            data_normal = np.concatenate(normls_views, axis=0)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(data_points)
            pcd.normals = o3d.utility.Vector3dVector(data_normal)
            pcd = pcd.normalize_normals()
            print("Saving the point cloud...")
            o3d.io.write_point_cloud(pc_file, pcd)
    
    print("\n")

    #################################################################################
    # With object and gripper model
    # Simply load the object urdf inside the loop
    print("\nPC and Mesh Generation for combined (object+gripper)\n")
    for model in models:
        print(f"Model: {model}")
        # object_urdf_file = "./003_cracker_box_custom.urdf"
        object_urdf_file = os.path.join(args.obj_urdf_dir, model + ".urdf")
        obj_id = p.loadURDF(object_urdf_file)

        pc_folder = os.path.join(args.path_out, model,
                                 'point_cloud', gripper_name)
        # mesh_folder = os.path.join(args.path_out, model, 'mesh', gripper_name)
        grasps_file = 'refined_{}-{}.json'.format(model, gripper_name)
        grasp_fpath = os.path.join(
            args.path_out, 'refined_grasps', grasps_file)
        if not os.path.isdir(pc_folder):
            print('Folder does not exist for the output point clouds')
            print('Creating the folder...')
            os.makedirs(pc_folder)
        if not os.path.isfile(grasp_fpath):
            print('Refined grasps file (.json) does exist!')
            print('Please check the location:', grasp_fpath)
            print('Continuing to next object model without saving point clouds....')
            continue

        with open(grasp_fpath, "r") as gf:
            all_data = json.load(gf)
        grasp_data = all_data['grasps']
        print("\nLoaded {} grasps \n".format(len(grasp_data)))
        for idx in range(len(grasp_data)):
            pc_fname = 'combined_graspnum_{}.ply'.format(idx)
            pc_file = os.path.join(pc_folder, pc_fname)
            # mesh_fname = 'combined_graspnum_{}.obj'.format(idx)
            # mesh_file = os.path.join(mesh_folder, mesh_fname)

            print("Generating point cloud for grasp num:", idx)
            full_pose = grasp_data[idx]['pose']
            dofs = grasp_data[idx]['dofs']
            set_pose_dofs(gripper_name, robot_id, full_pose, dofs)
            # Render the grasp!
            points_normals_views = [get_points_from_random_viewpoint(
                robot_id, params) for i in range(num_views)]
            points_views = [points_normals_views[i][0]
                            for i in range(num_views)]
            normls_views = [points_normals_views[i][1]
                            for i in range(num_views)]
            data_points = np.concatenate(points_views, axis=0)
            data_normal = np.concatenate(normls_views, axis=0)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(data_points)
            pcd.normals = o3d.utility.Vector3dVector(data_normal)
            pcd = pcd.normalize_normals()
            print("Saving the point cloud...")
            o3d.io.write_point_cloud(pc_file, pcd)
        # REMOVE the model from the scene -- otherwise the objects are not removed!!
        p.removeBody(obj_id)


if __name__ == '__main__':
    main(parser.parse_args())
