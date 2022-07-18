import numpy as np
import json
import os
import argparse

def farthest_point_sampling(points, n_samples):
    """
    points: [N, 3] array containing the whole point cloud
    n_samples: samples you want in the sampled point cloud typically << N 
    """
    points = np.array(points)
    
    # Represent the points by their indices in points
    points_left = np.arange(len(points)) # [P]

    # Initialise an array for the sampled indices
    sample_inds = np.zeros(n_samples, dtype='int') # [S]

    # Initialise distances to inf
    dists = np.ones_like(points_left) * float('inf') # [P]

    # Select a point from points by its index, save it
    selected = 0
    sample_inds[0] = points_left[selected]

    # Delete selected 
    points_left = np.delete(points_left, selected) # [P - 1]

    # Iteratively select points for a maximum of n_samples
    for i in range(1, n_samples):
        # Find the distance to the last added point in selected
        # and all the others
        last_added = sample_inds[i-1]
        
        dist_to_last_added_point = (
            (points[last_added] - points[points_left])**2).sum(-1) # [P - i]

        # If closer, updated distances
        dists[points_left] = np.minimum(dist_to_last_added_point, 
                                        dists[points_left]) # [P - i]

        # We want to pick the one that has the largest nearest neighbour
        # distance to the sampled points
        selected = np.argmax(dists[points_left])
        sample_inds[i] = points_left[selected]

        # Update points_left
        points_left = np.delete(points_left, selected)

    # return points[sample_inds]
    # We need the sample_inds to also obtain the full pose information
    return points[sample_inds], sample_inds


parser = argparse.ArgumentParser(description='Grasp refinement by farthest point sampling')
parser.add_argument('-m', '--models', nargs='*', default=['003_cracker_box_google_16k_textured_scale_1000'])
parser.add_argument('-l', '--models_file', type=str, default='') # one model entry per line
parser.add_argument('-o', '--path_out', type=str, default='')
# Data set directory
# Assumes that the initial grasps will be in: datadir/initial_grasps/obj-gripper.json

parser.add_argument('-ds', '--dataset', type=str, default='ycb', help="dataset name")
parser.add_argument('-r', '--gripper', type=str, default='fetch_gripper', help="robot gripper")
parser.add_argument('--num_samples', type=int, default=50, help="desired number of samples")


def main(args):
    if args.models_file and not os.path.isfile(args.models_file):
        print('File not exists: "{}"'.format(args.models_file))
        exit(0)

    if not args.path_out:
        print('Output directory not specified')
        exit(0)

    if not os.path.isdir(os.path.join(args.path_out, 'initial_grasps')):
        print('Initial grasps are not in specified folder')
        print(os.path.join(args.path_out, 'initial_grasps'))
        exit(0)

    if not os.path.isdir(os.path.join(args.path_out, 'refined_grasps')):
        print('Folder does not exist for the output refined grasps')
        print('Creating the folder...')
        os.mkdir(os.path.join(args.path_out, 'refined_grasps'))

    if not args.models_file and args.models:
        models = args.models
    else:
        with open(args.models_file) as f:
            models = f.read().splitlines()
    
    gripper_name = args.gripper
    num_samples = args.num_samples
    
    for model in models:
        grasp_file = '{}-{}.json'.format(model, gripper_name)
        grasps_fpath = os.path.join(args.path_out, 'initial_grasps', grasp_file)
        print("Loading the initial set of grasps...")
        if not os.path.isfile(grasps_fpath):
            print("Grasps file does not exist?! Try checking the path")
            print("Not saving the refined grasps for the {} object".format(model))
            continue
        
        with open(grasps_fpath, "r") as gf:
            all_data = json.load(gf)
        grasp_data = all_data['grasps']
        N = len(grasp_data)
        data_grasp_position = np.zeros((N,3))
        
        for i,item in enumerate(grasp_data):
            full_pose = item['pose']
            data_grasp_position[i] = full_pose[:3]
        print("Sampling from the initial set of grasps...")
        _, sampled_idxs = farthest_point_sampling(data_grasp_position, num_samples)
        sampled_grasps = [grasp_data[idx] for idx in sampled_idxs]
        # Create a dict to save the refined json
        sampled_grasps_description = {
            'grasps': sampled_grasps,
            'gripper': all_data['gripper'],
            'dataset': all_data['dataset'],
            'object_scale': all_data['object_scale'],
            'object_cat': '', # category, not imporatnt for us
            'object_id': all_data['object_id']
        }
        print("Saving the sampled refined set of grasps...")
        # Save the json
        refined_grasp_file = 'refined_{}-{}.json'.format(model, gripper_name)
        new_grasp_fpath = os.path.join(args.path_out, 'refined_grasps', refined_grasp_file)
        with open(new_grasp_fpath, 'w') as outf:
            json.dump(sampled_grasps_description, outf)


if __name__ == '__main__':
    main(parser.parse_args())