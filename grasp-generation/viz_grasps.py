#!/usr/bin/env python2

import argparse
import json
import os
import time

from graspit_process import GraspitProcess
from graspit_scene import GraspitScene
from grasp_miner import GraspMiner
from grasp_saver import GraspSaver

parser = argparse.ArgumentParser(description='Mined Grasps Viz in Graspit')
parser.add_argument('-m', '--models', nargs='*', default=['003_cracker_box_google_16k_textured_scale_1000'])
# parser.add_argument('-m', '--models', nargs='*', default=['glass'])
parser.add_argument('-l', '--models_file', type=str, default='')
parser.add_argument('-o', '--path_out', type=str, default='')
parser.add_argument('-x',
                    '--xvfb',
                    action='store_true',
                    help="Start with Xserver Virtual Frame Buffer (Xvfb)")
parser.add_argument('--graspit_dir',
                    type=str,
                    default=os.environ['GRASPIT'],
                    help="Path to GraspIt root directory")
parser.add_argument('--plugin_dir',
                    type=str,
                    default=os.environ['GRASPIT_PLUGIN_DIR'],
                    help="Path to directory with a graspit_interface plugin")
parser.add_argument('-ds', '--dataset', type=str, default='', help="dataset name")

parser.add_argument('-r', '--gripper', type=str, default='ManoHand', help="robot gripper")

def main(args):
    if not os.path.isdir(args.graspit_dir):
        print('Wrong GraspIt path: "{}"'.format(args.graspit_dir))
        exit(0)

    if not os.path.isdir(args.plugin_dir):
        print('Wrong plugins path: "{}"'.format(args.plugin_dir))
        exit(0)

    if args.models_file and not os.path.isfile(args.models_file):
        print('File not exists: "{}"'.format(args.models_file))
        exit(0)

    if not args.path_out:
        print('Output directory not specified')
        exit(0)

    if not os.path.isdir(args.path_out):
        os.makedirs(args.path_out)

    if not args.models_file and args.models:
        models = args.models
    else:
        with open(args.models_file) as f:
            models = f.read().splitlines()

    gripper = args.gripper
    path_out = args.path_out
    body = models[0]
    
    print("Visualizing {} grasps for {} robot gripper".format(body, gripper))

    grasps_filename = os.path.join(path_out, 'refined_{}-{}.json'.format(
        body, 
        gripper))
    
    if os.path.exists(grasps_filename):
        with open(grasps_filename, 'r') as grasps_file:
            existing_grasps_dict = json.load(grasps_file)
        # body_grasps.extend(existing_grasps['grasps'])
        body_grasps = existing_grasps_dict['grasps'] 
        object_id = existing_grasps_dict['object_id']
        print('{},{}: loading {} grasps'.format(
            object_id,
            body,
            len(body_grasps),
        ))
    else:
        print("Grasp file: {} , not found!".format(grasps_filename))
        print("The object model is:{}".format(models[0]))
    
    with GraspitProcess(graspit_dir=args.graspit_dir, plugin_dir=args.plugin_dir) as p:            
        print("{} : total {} grasps".format(body, len(body_grasps)))
        scene = GraspitScene(p.graspit, gripper, body)
        for i, grasp in enumerate(body_grasps):
            scene.grasp(grasp['pose'], grasp['dofs'])
            print("grasp-number: {}".format(i))
            time.sleep(5.0)


if __name__ == '__main__':
    main(parser.parse_args())
