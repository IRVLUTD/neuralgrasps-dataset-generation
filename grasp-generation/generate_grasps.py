#!/usr/bin/env python2

import argparse
import json
import os
import time

from graspit_process import GraspitProcess
from graspit_scene import GraspitScene
from grasp_miner import GraspMiner
from grasp_saver import GraspSaver

parser = argparse.ArgumentParser(description='Grasp mining')
parser.add_argument('-m', '--models', nargs='*', default=['003_cracker_box_google_16k_textured_scale_1000'])
# parser.add_argument('-m', '--models', nargs='*', default=['phone'])
parser.add_argument('-l', '--models_file', type=str, default='') # one model entry per line
parser.add_argument('-n', '--n_jobs', type=int, default=1)
parser.add_argument('-o', '--path_out', type=str, default='')
parser.add_argument('-v', '--verbose', action='store_true')
parser.add_argument('-d', '--debug', action='store_true')
parser.add_argument('-e', '--headless', action='store_true', help="Start in headless mode")
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
                  
parser.add_argument('--change_speed', action='store_true', help="Try several joint's speed ratios")
parser.add_argument('--relax_fingers',
                    action='store_true',
                    help="Randomize squezzed fingers positions")

parser.add_argument('-ds', '--dataset', type=str, default='ycb', help="dataset name")
parser.add_argument('-s', '--max_steps', type=int, default=0, help="Max search steps per object")
parser.add_argument('-g', '--max_grasps', type=int, default=0, help="KEEP IT ZERO SINCE WE ARE NOT USING IT (Max best grasps per object)")

parser.add_argument('-r', '--gripper', type=str, default='fetch_gripper', help="robot gripper")
parser.add_argument('--num_grasps', type=int, default=2500, help="desired number of grasps")
# parser.add_argument('--planner', type=str, default="simann", help="grasp planning optimization algorithm. Options: simann, multi")
# parser.add_argument('--energy', type=str, default="CONTACT_ENERGY", help="grasp planning search energy. Options: GUIDED_POTENTIAL_QUALITY_ENERGY, CONTACT_ENERGY")


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

    gripper_name = args.gripper
    NUM_GRASPS = args.num_grasps

    proccess = GraspitProcess(graspit_dir=args.graspit_dir,
                              plugin_dir=args.plugin_dir,
                              headless=args.headless,
                              xvfb_run=args.xvfb,
                              verbose=args.verbose)

    saver = GraspSaver(path_out=args.path_out, dataset=args.dataset)
    
    generator = GraspMiner(graspit_process=proccess,
                           num_grasps=NUM_GRASPS,
                           max_steps=args.max_steps,
                           max_grasps=args.max_grasps,
                           relax_fingers=args.relax_fingers,
                           change_speed=args.change_speed,
                           robot_names=[gripper_name], # ADD multiple grippers here
                           saver=saver)

    if args.n_jobs > 1:
        from joblib import Parallel, delayed
        grasp_result = Parallel(n_jobs=args.n_jobs, verbose=50)(delayed(generator)(m) for m in models)
    else:
        grasp_result = [generator(body) for body in models]

    if args.debug:
        with GraspitProcess(graspit_dir=args.graspit_dir, plugin_dir=args.plugin_dir) as p:
            for info in grasp_result:                
                body_name, gripper, body_grasps = info[0]
                print("[Debug Load] {} : total {} grasps for {} robot gripper".format(body_name, len(body_grasps), gripper))
                scene = GraspitScene(p.graspit, gripper, body_name)
                for grasp in body_grasps:
                    scene.grasp(grasp['pose'], grasp['dofs'])
                    time.sleep(0.5)


if __name__ == '__main__':
    main(parser.parse_args())
