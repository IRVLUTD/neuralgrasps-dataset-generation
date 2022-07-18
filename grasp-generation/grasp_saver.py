import yaml
import json
import itertools
import os
import numpy as np
from kinematics import Kinematics
from grasp_utils import *


class GraspSaver:
    """ Grasp saver """

    def __init__(self, path_out, dataset):
        """Constructor
        
        Arguments:
            graspit_process {GraspitProcess} -- process
        """
        self._path_out = path_out
        self._dataset = dataset

    def __call__(self, body_name, body_grasps, gripper_name, custom_ext):
        """Generated grasps for specific object
        
        Arguments:
            body_name {str}:  object name
            body_grasps {list}: list of grasps
            gripper_name {str}: robot gripper name
            custom_ext {str}: custom additions to the filename (usually other args) 
        
        Returns:
            None (saves a json file)
        """
        # File for saving the incremental progress
        # The Json dump includes an additional "iteration" field for tracking grasps numbers
        tmp_grasps_file = os.path.join(self._path_out, 'tmp-{}-{}-{}.json'.format(
            body_name, 
            gripper_name,
            custom_ext)
        )                        
        with open(tmp_grasps_file, 'w') as grasps_file:
            scale = 1
            split = body_name.split('_scale_')
            if len(split) > 1:
                scale = float(split[1])
            object_id = split[0]
            grasps_description = {
                'grasps': body_grasps,
                'gripper': gripper_name,
                'iteration': len(body_grasps),
                'dataset': self._dataset,
                'object_scale': scale,
                'object_cat': '',
                'object_id': object_id
            }
            json.dump(grasps_description, grasps_file)
        
        print('{}: Saved {} grasps for {}'.format(
            body_name,
            len(body_grasps),
            gripper_name
        ))
        
        
        grasps_filename = os.path.join(self._path_out, '{}-{}.json'.format(
            body_name, 
            gripper_name))
        # if os.path.exists(grasps_filename):
        #     with open(grasps_filename, 'r') as grasps_file:
        #         existing_grasps = json.load(grasps_file)
        #     body_grasps.extend(existing_grasps['grasps'])
        #     print('{}: loading {} grasps'.format(
        #         body_name,
        #         len(existing_grasps['grasps']),
        #     ))
        
        print('{}: Saving {} grasps for {} gripper'.format(body_name, len(body_grasps), gripper_name))

        with open(grasps_filename, 'w') as grasps_file:
            scale = 1
            split = body_name.split('_scale_')
            if len(split) > 1:
                scale = float(split[1])
            object_id = split[0]
            grasps_description = {
                'grasps': body_grasps,
                'gripper': gripper_name,
                'dataset': self._dataset,
                'object_scale': scale,
                'object_cat': '', # category, not imporatnt for us
                'object_id': object_id
            }
            json.dump(grasps_description, grasps_file)
            
