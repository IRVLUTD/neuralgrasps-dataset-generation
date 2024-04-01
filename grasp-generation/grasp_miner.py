import json
import itertools
import os
import numpy as np
from graspit_process import GraspitProcess
from graspit_scene import GraspitScene
from kinematics import Kinematics
from grasp_utils import *


class GraspMiner:
    """ Grasp generator """

    def __init__(self,
                 graspit_process,
                 num_grasps=100,
                 max_steps=0,
                 max_grasps=0,
                 relax_fingers=False,
                 change_speed=False,
                 robot_names=['ManoHand'],
                 saver=None):
        """Constructor
        
        Arguments:
            graspit_process {GraspitProcess} -- process
        
        Keyword Arguments:
            max_steps {int} -- max search steps per object (default: {auto})
            max_grasps {int} -- return only N best grasps per object (default: {auto})
            change_speed {bool} -- try several joint's speed ratios (default: {False})
            relax_fingers {bool} -- randomize angles of squezzed fingers (default: {False})
        """
        self._NUM_GRASPS = num_grasps
        self._process = graspit_process
        self._max_steps = max_steps
        self._max_grasps = max_grasps
        self._relax_fingers = relax_fingers
        self._robot_names = robot_names
        self._saver = saver
        # we can't change a joints speed ratios on the fly, so use several hand models
        if change_speed:
            self._robot_names += ['ManoHand_v2', 'ManoHand_v3']

    def __call__(self, object_name):
        """Generated grasps for specific object
        
        Arguments:
            object_name {str} -- object
        
        Returns:
            tuple -- object_name, generated grasps
        """
        if not self._process.run:
            self._process.start()       

        collision_path = os.path.join(self._process.dir,
            'models/objects/{}_collision.json'.format(object_name))
        collisions = []
        if os.path.exists(collision_path):
            collisions = json.load(open(collision_path, 'r'))
        if not collisions:
            collisions = [None]
        all_grasps = [] # list contianing tuples of (obj_name, robot_name, grasp_list)

        for robot_name, collision in itertools.izip(itertools.cycle(self._robot_names), collisions):
            # load hand and body and optionally collision object
            gripper = None
            if (collision is not None) and 'gripper' in collision.keys():
                gripper = collision['gripper']
            
            grasps_gripper_all = [] # Tmp list to store the grasps sepcific to a gripper (robot_name)
            total_plans = 0 # counter to store total number of plans (grasps) generated so far
            # Repeatedly plan and select *good quality* grasps until the desired number of
            # grasps is met.
            while total_plans < self._NUM_GRASPS:
                scene = GraspitScene(self._process.graspit, robot_name, object_name, gripper)
                # plan grasps with a standard procedure
                plans = scene.planGrasps(max_steps=self._max_steps)
                # execute grasps with different euristics
                variants = (
                    dict(approach=False, auto_open=False),  #
                    dict(approach=False, auto_open=True, full_open=True),
                    dict(approach=True, auto_open=True, full_open=False),
                    dict(approach=True, auto_open=True, full_open=True)
                )                
                # print("[Miner] robot: {}, | PlanGrasps generated {} grasps".format(robot_name, len(plans)))
                # print("[Miner] robot: {}, | Iteration Stage:  current {} plans out of total={}".format(robot_name, total_plans, self._NUM_GRASPS))
                total_plans += len(plans)

                grasps_to_save = []
                for plan in plans:
                    pose = plan['pose']
                    dofs = plan['dofs']
                    for args in variants:
                        # this return grasp is in our format! controlled via grasp_utils.grasp_from_robot_state()
                        grasp = scene.grasp(pose, dofs, object_name, **args)
                        if grasp is not None:
                            grasps_to_save.append(grasp)
                
                # print("[Miner] robot: {} | Total {} grasps selected from plans at this iteration".format(robot_name, len(grasps_to_save)))
                
                # [OPTIONAL] sort by quality
                # grasps_to_save.sort(key=lambda g: g['quality'], reverse=True)
                # cut best grasps
                # if self._max_grasps > 0:
                    # grasps_to_save = grasps_to_save[:self._max_grasps]

                # GraspIt has a tendency to squeeze the fingers even
                # they aren't in contact with an object.
                # Below we randomize joints positions in such case
                # if self._relax_fingers:
                #     rs = np.random.RandomState(0)
                #     for i, joints in squeezed(grasps):
                #         pose = grasps[i]['pose']
                #         dofs = list(grasps[i]['dofs'])
                #         for attempt in range(20):
                #             angles = rs.uniform(0.0, 2.0, size=len(joints))
                #             for j, a in zip(joints, angles):
                #                 dofs[j] = a
                #             grasp = scene.grasp(pose, dofs, object_name)
                #             if grasp is not None:
                #                 grasps[i]['dofs'] = tuple(dofs)

                grasps_gripper_all.extend(grasps_to_save)
                # print("[Miner] robot: {}, | Adding to grasps_gripper_all: {} grasps".format(robot_name, len(grasps_gripper_all)))
                # Save the current progress (over-write the existing file)
                if self._saver:
                    self._saver(object_name, grasps_gripper_all, robot_name, str(self._NUM_GRASPS), str(total_plans))
            
            # Append all the selected grasps for the (object, gripper) pair to the global list
            all_grasps.append((object_name, robot_name, grasps_gripper_all))
        
        return all_grasps
