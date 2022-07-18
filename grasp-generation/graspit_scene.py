import os
import numpy as np
from geometry_msgs.msg import Pose

from kinematics import Kinematics
from grasp_utils import *


class GraspitScene:
    """ Scene with a hand (robot) and a body """

    def __init__(self, graspit, robot, body, collision_object=None):
        default_pose = Pose()
        default_pose.position.y = 0.5 # does not matter that much!
        default_pose.orientation.w = 1

        graspit.clearWorld()
        graspit.importRobot(robot)
        self._robot_ids = graspit.getBodies().ids
        graspit.setRobotPose(default_pose)
        graspit.importGraspableBody(body)
        
        self._body_id = graspit.getBodies().ids[-1]

        self._collision_object = collision_object
        self._collision_object_id = -1
        if self._collision_object is not None:
            graspit.importObstacle(self._collision_object['id'], msg_from_pose(self._collision_object['pose']))
            self._collision_object_id = graspit.getBodies().ids[-1]
            graspit.toggleCollisions(False, self._collision_object_id, self._body_id)

        self._graspit = graspit
        self._robot = robot
        self._body = body
        # self._kinematics = Kinematics('{}/models/robots/{}'.format(os.environ['GRASPIT'], robot))
        self._kinematics = None

    def planGrasps(self, max_steps=70000):
        """Plan grasps
        
        Keyword Arguments:
            max_steps {int} -- planner max steps (default: {70000})
        
        Returns:
            list -- list of planned grasps
        """        
        from graspit_interface.msg import Planner
        result = self._graspit.planGrasps(max_steps=max_steps)
        # result = self._graspit.planGrasps(max_steps=max_steps)
        return [grasp_from_msg(g) for g in result.grasps]

    def grasp(self, pose, dofs, body='', approach=False, auto_open=False, full_open=False):
        """Execute a grasp
        
        Arguments:
            pose {list} -- hand root position
            dofs {list} -- hand dofs angles
            body {str} -- grasping body name
            approach {bool} -- approch to contact before close fingers
            auto_open {bool} -- open fingers before closing
            full_open {bool} -- disable collision checking while opening
        
        Returns:
            dict -- grasp data
        """
        graspit = self._graspit
        kinematics = self._kinematics
        try:
            # execute grasp
            graspit.toggleAllCollisions(False)
            graspit.setRobotPose(msg_from_pose(pose))
            graspit.forceRobotDof(dofs)
            if auto_open:
                if not full_open:
                    graspit.toggleAllCollisions(True)
                graspit.autoOpen()
            graspit.toggleAllCollisions(True)
            if approach:
                graspit.approachToContact()
            
            # AUTO GRASP 
            graspit.autoGrasp()
            
            # COMPUTER QUALITY
            quality = graspit.computeQuality()
            qual_res, qual_w, qual_eps = quality.result, quality.volume, quality.epsilon

            energy_types = ['POTENTIAL_QUALITY_ENERGY', 'GUIDED_POTENTIAL_QUALITY_ENERGY', 'CONTACT_ENERGY']
            energy_vals  = [0,0,0]
            for i, energy_t in enumerate(energy_types):
                energy_vals[i] = graspit.computeEnergy(energy_t)
            
            # if True or (quality.result == 0 and quality.epsilon > -1):
            if (quality.result == 0 and quality.epsilon > -1):
            # if (quality.result == 0):
                # print("Good Grasp!")
                response = graspit.getRobot()
                robot = response.robot
                # grasp = grasp_from_robot_state(robot, quality, body, kinematics)
                grasp = grasp_from_robot_state(robot, quality, self._body, kinematics)
                
                if self._collision_object:
                    distances = [graspit.getDistance(i,self._collision_object_id).distance for i in self._robot_ids]
                    distances.extend([graspit.getDistance(i,self._body_id).distance for i in self._robot_ids])
                    collision_contacts = [c for c in robot.contacts if c.body2 == self._collision_object['id']]
                    if collision_contacts:
                        return
                    grasp['handpart_distances'] = distances
                    grasp['collision_object'] = self._collision_object
                
                return grasp
        except Exception:
            pass

    def __repr__(self):
        return "Scene {} -> {}".format(self._robot, self._body)