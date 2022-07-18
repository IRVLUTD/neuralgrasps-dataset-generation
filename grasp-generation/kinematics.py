import collections
import json
import numpy as np
import os

from math_utils import *

CHAIN_NAME = collections.OrderedDict([('chain0', 'index'), ('chain1', 'mid'), ('chain2', 'pinky'),
                                      ('chain3', 'ring'), ('chain4', 'thumb')])


class Kinematics:
    """ Kinematics converter GraspIt -> MANO """

    def __init__(self, path=''):
        """Constructor

        Keyword Arguments:
            path {str} -- path to to directory with a kinematics.json (default: {''})
        """
        with open(os.path.join(path, 'kinematics.json'), 'r') as f:
            data = json.load(f)
        self._chains = [Chain(n, data) for n in CHAIN_NAME.values()]
        self._origin = data['origin']

    def getManoPose(self, xyz, quat, dofs):
        """Convert a hand pose from GraspIt to MANO

        Arguments:
            xyz  -- root position, vector x,y,z
            quat -- root orientation, quaternion x,y,z,w
            dofs -- joint angles, 20-length vector

        Returns:
            trans -- MANO hand's translation
            pose  -- MANO hand's pose
        """
        pose = [rvec_from_quat(quat)]
        for chain in self._chains:
            p0 = chain.mano_root_mat
            m0 = chain.solid_root_mat
            m = m0.copy()
            for i in range(4):
                theta = dofs[chain.dof_index[i]] * chain.dof_coeff[i]
                m0i = chain.tau0[i]
                m0 = m0 * m0i
                mi = mat_rotate_z(theta) * m0i
                zi = m * [[0], [0], [1]]
                m = m * mi

                if i == 1:
                    p = m * m0.T * p0
                    rvec = rvec_from_mat(p)
                    pose.append(rvec)
                elif i > 1:
                    axis = np.array(p.T * zi).reshape(3)
                    rvec = axis * theta
                    p = p.dot(mat_from_rvec(rvec))
                    pose.append(rvec)
        pose = np.array(pose).flatten()

        m = mat_from_quat(quat)
        trans = np.array(xyz) - self._origin + np.dot(m, self._origin)

        return trans.tolist(), pose.tolist()


class Chain:

    def __init__(self, name, data):
        self.name = name
        self.solid_root_mat = np.matrix(data['{}_graspit_origin'.format(name)])
        self.mano_root_mat = np.matrix(data['{}_mano_origin'.format(name)])
        self.dof_index = [data['{}_{}_dof_index'.format(name, i)] for i in range(4)]
        self.dof_coeff = [data['{}_{}_dof_coeff'.format(name, i)] for i in range(4)]
        self.tau0 = [np.matrix(data['{}_{}_tau0'.format(name, i)]) for i in range(4)]


if __name__ == '__main__':
    # test kinematics calculation
    k = Kinematics('../models/ManoHand/')
    dofs = np.array([ 0.3490658503988659, 1.5707963267948966, 1.5707963267948966,
                      0.0, 1.2937379824869724, 1.5707963267948966, -0.0015296899255567709,
                      0.5767170748005673, 0.5767170748005673, -0.5235987755982988,
                      0.35671707480056725, 0.35671707480056725, -0.17453292519943295,
                      -0.8726646259971648, 0.5587500000000003, 0.5587500000000003 ])
    pose = np.array([ -0.0382290891836463, 0.051242782153914326, -0.09341721796180356,
                      -0.706356308200317, 0.0039194533509700775, 0.01662453110243378, 0.7076503576743761 ])
    mano_pose_groundtruth = np.array([ -1.5687996855919637, 0.008705007817315995, 0.036922667588346496,
                                        0.4441922704667301, 0.09948998100119684, 1.5238331466989072,
                                        0.3426444655637463, -0.0027020773869509598, 1.5329672433775843,
                                        0.20558667933824776, -0.0016212464321705593, 0.9197803460265508,
                                        0.0, 0.0, 1.2937379824869724,
                                        0.0, 0.0, 1.5707963267948966,
                                        0.0, 0.0, 0.9424777960769379,
                                        -0.1722578186871779, 0.024150281322695307, 0.2691696044339346,
                                        -0.18633526480949508, -0.00375421288114578, 0.3041580944645266,
                                        -0.11180115888569703, -0.0022525277286874856, 0.18249485667871596,
                                        -0.040046038287493785, 0.13205968265590406, 0.5659416592238729,
                                        -0.07901608599131381, -0.0003884841925069772, 0.5712782961054697,
                                        -0.04740965159478827, -0.00023309051550419324, 0.3427669776632819,
                                        1.1154200483134478, 0.03490624497342323, 0.7449772734709914,
                                        -0.03775696957956241, -0.556539754634176, 0.031096427859256585,
                                        0.11962592410171811, -0.4792618385302032, 0.26101647278238155 ])
    mano_trans_groundtruth = np.array([-0.03858142232859295,0.052775923707074214,-0.10874888745363441])

    mano_trans, mano_pose =  k.getManoPose(pose[:3], pose[3:], dofs)
    print ("mano_pose comparison", np.isclose(mano_pose, mano_pose_groundtruth))
    print ("mano_trans comparison", np.isclose(mano_trans, mano_trans_groundtruth))