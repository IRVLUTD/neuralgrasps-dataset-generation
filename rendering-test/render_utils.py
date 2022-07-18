import pybullet as p
import numpy as np


##########################################################################################
## UTILITY FUNCTIONS
################################
## See https://github.com/IRVLUTD/few-shot-dataset/blob/af5107fb231c6d02b4c847a7182c0b58aac974e5/tools/simulation_util.py
#################################
def rotX(rotz):
    RotZ = np.array([[1, 0, 0, 0], 
                     [0, np.cos(rotz), -np.sin(rotz), 0], 
                     [0, np.sin(rotz), np.cos(rotz), 0], 
                     [0, 0, 0, 1]])
    return RotZ


def view_to_extrinsics(mat):
    pose = np.array(mat).reshape([4, 4]).T
    return rotX(np.pi).dot(pose)


def projection_to_intrinsics(mat, width=224, height=224):
    intrinsic_matrix = np.eye(3)
    mat = np.array(mat).reshape([4, 4]).T
    fv = width / 2 * mat[0, 0]
    fu = height / 2 * mat[1, 1]
    u0 = width / 2
    v0 = height / 2

    intrinsic_matrix[0, 0] = fu
    intrinsic_matrix[1, 1] = fv
    intrinsic_matrix[0, 2] = u0
    intrinsic_matrix[1, 2] = v0
    return intrinsic_matrix



######################
## Ref: https://github.com/IRVLUTD/few-shot-dataset/blob/af5107fb231c6d02b4c847a7182c0b58aac974e5/tools/pybullet_suncg/simulator.py#L830
#####################
def sample_camera_up_vector(camera_pos, lookat_pos, max_camera_rotation=np.pi/15):

    # To do this, I first generate the camera view with [0,1,0] as the up-vector, then sample a rotation from the camera x-y axis and apply it
    # truncated normal sampling. clip it to 2*sigma range, meaning ~5% of samples are maximally rotated
     # Default value
    theta = np.random.normal(0, max_camera_rotation / 2, size=[1])
    theta = theta.clip(-max_camera_rotation, max_camera_rotation)[0]
    my_rot_mat = np.array([[np.cos(theta), -np.sin(theta), 0],
                            [np.sin(theta), np.cos(theta), 0],
                            [0, 0, 1]])
    y_axis = np.array([0,1,0])
    camera_up_vector = my_rot_mat.dot(y_axis) # this is in camera coordinate frame. pybullet needs this in world coordinate frame
    camera_rotation_matrix = np.asarray(p.computeViewMatrix(camera_pos, lookat_pos, y_axis)).reshape(4,4, order='F')[:3,:3].T # Note that transpose is the inverse since it's orthogonal. this is camera rotation matrix
    camera_up_vector = camera_rotation_matrix.dot(camera_up_vector) # camera up vector in world coordinate frame

    return camera_up_vector




## See: https://github.com/IRVLUTD/few-shot-dataset/blob/af5107fb231c6d02b4c847a7182c0b58aac974e5/tools/pybullet_suncg/simulator.py#L472
def get_state(obj_id):    
    pos, q = p.getBasePositionAndOrientation(bodyUniqueId=obj_id)
    # rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    rotation = np.asarray(q)
    return pos, rotation


############################
## See https://github.com/IRVLUTD/few-shot-dataset/blob/af5107fb231c6d02b4c847a7182c0b58aac974e5/tools/pybullet_suncg/simulator.py#L574
##############################
def get_object_bbox_coordinates(obj_id, linkIndex=-1):
    """ Return min/max coordinates of an encapsulating bounding box in x,y,z dims

        @param obj_id: ID of object
        @return: an np.array: [ [xmin, ymin, zmin],
                                [xmax, ymax, zmax] ]
    """    
    obj_min, obj_max = p.getAABB(obj_id, linkIndex=linkIndex)

    return {'xmin' : obj_min[0],
            'ymin' : obj_min[1],
            'zmin' : obj_min[2],
            'xmax' : obj_max[0],
            'ymax' : obj_max[1],
            'zmax' : obj_max[2],
            'xsize' : obj_max[0] - obj_min[0],
            'ysize' : obj_max[1] - obj_min[1],
            'zsize' : obj_max[2] - obj_min[2]
           }

def swap_axes_values(coords, swap_type='yz'):
    if swap_type == 'yz':
        coords['ymin'], coords['zmin'] = coords['zmin'], coords['ymin']
        coords['ymax'], coords['zmax'] = coords['zmax'], coords['ymax']
        coords['ysize'], coords['zsize'] = coords['zsize'], coords['ysize']
    elif swap_type == 'xy':
        coords['ymin'], coords['xmin'] = coords['xmin'], coords['ymin']
        coords['ymax'], coords['xmax'] = coords['xmax'], coords['ymax']
        coords['ysize'], coords['xsize'] = coords['xsize'], coords['ysize']
    elif swap_type == 'xz':
        coords['xmin'], coords['zmin'] = coords['zmin'], coords['xmin']
        coords['xmax'], coords['zmax'] = coords['zmax'], coords['xmax']
        coords['xsize'], coords['zsize'] = coords['zsize'], coords['xsize']
