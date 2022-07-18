import pybullet as p
import numpy as np
import open3d as o3d

from render_utils import *



##########################################################################################

########################
## Sampling a View and obtaining the depth image
## 1. Sample a view around the object of interest: https://github.com/IRVLUTD/few-shot-dataset/blob/d3022b2ff8990c8e5434644f36c5573bdc1ecefb/tools/pybullet_suncg/fewshot_simulator.py#L598
## 2. Get the camera depth image: https://github.com/IRVLUTD/few-shot-dataset/blob/d3022b2ff8990c8e5434644f36c5573bdc1ecefb/tools/pybullet_suncg/fewshot_simulator.py#L740
########################

def get_camera_images(camera_pos, lookat_pos, params, camera_up_vector=None):
    """ Get RGB/Depth/Segmentation images
        params should have:
            - 'img_width'
            - 'img_height'
            - 'far' : far clipping plane distance
            - 'near': near clipping plane distance
            - 'fov' : field of view for camera

    """

    if camera_up_vector is None:
        camera_up_vector = sample_camera_up_vector(camera_pos, lookat_pos)

    # Compute view/projection matrices and get images
    aspect_ratio = params['img_width'] / params['img_height']
    view_matrix = p.computeViewMatrix(camera_pos, lookat_pos, camera_up_vector)
    proj_matrix = p.computeProjectionMatrixFOV(params['fov'], aspect_ratio, params['near'], params['far'])
    intrinsic_matrix = projection_to_intrinsics(proj_matrix, params['img_width'], params['img_height'])
    temp = p.getCameraImage(params['img_width'], 
                            params['img_height'], 
                            viewMatrix=view_matrix,
                            projectionMatrix=proj_matrix,
                            renderer=p.ER_BULLET_HARDWARE_OPENGL) 
    # returns a tuple of: width, height, rgbPixels, depthPixels, segmentation

    # RGB image
    rgb_img = np.reshape(temp[2], (params['img_height'], params['img_width'], 4))[..., :3]

    # Depth image
    depth_buffer = np.array(temp[3]).reshape(params['img_height'],params['img_width'])
    depth_img = params['far'] * params['near'] / \
                (params['far'] - (params['far'] - params['near']) * depth_buffer)
    # Note: this gives positive z values. this equation multiplies the actual negative z values by -1
    #       Negative z values are because OpenGL camera +z axis points away from image

    # Segmentation image
    seg_img = np.array(temp[4]).reshape(params['img_height'],params['img_width'])

    # Set near/far clipped depth values to 0. This is indicated by seg_img == -1
    depth_img[seg_img == -1] = 0.

    # construct meta data
    meta = {} # dict to store the meta data
    meta['intrinsic_matrix'] = intrinsic_matrix
    meta['view_matrix'] = view_matrix
    meta['proj_matrix'] = proj_matrix
    meta['image_width'] = params['img_width']
    meta['image_height'] = params['img_height']
    meta['near'] = params['near']
    meta['far'] = params['far']
    meta['fov'] = params['fov']
    camera_pose = view_to_extrinsics(view_matrix)
    meta['camera_pose'] = camera_pose

    return {'rgb' : rgb_img,
            'depth' : depth_img,
            'depth_buffer': depth_buffer,
            'orig_seg_img' : seg_img,
            'meta' : meta,
            'view_params' : {
                            'camera_pos' : camera_pos.tolist() if type(camera_pos) == np.ndarray else camera_pos,
                            'lookat_pos' : lookat_pos.tolist() if type(lookat_pos) == np.ndarray else lookat_pos,
                            'camera_up_vector' : camera_up_vector.tolist() if type(camera_up_vector) == np.ndarray else camera_up_vector,
                            }
            }


# Assume the XZ alignment i.e the "distance" from object is along Y axis
def sample_camera_pos_lookat(robot_coords, robot_state_pos):

    xz_bbox_side_probs = np.array([robot_coords['xsize'], # x side 1
                                    robot_coords['zsize'], # z side 1
                                    robot_coords['xsize'], # x side 2
                                    robot_coords['zsize']] # z side 2
                                    )

    xz_bbox_side_probs = xz_bbox_side_probs / np.sum(xz_bbox_side_probs)

    side = np.random.choice(range(4), p=xz_bbox_side_probs)
    if side == 0: # x side 1
        p1 = np.array([robot_coords['xmin'], robot_coords['zmin']])
        p2 = np.array([robot_coords['xmax'], robot_coords['zmin']])
        side_length = np.linalg.norm(p2 - p1)
        other_side_length = np.linalg.norm(p2 - np.array([robot_coords['xmax'], robot_coords['zmax']]))
    elif side == 1: # z side 1
        p1 = np.array([robot_coords['xmax'], robot_coords['zmin']])
        p2 = np.array([robot_coords['xmax'], robot_coords['zmax']])
        side_length = np.linalg.norm(p2 - p1)
        other_side_length = np.linalg.norm(p2 - np.array([robot_coords['xmin'], robot_coords['zmax']]))
    elif side == 2: # x side 2
        p1 = np.array([robot_coords['xmax'], robot_coords['zmax']])
        p2 = np.array([robot_coords['xmin'], robot_coords['zmax']])
        side_length = np.linalg.norm(p2 - p1)
        other_side_length = np.linalg.norm(p2 - np.array([robot_coords['xmin'], robot_coords['zmin']]))
    elif side == 3: # z side 2
        p1 = np.array([robot_coords['xmin'], robot_coords['zmax']])
        p2 = np.array([robot_coords['xmin'], robot_coords['zmin']])
        side_length = np.linalg.norm(p2 - p1)
        other_side_length = np.linalg.norm(p2 - np.array([robot_coords['xmax'], robot_coords['zmin']]))

    # Select point on that side uniformly
    point = p1 + (p2 - p1) * np.random.uniform(0.4, 0.6)

    # Sample xz distance from that point
    dist_from_object = np.random.uniform(0.0, 0.15)
    theta = np.radians(-90)
    rot_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                            [np.sin(theta), np.cos(theta)]])
    away_from_object_direction = rot_matrix.dot ( (p2 - p1) / np.linalg.norm(p2 - p1) )
    camera_x, camera_z = point + dist_from_object * away_from_object_direction

    # Sample y distance
    # height_from_object = np.random.uniform(.5*robot_coords['ysize'], 1.0*robot_coords['ysize'])
    height_from_object = np.random.uniform(.5, 1.0) # anywhere from .5m to 1.2m above object
    
    options = ['top', 'bottom'] # top (0) or bottom (1) of the object
    flip = np.random.choice(2) # equally likely
    y_choice = options[flip]
    if y_choice == 'top':
        camera_y = robot_coords['ymax'] + height_from_object
    else:
        camera_y = robot_coords['ymin'] - height_from_object
    
    # Final camera position
    camera_pos = np.array([camera_x, camera_y, camera_z])

    # look at near the object center
    lookat_xmin = (robot_coords['xmin'] + robot_coords['xmax']) * 0.5 - robot_coords['xsize'] * 0.2
    lookat_xmax = (robot_coords['xmin'] + robot_coords['xmax']) * 0.5 + robot_coords['xsize'] * 0.2
    lookat_zmin = (robot_coords['zmin'] + robot_coords['zmax']) * 0.5 - robot_coords['zsize'] * 0.2
    lookat_zmax = (robot_coords['zmin'] + robot_coords['zmax']) * 0.5 + robot_coords['zsize'] * 0.2

    # Sample lookat position
    lookat_pos = np.array(robot_state_pos)
    lookat_pos[0] = np.random.uniform(lookat_xmin, lookat_xmax)
    # if sampling from top-view, then look at top (ymax) else bottom (ymin)
    lookat_pos[1] = robot_coords['ymax'] if y_choice == 'top' else robot_coords['ymin']
    lookat_pos[2] = np.random.uniform(lookat_zmin, lookat_zmax)
    
    return camera_pos, lookat_pos


def sample_view(robot_id, params):
    robot_coords = get_object_bbox_coordinates(robot_id)
    robot_state_pos = get_state(robot_id)[0]
    # Three alignments possible: (1) XZ, (2) YZ, (3) XY
    # sample_camera_pose() already does it for XZ
    # So we modify the input (robot_coords) and return values (campos, lookat) for others
    options = ['xz', 'yz', 'xy']
    flip = np.random.choice(3)
    dirn = options[flip]
    
    if dirn == 'xz':
        # Default, no need to change!
        camera_pos, lookat_pos = sample_camera_pos_lookat(robot_coords, robot_state_pos)
    elif dirn == 'yz':
        swap_axes_values(robot_coords, 'yz')
        camera_pos, lookat_pos = sample_camera_pos_lookat(robot_coords, robot_state_pos)
        # Finally swap the values for campos and lookat
        camera_pos[1], camera_pos[2] = camera_pos[2], camera_pos[1]
        lookat_pos[1], lookat_pos[2] = lookat_pos[2], lookat_pos[1]
    elif dirn == 'xy':
        swap_axes_values(robot_coords, 'xy')
        camera_pos, lookat_pos = sample_camera_pos_lookat(robot_coords, robot_state_pos)
        camera_pos[1], camera_pos[0] = camera_pos[0], camera_pos[1]
        lookat_pos[1], lookat_pos[0] = lookat_pos[0], lookat_pos[1]
    
    return get_camera_images(camera_pos, lookat_pos, params)

##########################################################################################

#############
## Backprojecting the points to 3D
## See: https://github.com/NVlabs/UnseenObjectClustering/blob/41377eca18c5fdf75502bf8dd824905db112bd33/lib/datasets/imdb.py#L47
#############

def backproject_to_wcs(depth_cv, intrinsic_matrix, camera_pose_matrix, factor=1):

    # factor = 1 since we do not need the conversion to png. Earlier in the other code base
    # where this is taken from, they have a multiply by 1000, so they divide it here. But
    # we dont need that, so factor's default value is kept as 1.

    depth = depth_cv.astype(np.float32, copy=True) / factor

    index = np.where(~np.isfinite(depth))
    depth[index[0], index[1]] = 0

    # get intrinsic matrix
    K = intrinsic_matrix
    Kinv = np.linalg.inv(K)

    # compute the 3D points
    width = depth.shape[1]
    height = depth.shape[0]

    # construct the 2D points matrix
    x, y = np.meshgrid(np.arange(width), np.arange(height))
    ones = np.ones((height, width), dtype=np.float32)
    x2d = np.stack((x, y, ones), axis=2).reshape(width*height, 3)

    # backprojection
    R = np.dot(Kinv, x2d.transpose())

    # compute the 3D points
    X = np.multiply(np.tile(depth.reshape(1, width*height), (3, 1)), R)
    ccs_points =  np.array(X).transpose().reshape((height, width, 3))
    ccs_points = ccs_points.reshape(-1, 3)
    # Filter ccs_points who have depth value of zero
    # Crucial to reduce the memory footprint since majority are useless points
    ccs_points = ccs_points[ccs_points[:, -1] != 0]

    ### CCS to WCS: apply inverse of camera_pose_matrix (4,4)
 
    Rt = camera_pose_matrix
    inv_Rt = np.linalg.inv(Rt)
    ## Method-1: Apply the inverse of rotation matrix + translation manually
    # translation = inv_Rt[:-1,-1]
    # rotation = inv_Rt[:-1,:-1]
    # wcs_points = np.dot(ccs_points, rotation.T) + translation
    
    ## Method-2: Using homogenous coordinates
    ## 1. we need to convert ccs_points (N,3) -> (N,4). Set fourth dim to be 1
    ## 2. apply inv(camera_pose_matrix) to the points.
    ## 3. recover from the homog coordinates (N,4) -> (N,3) by dividing across the fourth dim value.
    
    N = ccs_points.shape[0]
    homog_ccs_points = np.ones((N, 4))
    homog_ccs_points[:N,:-1] = ccs_points
    wcs_homog = np.dot(homog_ccs_points, inv_Rt.T)
    ### See: https://stackoverflow.com/a/43925621
    wcs_points = wcs_homog[:, :-1] / wcs_homog[:, [-1]]
    return wcs_points
    

## Alternate function: NOT WORKING
def convert_depth_to_pc(sample_view):
    # https://stackoverflow.com/questions/59128880/getting-world-coordinates-from-opengl-depth-buffer
    # AND: https://github.com/irvingvasquez/nbv_regression/blob/main/range_simulation_tutorial/pybullet_sim_tutorial.ipynb
    imgH = sample_view['meta']['image_height']
    imgW = sample_view['meta']['image_width']
    projectionMatrix = sample_view['meta']['proj_matrix']
    viewMatrix = sample_view['meta']['view_matrix']

    depthImg = sample_view['depth_buffer']

    stepX = 1
    stepY = 1        
    pointCloud = np.empty([np.int32(imgH/stepY), np.int32(imgW/stepX), 4])

    projectionMatrix = np.asarray(projectionMatrix).reshape([4,4],order='F')

    viewMatrix = np.asarray(viewMatrix).reshape([4,4],order='F')

    tran_pix_world = np.linalg.inv(np.matmul(projectionMatrix, viewMatrix))

    for h in range(0, imgH, stepY):
        for w in range(0, imgW, stepX):
                x = (2*w - imgW)/imgW
                y = -(2*h - imgH)/imgH  # be careful！ deepth and its corresponding position
                # duda
                z = 2*depthImg[h,w] - 1
                #z = realDepthImg[h,w]
                pixPos = np.asarray([x, y, z, 1])
                #print(pixPos)
                position = np.matmul(tran_pix_world, pixPos)
                pointCloud[np.int32(h/stepY), np.int32(w/stepX), :] = position / position[3]
                
    return pointCloud


def main():
    p.connect(p.DIRECT)
    obj = p.loadURDF("./003_cracker_box_custom.urdf")
    params = {}
    params['img_width'] = 320
    params['img_height'] = 240
    params['near'] = 0.01
    params['far'] = 10
    params['fov'] = 45

    samp_view = sample_view(obj, params)
    depth_img = samp_view['depth']
    intrinsic = samp_view['meta']['intrinsic_matrix']
    camera_pose = samp_view['meta']['camera_pose']
    wcs_points = backproject_to_wcs(depth_img, intrinsic, camera_pose, 1)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(wcs_points)
    o3d.io.write_point_cloud("./images/new1_test_pc.ply", pcd)

if __name__ == "__main__":
    # main()
    pass