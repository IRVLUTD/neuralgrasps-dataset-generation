import numpy as np

# Reference: https://github.com/marian42/mesh_to_sdf/blob/master/mesh_to_sdf/utils.py

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


def get_bbox(points):
    xmin, ymin, zmin = np.min(points, axis=0)
    xmax, ymax, zmax = np.max(points, axis=0)
    return (xmin, xmax, ymin, ymax, zmin, zmax)

# Buffer is to ensure that we also have enough space in the unit sphere for the gripper.
# Since we are normalizing only using the object points, it is possible that a "tight"
# scaling will leave no room inside the sphere for the gripper points.
def get_norm_params_sphere(points, buffer=2.0):
    xmin, xmax, ymin, ymax, zmin, zmax = get_bbox(points)
    # bbox_centroid = np.array(
    #     [(xmax-xmin)/2.0, (ymax-ymin)/2.0, (zmax-zmin)/2.0])
    bbox_centroid = np.mean(points, axis=0)
    scale_down = np.max(np.linalg.norm(points - bbox_centroid, axis=1))
    return bbox_centroid, scale_down * buffer


def get_norm_params_cube(points, buffer=2.0):
    xmin, xmax, ymin, ymax, zmin, zmax = get_bbox(points)
    # bbox_centroid = np.array(
    #     [(xmax-xmin)/2.0, (ymax-ymin)/2.0, (zmax-zmin)/2.0])
    bbox_centroid = np.mean(points, axis=0)
    scale_down = np.max(bbox_centroid)
    # Note that for cube, the scale down is just the max along each dimension's range
    return bbox_centroid, scale_down * buffer


def scale_to_unit(points, method='sphere'):
    if method == 'sphere':
        offset, scale = get_norm_params_sphere(points)
    elif method == 'cube':
        offset, scale = get_norm_params_cube(points)
    # points = points - offset
    # points /= scale
    return scale_using_params(points, scale, offset)


def scale_using_params(points, scale, offset):
    return (points - offset) / scale


def sample_uniform_points_in_unit_sphere(amount):
    unit_sphere_points = np.random.uniform(-1, 1, size=(amount * 2 + 20, 3))
    unit_sphere_points = unit_sphere_points[np.linalg.norm(
        unit_sphere_points, axis=1) < 1]

    points_available = unit_sphere_points.shape[0]
    if points_available < amount:
        # This is a fallback for the rare case that too few points are inside the unit sphere
        result = np.zeros((amount, 3))
        result[:points_available, :] = unit_sphere_points
        result[points_available:, :] = sample_uniform_points_in_unit_sphere(
            amount - points_available)
        return result
    else:
        return unit_sphere_points[:amount, :]


def get_random_surface_points(surface_points, count):
    indices = np.random.choice(surface_points.shape[0], count)
    return surface_points[indices, :]


def sample_query_points(surface_points, number_of_points):
    query_points = []
    surface_sample_count = int(number_of_points * 47 / 50) // 2
    surface_points = get_random_surface_points(
        surface_points, surface_sample_count)
    query_points.append(
        surface_points + np.random.normal(scale=0.0025, size=(surface_sample_count, 3)))
    query_points.append(
        surface_points + np.random.normal(scale=0.00025, size=(surface_sample_count, 3)))

    unit_sphere_sample_count = number_of_points - surface_points.shape[0] * 2
    unit_sphere_points = sample_uniform_points_in_unit_sphere(
        unit_sphere_sample_count)
    query_points.append(unit_sphere_points)
    query_points = np.concatenate(query_points).astype(np.float32)
    return query_points
