import math
import numpy as np
from sklearn.neighbors import KDTree

from sdf_utils import sample_uniform_points_in_unit_sphere, farthest_point_sampling

# Reference: https://github.com/marian42/mesh_to_sdf/blob/66036a747e82e7129f6afc74c5325d676a322114/mesh_to_sdf/surface_point_cloud.py


class PointCloudData:

    def __init__(self, points, normals) -> None:
        self.points = points
        self.normals = normals
        self.kd_tree = KDTree(points)
        print("number of points in point cloud:", points.shape[0])

    def get_random_surface_points(self, count):
        if count >= self.points.shape[0]:
            indices = np.random.choice(self.points.shape[0], count)
        else:
            print("Doing FPS!")
            _, indices = farthest_point_sampling(self.points, count)
        print("Sampled random surface points")
        return self.points[indices, :]

    def get_sdf(self, query_points, sample_count=11):
        distances, indices = self.kd_tree.query(query_points, k=sample_count)
        distances = distances.astype(np.float32)

        closest_points = self.points[indices]
        direction_from_surface = query_points[:,
                                              np.newaxis, :] - closest_points
        inside = np.einsum(
            'ijk,ijk->ij', direction_from_surface, self.normals[indices]) < 0
        inside = np.sum(inside, axis=1) > sample_count * 0.5
        distances = distances[:, 0]
        distances[inside] *= -1
        return distances

    def get_sdf_in_batches(self, query_points, sample_count=11, batch_size=1000000):
        if query_points.shape[0] <= batch_size:
            return self.get_sdf(query_points, sample_count=sample_count)

        n_batches = int(math.ceil(query_points.shape[0] / batch_size))
        batches = [
            self.get_sdf(points, sample_count=sample_count)
            for points in np.array_split(query_points, n_batches)
        ]
        return np.concatenate(batches)  # distances

    def sample_query_points(self, number_of_points=500000):
        query_points = []
        surface_sample_count = int(number_of_points * 47 / 50) // 2
        surface_points = self.get_random_surface_points(surface_sample_count)
        query_points.append(
            surface_points + np.random.normal(scale=0.0025, size=(surface_sample_count, 3)))
        query_points.append(
            surface_points + np.random.normal(scale=0.00025, size=(surface_sample_count, 3)))

        unit_sphere_sample_count = number_of_points - \
            surface_points.shape[0] * 2
        unit_sphere_points = sample_uniform_points_in_unit_sphere(
            unit_sphere_sample_count)
        query_points.append(unit_sphere_points)
        query_points = np.concatenate(query_points).astype(np.float32)
        return query_points

    def sample_sdf_near_surface(self, number_of_points=500000, normal_sample_count=200, min_size=0):
        query_points = []
        surface_sample_count = int(number_of_points * 47 / 50) // 2
        surface_points = self.get_random_surface_points(surface_sample_count)
        query_points.append(
            surface_points + np.random.normal(scale=0.0025, size=(surface_sample_count, 3)))
        query_points.append(
            surface_points + np.random.normal(scale=0.00025, size=(surface_sample_count, 3)))

        unit_sphere_sample_count = number_of_points - \
            surface_points.shape[0] * 2
        unit_sphere_points = sample_uniform_points_in_unit_sphere(
            unit_sphere_sample_count)
        query_points.append(unit_sphere_points)
        query_points = np.concatenate(query_points).astype(np.float32)

        sdf = self.get_sdf_in_batches(
            query_points, sample_count=normal_sample_count)

        if min_size > 0:
            model_size = np.count_nonzero(
                sdf[-unit_sphere_sample_count:] < 0) / unit_sphere_sample_count
            if model_size < min_size:
                raise ValueError('BadMeshException()')

        return query_points, sdf

    def closest_point_dist(self, query_points):
        distances, _ = self.kd_tree.query(query_points)
        return np.squeeze(distances)