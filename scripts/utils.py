from typing import NamedTuple
from shapely.geometry import Point, Polygon, MultiPolygon
from shapely.affinity import rotate, translate
import random
import math
import rospy
import time
from shapely.ops import unary_union
import geopandas as gpd
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import copy


class RectangleShapeInfo:
    def __init__(self, position, width, length, angle):
        self.pos = position  # should be shapely Point type
        self.width = width
        self.length = length
        self.angle = angle

    def __str__(self):
        members = ', '.join(f"{key}={value}" for key, value in self.__dict__.items())
        return f"{self.__class__.__name__}({members})"

    def __repr__(self):
        return self.__str__()


class CircleShapeInfo:
    def __init__(self, position, radius):
        self.pos = position  # should be shapely Point type
        self.radius = radius

    def __str__(self):
        members = ', '.join(f"{key}={value}" for key, value in self.__dict__.items())
        return f"{self.__class__.__name__}({members})"

    def __repr__(self):
        return self.__str__()


class MapLimits(NamedTuple):
    x_max: float
    x_min: float
    y_max: float
    y_min: float


class GenerateShape:
    def __init__(self, shape_type, shape_info, object_name):
        self.type = shape_type
        self.info = shape_info
        self.obj_name = object_name
        self.shapely_object = None

    def __str__(self):
        members = ', '.join(f"{key}={value}" for key, value in self.__dict__.items())
        return f"{self.__class__.__name__}({members})"

    def __repr__(self):
        return self.__str__()


def return_shapely_object(shape, clearance=0.0):
    if shape.type == 'circle':
        center = shape.info.pos
        radius = shape.info.radius + clearance
        return center.buffer(radius)

    elif shape.type == 'rectangle':
        center = shape.info.pos
        width = shape.info.width + 2 * clearance
        length = shape.info.length + 2 * clearance
        coords = [
            (center.x - width / 2, center.y - length / 2),
            (center.x - width / 2, center.y + length / 2),
            (center.x + width / 2, center.y + length / 2),
            (center.x + width / 2, center.y - length / 2),
            (center.x - width / 2, center.y - length / 2)
        ]
        rectangle_polygon = Polygon(coords)
        return rotate(rectangle_polygon, shape.info.angle, origin='centroid')

    else:
        raise ValueError(f"Unknown shape type: {shape.type}, shape name: {shape.obj_name}")


def check_overlap_all_shapes(shapes):
    for i in range(len(shapes)):
        for j in range(i + 1, len(shapes)):
            if shapes[i].intersects(shapes[j]):
                return True
    return False


def check_overlap_new_shape(rectangle, polygons):
    return any(rectangle.intersects(polygon) for polygon in polygons)


def remove_largest_obstacle(shapes_list):
    largest_index = max(range(len(shapes_list)), key=lambda i: (
        shapes_list[i].info.width * shapes_list[i].info.length if shapes_list[i].type == 'rectangle'
        else math.pi * shapes_list[i].info.radius ** 2
    ))
    del shapes_list[largest_index]
    return shapes_list


def random_points_in_bounds(polygon, number):
    minx, miny, maxx, maxy = polygon.bounds
    x = np.random.uniform(minx, maxx, number)
    y = np.random.uniform(miny, maxy, number)
    return x, y


def sample_points_in_polygon(polygon, num_points):
    gdf_poly = gpd.GeoDataFrame(index=["myPoly"], geometry=[polygon])
    x, y = random_points_in_bounds(polygon, num_points)
    df = pd.DataFrame({'points': list(zip(x, y))})
    df['points'] = df['points'].apply(Point)
    gdf_points = gpd.GeoDataFrame(df, geometry='points')
    Sjoin = gpd.tools.sjoin(gdf_points, gdf_poly, predicate="within", how='left')
    pnts_in_poly = gdf_points[Sjoin.index_right == 'myPoly']
    return pnts_in_poly, gdf_poly


def move_polygon_to_new_center(polygon, new_center):
    current_center = polygon.centroid
    dx = new_center.x - current_center.x
    dy = new_center.y - current_center.y
    return translate(polygon, xoff=dx, yoff=dy)


def plot_polygons(polygons, bounds=None):
    fig, ax = plt.subplots()
    for polygon in polygons:
        x, y = polygon.exterior.xy
        ax.plot(x, y)
    ax.set_aspect('equal', adjustable='box')
    if bounds:
        plt.xlim(bounds.x_min, bounds.x_max)
        plt.ylim(bounds.y_min, bounds.y_max)
    plt.show()


def make_map_from_shapes(shape_list, map_limits, clearance=0.0):
    rospy.logdebug('Start make_map_from_shapes')

    shapes_assigned = []
    shapes_not_assigned = []
    num_points_to_sample = 10000
    buffer_distance = 0.8

    for curr_shape in shape_list:
        if curr_shape.info.pos is None:
            curr_shape.info.pos = Point(0.0, 0.0)
            curr_shape.info.angle = 0.0
            curr_shape.shapely_object = return_shapely_object(curr_shape, clearance)
            shapes_not_assigned.append(curr_shape)
        else:
            curr_shape.shapely_object = return_shapely_object(curr_shape)
            shapes_assigned.append(curr_shape)

    shapely_assigned = [shape.shapely_object for shape in shapes_assigned]
    assert not check_overlap_all_shapes(shapely_assigned), "there should be no overlap between pre-assigned shapes"

    shapely_assigned_multipolygon = MultiPolygon(shapely_assigned)
    thickened_polygons = [polygon.buffer(buffer_distance) for polygon in shapely_assigned_multipolygon.geoms]
    shapely_assigned_multipolygon = MultiPolygon(thickened_polygons)

    map_limit_bb = Polygon([
        (map_limits.x_min, map_limits.y_min),
        (map_limits.x_min, map_limits.y_max),
        (map_limits.x_max, map_limits.y_max),
        (map_limits.x_max, map_limits.y_min)
    ])

    shapely_assigned_inv_polygon = map_limit_bb.difference(unary_union(shapely_assigned_multipolygon))
    points_outside_assigned, gdf_polygon = sample_points_in_polygon(shapely_assigned_inv_polygon, num_points_to_sample)

    overlap = True
    counter = -1
    limit_for_map_creation = 2

    while overlap:
        counter += 1
        if counter > limit_for_map_creation:
            rospy.logwarn(f'Could not create map, deleting first obstacle. Num obstacles: {len(shapes_not_assigned)}')
            del shapes_not_assigned[0]
            counter = -1

        shapely_not_assigned = []
        free_points = copy.deepcopy(points_outside_assigned)
        angle_range = 180.0

        for shape in shapes_not_assigned:
            if len(free_points) == 0:
                break
            curr_shape = shape.shapely_object

            if shape.type == 'rectangle':
                angle = random.uniform(0, angle_range)
                curr_shape = rotate(curr_shape, angle)
                shape.info.angle = angle - 90.0

            sampled_point = free_points.sample(1).geometry.iloc[0].coords[0]
            new_center = Point(sampled_point[0], sampled_point[1])
            curr_shape = move_polygon_to_new_center(curr_shape, new_center)
            curr_shape_thickened = curr_shape.buffer(buffer_distance)
            free_points = free_points[~free_points.within(curr_shape_thickened)]

            shape.info.pos = new_center
            shapely_not_assigned.append(curr_shape)

        current_overlap = any(
            check_overlap_new_shape(curr_shape, shapely_assigned + shapely_not_assigned[i + 1:])
            for i, curr_shape in enumerate(shapely_not_assigned)
        )

        if not current_overlap:
            overlap = False
            if len(shapes_not_assigned) != len(shapely_not_assigned):
                overlap = True
                rospy.logerr(f'Mismatch in lengths: shapes_not_assigned has {len(shapes_not_assigned)}, shapely_not_assigned has {len(shapely_not_assigned)}')
            else:
                for i in range(len(shapes_not_assigned)):
                    shapes_not_assigned[i].shapely_object = shapely_not_assigned[i]

    shapes_assigned.extend(shapes_not_assigned)
    shapely_assigned = [shape.shapely_object for shape in shapes_assigned]
    #plot_polygons(shapely_assigned, bounds=map_limits)

    return shapes_assigned


if __name__ == "__main__":
    map_limits = MapLimits(2.0, -2.0, 2.0, -2.0)

    test_shapes = [
        GenerateShape('rectangle', RectangleShapeInfo(Point(0.0, 0.0), 0.4, 0.6, 0.0), 'rectangle_0'),
        GenerateShape('rectangle', RectangleShapeInfo(Point(1.0, 0.0), 0.2, 0.5, 90.0), 'rectangle_1'),
        GenerateShape('rectangle', RectangleShapeInfo(Point(1.5, 1.0), 0.3, 0.2, 45.0), 'rectangle_2')
    ]

    widths = [0.4, 0.2, 0.3, 0.4, 0.2, 0.3]
    lengths = [0.6, 0.5, 0.2, 0.6, 0.5, 0.2]
    for i, (w, l) in enumerate(zip(widths, lengths)):
        test_shapes.append(GenerateShape('rectangle', RectangleShapeInfo(None, w, l, None), f'rectangle_{i+3}'))

    start_time = time.time()
    make_map_from_shapes(test_shapes, map_limits)
    print("total time:", time.time() - start_time)
