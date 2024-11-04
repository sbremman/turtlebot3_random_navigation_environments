#!/usr/bin/env python3

import rospy
import rospkg
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from turtlebot3_random_navigation_environments.srv import RandomizeWorld, RandomizeWorldResponse, RandomizeWorldRequest
import random
import xml.etree.ElementTree as ET

from xml.dom import minidom
import time
import os
import math
import re

from utils import make_map_from_shapes, GenerateShape, RectangleShapeInfo, CircleShapeInfo, MapLimits
from shapely import Point
from find_bounding_box_from_sdf import get_bounding_box_from_sdf

import gc
import subprocess

# Checks if Gazebo is currently running by attempting to access a service
def is_gazebo_running():
        try:
            rospy.wait_for_service('/gazebo/get_model_state', timeout=5)
            return True
        except (rospy.ROSException, rospy.ServiceException):
            return False

# Restarts Gazebo by killing current instances and launching it again
def restart_gazebo():
    rospy.loginfo("Restarting Gazebo...")
    subprocess.call(["pkill", "gzserver"])
    time.sleep(5) 
    subprocess.call(["pkill", "gzclient"])
    time.sleep(5) 
    subprocess.Popen(["roslaunch", "turtlebot3_random_navigation_environments", "random_world_only_gazebo.launch"])  # Adjust the package name and launch file path
    time.sleep(5)  # Give Gazebo some time to restart
    rospy.loginfo("!!!Gazebo has been restarted...!!!")

# Converts XML element to a dictionary representation
def xml_to_dict(elem):
    def inner_xml_to_dict(e):
        return {
            "tag": e.tag,
            "attrib": e.attrib,
            "text": e.text.strip() if e.text else None,
            "children": [inner_xml_to_dict(c) for c in e]
        }
    return inner_xml_to_dict(elem)

# Pretty prints XML recursively
def pretty_print_xml(elem, level=0):
    indent = "  " * level
    text = indent + f"<{elem.tag}"
    for key, value in elem.attrib.items():
        text += f" {key}=\"{value}\""
    text += ">"
    if elem.text and elem.text.strip():
        text += elem.text.strip()
    else:
        text += "\n"
    for child in elem:
        text += pretty_print_xml(child, level + 1) + "\n"
    text += indent + f"</{elem.tag}>"
    return text

# Converts an XML element to a pretty-printed string for readability
def prettify(element):
    rough_string = ET.tostring(element, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

# Creates an SDF for a wall with given properties
def create_wall_sdf(length, width, height, wall_name, position):
    sdf = ET.Element("sdf", version="1.6")
    model = ET.SubElement(sdf, "model", name=wall_name)
    static = ET.SubElement(model, "static")
    static.text = "true"
    
    link = ET.SubElement(model, "link", name="link")
    
    pose = ET.SubElement(link, "pose")
    pose.text = "{} {} {} 0 0 0".format(position[0], position[1], position[2])
    
    for tag in ["collision", "visual"]:
        elem = ET.SubElement(link, tag, name=tag)
        geometry = ET.SubElement(elem, "geometry")
        size = ET.SubElement(ET.SubElement(geometry, "box"), "size")
        size.text = f"{length} {width} {height}"
    
    material = ET.SubElement(link.find("visual"), "material")
    script = ET.SubElement(material, "script")
    ET.SubElement(script, "uri").text = "file://media/materials/scripts/gazebo.material"
    ET.SubElement(script, "name").text = "Gazebo/Wood"

    return sdf

# Combines multiple SDF elements into a single SDF element
def combine_sdfs(list_of_sdfs):
    sdf = ET.Element("sdf", version="1.6")
    model = ET.SubElement(sdf, "model", name="combinedobject")
    static = ET.SubElement(model, "static")
    static.text = "true"

    for i in range(len(list_of_sdfs)):
        link = find_first_child_with_tag(list_of_sdfs[i], "link")
        if link is None:
            raise ValueError("SDF does not contain link")
        link.set('name', f"link_{i}")
        model.append(link)
    
    return ET.tostring(sdf, encoding='unicode'), sdf

# Lists all folders within a given directory
def list_folders_in_directory(directory_path):
    try:
        entries = os.listdir(directory_path)
        return [entry for entry in entries if os.path.isdir(os.path.join(directory_path, entry))]
    except Exception as e:
        print(f"An error occurred: {e}")
        return []

# Finds the ROS package path by package name
def find_package_path(package_name):
    rospack = rospkg.RosPack()
    try:
        return rospack.get_path(package_name)
    except rospkg.ResourceNotFound:
        return None

# Generates an SDF representation of a shape
def generate_shape_to_sdf(shape):
    if shape.type == 'rectangle':
        return create_rectangle_sdf(shape)
    elif shape.type == 'circle':
        return create_circle_sdf(shape)
    else:
        raise NotImplementedError(f"Shape type '{shape.type}' is not supported")

# Creates an SDF for a rectangle shape
def create_rectangle_sdf(shape):
    pos = shape.info.pos
    width = shape.info.width
    length = shape.info.length
    angle = math.radians(shape.info.angle)  # Convert angle to radians
    obj_name = shape.obj_name

    sdf = ET.Element("sdf", version="1.6")
    model = ET.SubElement(sdf, "model", name=obj_name)
    static = ET.SubElement(model, "static")
    static.text = "true"

    link = ET.SubElement(model, "link", name="link")
    pose = ET.SubElement(link, "pose")
    pose.text = f"{pos.x} {pos.y} 0 0 0 {angle}"

    for tag in ["collision", "visual"]:
        elem = ET.SubElement(link, tag, name=tag)
        geometry = ET.SubElement(elem, "geometry")
        size = ET.SubElement(ET.SubElement(geometry, "box"), "size")
        size.text = f"{length} {width} 0.5"  # Assuming height is 0.5
    
    material = ET.SubElement(link.find("visual"), "material")
    script = ET.SubElement(material, "script")
    ET.SubElement(script, "uri").text = "file://media/materials/scripts/gazebo.material"
    ET.SubElement(script, "name").text = "Gazebo/White"

    return sdf

# Creates an SDF for a circle shape
def create_circle_sdf(shape):
    pos = shape.info.pos
    radius = shape.info.radius
    obj_name = shape.obj_name

    sdf = ET.Element("sdf", version="1.6")
    model = ET.SubElement(sdf, "model", name=obj_name)
    static = ET.SubElement(model, "static")
    static.text = "true"

    link = ET.SubElement(model, "link", name="link")
    pose = ET.SubElement(link, "pose")
    pose.text = f"{pos.x} {pos.y} 0 0 0 0"

    for tag in ["collision", "visual"]:
        elem = ET.SubElement(link, tag, name=tag)
        geometry = ET.SubElement(elem, "geometry")
        cylinder = ET.SubElement(geometry, "cylinder")
        ET.SubElement(cylinder, "radius").text = f"{radius}"
        ET.SubElement(cylinder, "length").text = "0.5"  # Assuming height is 0.5

    material = ET.SubElement(link.find("visual"), "material")
    script = ET.SubElement(material, "script")
    ET.SubElement(script, "uri").text = "file://media/materials/scripts/gazebo.material"
    ET.SubElement(script, "name").text = "Gazebo/White"

    return sdf

# Reads an SDF file from the given path
def read_sdf(file_path):
    try:
        tree = ET.parse(file_path)
        return tree.getroot()
    except ET.ParseError as e:
        print(f"Error parsing SDF file: {e}")
        return None

# Finds a direct child with a specific tag
def find_direct_child_with_tag(root, tag):
    for child in root:
        if child.tag == tag:
            return child
    return None

# Recursively finds the first child with a specific tag
def find_first_child_with_tag(root, tag):
    for child in root:
        if child.tag == tag:
            return child
        result = find_first_child_with_tag(child, tag)
        if result is not None:
            return result
    return None

# Updates the pose of an SDF element with a new position and angle
def update_pose(root, new_position, angle_deg):
    angle_rad = math.radians(angle_deg)
    for link in root.iter('link'):
        pose = find_direct_child_with_tag(link, "pose")
        if pose is None:
            pose = ET.SubElement(link, "pose")
        pose.text = f"{new_position[0]} {new_position[1]} 0 0 0 {angle_rad + math.pi / 2}"

# Removes newline characters from XML elements
def remove_newline_text(element):
    if element.text == '\n  ':
        element.text = None
    for child in element:
        remove_newline_text(child)

# Cleans an XML string by removing extraneous whitespace
def clean_xml_string(xml_string):
    return re.sub(r'>\s+<', '><', xml_string)

# Generates a random goal location within defined distance limits
def sample_goal_location(goal_min_distance, goal_max_distance_x, goal_max_distance_y):
    while True:
        x = random.uniform(-goal_max_distance_x, goal_max_distance_x)
        y = random.uniform(-goal_max_distance_y, goal_max_distance_y)
        if min(abs(x), abs(y)) >= goal_min_distance:
            return [x, y]

# The main class for randomizing the Gazebo world environment
class WorldRandomizer:
    def __init__(self, gazebo_ns='/gazebo', bot_dim=[0.281, 0.306, 0.141]):
        # Initializes the world randomizer with default namespaces and robot dimensions
        self.gazebo_ns = gazebo_ns
        self._spawn_model = rospy.ServiceProxy(self.gazebo_ns + '/spawn_sdf_model', SpawnModel)
        self._delete_model = rospy.ServiceProxy(self.gazebo_ns + '/delete_model', DeleteModel)
        self._obj_names = []
        
        # Set up paths and list available SDF folders
        package_path = rospkg.RosPack().get_path('turtlebot3_random_navigation_environments')
        self.sdf_folder_path = package_path+'/sdf_obstacles/'
        self.sdf_name_list = list_folders_in_directory(self.sdf_folder_path)

        # Generate default TurtleBot and goal objects
        self.turtlebot_object = GenerateShape('rectangle',
                                              RectangleShapeInfo(Point(0.0,0.0),
                                                                 bot_dim[1],
                                                                 bot_dim[0],
                                                                 0.0),
                                              'turtlebot')
        
        self.goal_object = GenerateShape('rectangle',
                                        RectangleShapeInfo(Point(1.0,1.0),
                                                            bot_dim[1]*2,
                                                            bot_dim[1]*2,
                                                            0.0),
                                        'goal')
        
        # Load parameters from ROS
        self.chance_for_circle = rospy.get_param('chance_for_circle')
        self.chance_for_sdf = rospy.get_param('chance_for_sdf')
        self.obstacle_clearance = rospy.get_param('obstacle_clearance')
        self.goal_min_distance = rospy.get_param('goal_min_distance')

        # Load goal SDF from model path
        self.goalPath = find_package_path('turtlebot3_gazebo') + '/models/turtlebot3_square/goal_box/model.sdf'
        self.f = open(self.goalPath, 'r')
        self.goal_sdf = read_sdf(self.goalPath)

        rospy.Service('world_randomizer', RandomizeWorld, self.handle_world_randomizer)
        self.num_spawned = 0

    # Creates SDFs for enclosing walls based on environment size
    def create_sdfs_for_walls(self, env_length, env_width, wall_thickness=0.1):
        wall_height = 0.5
        walls = [
            (env_length, wall_thickness, wall_height, [0.0, env_width / 2.0, wall_height / 2]),  # Wall 1
            (env_length, wall_thickness, wall_height, [0.0, -env_width / 2.0, wall_height / 2]), # Wall 2
            (wall_thickness, env_width, wall_height, [env_length / 2.0, 0.0, wall_height / 2]),   # Wall 3
            (wall_thickness, env_width, wall_height, [-env_length / 2.0, 0.0, wall_height / 2])  # Wall 4
        ]

        wall_sdf_list = []
        for i, (length, width, height, position) in enumerate(walls):
            wall_name = 'wall_' + str(i)
            wall_sdf = create_wall_sdf(length, width, height, wall_name, position)
            wall_sdf_list.append(wall_sdf)

        return wall_sdf_list, walls

    # Spawns an object in the Gazebo environment
    def spawn_object(self, obj_name, obj_sdf, pos):
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            try:
                pose = Pose()
                pose.position.x = pos[0]
                pose.position.y = pos[1]
                pose.position.z = pos[2]
                pose.orientation.w = 1.0
                self._spawn_model(obj_name, obj_sdf, "", pose, "world")
                self._obj_names.append(obj_name)
            except rospy.ServiceException as e:
                rospy.logerr("Spawn SDF model service call failed: {0}".format(e))
                if not is_gazebo_running():
                    restart_gazebo()
                    # Retry spawning the object after restarting Gazebo
                    try:
                        self._spawn_model(obj_name, obj_sdf, "", pose, "world")
                        self._obj_names.append(obj_name)
                    except rospy.ServiceException as e:
                        rospy.logerr("Retrying spawn SDF model service call failed: {0}".format(e))

        # Deletes an object from the Gazebo environment
    def delete_object(self, object_name):
            rospy.wait_for_service('/gazebo/delete_model')
            try:
                self._delete_model(object_name)
            except rospy.ServiceException as e:
                rospy.logerr("Delete model service call failed: {0}".format(e))
                if not is_gazebo_running():
                    restart_gazebo()

        # Frees up memory
    def cleanup(self):
            gc.collect()

        # Handles world randomization request from ROS service
    def handle_world_randomizer(self, req):
            world_randomizer.cleanup()

            for obj_name in self._obj_names:
                self.delete_object(obj_name)
            self._obj_names.clear()
            rospy.logdebug("Objects have been deleted")
            start_time = time.time()

            wall_thickness = 0.1

            # Sample new goal location
            goal_location = sample_goal_location(self.goal_min_distance, 
                                                req.env_size_x / 2 - self.goal_object.info.width / 2 - wall_thickness,
                                                req.env_size_y / 2 - self.goal_object.info.length / 2 - wall_thickness)
            
            self.goal_object.info.pos = Point(goal_location[0], goal_location[1])
            shape_list = [self.turtlebot_object, self.goal_object]
            wall_sdf_list = []

            # Generate walls based on a random chance
            if random.random() < req.wall_spawn_chance:
                wall_sdf_list, walls = self.create_sdfs_for_walls(req.env_size_x, req.env_size_y, wall_thickness=wall_thickness)
                for i in range(len(walls)):
                    wall = walls[i]
                    length = wall[1] + [-wall_thickness/10, -wall_thickness/10, 0.0, 0.0][i]
                    width = wall[0] + [0.0, 0.0, -wall_thickness/10, -wall_thickness/10][i]
                    pos = Point(wall[3][0] + [-wall_thickness/2, wall_thickness/2, 0.0, 0.0][i], 
                                wall[3][1] + [0.0, 0.0, wall_thickness/2, -wall_thickness/2][i])
                    shape_info = RectangleShapeInfo(pos, width, length, 0.0)
                    wall_name = 'wall_' + str(i)
                    shape_object = GenerateShape('rectangle', shape_info, wall_name)
                    shape_list.append(shape_object)

            # Iterate through requested number of obstacles and generate them
            for i in range(req.num_obstacles):
                shape_chance = random.random()

                if shape_chance <= self.chance_for_circle:
                    radius = random.uniform(req.obstacle_min_size, req.obstacle_max_size) / 2
                    shape_object = GenerateShape('circle', CircleShapeInfo(None, radius), 'circle')
                    shape_list.append(shape_object)

                elif shape_chance <= self.chance_for_circle + self.chance_for_sdf:
                    sdf_name = random.choice(self.sdf_name_list)
                    sdf_model_path = self.sdf_folder_path + sdf_name + '/model.sdf'
                    bounding_box = get_bounding_box_from_sdf(sdf_model_path, [self.sdf_folder_path])

                    if bounding_box is None:
                        raise ValueError(f"Bounding box cannot be calculated for {sdf_name}")

                    width = bounding_box[1][0] - bounding_box[0][0]
                    length = bounding_box[1][1] - bounding_box[0][1]
                    shape_info = RectangleShapeInfo(None, width, length, 0.0)
                    shape_object = GenerateShape('rectangle', shape_info, sdf_name)
                    shape_list.append(shape_object)

                else:
                    width = random.uniform(req.obstacle_min_size, req.obstacle_max_size)
                    length = random.uniform(req.obstacle_min_size, req.obstacle_max_size)
                    shape_info = RectangleShapeInfo(None, width, length, 0.0)
                    shape_object = GenerateShape('rectangle', shape_info, 'rectangle')
                    shape_list.append(shape_object)

            # Determine positions and orientations for all shapes in the list
            shapes = make_map_from_shapes(shape_list, 
                                        MapLimits(req.env_size_x / 2.0, 
                                                    -req.env_size_x / 2.0,
                                                    req.env_size_y / 2.0,
                                                    -req.env_size_y / 2.0),
                                        self.obstacle_clearance)
            
            sdf_obstacle_list = []
            for shape in shapes:
                if shape.obj_name in self.sdf_name_list:
                    sdf_model_path = self.sdf_folder_path + shape.obj_name + '/model.sdf'
                    sdf = read_sdf(sdf_model_path)
                    remove_newline_text(sdf)
                    update_pose(sdf, [shape.info.pos.x, shape.info.pos.y], shape.info.angle)
                    sdf_obstacle_list.append(sdf)
                elif shape.obj_name in ['rectangle', 'circle', 'goal']:
                    sdf = generate_shape_to_sdf(shape)
                    sdf_obstacle_list.append(sdf)

            combined_sdf_list, sdf_tree = combine_sdfs(wall_sdf_list + sdf_obstacle_list)
            combined_sdf_list = clean_xml_string(combined_sdf_list)
            name = "walls" + str(self.num_spawned)
            self.num_spawned += 1
            self.spawn_object(name, combined_sdf_list, [0.0, 0.0, 0.0])

            return RandomizeWorldResponse(success=True, 
                                        message="World randomized successfully",
                                        goal_x=goal_location[0],
                                        goal_y=goal_location[1])

if __name__ == "__main__":
    rospy.init_node('world_randomizer')
    world_randomizer = WorldRandomizer()
    rospy.spin()

    test_req = RandomizeWorldRequest()
    test_req.env_size_x = 30.0
    test_req.env_size_y = 30.0
    test_req.num_obstacles = 30
    test_req.obstacle_max_size = 0.301
    test_req.obstacle_min_size = 0.3
    test_req.wall_spawn_chance = 1.0

    response = world_randomizer.handle_world_randomizer(test_req)
    print(response)
    rospy.sleep(4)