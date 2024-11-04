#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel
from random_navigation_environments.srv import RandomizeWorld, RandomizeWorldResponse, RandomizeWorldRequest
import random
import xml.etree.ElementTree as ET
from xml.dom import minidom


def prettify(element):
    # Converts an XML element into a pretty-printed string for readability
    rough_string = ET.tostring(element, 'utf-8')
    return minidom.parseString(rough_string).toprettyxml(indent="  ")


def create_wall_sdf(length, width, height, wall_name):
    # Create the SDF XML for a wall model with given dimensions and name
    sdf = ET.Element("sdf", version="1.6")
    model = ET.SubElement(sdf, "model", name=wall_name)
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="link")
    ET.SubElement(link, "pose").text = "0 0 0 0 0 0"

    for tag in ["collision", "visual"]:
        elem = ET.SubElement(link, tag, name=tag)
        geometry = ET.SubElement(elem, "geometry")
        size = ET.SubElement(ET.SubElement(geometry, "box"), "size")
        size.text = f"{length} {width} {height}"
    
    material = ET.SubElement(link.find("visual"), "material")
    script = ET.SubElement(material, "script")
    ET.SubElement(script, "uri").text = "file://media/materials/scripts/gazebo.material"
    ET.SubElement(script, "name").text = "Gazebo/Wood"

    return ET.tostring(sdf, encoding='unicode')


class WorldRandomizer:
    def __init__(self, gazebo_ns='/gazebo'):
        # Initialize the WorldRandomizer with Gazebo namespace and service proxies
        self.gazebo_ns = gazebo_ns
        self._spawn_model = rospy.ServiceProxy(f"{gazebo_ns}/spawn_sdf_model", SpawnModel)
        self._delete_model = rospy.ServiceProxy(f"{gazebo_ns}/delete_model", DeleteModel)
        self._obj_names = []

    def spawn_walls(self, env_length, env_width):
        # Spawns four walls to enclose the environment
        wall_thickness, wall_height = 0.1, 0.5
        walls = [
            (env_length, wall_thickness, wall_height, [0.0, env_width / 2.0, wall_height / 2]),
            (env_length, wall_thickness, wall_height, [0.0, -env_width / 2.0, wall_height / 2]),
            (wall_thickness, env_width, wall_height, [env_length / 2.0, 0.0, wall_height / 2]),
            (wall_thickness, env_width, wall_height, [-env_length / 2.0, 0.0, wall_height / 2])
        ]
        for i, (length, width, height, position) in enumerate(walls):
            wall_name = f'wall_{i}'
            self.spawn_object(wall_name, create_wall_sdf(length, width, height, wall_name), position)

    def spawn_object(self, obj_name, obj_sdf, pos):
        # Spawns an object in Gazebo using the spawn SDF service
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            pose = Pose(position=Pose().position)
            pose.position.x, pose.position.y, pose.position.z = pos
            pose.orientation.w = 1.0
            self._spawn_model(obj_name, obj_sdf, "", pose, "world")
            self._obj_names.append(obj_name)
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn SDF model service call failed: {e}")

    def delete_object(self, object_name):
        # Deletes an object in Gazebo using the delete model service
        rospy.wait_for_service('/gazebo/delete_model')
        try:
            self._delete_model(object_name)
        except rospy.ServiceException as e:
            rospy.logerr(f"Delete model service call failed: {e}")

    def handle_world_randomizer(self, req):
        # Handles the randomization request by deleting existing objects and spawning new walls
        for obj_name in self._obj_names:
            self.delete_object(obj_name)
        self._obj_names.clear()

        if random.random() < req.wall_spawn_chance:
            self.spawn_walls(req.env_size_x, req.env_size_y)

        return RandomizeWorldResponse(success=True, message="World randomized successfully")


if __name__ == "__main__":
    rospy.init_node('world_randomizer')
    world_randomizer = WorldRandomizer()
    rospy.wait_for_service('/gazebo/set_physics_properties')
    rospy.wait_for_service('/gazebo/get_physics_properties')

    # Test request to randomize the environment
    test_req = RandomizeWorldRequest(env_size_x=5.0, env_size_y=2.5, num_obstacles=5, obstacle_max_size=1.5, obstacle_min_size=0.1, wall_spawn_chance=1.0, include_sdfs=False)

    for i in range(1, 9):
        rospy.loginfo(f"Loop: {i}")
        test_req.env_size_x, test_req.env_size_y = i, 10 - i
        world_randomizer.handle_world_randomizer(test_req)
        rospy.sleep(4)
