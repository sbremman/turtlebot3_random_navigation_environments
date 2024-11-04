import xml.etree.ElementTree as ET
import trimesh
import os

def get_bounding_box_from_sdf(sdf_path, model_paths=None):
    if model_paths is None:
        model_paths = []
    
    def parse_sdf(file_path):
        tree = ET.parse(file_path)
        root = tree.getroot()
        return root

    def get_geometries(root):
        geometries = []
        for link in root.findall('.//link'):
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry is not None:
                    geometries.append(geometry)
        return geometries

    def extract_box_dimensions(geometry):
        box = geometry.find('box')
        if box is not None:
            size = box.find('size').text.split()
            return list(map(float, size))
        return None

    def extract_sphere_dimensions(geometry):
        sphere = geometry.find('sphere')
        if sphere is not None:
            radius = float(sphere.find('radius').text)
            return [radius * 2] * 3  # Diameter in all dimensions
        return None

    def extract_cylinder_dimensions(geometry):
        cylinder = geometry.find('cylinder')
        if cylinder is not None:
            radius = float(cylinder.find('radius').text)
            length = float(cylinder.find('length').text)
            return [radius * 2, radius * 2, length]  # Diameter and length
        return None

    def calculate_bounding_box(geometry):
        dimensions = (extract_box_dimensions(geometry) or
                      extract_sphere_dimensions(geometry) or
                      extract_cylinder_dimensions(geometry) or
                      None)
        if dimensions:
            min_coords = [-dim / 2 for dim in dimensions]
            max_coords = [dim / 2 for dim in dimensions]
            return min_coords, max_coords
        return None

    def extract_mesh_file(geometry):
        mesh = geometry.find('mesh')
        if mesh is not None:
            uri = mesh.find('uri').text
            return uri
        return None

    def resolve_mesh_path(sdf_dir, uri):
        if uri.startswith("model://"):
            model_name = uri[len("model://"):].split('/')[0]
            relative_path = '/'.join(uri[len("model://"):].split('/')[1:])
            for model_path in model_paths:
                candidate_path = os.path.join(model_path, model_name, relative_path)
                if os.path.exists(candidate_path):
                    return candidate_path
        else:
            resolved_path = os.path.join(sdf_dir, uri)
            if os.path.exists(resolved_path):
                return resolved_path
        return None

    def calculate_mesh_bounding_box(mesh_path):
        try:
            mesh = trimesh.load(mesh_path)
            bounding_box = mesh.bounds
            min_coords = bounding_box[0].tolist()
            max_coords = bounding_box[1].tolist()
            return min_coords, max_coords
        except Exception as e:
            print(f"Error loading mesh: {e}")
            return None

    def combine_bounding_boxes(boxes):
        min_coords = [min(box[0][i] for box in boxes) for i in range(3)]
        max_coords = [max(box[1][i] for box in boxes) for i in range(3)]
        return min_coords, max_coords

    sdf_dir = os.path.dirname(sdf_path)
    sdf_root = parse_sdf(sdf_path)
    geometries = get_geometries(sdf_root)

    basic_bounding_boxes = [calculate_bounding_box(geom) for geom in geometries if calculate_bounding_box(geom) is not None]

    mesh_uris = [extract_mesh_file(geom) for geom in geometries if extract_mesh_file(geom) is not None]
    mesh_files = [resolve_mesh_path(sdf_dir, uri) for uri in mesh_uris if uri is not None]
    mesh_bounding_boxes = [calculate_mesh_bounding_box(mesh_file) for mesh_file in mesh_files if mesh_file is not None]

    """print(f"Basic bounding boxes: {basic_bounding_boxes}")
    print(f"Mesh URIs: {mesh_uris}")
    print(f"Resolved mesh files: {mesh_files}")
    print(f"Mesh bounding boxes: {mesh_bounding_boxes}")"""

    all_bounding_boxes = basic_bounding_boxes + mesh_bounding_boxes
    if not all_bounding_boxes:
        return None

    final_bounding_box = combine_bounding_boxes(all_bounding_boxes)

    return final_bounding_box

if __name__ == "__main__":

    # Usage example
    model_paths = [
        '/usr/share/gazebo-11/models',  # Add common Gazebo model paths or your custom paths here
        '/home/user/.gazebo/models',
        '../sdf_obstacles'
    ]
    sdf_path = '../sdf_obstacles/OfficeChairBlack/model.sdf'
    bounding_box = get_bounding_box_from_sdf(sdf_path, model_paths=model_paths)
    if bounding_box:
        print(f"Bounding Box: Min Coordinates: {bounding_box[0]}, Max Coordinates: {bounding_box[1]}")
    else:
        print("No bounding box could be calculated.")
