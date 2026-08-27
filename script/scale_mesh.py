import argparse
import os

import open3d as o3d
import numpy as np
import math

class Node:
    idx: int
    x: float
    y: float
    z: float

    def __init__(self, idx: int, x: float, y: float, z: float):
        self.idx = idx
        self.x = x
        self.y = y
        self.z = z

    @staticmethod
    def from_line(line):
        index, *floats = line.split()
        index = int(index)
        floats = list(map(float, floats))
        return Node(index, *floats)


    def vertex(self):
        return [self.x,self.y,self.z]


class Face:
    idx: int
    v_idx1: int
    v_idx2: int
    v_idx3: int

    def __init__(self, idx: int, v_idx1: int, v_idx2: int, v_idx3: int):
        self.idx = idx
        self.v_idx1 = v_idx1
        self.v_idx2 = v_idx2
        self.v_idx3 = v_idx3


    @staticmethod
    def from_line(line):
        results = list(map(int, line.split()))
        return Face(*results)

    def triangle(self):
        return [self.v_idx1, self.v_idx2, self.v_idx3]


def fetch_data(node_file, face_file):
    with open(node_file, "r") as f:
        node_lines = f.readlines()
    with open(face_file, "r") as f:
        face_lines = f.readlines()
    nodes = []
    faces = []
    skipped_first = False
    for line in node_lines:
        if line.strip().startswith("#"):
            continue
        if not skipped_first:
            skipped_first = True
            continue
        nodes.append(Node.from_line(line))

    skipped_first = False
    for line in face_lines:
        if line.strip().startswith("#"):
            continue
        if not skipped_first:
            skipped_first = True
            continue
        faces.append(Face.from_line(line))
    return nodes, faces

def mesh_from_node_face(nodes, faces):
    vertices = list(map(lambda n: n.vertex(), nodes))
    triangles = list(map(lambda f: f.triangle(), faces))
    return o3d.geometry.TriangleMesh(vertices=o3d.utility.Vector3dVector(vertices), triangles=o3d.utility.Vector3iVector(triangles))


def node_face_from_mesh(mesh):
    vertices = np.asarray(mesh.vertices).tolist()
    triangles = np.asarray(mesh.triangles).tolist()
    nodes = [Node(i, *vertex) for i, vertex in enumerate(vertices)]
    faces = [Face(i, *triangle) for i, triangle in enumerate(triangles)]
    return nodes, faces


def scale_mesh(mesh, number_of_faces):
    num_triangles = len(mesh.triangles)
    if num_triangles == 0:
        raise ValueError("The mesh has no triangles. Cannot scale an empty mesh.")
    iterations = max(0, math.ceil(math.log(number_of_faces / num_triangles) / math.log(4)))
    if iterations > 0:
        mesh = mesh.subdivide_loop(number_of_iterations=iterations)
    mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=number_of_faces)
    mesh = mesh.remove_unreferenced_vertices()
    return mesh


def write_to_file(nodes, faces, filename):
    with open(f"{filename}.node", "w") as f:
        f.write("# Adapted from https://github.com/darioizzo/geodesyNets/tree/master/3dmeshes\n")
        f.write("# Node count, 3 dimensions, no attribute, no boundary marker\n")
        f.write(f"{len(nodes)} 3 0 0\n")
        f.write("# Node index, node coordinates\n")

        for node in nodes:
            line = f"{node.idx} {node.x} {node.y} {node.z}\n"
            f.write(line)

    with open(f"{filename}.face", "w") as f:
        f.write("# Adapted from https://github.com/darioizzo/geodesyNets/tree/master/3dmeshes\n")
        f.write("# Number of faces, boundary marker off\n")
        f.write(f"{len(faces)} 0\n")
        f.write("# Face index, nodes of face\n")

        for face in faces:
            line = f"{face.idx} {face.v_idx1} {face.v_idx2} {face.v_idx3}\n"
            f.write(line)


def main():
    parser = argparse.ArgumentParser(description="Scale a mesh into several progressively larger variants.")
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--node-face", nargs=2, metavar=("NODE_FILE", "FACE_FILE"),
                              help="Path to a .node/.face mesh pair to scale (tetgen format).")
    input_group.add_argument("--ply", metavar="PLY_FILE",
                              help="Path to a single .ply mesh file to scale.")
    parser.add_argument("--face-amounts", type=int, nargs="+",
                         default=[round(1000 * math.sqrt(3) ** k) for k in range(10)],
                         help="Target face counts to generate scaled variants for "
                              "(default: round(1000 * sqrt(3)^k) for k in range(10)).")
    args = parser.parse_args()

    if args.node_face:
        node_file, face_file = args.node_face
        filename = os.path.splitext(node_file)[0]
        nodes, faces = fetch_data(node_file, face_file)
        base_mesh = mesh_from_node_face(nodes, faces)
        for amount in args.face_amounts:
            scaled_mesh = scale_mesh(base_mesh, amount)
            scaled_nodes, scaled_faces = node_face_from_mesh(scaled_mesh)
            write_to_file(scaled_nodes, scaled_faces, f"{filename}_scaled-{amount}")
    else:
        filename = os.path.splitext(args.ply)[0]
        base_mesh = o3d.io.read_triangle_mesh(args.ply)
        for amount in args.face_amounts:
            scaled_mesh = scale_mesh(base_mesh, amount)
            o3d.io.write_triangle_mesh(f"{filename}_scaled-{amount}.ply", scaled_mesh)


if __name__ == "__main__":
    main()
