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
        try:
            index, *floats = line.split()
            index = int(index)
            floats = list(map(float, floats))
            if len(floats) != 3:
                raise ValueError("Node line must contain exactly 3 coordinates.")
            return Node(index, *floats)
        except Exception as e:
            print(f"Error parsing node line: {line}")
            raise e


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


class MeshScaler:
    """Scales a base mesh to several face counts, sharing Loop subdivisions between targets.

    Subdividing is the expensive step and its result only depends on the iteration count, so each
    subdivision level is computed once (building on the previous level) and reused by every
    target needing it. Every target is still decimated from a subdivided base mesh, never from
    another decimated one, so output is identical to scaling each target independently.
    """

    def __init__(self, base_mesh):
        if len(base_mesh.triangles) == 0:
            raise ValueError("The mesh has no triangles. Cannot scale an empty mesh.")
        self.base_triangles = len(base_mesh.triangles)
        self.levels = {0: base_mesh}

    def _subdivided(self, iterations):
        if iterations not in self.levels:
            closest = max(level for level in self.levels if level < iterations)
            self.levels[iterations] = self.levels[closest].subdivide_loop(
                number_of_iterations=iterations - closest)
        return self.levels[iterations]

    def scale(self, number_of_faces):
        iterations = max(0, math.ceil(math.log(number_of_faces / self.base_triangles) / math.log(4)))
        mesh = self._subdivided(iterations)
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=number_of_faces)
        return mesh.remove_unreferenced_vertices()


def write_to_file(mesh, filename):
    vertices = np.asarray(mesh.vertices).tolist()
    triangles = np.asarray(mesh.triangles).tolist()

    with open(f"{filename}.node", "w") as f:
        f.write("# Adapted from https://github.com/darioizzo/geodesyNets/tree/master/3dmeshes\n")
        f.write("# Node count, 3 dimensions, no attribute, no boundary marker\n")
        f.write(f"{len(vertices)} 3 0 0\n")
        f.write("# Node index, node coordinates\n")
        f.write("".join(f"{i} {x} {y} {z}\n" for i, (x, y, z) in enumerate(vertices)))

    with open(f"{filename}.face", "w") as f:
        f.write("# Adapted from https://github.com/darioizzo/geodesyNets/tree/master/3dmeshes\n")
        f.write("# Number of faces, boundary marker off\n")
        f.write(f"{len(triangles)} 0\n")
        f.write("# Face index, nodes of face\n")
        f.write("".join(f"{i} {a} {b} {c}\n" for i, (a, b, c) in enumerate(triangles)))


def main():
    parser = argparse.ArgumentParser(description="Scale a mesh into several progressively larger variants.")
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--node-face", nargs=2, metavar=("NODE_FILE", "FACE_FILE"),
                              help="Path to a .node/.face mesh pair to scale (tetgen format).")
    input_group.add_argument("--ply", metavar="PLY_FILE",
                              help="Path to a single .ply mesh file to scale.")
    parser.add_argument("--face-amounts", type=int, nargs="+",
                         default=[round(1000 * math.sqrt(3) ** k) for k in range(14)],
                         help="Target face counts to generate scaled variants for "
                              "(default: round(1000 * sqrt(3)^k) for k in range(14)).")
    args = parser.parse_args()
    # Ascending order lets MeshScaler build each subdivision level on top of the previous one.
    args.face_amounts = sorted(set(args.face_amounts))

    if args.node_face:
        node_file, face_file = args.node_face
        filename = os.path.splitext(node_file)[0]
        nodes, faces = fetch_data(node_file, face_file)
        base_mesh = mesh_from_node_face(nodes, faces)
        scaler = MeshScaler(base_mesh)
        for amount in args.face_amounts:
            write_to_file(scaler.scale(amount), f"{filename}_scaled-{amount}")
    else:
        filename = os.path.splitext(args.ply)[0]
        scaler = MeshScaler(o3d.io.read_triangle_mesh(args.ply))
        for amount in args.face_amounts:
            scaled_mesh = scaler.scale(amount)
            o3d.io.write_triangle_mesh(f"{filename}_scaled-{amount}.ply", scaled_mesh, write_ascii=True)


if __name__ == "__main__":
    main()
