import os
import pathlib

import numpy as np
import pathlib as pathlib
from scipy import spatial as sp_spatial
from scipy.spatial import Delaunay


def icosahedron():
    h = 0.5*(1+np.sqrt(5))
    p1 = np.array([[0, 1, h], [0, 1, -h], [0, -1, h], [0, -1, -h]])
    p2 = p1[:, [1, 2, 0]]
    p3 = p1[:, [2, 0, 1]]
    return np.vstack((p1, p2, p3))


def cube():
    return np.array([
        [0.5, -0.5, 0.5], [-0.5, -0.5, 0.5], [0.5, 0.5, 0.5], [-0.5, 0.5, 0.5],
        [0.5, 0.5, -0.5], [-0.5, 0.5, -0.5], [0.5, -0.5, -0.5], [-0.5, -0.5, -0.5]
    ])


def custom():

    file = open(os.path.join(pathlib.Path(__file__).parent.absolute(), "vertices.txt"), "r")
    content = file.read()
    file.close()
    lines = content.split("\n")
    count = int(lines[0])
    # print(count)
    vertices = []
    for i in range(count):
        line = lines[i + 1]

        numbers = line.split(" ")
        vertex = []
        for number in numbers:
            vertex.append(float(number.replace(",", ".")))
        vertices.append(vertex)

    return np.array(vertices)


# points = icosahedron()
# points = cube()
points = custom()


# hull = Delaunay(points)
hull = sp_spatial.ConvexHull(points)
indices = hull.simplices
faces = points[indices]

lines = [str(len(faces))]
for f in faces:
    coords = []
    for p in f:
        coords.append(str(p[0]))
        coords.append(str(p[1]))
        coords.append(str(p[2]))

    lines.append(" ".join(coords))

print(os.path.join(pathlib.Path(__file__).parent.absolute(), "triangles.txt"))
f = open(os.path.join(pathlib.Path(__file__).parent.absolute(), "triangles.txt"), "wb")
f.write(bytes("\n".join(lines), encoding='utf8'))
f.close()
# plt.show()
