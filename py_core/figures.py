import os.path
import tkinter
import numpy
from matplotlib import pyplot
from mpl_toolkits import mplot3d
from tkinter import *
import stl

basePath = 'G:\\Projects\\PostgraduateResearch\\data'
figures = [
    'long_angle',
    'long_inv_angle',
    'short_shear_waves_40',
    'zz44_200',
    'long_concave_line_1000',
    'long_convex_line_1000',
]


# root = tkinter.Tk
# canvas = tkinter.Canvas(root, width=1000, height=1000)

def is2dPointsEqual(p1, p2):
    return p1['x'] == p2['x'] and p1['y'] == p2['y']

figuresData = []
index = 0
for f in figures:
    fName = os.path.join(basePath, f, 'data.stl')
    # file = os.open(fName, 'rb')
    mesh = stl.mesh.Mesh.from_file(fName)
    # figure = pyplot.figure()
    # axes = mplot3d.Axes3D(figure)
    # axes.add_collection3d(mplot3d.art3d.Poly3DCollection(mesh.vectors))
    # scale = mesh.points.flatten()
    # axes.auto_scale_xyz(scale, scale, scale)

    points2d = []
    for point in mesh.points:
        points2d.append(
            [{'x': point[2], 'y': point[1]}, {'x': point[5], 'y': point[4]}, {'x': point[8], 'y': point[7]}])

    tPoints = []
    for p in points2d:
        if is2dPointsEqual(p[0], p[1]) or is2dPointsEqual(p[0], p[2]):
            tPoints.append(p[0])
        else:
            tPoints.append(p[1])
        # print(p, tPoints[-1])

    # print('-------')
    fPoints = []
    for i in range(len(tPoints)):
        if i + 1 == len(tPoints) or not is2dPointsEqual(tPoints[i], tPoints[i + 1]):
            fPoints.append(tPoints[i])


    minX = 1.0e10
    maxX = 0.0
    minY = 1.0e10
    maxY = 0.0
    for p in fPoints:
        minX = min(minX, p['x'])
        maxX = max(maxX, p['x'])
        minY = min(minY, p['y'])
        maxY = max(maxY, p['y'])

    figuresData.append({
        'fPoints': fPoints,
        'figure': f,
        'pos': index,
        'minX': minX,
        'maxX': maxX,
        'minY': minY,
        'maxY': maxY,
    })
    index += 1
    # for p in fPoints:
    #     print(p)


def sign(a):
    if a > 0:
        return 1
    if a < 0:
        return -1
    return 0

alpha = 60
scale = 2.0
columnsCount = 3
columnWidth = 160.0 * scale
borderWidth = 32.0 * scale
rowHeight = 160 * scale - alpha * scale
borderHeight = 0.0 * scale
rowCount = (len(figuresData) // columnsCount + sign(len(figuresData) % columnsCount))
width = (columnWidth + borderWidth) * columnsCount + borderWidth
height = (rowHeight + borderHeight) * rowCount + borderHeight

print(width, height)

root = Tk()
canvas = Canvas(root, bg='white', width=width, height=height)

for i in range(len(figuresData)):
    row = i // columnsCount
    column = i % columnsCount

    xOffset = (column + 0.5) * columnWidth + (column + 1) * borderWidth
    yOffset = (row + 0.5) * rowHeight + (row + 1) * borderHeight

    data = figuresData[i]
    print(row, column, data['figure'])

    xNorm = columnWidth / (data['maxX'] - data['minX'])
    yNorm = rowHeight / (data['maxY'] - data['minY'])
    norm = min(xNorm, yNorm)

    dx = -(data['maxX'] + data['minX']) / 2.0
    dy = -(data['maxY'] + data['minY']) / 2.0

    for j in range(len(data['fPoints']) - 1):
        p0 = data['fPoints'][j]
        p1 = data['fPoints'][j + 1]

        x0 = (p0['x'] + dx) * norm + xOffset
        y0 = (p0['y'] + dy) * norm + yOffset
        x1 = (p1['x'] + dx) * norm + xOffset
        y1 = (p1['y'] + dy) * norm + yOffset

        print(x0, y0, x1, y1, p0['x'], p1['x'], data['minX'], data['maxX'])

        canvas.create_line(x0, y0, x1, y1, width=3)

canvas.create_text(30 * scale, 70 * scale - 0.5 * alpha * scale, text='а)', font='Helvetica 24')
canvas.create_text(230 * scale, 70 * scale - 0.5 * alpha * scale, text='б)', font='Helvetica 24')
canvas.create_text(400 * scale, 70 * scale - 0.5 * alpha * scale, text='в)', font='Helvetica 24')
canvas.create_text(30 * scale, 210 * scale - 1.5 * alpha * scale, text='г)', font='Helvetica 24')
canvas.create_text(230 * scale, 210 * scale - 1.5 * alpha * scale, text='д)', font='Helvetica 24')
canvas.create_text(400 * scale, 210 * scale - 1.5 * alpha * scale, text='е)', font='Helvetica 24')

canvas.create_text(485 * scale, 103 * scale - 0.5 * alpha * scale, text='2Pi / A', font='Helvetica 24')
canvas.create_line(461 * scale, 93 * scale - 0.5 * alpha * scale, 509 * scale, 93 * scale - 0.5 * alpha * scale, fill='gray', width=3)
# canvas.create_line(461 * scale, 93 * scale, 461 * scale, 73 * scale, fill='gray', width=3)
# canvas.create_line(509 * scale, 93 * scale, 509 * scale, 73 * scale, fill='gray', width=3)

canvas.create_text(67.5 * scale, 260 * scale - 1.5 * alpha * scale, text='T', font='Helvetica 24')
canvas.create_line(63 * scale, 250 * scale - 1.5 * alpha * scale, 73 * scale, 250 * scale - 1.5 * alpha * scale, fill='gray', width=3)

canvas.create_text(313 * scale, 230 * scale - 1.5 * alpha * scale, text='R', font='Helvetica 24')
canvas.create_line(304 * scale, 196 * scale - 1.5 * alpha * scale, 304 * scale, 250 * scale - 1.5 * alpha * scale, fill='gray', width=3)

canvas.create_text(505 * scale, 250 * scale - 1.5 * alpha * scale, text='R', font='Helvetica 24')
canvas.create_line(496 * scale, 230 * scale - 1.5 * alpha * scale, 496 * scale, 284 * scale - 1.5 * alpha * scale, fill='gray', width=3)


canvas.pack()
root.mainloop()

# pyplot.show()
