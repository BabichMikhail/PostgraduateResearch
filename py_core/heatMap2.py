import math
import matplotlib.pyplot as plt
import numpy as np
import sys


def Sub(p1, p2):
    return {"x": p1["x"] - p2["x"], "z": p1["z"] - p2["z"]}


def GetLength(p):
    return math.sqrt(p["x"] * p["x"] + p["z"] * p["z"])


def GetDistance(p1, p2):
    return GetLength(Sub(p1, p2))


def GetSquare(p1, p2, p3):
    d1 = GetDistance(p2, p3)
    d2 = GetDistance(p1, p3)
    d3 = GetDistance(p1, p2)
    pp = (d1 + d2 + d3) / 2.0
    return math.sqrt(math.fabs(pp * (pp - d1) * (pp - d2) * (pp - d3)))



def Dot(p1, p2):
    l = GetLength(p1) * GetLength(p2)
    if l == 0:
        return 0
    return (p1["x"] * p2["x"] + p1["z"] * p2["z"]) / l


def Sign(p1, p2, p3):
    return (p1["x"] - p3["x"]) * (p2["z"] - p3["z"]) - (p2["x"] - p3["x"]) * (p1["z"] - p3["z"])


def IsPointInTriangle(pt, p1, p2, p3):
    # st = GetSquare(p1, p2, p3)
    # s1 = GetSquare(pt, p2, p3)
    # s2 = GetSquare(pt, p1, p3)
    # s3 = GetSquare(pt, p1, p2)
    # return math.fabs(st - (s1 + s2 + s3)) < 1e-4

    d1 = Sign(pt, p1, p2)
    d2 = Sign(pt, p2, p3)
    d3 = Sign(pt, p3, p1)

    has_neg = d1 < 0 or d2 < 0 or d3 < 0
    has_pos = d1 > 0 or d2 > 0 or d3 > 0
    return not has_neg or not has_pos


def getHeatMap(fileName, mapLength):
    sumV = 0.0
    triangles = []

    file = open(fileName, "r")
    while True:
        line = file.readline()
        if not line:
            break

        parts = line.strip().replace(",", ".").split('\t')
        t = []
        for i in range(3):
            pointData = parts[2 * i].strip("()")
            amount = float(parts[2 * i + 1])
            coords = pointData.split(";")
            x = float(coords[0])
            y = float(coords[1])
            z = float(coords[2])
            t.append({"point": {"x": x, "y": y, "z": z}, "q": amount})

        print(t)
        triangles.append(t)
        square = GetSquare(t[0]["point"], t[1]["point"], t[2]["point"])
        sumV += (t[0]["q"] + t[1]["q"] + t[2]["q"]) * square

    print(sumV)

    minX = sys.float_info.max
    maxX = -sys.float_info.max
    minZ = sys.float_info.max
    maxZ = -sys.float_info.max

    for t in triangles:
        for item in t:
            p = item["point"]
            minX = min(minX, p["x"])
            maxX = max(maxX, p["x"])
            minZ = min(minZ, p["z"])
            maxZ = max(maxZ, p["z"])

    print(minX)
    print(maxX)
    print(minZ)
    print(maxZ)

    step = (maxZ - minZ) / (mapLength - 1)
    mapWidth = int(mapLength)
    mapHeight = int((maxX - minX) / step + 1)

    print((mapHeight, mapWidth))
    hotMap = np.zeros((mapHeight, mapWidth))
    print(hotMap)
    hotMapCounts = np.zeros((mapHeight, mapWidth))

    for t in triangles:
        tMinX = sys.float_info.max
        tMaxX = -sys.float_info.max
        tMinZ = sys.float_info.max
        tMaxZ = -sys.float_info.max
        for item in t:
            p = item["point"]
            tMinX = min(tMinX, p["x"])
            tMaxX = max(tMaxX, p["x"])
            tMinZ = min(tMinZ, p["z"])
            tMaxZ = max(tMaxZ, p["z"])

        minI = math.floor((tMinX - minX) / step)
        maxI = math.ceil((tMaxX - minX) / step)
        minJ = math.floor((tMinZ - minZ) / step)
        maxJ = math.ceil((tMaxZ - minZ) / step)

        for i in range(maxI - minI + 1):
            for j in range(maxJ - minJ + 1):
                ii = minI + i
                jj = minJ + j
                pt = {"x": minX + step * ii, "z": minZ + step * jj}
                if IsPointInTriangle(pt, t[0]["point"], t[1]["point"], t[2]["point"]):
                    p0 = t[0]["point"]
                    p1 = t[1]["point"]
                    p2 = t[2]["point"]

                    d0 = GetDistance(pt, p0)
                    d1 = GetDistance(pt, p1)
                    d2 = GetDistance(pt, p2)

                    d01 = GetDistance(p0, p1)
                    d02 = GetDistance(p0, p2)
                    d12 = GetDistance(p1, p2)

                    ts = GetSquare(p0, p1, p2)
                    t01 = GetSquare(pt, p0, p1)
                    t02 = GetSquare(pt, p0, p2)
                    t12 = GetSquare(pt, p1, p2)
                    w01 = pow((ts - t01) / ts, 1)
                    w02 = pow((ts - t02) / ts, 1)
                    w12 = pow((ts - t12) / ts, 1)

                    q01 = \
                        t[0]["q"] * (d01 - d0 * Dot(Sub(pt, p0), Sub(p1, p0))) / d01 + \
                        t[1]["q"] * (d01 - d1 * Dot(Sub(pt, p1), Sub(p0, p1))) / d01
                    q02 = \
                        t[0]["q"] * (d02 - d0 * Dot(Sub(pt, p0), Sub(p2, p0))) / d02 + \
                        t[2]["q"] * (d02 - d2 * Dot(Sub(pt, p2), Sub(p0, p2))) / d02
                    q12 = \
                        t[1]["q"] * (d12 - d1 * Dot(Sub(pt, p1), Sub(p2, p1))) / d12 + \
                        t[2]["q"] * (d12 - d2 * Dot(Sub(pt, p2), Sub(p1, p2))) / d12
                    q = q01 * w01 + q02 * w02 + q12 * w12
                    q /= 2
                    hotMap[ii][jj] = q
                    hotMapCounts[ii][jj] += 1

    return hotMap

file1 = "T:\\Projects\\PostgraduateResearch\\store\\articleExperiments36\\angleWide12S_texture.txt"
file2 = "T:\\Projects\\PostgraduateResearch\\store\\articleExperiments36\\angleWide08A_texture.txt"
mapLength = 101

hotMap1 = getHeatMap(file1, mapLength)
hotMap2 = getHeatMap(file2, mapLength)

plt.imshow(hotMap1 - hotMap2, cmap='hot', interpolation='nearest')
# plt.title(sys.argv[1])
plt.show()
