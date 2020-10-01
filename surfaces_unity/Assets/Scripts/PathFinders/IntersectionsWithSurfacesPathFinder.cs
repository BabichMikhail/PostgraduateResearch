using System;
using System.Collections.Generic;
using System.Linq;
using Generic;
using Unity.Profiling;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Experimental.AI;
using Edge = Generic.Edge;
using Plane = Generic.Plane;
using Point = Generic.Point;
using Triangle = Generic.Triangle;

namespace PathFinders
{
    public class IntersectionsWithSurfacesPathFinder : IPathFinder {
        private class TrianglePath {
            public readonly Triangle triangle;
            public readonly Edge edge;

            public TrianglePath(Triangle aTriangle, Edge aEdge) {
                triangle = aTriangle;
                edge = aEdge;
            }
        }

        public float paintHeight;
        public float paintRadius;

        public IntersectionsWithSurfacesPathFinder(float aPaintRadius, float aPaintHeight) {
            paintHeight = aPaintHeight;
            paintRadius = aPaintRadius;
        }

        Point PreparePoint(Point v) {
            var precision = 1e4f;
            return new Point(Mathf.Round(v.x * precision) / precision, Mathf.Round(v.y * precision) / precision, Mathf.Round(v.z * precision) / precision);
        }

        public List<Position> GetPath(ref List<Triangle> originTriangles) {
            var triangles = new List<Triangle>();
            foreach (var t in originTriangles) {
                triangles.Add(new Triangle(PreparePoint(t.p1), PreparePoint(t.p2), PreparePoint(t.p3)));
            }

            var a = new Point(1, 0, 0);
            var b = new Point(1, 1, 0);
            var c = new Point(1, 1, 1);

            var basePlane = new Plane(a, b, c);

            var a1 = new Point(a.x * Mathf.Cos(Mathf.PI / 2) - a.z * Mathf.Sin(Mathf.PI / 2), a.y, a.x * Mathf.Sin(Mathf.PI / 2) + a.z * Mathf.Cos(Mathf.PI / 2));
            var b1 = new Point(b.x * Mathf.Cos(Mathf.PI / 2) - b.z * Mathf.Sin(Mathf.PI / 2), b.y, b.x * Mathf.Sin(Mathf.PI / 2) + b.z * Mathf.Cos(Mathf.PI / 2));
            var c1 = new Point(c.x * Mathf.Cos(Mathf.PI / 2) - c.z * Mathf.Sin(Mathf.PI / 2), c.y, c.x * Mathf.Sin(Mathf.PI / 2) + c.z * Mathf.Cos(Mathf.PI / 2));
            var plane1 = new Plane(a1, b1, c1);
            var plane2 = new Plane(a1, b1, c1);

            var maxPlaneD = 0.0f;
            foreach (var t in triangles) {
                foreach (var p in t.GetPoints()) {
                    maxPlaneD = Mathf.Max(p.magnitude, maxPlaneD);
                }
            }

            var basePlaneD = 2 * maxPlaneD * (float)basePlane.GetDenominator();
            var plane1D = 2 * maxPlaneD * (float)plane1.GetDenominator();
            var plane2D = 2 * maxPlaneD * (float)plane2.GetDenominator();

            basePlane.D = basePlaneD;
            plane1.D = plane1D;
            plane2.D = -plane2D;

            foreach (var t in triangles) {
                foreach (var p in t.GetPoints()) {
                    basePlaneD = Mathf.Min((float)basePlane.GetDistance(p), basePlaneD);
                    plane1D = Mathf.Min((float)plane1.GetDistance(p), plane1D);
                    plane2D = Mathf.Min((float)plane2.GetDistance(p), plane2D);
                }
            }

            basePlane.D -= (basePlaneD - 10) * (float)basePlane.GetDenominator();
            plane1.D -= (plane1D - 10) * (float)plane1.GetDenominator();
            plane2.D += (plane2D - 10) * (float)plane2.GetDenominator();

            {
                var maxDistance = plane2.GetDistance(plane1.GetSomePoint());
                foreach (var t in triangles) {
                    foreach (var p in t.GetPoints()) {
                        Debug.Assert(plane1.GetDistance(p) < maxDistance);
                        Debug.Assert(plane2.GetDistance(p) < maxDistance);
                    }
                }
            }

            var fixedPathPartsByDistance = new Dictionary<double, List<TrianglePath>>();
            foreach (var pathPart in SplitObjectByDistance(basePlane, triangles)) {
                var edgesByPoint = new Dictionary<Point, List<Edge>>();
                foreach (var tp in pathPart.Value) {
                    AddEdge(ref edgesByPoint, tp.edge.p1, tp.edge);
                    AddEdge(ref edgesByPoint, tp.edge.p2, tp.edge);
                    // AddEdge(ref edgesByPoint, tp.edge.p1, edgeByAddedPoint[tp.edge.p1]);
                    // AddEdge(ref edgesByPoint, tp.edge.p2, edgeByAddedPoint[tp.edge.p2]);
                }

                var singlePoints = new List<Point>();
                var doublePoints = new List<Point>();
                var wrongPoints = new List<Point>();
                foreach (var ep in edgesByPoint) {
                    if (ep.Value.Count == 1) {
                        singlePoints.Add(ep.Key);
                    }
                    else if (ep.Value.Count == 2) {
                        doublePoints.Add(ep.Key);
                    }
                    else {
                        wrongPoints.Add(ep.Key);
                    }
                }

                if (wrongPoints.Count > 0) {
                    Debug.Assert(false);
                }

                var substitutes = new Dictionary<Point, Point>();
                var processedPointIndexes = new Dictionary<int, bool>();
                for (var i = 0; i < singlePoints.Count; ++i) {
                    var minDistance = 0.0;
                    var nearestIdx = -1;
                    for (var j = i + 1; j < singlePoints.Count; ++j) {
                        var d = (singlePoints[i] - singlePoints[j]).magnitude;
                        if (i != j && !processedPointIndexes.ContainsKey(i) && !processedPointIndexes.ContainsKey(j) && (nearestIdx == -1 || minDistance > d)) {
                            nearestIdx = j;
                            minDistance = d;
                        }
                    }

                    if (nearestIdx != -1) {
                        processedPointIndexes.Add(i, true);
                        processedPointIndexes.Add(nearestIdx, true);

                        var m = (singlePoints[i] + singlePoints[nearestIdx]) / 2;
                        substitutes.Add(singlePoints[i], m);
                        substitutes.Add(singlePoints[nearestIdx], m);

                        Debug.Assert((singlePoints[i] - singlePoints[nearestIdx]).magnitude < 1e-5);
                    }
                }

                var trianglePaths = new List<TrianglePath>();
                foreach (var tp in pathPart.Value) {
                    var p1 = tp.edge.p1;
                    if (substitutes.ContainsKey(p1)) {
                        p1 = substitutes[p1];
                    }

                    var p2 = tp.edge.p2;
                    if (substitutes.ContainsKey(p2)) {
                        p2 = substitutes[p2];
                    }

                    var edge = new Edge(p1, p2);
                    trianglePaths.Add(new TrianglePath(tp.triangle, edge));
                }

                fixedPathPartsByDistance.Add(pathPart.Key, trianglePaths);

                Debug.Assert(true);
            }

            var pathPartsByDistance = fixedPathPartsByDistance;

            var result = new List<Position>();
            foreach (var pathPart in pathPartsByDistance) {
                var fromPlane = plane1;
                var toPlane = plane2;

                var edges = new List<Edge>();
                var originalNormals = new List<Point>();
                var normals = new List<Point>();
                foreach (var tp in pathPart.Value) {
                    edges.Add(tp.edge);
                    originalNormals.Add(tp.triangle.GetPlane().GetNormal());
                }

                var pointsDict = new Dictionary<Point, bool>();
                var edgesByPoint = new Dictionary<Point, List<Edge>>();

                foreach (var edge in edges) {
                    if (!pointsDict.ContainsKey(edge.p1)) {
                        pointsDict.Add(edge.p1, true);
                    }

                    if (!pointsDict.ContainsKey(edge.p2)) {
                        pointsDict.Add(edge.p2, true);
                    }

                    if (!edgesByPoint.ContainsKey(edge.p1)) {
                        edgesByPoint.Add(edge.p1, new List<Edge>());
                    }

                    if (!edgesByPoint.ContainsKey(edge.p2)) {
                        edgesByPoint.Add(edge.p2, new List<Edge>());
                    }

                    edgesByPoint[edge.p1].Add(edge);
                    edgesByPoint[edge.p2].Add(edge);
                }

                var points = pointsDict.Keys.ToList();

                var k1 = 0;
                var k2 = 0;
                var pointToId = new Dictionary<Point, int>();
                var processedPoints = new Dictionary<int, bool>();
                for (var i = 0; i < points.Count; ++i) {
                    if (fromPlane.GetDistance(points[i]) < fromPlane.GetDistance(points[k1])) {
                        k1 = i;
                    }

                    if (fromPlane.GetDistance(points[i]) > fromPlane.GetDistance(points[k2])) {
                        k2 = i;
                    }

                    pointToId.Add(points[i], i);
                }

                var subResult = new List<Position>();
                subResult.Add(new Position(points[k1] + originalNormals[k1], originalNormals[k1], points[k1], Position.PointType.MIDDLE));
                processedPoints.Add(k1, true);
                while (!processedPoints.ContainsKey(k2)) {
                    var nextPointIds = new List<int>();

                    bool canBeNextPoint(int k) => !processedPoints.ContainsKey(k) && fromPlane.GetDistance(points[k]) > fromPlane.GetDistance(points[k1]);
                    foreach (var edge in edgesByPoint[points[k1]]) {
                        if (canBeNextPoint(pointToId[edge.p1])) {
                            nextPointIds.Add(pointToId[edge.p1]);
                        }

                        if (canBeNextPoint(pointToId[edge.p2])) {
                            nextPointIds.Add(pointToId[edge.p2]);
                        }
                    }

                    var k3 = -1;
                    if (nextPointIds.Count > 0) {
                        var dy = points[k1].y;
                        var dz = points[k1].z;
                        k3 = nextPointIds[0];
                        foreach (var i in nextPointIds) {
                            var p1 = points[k3];
                            var p2 = points[i];
                            var d = Math.Abs(p1.z - dz) < 10e-2 || Math.Abs(p2.z - dz) < 10e-2 ? 1 : 0;
                            if (fromPlane.GetDistance(points[k1]) < fromPlane.GetDistance(p2) && (p1.y - dy) / (p1.z - dz + d) < (p2.y - dy) / (p1.z - dz + d)) {
                                k3 = i;
                            }
                        }
                    }
                    else {
                        for (var i = 0; i < points.Count; ++i) {
                            if (canBeNextPoint(i) && (k3 == -1 || fromPlane.GetDistance(points[i]) < fromPlane.GetDistance(points[k3]))) {
                                k3 = i;
                            }
                        }
                    }

                    if (k3 == -1) {
                        throw new Exception("Magic");
                    }

                    subResult.Add(new Position(points[k3] + originalNormals[k1], originalNormals[k1], points[k3], Position.PointType.MIDDLE));
                    subResult.Add(new Position(points[k3] + originalNormals[k3], originalNormals[k3], points[k3], Position.PointType.MIDDLE));
                    processedPoints.Add(k3, true);
                    k1 = k3;
                }

                {
                    var i0 = 0;
                    var i1 = 1;

                    var pos0 = subResult[i0];
                    var n0 = subResult[i0].paintDirection;
                    var surface0 = subResult[i0].surfacePosition;
                    var surface1 = subResult[i1].surfacePosition;
                    var dir = (surface0 - surface1).normalized;
                    result.Add(new Position(pos0.originPosition + dir * paintRadius * 3, n0, pos0.surfacePosition + dir * paintRadius * 3, Position.PointType.START));
                }

                result.AddRange(subResult);

                {
                    var i0 = subResult.Count - 1;
                    var i1 = subResult.Count - 2;

                    var pos0 = subResult[i0];
                    var n0 = subResult[i0].paintDirection;
                    var surface0 = subResult[i0].surfacePosition;
                    var surface1 = subResult[i1].surfacePosition;
                    var dir = (surface0 - surface1).normalized;
                    result.Add(new Position(pos0.originPosition + dir * paintRadius * 3, n0, pos0.surfacePosition + dir * paintRadius * 3, Position.PointType.FINISH));
                }
            }

            return result;
        }

        Point GetPointUsingBinarySearch(Point a, Point b, Plane basePlane, double distance) {
            var da = basePlane.GetDistance(a);
            var db = basePlane.GetDistance(b);
            if (da > db) {
                var c = a;
                var dc = da;

                a = b;
                da = db;

                b = c;
                db = dc;
            }

            while (Math.Abs(da - distance) > 1e-4) {
                var p = (a + b) / 2;
                var dp = basePlane.GetDistance(p);

                if (Math.Abs(dp - distance) < 1e-4 || dp <= distance) {
                    a = p;
                    da = dp;
                }
                else if (dp > distance) {
                    b = p;
                    db = dp;
                }
                else {
                    throw new Exception("Unexpected behavior in binary search.");
                }
            }

            return a;
        }

        List<Point> GetFilteredPoints(List<Point> originalVertices, Triangle triangle, Plane basePlane) {
            var dict = new Dictionary<int, Point>();
            foreach (var v in originalVertices) {
                var code = v.GetHashCode();
                if (!dict.ContainsKey(code)) {
                    dict.Add(code, v);
                }
            }

            var vertices = dict.Values.ToList();
            while (vertices.Count > 2) {
                // Тут была битва при точности расчета точек. Короче херня какая-то.
                var m = new List<List<double>>();
                for (var i = 0; i < vertices.Count; ++i) {
                    m.Add(new List<double>());
                    for (var j = 0; j < vertices.Count; ++j) {
                        m[i].Add(0);
                    }
                }

                var i1 = -1;
                var i2 = -1;
                for (var i = 0; i < vertices.Count; ++i) {
                    for (var j = i + 1; j < vertices.Count; ++j) {
                        m[i][j] = m[j][i] = (vertices[i] - vertices[j]).sqrMagnitude;
                        if (i1 == -1 || m[i][j] < m[i1][i2]) {
                            i1 = i;
                            i2 = j;
                        }
                    }
                }

                var d1 = basePlane.GetDistance(vertices[i1]);
                var d2 = basePlane.GetDistance(vertices[i2]);

                var newVertices = new List<Point>();
                for (var i = 0; i < vertices.Count; ++i) {
                    if (i != i1 && i != i2) {
                        newVertices.Add(vertices[i]);
                    }
                }

                if (d1 < d2) {
                    newVertices.Add(vertices[i1]);
                }
                else {
                    newVertices.Add(vertices[i2]);
                }

                Debug.Assert(newVertices.Count == 2);
                if (newVertices.Count != 2) {
                    Debug.Assert(false);
                }
                vertices = newVertices;
            }

            if (vertices.Count > 2) {
                Debug.Assert(false);

                // foreach (var e in edges) {
                //     var d1 = basePlane.GetDistance(e.p1);
                //     var d2 = basePlane.GetDistance(e.p2);
                //
                //     var minD = Math.Min(d1, d2);
                //     var maxD = Math.Max(d1, d2);
                //     if (vertices.Count != 2 && maxD - minD > 1e-3 && minD - distance <= 1e-3 && distance - maxD <= 1e-3) {
                //         Debug.Assert(false);
                //         // vertices.Add(e.p1 + (e.p2 - e.p1) * ((distance - d1) / (d2 - d1)));
                //     }
                // }
            }

            return vertices;
        }

        Dictionary<double, List<TrianglePath>> SplitObjectByDistance(Plane basePlane, List<Triangle> triangles) {
            var lineWidth = paintRadius;
            var edgeByAddedPoint = new Dictionary<Point, Edge>();
            var pathPartsByDistance = new Dictionary<double, List<TrianglePath>>();
            foreach (var t in triangles) {
                var minDistance = basePlane.GetDistance(t.p1);
                var maxDistance = basePlane.GetDistance(t.p1);

                foreach (var v in new List<Point>{t.p2, t.p3}) {
                    var distance = basePlane.GetDistance(v);
                    minDistance = Math.Min(minDistance, distance);
                    maxDistance = Math.Max(maxDistance, distance);
                }

                var edges = t.GetEdges();
                for (var distance = Math.Ceiling(minDistance / lineWidth) * lineWidth; distance <= maxDistance; distance += lineWidth) {
                    var minTDistance = basePlane.GetDistance(edges.First().p1);
                    var maxTDistance = basePlane.GetDistance(edges.First().p1);

                    foreach (var e in edges) {
                        foreach (var p in e.GetPoints()) {
                            var d = basePlane.GetDistance(p);
                            maxTDistance = Math.Max(d, maxTDistance);
                            minTDistance = Math.Min(d, minTDistance);
                        }
                    }

                    var vertices = new List<Point>();
                    if (maxTDistance - minTDistance >= 0) {
                        foreach (var e in edges) {
                            var d1 = basePlane.GetDistance(e.p1);
                            var d2 = basePlane.GetDistance(e.p2);

                            var minD = Math.Min(d1, d2);
                            var maxD = Math.Max(d1, d2);
                            if (maxD >= minD && minD <= distance && distance <= maxD) {
                                // vertices.Add(e.p1 + (e.p2 - e.p1) * (float)((distance - d1) / (d2 - d1)));
                                // var point = (e.p1 * (float)(d2 - d1) + (e.p2 - e.p1) * (float)(distance - d1)) / (float)(d2 - d1);
                                var point = GetPointUsingBinarySearch(e.p1, e.p2, basePlane, distance);
                                if (Math.Abs(basePlane.GetDistance(point) - distance) > 1e-4) {
                                    Debug.Assert(false);
                                }

                                vertices.Add(point);
                                if (!edgeByAddedPoint.ContainsKey(point)) {
                                    edgeByAddedPoint.Add(point, e);
                                }
                            }
                        }

                        var fPoints = GetFilteredPoints(vertices, t, basePlane);

                        Edge newEdge = null;
                        if (fPoints.Count == 1 && vertices.Count == 2 && fPoints.GetHashCode() == vertices[0].GetHashCode() && vertices[0].GetHashCode() == vertices[1].GetHashCode()) {
                            newEdge = new Edge(fPoints[0], fPoints[0]);
                        }
                        else if (fPoints.Count == 2) {
                            newEdge = new Edge(fPoints[0], fPoints[1]);
                        }

                        if (newEdge is null) {
                            Debug.Log(false);
                        }

                        if (!(newEdge is null)) {
                            if (!pathPartsByDistance.ContainsKey(distance)) {
                                pathPartsByDistance.Add(distance, new List<TrianglePath>());
                            }
                            pathPartsByDistance[distance].Add(new TrianglePath(t, newEdge));
                        }
                    }
                }
            }

            return pathPartsByDistance;
        }

        void AddEdge(ref Dictionary<Point, List<Edge>> d, Point p, Edge e) {
            var p0 = p;
            if (d.ContainsKey(p0)) {
                d[p0].Add(e);
            }
            else {
                d.Add(p0, new List<Edge>{e});
            }
        }

        void AddTriangle(ref Dictionary<Edge, Triangle> d, Edge e, Triangle t) {
            if (d.ContainsKey(e)) {
                // MAGIC!!
                Debug.Assert(false);
                return;
            }
            d.Add(e, t);
        }

        void AddTriangle(ref Dictionary<Point, Triangle> d, Point p, Triangle t) {
            if (d.ContainsKey(p)) {
                // MAGIC!!
                Debug.Assert(false);
                return;
            }
            d.Add(p, t);
        }

        List<Position> GetOldSubResult(bool isForwardDirection, Dictionary<Point, List<Edge>> edgesByPoint, Plane fromPlane, Plane toPlane) {
            var subResult = new List<Position>();

            var edgeDict = new Dictionary<Edge, bool>();
            foreach (var ep in edgesByPoint) {
                foreach (var e in ep.Value) {
                    if (!edgeDict.ContainsKey(e)) {
                        edgeDict.Add(e, true);
                    }
                }
            }

            Debug.Assert(edgeDict.Count == edgesByPoint.Count);
            var edges = edgeDict.Keys.ToList();
            var sequences = GetSequences(edges, edgesByPoint);
            throw new Exception("qwe");

            return subResult;
        }

        List<List<Point>> GetSequences(List<Edge> edges, Dictionary<Point, List<Edge>> edgesByPoint) {
            var processedEdge = new Dictionary<Edge, bool>();
            var sequences = new List<List<Point>>();
            while (processedEdge.Count < edges.Count) {
                Edge baseEdge = null;
                foreach (var e in edges) {
                    if (!processedEdge.ContainsKey(e)) {
                        baseEdge = e;
                    }
                }

                var sequence = new List<Point>();
                while (true) {
                    processedEdge.Add(baseEdge, true);
                    sequence.Add(baseEdge.p1);

                    Edge nextEdge = null;
                    foreach (var e in edgesByPoint[baseEdge.p1]) {
                        if (!processedEdge.ContainsKey(e)) {
                            nextEdge = e;
                            break;
                        }
                    }

                    baseEdge = nextEdge;
                    if (baseEdge is null) {
                        break;
                    }
                }

                sequences.Add(sequence);
            }

            return sequences;
        }

        List<Position> GetSubResult(bool isForwardDirection, Point firstPoint, Point lastPoint, List<Point> points, Plane fromPlane, Dictionary<Point, Triangle> triangleByPoint) {

        // List<Position> GetSubResult(bool isForwardDirection, Dictionary<Point, List<Edge>> edgesByPoint, Point firstPoint, Point lastPoint, Dictionary<Edge, Triangle> triangleByEdge) {
            var subResult = new List<Position>();

            var processedPoints = new Dictionary<Point, bool>();
            var sequences = new List<List<Point>>();
            while (processedPoints.Count < points.Count) {
                var initialPoint = points[0];
                foreach (var p in points) {
                    if (processedPoints.ContainsKey(initialPoint) || !processedPoints.ContainsKey(p) && fromPlane.GetDistance(p) > fromPlane.GetDistance(initialPoint)) {
                        initialPoint = p;
                    }
                }

                var s = GetSequence(initialPoint, points, triangleByPoint);
                foreach (var p in s) {
                    processedPoints.Add(p, true);
                }
                throw new Exception();
                sequences.Add(s);
            }

            return subResult;
        }

        List<Point> GetSequence(Point firstPoint, List<Point> points, Dictionary<Point, Triangle> triangleByPoint) {
            var sequence = new List<Point>();
            var pointsByTriangle = new Dictionary<Triangle, List<Point>>();
            foreach (var point in points) {
                var t = triangleByPoint[point];
                if (!pointsByTriangle.ContainsKey(t)) {
                    pointsByTriangle.Add(t, new List<Point>());
                }
                pointsByTriangle[t].Add(point);
            }

            foreach (var p in points) {
                Debug.Assert(pointsByTriangle[triangleByPoint[p]].Count == 2);
            }

            // Point nextPoint = null;
            // var currentPoint = firstPoint;

            Point previousPoint = null;
            var currentPoint = firstPoint;
            // result.Add(firstPoint);
            while (previousPoint is null || currentPoint != firstPoint) {
                foreach (var p in pointsByTriangle[triangleByPoint[currentPoint]]) {
                    if (p != currentPoint) {
                        previousPoint = currentPoint;
                        currentPoint = p;
                        break;
                    }
                }

                sequence.Add(currentPoint);
            }

            for (var i = 0; i < sequence.Count - 1; ++i) {
                Debug.Assert(sequence[i] != sequence[i + 1]);
            }

            return sequence;
        }
    }
}
