using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using Generic;
using UnityEditor;
using UnityEngine;
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

                        Debug.Assert((singlePoints[i] - singlePoints[nearestIdx]).magnitude < 1e-4);
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
                var originalNormalByEdge = new Dictionary<Edge, Point>();
                var normals = new List<Point>();
                foreach (var tp in pathPart.Value) {
                    edges.Add(tp.edge);

                    var n = tp.triangle.GetPlane().GetNormal().normalized;
                    originalNormals.Add(n);
                    originalNormalByEdge.Add(tp.edge, n);
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

                // var points = pointsDict.Keys.ToList();

                // var resultPoints = getBodyPoints(points, fromPlane, toPlane, originalNormalByEdge, edgesByPoint);
                // var subResult = getBodyPoints2(points, fromPlane, toPlane, originalNormals, edgesByPoint);
                var subResult = getBodyPoint3(edges, fromPlane, toPlane, originalNormalByEdge);

                // Point GetNormal(Dictionary<Edge, Point> normalByEdge, Dictionary<Point, List<Edge>> aEdgesByPoint, Point p1, Point p2) {
                //     var q = new Dictionary<Edge, bool>();
                //     foreach (var e in aEdgesByPoint[p1]) {
                //         q.Add(e, true);
                //     }
                //
                //     foreach (var e in aEdgesByPoint[p2]) {
                //         if (q.ContainsKey(e)) {
                //             return normalByEdge[e];
                //         }
                //         q.Add(e, true);
                //     }
                //
                //     return null;
                // }

                // var subResult = new List<Position>();
                // for (var i = 0; i < resultPoints.Count; ++i) {
                //     if (i < resultPoints.Count - 1) {
                //         var n = GetNormal(originalNormalByEdge, edgesByPoint, resultPoints[i], resultPoints[i + 1]);
                //         if (!(n is null)) {
                //             subResult.Add(new Position(resultPoints[i] + n * paintHeight, -n, resultPoints[i], Position.PointType.MIDDLE));
                //         }
                //     }
                //
                //     if (i > 0) {
                //         var n = GetNormal(originalNormalByEdge, edgesByPoint, resultPoints[i - 1], resultPoints[i]);
                //         if (!(n is null)) {
                //             subResult.Add(new Position(resultPoints[i] + n * paintHeight, -n, resultPoints[i], Position.PointType.MIDDLE));
                //         }
                //     }
                // }

                {
                    var i0 = 0;
                    var i1 = i0 + 1;
                    while (i1 < subResult.Count - 1 && (subResult[i0].surfacePosition - subResult[i1].surfacePosition).magnitude < 2) {
                        ++i1;
                    }

                    var pos0 = subResult[i0];
                    var pd = subResult[i0].paintDirection;
                    var surface0 = subResult[i0].surfacePosition;
                    var surface1 = subResult[i1].surfacePosition;
                    var dir = (surface0 - surface1).normalized;
                    result.Add(new Position(pos0.originPosition + dir * paintRadius * 3, pd, pos0.surfacePosition + dir * paintRadius * 3, Position.PointType.START));
                }

                result.AddRange(subResult);

                {
                    var i0 = subResult.Count - 1;
                    var i1 = i0 - 1;
                    while (i1 > 0 && (subResult[i0].surfacePosition - subResult[i1].surfacePosition).magnitude < 2) {
                        --i1;
                    }

                    var pos0 = subResult[i0];
                    var pd = subResult[i0].paintDirection;
                    var surface0 = subResult[i0].surfacePosition;
                    var surface1 = subResult[i1].surfacePosition;
                    var dir = (surface0 - surface1).normalized;
                    result.Add(new Position(pos0.originPosition + dir * paintRadius * 3, pd, pos0.surfacePosition + dir * paintRadius * 3, Position.PointType.FINISH));
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

        Dictionary<Point, List<Edge>> GetEdgesByPoint(List<Edge> edges) {
            var edgesByPoint = new Dictionary<Point, List<Edge>>();
            foreach (var edge in edges) {
                if (!edgesByPoint.ContainsKey(edge.p1)) {
                    edgesByPoint.Add(edge.p1, new List<Edge>{edge});
                }
                else {
                    edgesByPoint[edge.p1].Add(edge);
                }

                if (!edgesByPoint.ContainsKey(edge.p2)) {
                    edgesByPoint.Add(edge.p2, new List<Edge>{edge});
                }
                else {
                    edgesByPoint[edge.p2].Add(edge);
                }
            }

            return edgesByPoint;
        }

        List<Position> getBodyPoint3(List<Edge> edges, Plane fromPlane, Plane toPlane, Dictionary<Edge, Point> normalByEdge) {
            var result = new List<Position>();

            var edgesByPoint = GetEdgesByPoint(edges);
            var eSequence = new List<Edge>();
            var usedEdges = new Dictionary<Edge, bool>();
            var blockedEdges = new Dictionary<Edge, bool>();

            while (true) {
                {
                    Edge e0 = null;
                    foreach (var e in edges) {
                        if (!usedEdges.ContainsKey(e)) {
                            Edge de = null; // directive edge;
                            Edge le = null; // last edge;
                            for (var j = eSequence.Count - 1 ; de is null && j >= 0; --j) {
                                if (!(le is null)) {
                                    de = eSequence[j];
                                }
                                if (MyMath.GetDistance(e, eSequence[j]) > 2) {
                                    le = eSequence[j];
                                }
                            }

                            var n1 = normalByEdge[e];
                            if (le is null) {
                                if (n1.y > -1 / Math.Sqrt(2) && (e0 is null || MyMath.GetDistance(fromPlane, e) < MyMath.GetDistance(fromPlane, e0))) {
                                    e0 = e;
                                }
                            }
                            else if (e0 is null || MyMath.GetDistance(le, e) < MyMath.GetDistance(le, e0)) {
                                var directionOk = true;
                                if (!(de is null)) {
                                    var d = e.p1 + e.p2 - le.p1 - le.p2;
                                    var d0 = le.p1 + le.p2 - de.p1 - de.p2;
                                    directionOk = d.x * d0.x + d.y * d0.y + d.z * d0.z > 0;
                                }

                                var n2 = normalByEdge[le];
                                if (directionOk && n1.y > -1 / Math.Sqrt(2) && (n1.normalized - n2.normalized).magnitude < 0.3) {
                                    e0 = e;
                                }
                            }
                        }
                    }

                    if (e0 is null) {
                        break;
                    }

                    eSequence.Add(e0);
                    usedEdges.Add(e0, true);
                }

                // if (eSequence.Count == 0) {
                //     var i0 = -1;
                //     for (var i = 0; i < edges.Count; ++i) {
                //         if (!usedEdges.ContainsKey(edges[i]) && normalByEdge[edges[i]].y > 0 && (i0 == -1 || MyMath.GetDistance(fromPlane, edges[i]) < MyMath.GetDistance(fromPlane, edges[i0]))) {
                //             i0 = i;
                //         }
                //     }
                //
                //     eSequence.Add(edges[i0]);
                //     usedEdges.Add(edges[i0], true);
                // }
                // else {
                //     var i0 = -1;
                //     var le = eSequence.Last();
                //     for (var i = 0; i < edges.Count; ++i) {
                //         var e = edges[i];
                //         var n1 = normalByEdge[e];
                //         var n2 = normalByEdge[le];
                //         if (!usedEdges.ContainsKey(edges[i]) && (i0 == -1 || MyMath.GetDistance(le, edges[i]) < MyMath.GetDistance(le, edges[i0]))) {
                //             if (n1.y > 0 && (n1.normalized - n2.normalized).magnitude < 0.3 && n1.y > -1 / Math.Sqrt(2)) {
                //                 i0 = i;
                //             }
                //         }
                //     }
                //
                //     if (i0 == -1) {
                //         break;
                //     }
                //
                //     eSequence.Add(edges[i0]);
                //     usedEdges.Add(edges[i0], true);
                // }

                while (true) {
                    var i0 = -1;
                    var lastEdge = eSequence.Last();

                    var cEdges = new List<Edge>();
                    foreach (var p in lastEdge.GetPoints()) {
                        foreach (var e in edgesByPoint[p]) {
                            if (!usedEdges.ContainsKey(e) && (normalByEdge[e].normalized - normalByEdge[lastEdge].normalized).magnitude < 0.3) {
                                cEdges.Add(e);
                            }
                        }
                    }

                    if (cEdges.Count == 0) {
                        break;
                    }

                    eSequence.Add(cEdges[0]);
                    usedEdges.Add(cEdges[0], true);
                }
            }

            for (var i = 0; i < eSequence.Count; ++i) {
                var edge = eSequence[i];
                var p1 = edge.p1;
                var p2 = edge.p2;
                if (i == 0) {
                    if (fromPlane.GetDistance(p2) < fromPlane.GetDistance(p1)) {
                        var p0 = p1;
                        p1 = p2;
                        p2 = p0;
                    }
                }
                else {
                    var prevEdge = eSequence[i - 1];
                    if (prevEdge.p1 == p2 || prevEdge.p2 == p2) {
                        var p0 = p1;
                        p1 = p2;
                        p2 = p0;
                    }
                }

                var n = normalByEdge[edge];
                result.Add(new Position(p1 + n * paintHeight, -n, p1, Position.PointType.MIDDLE));
                result.Add(new Position(p2 + n * paintHeight, -n, p2, Position.PointType.MIDDLE));
            }

            return result;
        }

        List<Point> getBodyPoints(List<Point> points, Plane fromPlane, Plane toPlane, Dictionary<Edge, Point> normalByEdge, Dictionary<Point, List<Edge>> edgesByPoint) {
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

            var result = new List<Point>();
            // var subResult = new List<Position>();

            {
                // var n0 = normalByEdge[edgesByPoint[points[k1]][0]];
                // var n1 = normalByEdge[edgesByPoint[points[k1]][1]];
                result.Add(points[k1]);
                // subResult.Add(new Position(points[k1] + n0 * paintHeight, -n0, points[k1], Position.PointType.MIDDLE));
            }

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
                        if (canBeNextPoint(i) && fromPlane.GetDistance(points[k1]) < fromPlane.GetDistance(p2) && (p1.y - dy) / (p1.z - dz + d) < (p2.y - dy) / (p1.z - dz + d)) {
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

                result.Add(points[k1]);
                // var n1 = normalByEdge[edgesByPoint[points[k3]][0]];
                // var n2 = normalByEdge[edgesByPoint[points[k3]][1]];
                // subResult.Add(new Position(points[k3] + n1 * paintHeight, -n1, points[k3], Position.PointType.MIDDLE));
                // subResult.Add(new Position(points[k3] + n2 * paintHeight, -n2, points[k3], Position.PointType.MIDDLE));
                processedPoints.Add(k3, true);
                k1 = k3;
            }

            // return subResult;
            return result;
        }

        List<Position> getBodyPoints2(List<Point> points, Plane fromPlane, Plane toPlane, List<Point> originalNormals, Dictionary<Point, List<Edge>> edgesByPoint) {
            var sequences = getSequences(points, edgesByPoint, fromPlane);

            var nearestPoints = new List<Point>();
            var farestPoints = new List<Point>();
            foreach (var sequence in sequences) {
                var p1 = sequence[0];
                var p2 = sequence[0];
                for (var j = 1; j < sequence.Count; ++j) {
                    if (fromPlane.GetDistance(sequence[j]) < fromPlane.GetDistance(p1)) {
                        p1 = sequence[j];
                    }

                    if (fromPlane.GetDistance(sequence[j]) > fromPlane.GetDistance(p2)) {
                        p2 = sequence[j];
                    }
                }

                nearestPoints.Add(p1);
                farestPoints.Add(p2);
            }

            var orderedSequenceIds = new List<int>();
            {
                var usedSequenceIds = new Dictionary<int, bool>();
                while (true) {
                    var s1 = -1;
                    for (var i = 0; i < sequences.Count; ++i) {
                        if (!usedSequenceIds.ContainsKey(i) && (s1 == -1 || fromPlane.GetDistance(nearestPoints[i]) < fromPlane.GetDistance(nearestPoints[s1]))) {
                            s1 = i;
                        }
                    }

                    if (s1 == -1) {
                        break;
                    }

                    var nextSequenceIds = new List<int> {s1};
                    var fromDistance = fromPlane.GetDistance(nearestPoints[s1]);
                    var toDistance = fromPlane.GetDistance(farestPoints[s1]);
                    for (var i = 0; i < sequences.Count; ++i) {
                        var distance = fromPlane.GetDistance(nearestPoints[i]);
                        if (i != s1 && !usedSequenceIds.ContainsKey(i) && distance >= fromDistance && distance <= toDistance) {
                            nextSequenceIds.Add(i);
                        }
                    }

                    var sequenceHeight = new Dictionary<int, float>();
                    foreach (var i in nextSequenceIds) {
                        for (var j = 0; j < sequences[i].Count; ++j) {
                            if (fromPlane.GetDistance(sequences[i][j]) >= fromDistance && fromPlane.GetDistance(sequences[i][j]) <= toDistance) {
                                sequenceHeight.Add(i, sequences[i][j].y);
                                break;
                            }
                        }
                    }

                    var nextId = -1;
                    foreach (var sh in sequenceHeight) {
                        if (nextId == -1 || sh.Value > sequenceHeight[nextId]) {
                            nextId = sh.Key;
                        }
                    }

                    usedSequenceIds.Add(nextId, true);
                    orderedSequenceIds.Add(nextId);
                }
            }

            var distancesBetweenSequences = new List<List<double>>();
            var nearestPointIdsBetweenSequences = new List<List<int>>();
            {
                for (var i = 0; i < sequences.Count; ++i) {
                    distancesBetweenSequences.Add(new List<double>());
                    nearestPointIdsBetweenSequences.Add(new List<int>());
                    for (var j = 0; j < sequences.Count; ++j) {
                        distancesBetweenSequences[i].Add(0.0);
                        nearestPointIdsBetweenSequences[i].Add(-1);
                    }
                }

                var maxDistance = fromPlane.GetDistance(toPlane.GetSomePoint()) + 1;
                for (var i = 0; i < sequences.Count; ++i) {
                    for (var j = i + 1; j < sequences.Count; ++j) {
                        var minDistance = maxDistance * 10000;
                        var k0 = -1;
                        var p0 = -1;
                        for (var k = 0; k < sequences[i].Count; ++k) {
                            for (var p = 0; p < sequences[j].Count; ++p) {
                                var newDistance = (sequences[i][k] - sequences[j][p]).magnitude;
                                Debug.Assert(maxDistance > newDistance);
                                if (newDistance < minDistance) {
                                    minDistance = newDistance;
                                    k0 = k;
                                    p0 = p;
                                }
                            }
                        }

                        if (k0 == -1 || p0 == -1) {
                            Debug.Assert(false);
                        }

                        distancesBetweenSequences[i][j] = minDistance;
                        distancesBetweenSequences[j][i] = minDistance;
                        nearestPointIdsBetweenSequences[i][j] = k0;
                        nearestPointIdsBetweenSequences[j][i] = p0;
                    }
                }

                Debug.Assert(true);
            }

            var newOrderedSequenceIds = new List<int>();
            {
                var usedSequenceIds = new Dictionary<int, bool>();
                {
                    var sId = 0;
                    var pId = 0;
                    for (var i = 0; i < sequences.Count; ++i) {
                        for (var j = 0; j < sequences[i].Count; ++j) {
                            if (fromPlane.GetDistance(sequences[sId][pId]) > fromPlane.GetDistance(sequences[i][j])) {
                                sId = i;
                                pId = j;
                            }
                        }
                    }

                    newOrderedSequenceIds.Add(sId);
                    usedSequenceIds.Add(sId, true);

                    while (usedSequenceIds.Count < sequences.Count) {
                        var nextId = -1;
                        for (var i = 0; i < sequences.Count; ++i) {
                            if (!usedSequenceIds.ContainsKey(i)) {
                                if (nextId == -1 || distancesBetweenSequences[sId][nextId] > distancesBetweenSequences[sId][i]) {
                                    nextId = i;
                                }
                            }
                        }

                        newOrderedSequenceIds.Add(nextId);
                        usedSequenceIds.Add(nextId, true);
                        sId = nextId;
                    }
                }
            }
            // orderedSequenceIds = newOrderedSequenceIds;

            var normalByPoint = new Dictionary<Point, Point>();
            for (var i = 0; i < points.Count; ++i) {
                normalByPoint.Add(points[i], originalNormals[i]);
            }

            var firstLastSequencePoints = new List<KeyValuePair<int, int>>();
            var subResult = new List<Position>();
            for (var i = 0; i < orderedSequenceIds.Count; ++i) {
                var id = orderedSequenceIds[i];
                var sequence = sequences[id];

                var firstId = -1;
                if (firstLastSequencePoints.Count == 0) {
                    var pId = 0;
                    for (var j = 1; j < sequence.Count; ++j) {
                        if (fromPlane.GetDistance(sequence[j]) < fromPlane.GetDistance(sequence[pId])) {
                            pId = j;
                        }
                    }

                    firstId = pId;
                }
                else {
                    firstId = nearestPointIdsBetweenSequences[id][orderedSequenceIds[i - 1]];
                }

                var lastId = -1;
                if (firstLastSequencePoints.Count == sequences.Count - 1) {
                    var pId = 0;
                    for (var j = 1; j < sequence.Count; ++j) {
                        if (toPlane.GetDistance(sequence[j]) < toPlane.GetDistance(sequence[pId])) {
                            pId = j;
                        }
                    }

                    lastId = pId;
                }
                else {
                    lastId = nearestPointIdsBetweenSequences[id][orderedSequenceIds[i + 1]];
                }

                if (firstId == -1 || lastId == -1) {
                    Debug.Assert(false);
                }

                firstLastSequencePoints.Add(new KeyValuePair<int, int>(firstId, lastId));
            }

            var newSequences = new List<List<Point>>();
            for (var k = 0; k < firstLastSequencePoints.Count; ++k) {
                var first = firstLastSequencePoints[k].Key;
                var last = firstLastSequencePoints[k].Value;
                var sequence = sequences[orderedSequenceIds[k]];

                var sequence1 = new List<Point>();
                var sequence2 = new List<Point>();

                // Debug.Log(first + " " + last + " " + sequence.Count);
                if (last >= first) {
                    for (var i = first; i <= last; ++i) {
                        sequence1.Add(sequence[i]);
                    }

                    for (var i = first; i >= 0; --i) {
                        sequence2.Add(sequence[i]);
                    }

                    for (var i = sequence.Count - 1; i >= last; --i) {
                        sequence2.Add(sequence[i]);
                    }
                }
                else {
                    for (var i = first; i < sequence.Count; ++i) {
                        sequence1.Add(sequence[i]);
                    }

                    for (var i = 0; i <= last; ++i) {
                        sequence1.Add(sequence[i]);
                    }

                    for (var i = first; i >= last; --i) {
                        sequence2.Add(sequence[i]);
                    }
                }

                var newSequence = sequence1;
                if (first != last && sequence2[1].y > sequence1[1].y) {
                    newSequence = sequence2;
                }

                newSequences.Add(newSequence);
            }

            foreach (var sequence in newSequences) {
                foreach (var point in sequence) {
                    subResult.Add(new Position(point + normalByPoint[point] * paintHeight, -normalByPoint[point], point, Position.PointType.MIDDLE));
                }
            }

            return subResult;
        }

        List<List<Point>> getSequences(List<Point> points, Dictionary<Point, List<Edge>> edgesByPoint, Plane fromPlane) {
            var result = new List<List<Point>>();
            var processedPoints = new Dictionary<Point, int>();

            bool canProcessPoint(Point p, ref Dictionary<Point, bool> localProcessedPoints) {
                return !localProcessedPoints.ContainsKey(p) && (!processedPoints.ContainsKey(p) || processedPoints[p] < edgesByPoint[p].Count / 2);
            }

            void processPoint(Point p, ref Dictionary<Point, bool> localProcessedPoints) {
                localProcessedPoints.Add(p, true);
                if (!processedPoints.ContainsKey(p)) {
                    processedPoints.Add(p, 1);
                }
                else {
                    ++processedPoints[p];
                }
            }

            while (processedPoints.Count < points.Count) {
                var localProcessedPoints = new Dictionary<Point, bool>();
                var k1 = -1;
                var minD = 0.0;
                for (var i = 0; i < points.Count; ++i) {
                    var d = fromPlane.GetDistance(points[i]);
                    if (canProcessPoint(points[i], ref localProcessedPoints) && (k1 == -1 || minD > d)) {
                        minD = d;
                        k1 = i;
                    }
                }

                var basePoint = points[k1];
                processPoint(basePoint, ref localProcessedPoints);

                var sequence = new List<Point>();
                sequence.Add(basePoint);
                while (true) {
                    var nextPoints = new List<Point>();
                    foreach (var edge in edgesByPoint[basePoint]) {
                        if (canProcessPoint(edge.p1, ref localProcessedPoints)) {
                            nextPoints.Add(edge.p1);
                        }

                        if (canProcessPoint(edge.p2, ref localProcessedPoints)) {
                            nextPoints.Add(edge.p2);
                        }
                    }

                    if (nextPoints.Count == 0 && edgesByPoint[basePoint].Count < 2) {
                        throw new Exception("Magic");
                    }

                    if (nextPoints.Count == 0) {
                        break;
                    }

                    basePoint = nextPoints[0];
                    sequence.Add(basePoint);
                    processPoint(basePoint, ref localProcessedPoints);
                }

                result.Add(sequence);
            }

            return result;
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
