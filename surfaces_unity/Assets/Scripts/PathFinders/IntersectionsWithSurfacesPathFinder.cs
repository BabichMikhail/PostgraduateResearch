using System;
using System.Collections.Generic;
using System.Linq;
using Generic;
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
        public float paintLateralAllowance;
        public float paintLongitudinalAllowance;

        public IntersectionsWithSurfacesPathFinder(float aPaintRadius, float aPaintHeight, float aPaintLateralAllowance, float aPaintLongitudinalAllowance) {
            paintHeight = aPaintHeight;
            paintRadius = aPaintRadius;
            paintLateralAllowance = aPaintLateralAllowance;
            paintLongitudinalAllowance = aPaintLongitudinalAllowance;
        }

        Point PreparePoint(Point v) {
            var precision = 1e4f;
            return new Point(Mathf.Round(v.x * precision) / precision, Mathf.Round(v.y * precision) / precision, Mathf.Round(v.z * precision) / precision);
        }

        public List<Position> GetPath(ref List<Triangle> originTriangles) {
            var triangles = new List<Triangle>();
            foreach (var t in originTriangles) {
                triangles.Add(new Triangle(PreparePoint(t.P1), PreparePoint(t.P2), PreparePoint(t.P3)));
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
                    maxPlaneD = Mathf.Max(p.Magnitude, maxPlaneD);
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

            basePlane.D -= (basePlaneD - 2 * paintLateralAllowance) * (float)basePlane.GetDenominator();
            plane1.D -= (plane1D - 10 - paintLateralAllowance) * (float)plane1.GetDenominator();
            plane2.D += (plane2D - 10 - paintLateralAllowance) * (float)plane2.GetDenominator();

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
                    AddEdge(ref edgesByPoint, tp.edge.P1, tp.edge);
                    AddEdge(ref edgesByPoint, tp.edge.P2, tp.edge);
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
                        var d = (singlePoints[i] - singlePoints[j]).Magnitude;
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

                        Debug.Assert((singlePoints[i] - singlePoints[nearestIdx]).Magnitude < 1e-4);
                    }
                }

                var trianglePaths = new List<TrianglePath>();
                foreach (var tp in pathPart.Value) {
                    var p1 = tp.edge.P1;
                    if (substitutes.ContainsKey(p1)) {
                        p1 = substitutes[p1];
                    }

                    var p2 = tp.edge.P2;
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
                var originalNormalByEdge = new Dictionary<Edge, Point>();
                foreach (var tp in pathPart.Value) {
                    edges.Add(tp.edge);
                    originalNormalByEdge.Add(tp.edge, tp.triangle.GetPlane().GetNormal().Normalized);
                }

                var pointsDict = new Dictionary<Point, bool>();
                var edgesByPoint = new Dictionary<Point, List<Edge>>();
                foreach (var edge in edges) {
                    AddPoint(ref pointsDict, edge.P1);
                    AddPoint(ref pointsDict, edge.P2);

                    AddEdge(ref edgesByPoint, edge.P1, edge);
                    AddEdge(ref edgesByPoint, edge.P2, edge);
                }

                var subResult = GetBodyPositions(edges, fromPlane, toPlane, originalNormalByEdge);

                {
                    var i0 = 0;
                    var i1 = i0 + 1;
                    while (i1 < subResult.Count - 1 && (subResult[i0].surfacePosition - subResult[i1].surfacePosition).Magnitude < 2) {
                        ++i1;
                    }

                    var pos0 = subResult[i0];
                    var pos1 = subResult[i1];
                    var pd = pos0.paintDirection;
                    var surface0 = pos0.surfacePosition;
                    var surface1 = pos1.surfacePosition;
                    var dir = (surface0 - surface1).Normalized;
                    result.Add(new Position(pos0.originPosition + dir * paintLongitudinalAllowance, pd, pos0.surfacePosition + dir * paintLongitudinalAllowance, Position.PointType.START));
                }

                result.AddRange(subResult);

                {
                    var i0 = subResult.Count - 1;
                    var i1 = i0 - 1;
                    while (i1 > 0 && (subResult[i0].surfacePosition - subResult[i1].surfacePosition).Magnitude < 2) {
                        --i1;
                    }

                    var pos0 = subResult[i0];
                    var pos1 = subResult[i1];
                    var pd = pos0.paintDirection;
                    var surface0 = pos0.surfacePosition;
                    var surface1 = pos1.surfacePosition;
                    var dir = (surface0 - surface1).Normalized;
                    result.Add(new Position(pos0.originPosition + dir * paintLongitudinalAllowance, pd, pos0.surfacePosition + dir * paintLongitudinalAllowance, Position.PointType.FINISH));
                }
            }

            return result;
        }

        Point GetPointUsingBinarySearch(Point a, Point b, Plane basePlane, double distance) {
            var da = basePlane.GetDistance(a);
            var db = basePlane.GetDistance(b);
            if (da > db) {
                MyUtils.Swap(ref a, ref b);
                MyUtils.Swap(ref da, ref db);
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
                        m[i][j] = m[j][i] = (vertices[i] - vertices[j]).SqrMagnitude;
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
            }

            return vertices;
        }

        Dictionary<Point, List<Edge>> GetEdgesByPoint(List<Edge> edges) {
            var edgesByPoint = new Dictionary<Point, List<Edge>>();
            foreach (var edge in edges) {
                AddEdge(ref edgesByPoint, edge.P1, edge);
                AddEdge(ref edgesByPoint, edge.P2, edge);
            }

            return edgesByPoint;
        }

        private bool IsNearestEdge(Edge cEdge, Edge lastEdge, List<List<Edge>> sequences) {
            var ok = true;
            for (var i = 0; i < sequences.Count && ok; ++i) {
                for (var j = 0; j < sequences[i].Count && ok; ++j) {
                    if (lastEdge == sequences[i][j]) {
                        i = sequences.Count;
                        break;
                    }

                    if (MyMath.GetDistance(cEdge, lastEdge) > MyMath.GetDistance(cEdge, sequences[i][j])) {
                        ok = false;
                    }
                }
            }

            return ok;
        }

        private bool IsGoodDirection(Edge cEdge, Edge directionEdge, Edge lastEdge) {
            var leP1 = lastEdge.P1;
            var leP2 = lastEdge.P2;
            if (MyMath.GetDistance(directionEdge, leP1) > MyMath.GetDistance(directionEdge, leP2)) {
                MyUtils.Swap(ref leP1, ref leP2);
            }

            var eP1 = cEdge.P1;
            var eP2 = cEdge.P2;
            if (MyMath.GetDistance(leP2, eP1) > MyMath.GetDistance(leP2, eP2)) {
                MyUtils.Swap(ref eP1, ref eP2);
            }

            return MyMath.GetDistance(eP2, leP1) <= MyMath.GetDistance(eP2, leP2);
        }

        List<Position> GetBodyPositions(List<Edge> edges, Plane fromPlane, Plane toPlane, Dictionary<Edge, Point> normalByEdge) {
            var result = new List<Position>();

            var edgesByPoint = GetEdgesByPoint(edges);
            var eSequences = new List<List<Edge>>();
            var usedEdges = new Dictionary<Edge, bool>();

            while (true) {
                Edge de0 = null;
                {
                    Edge e0 = null;
                    foreach (var e in edges) {
                        if (!usedEdges.ContainsKey(e)) {
                            Edge de = null; // directive edge;
                            Edge le = null; // last edge;
                            for (var i = eSequences.Count - 1; de is null && i >= 0; --i) {
                                var eSequence = eSequences[i];
                                for (var j = eSequence.Count - 1 ; de is null && j >= 0; --j) {
                                    if (!(le is null)) {
                                        de = eSequence[j];
                                    }
                                    if (MyMath.GetDistance(e, eSequence[j]) > 20) {
                                        le = eSequence[j];
                                    }
                                }
                            }

                            var n1 = normalByEdge[e];
                            if (le is null) {
                                if (n1.y > -1 / Math.Sqrt(2) && (e0 is null || MyMath.GetDistance(fromPlane, e) < MyMath.GetDistance(fromPlane, e0))) {
                                    e0 = e;
                                    de0 = de;
                                }
                            }
                            else if (e0 is null || MyMath.GetDistance(le, e) < MyMath.GetDistance(le, e0)) {
                                var n2 = normalByEdge[le];
                                if (!(de is null) && IsGoodDirection(e, de, le) && (n1.Normalized - n2.Normalized).Magnitude < 0.6 && IsNearestEdge(e, le, eSequences)) {
                                    e0 = e;
                                    de0 = de;
                                }
                            }
                        }
                    }

                    if (e0 is null) {
                        break;
                    }

                    eSequences.Add(new List<Edge>{e0});
                    usedEdges.Add(e0, true);
                }

                while (true) {
                    var i0 = -1;
                    var le = eSequences.Last().Last();

                    var cEdges = new List<Edge>();
                    foreach (var p in le.GetPoints()) {
                        foreach (var e in edgesByPoint[p]) {
                            var n1 = normalByEdge[e];
                            var n2 = normalByEdge[le];
                            if (!usedEdges.ContainsKey(e) && (n1.Normalized - n2.Normalized).Magnitude < 0.6) {
                                    cEdges.Add(e);
                                // }
                            }
                        }
                    }

                    if (cEdges.Count == 0) {
                        break;
                    }

                    eSequences.Last().Add(cEdges[0]);
                    usedEdges.Add(cEdges[0], true);
                }

                var length = 0.0f;
                foreach (var e in eSequences.Last()) {
                    length += (e.P1 - e.P2).Magnitude;
                }
                if (length < 2) {
                    eSequences.RemoveAt(eSequences.Count - 1);
                }
            }

            {
                var eSequence = new List<Edge>();
                foreach (var sequence in eSequences) {
                    eSequence.AddRange(sequence);
                }

                for (var i = 0; i < eSequence.Count; ++i) {
                    var edge = eSequence[i];
                    var p1 = edge.P1;
                    var p2 = edge.P2;
                    if (i == 0) {
                        if (fromPlane.GetDistance(p2) < fromPlane.GetDistance(p1)) {
                            MyUtils.Swap(ref p1, ref p2);
                        }
                    }
                    else {
                        var prevEdge = eSequence[i - 1];
                        if (prevEdge.P1 == p2 || prevEdge.P2 == p2) {
                            MyUtils.Swap(ref p1, ref p2);
                        }
                    }

                    var n = normalByEdge[edge];
                    result.Add(new Position(p1 + n * paintHeight, -n, p1, Position.PointType.MIDDLE));
                    result.Add(new Position(p2 + n * paintHeight, -n, p2, Position.PointType.MIDDLE));
                }
            }

            return result;
        }

        Dictionary<double, List<TrianglePath>> SplitObjectByDistance(Plane basePlane, List<Triangle> triangles) {
            var minDistanceToFigure = 1e9;
            foreach (var t in triangles) {
                foreach (var p in t.GetPoints()) {
                    minDistanceToFigure = Math.Min(minDistanceToFigure, basePlane.GetDistance(p));
                }
            }
            var offset = minDistanceToFigure - paintLateralAllowance;

            var lineWidth = paintRadius;
            var edgeByAddedPoint = new Dictionary<Point, Edge>();
            var pathPartsByDistance = new Dictionary<double, List<TrianglePath>>();
            foreach (var t in triangles) {
                var minDistance = basePlane.GetDistance(t.P1);
                var maxDistance = basePlane.GetDistance(t.P1);

                foreach (var v in new List<Point>{t.P2, t.P3}) {
                    var distance = basePlane.GetDistance(v);
                    minDistance = Math.Min(minDistance, distance);
                    maxDistance = Math.Max(maxDistance, distance);
                }

                var edges = t.GetEdges();
                for (var distance = Math.Ceiling((minDistance - offset) / lineWidth) * lineWidth + offset; distance <= maxDistance; distance += lineWidth) {
                    var minTDistance = basePlane.GetDistance(edges.First().P1);
                    var maxTDistance = basePlane.GetDistance(edges.First().P1);

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
                            var d1 = basePlane.GetDistance(e.P1);
                            var d2 = basePlane.GetDistance(e.P2);

                            var minD = Math.Min(d1, d2);
                            var maxD = Math.Max(d1, d2);
                            if (maxD >= minD && minD <= distance && distance <= maxD) {
                                var point = GetPointUsingBinarySearch(e.P1, e.P2, basePlane, distance);
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

                        if (!(newEdge is null)) {
                            var tp = new TrianglePath(t, newEdge);
                            if (!pathPartsByDistance.ContainsKey(distance)) {
                                pathPartsByDistance.Add(distance, new List<TrianglePath>{tp});
                            }
                            else {
                                pathPartsByDistance[distance].Add(tp);
                            }
                        }
                    }
                }
            }

            return pathPartsByDistance;
        }

        void AddPoint(ref Dictionary<Point, bool> d, Point p) {
            if (!d.ContainsKey(p)) {
                d.Add(p, true);
            }
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
    }
}
