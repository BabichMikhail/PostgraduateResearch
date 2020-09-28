using System;
using System.Collections.Generic;
using System.Linq;
using Generic;
using UnityEngine;
using UnityEngine.Assertions;
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

            // var baseDistance = 1000000.0;
            // var distance1 = 1000000.0;
            // var distance2 = 1000000.0;
            // foreach (var t in triangles) {
            //     foreach (var p in t.GetPoints()) {
            //         // For debug.
            //         baseDistance = basePlane.GetDistance(p);
            //         distance1 = plane1.GetDistance(p);
            //         distance2 = plane2.GetDistance(p);
            //     }
            // }

            var lineWidth = paintRadius;
            // var pathParts = new List<TrianglePath>();
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
                    var vertices = new List<Point>();
                    var firstEdge = edges.First();
                    var minTDistance = basePlane.GetDistance(firstEdge.p1);
                    var maxTDistance = basePlane.GetDistance(firstEdge.p1);

                    foreach (var e in edges) {
                        foreach (var p in e.GetPoints()) {
                            var d = basePlane.GetDistance(p);
                            maxTDistance = Math.Max(d, maxTDistance);
                            minTDistance = Math.Min(d, minTDistance);
                        }
                    }

                    if (maxTDistance - minTDistance > 1e-4) {
                        foreach (var e in edges) {
                            var d1 = basePlane.GetDistance(e.p1);
                            var d2 = basePlane.GetDistance(e.p2);

                            var minD = Math.Min(d1, d2);
                            var maxD = Math.Max(d1, d2);
                            if (maxD - minD > 1e-4 && minD - distance <= 1e-4 && distance - maxD <= 1e-4) {
                                // vertices.Add(e.p1 + (e.p2 - e.p1) * (float)((distance - d1) / (d2 - d1)));
                                // var point = (e.p1 * (float)(d2 - d1) + (e.p2 - e.p1) * (float)(distance - d1)) / (float)(d2 - d1);
                                var point = GetPointUsingBinarySearch(e.p1, e.p2, basePlane, distance);
                                var testDistance = basePlane.GetDistance(point);
                                if (Math.Abs(testDistance - distance) > 1e-4) {
                                    Debug.Assert(false);
                                }

                                vertices.Add(point);
                                if (!edgeByAddedPoint.ContainsKey(point)) {
                                    edgeByAddedPoint.Add(point, e);
                                }
                            }
                        }

                        vertices = GetFilteredPoints(vertices, t, basePlane);

                        if (vertices.Count == 2) {
                            var v0d = basePlane.GetDistance(vertices[0]);
                            var v1d = basePlane.GetDistance(vertices[1]);
                            if (Math.Abs(v0d - v1d) > 1e-4) {
                                Debug.Assert(false);
                            }
                            // Debug.Assert(Math.Abs(basePlane.GetDistance(vertices[0]) - basePlane.GetDistance(vertices[1])) <= 1e-4);

                            var v1 = vertices[0];
                            var v2 = vertices[1];
                            if (v2.x < v1.x) {
                                var v = v1;
                                v1 = v2;
                                v2 = v;
                            }

                            if ((v1 - v2).sqrMagnitude > 1e-4) {
                                if (!pathPartsByDistance.ContainsKey(distance)) {
                                    pathPartsByDistance.Add(distance, new List<TrianglePath>());
                                }
                                pathPartsByDistance[distance].Add(new TrianglePath(t, new Edge(v1, v2)));
                            }
                        }
                    }
                }
            }

            var sign = 1;
            var result = new List<Position>();
            foreach (var pathPart in pathPartsByDistance) {
                // var edgesByPoint = new Dictionary<Point, List<Edge>>();
                var triangleByPoint = new Dictionary<Point, Triangle>();
                // var triangleByEdge = new Dictionary<Edge, Triangle>();
                foreach (var tp in pathPart.Value) {
                    // AddEdge(ref edgesByPoint, tp.edge.p1, edgeByAddedPoint[tp.edge.p1]);
                    // AddEdge(ref edgesByPoint, tp.edge.p2, edgeByAddedPoint[tp.edge.p2]);
                    if (tp.edge is null) {
                        Debug.Assert(false);
                    }

                    AddTriangle(ref triangleByPoint, tp.edge.p1, tp.triangle);
                    AddTriangle(ref triangleByPoint, tp.edge.p2, tp.triangle);
                    // AddTriangle(ref triangleByEdge, tp.edge, tp.triangle);
                }

                var duplicatePoints = 0; // for debug
                var pointDict = new Dictionary<Point, bool>();
                foreach (var tp in pathPart.Value) {
                    foreach (var point in tp.edge.GetPoints()) {
                        if (!pointDict.ContainsKey(point)) {
                            pointDict.Add(point, true);
                        }
                        else {
                            ++duplicatePoints;
                        }
                    }
                }

                var points = pointDict.Keys.ToList();
                var nearestPointsForPoint = new Dictionary<Point, Edge>(); // TODO всё гавно.
                for (var i = 0; i < points.Count; ++i) {
                    var j1 = -1;
                    var j2 = -1;
                    for (var j = 0; j < points.Count; ++j) {
                        if (i == j) {
                            continue;
                        }

                        var d = (points[i] - points[j]).magnitude;
                        if (j1 == -1) {
                            j1 = j;
                        }
                        else if (j2 == -1) {
                            j2 = j;
                        }
                        else if (d < (points[i] - points[j1]).magnitude && true) { // TODO условие с нормалями
                            j1 = j;
                        }

                        if (j1 != -1 && j2 != -1 && (points[i] - points[j1]).magnitude < (points[i] - points[j2]).magnitude) {
                            var j0 = j1;
                            j1 = j2;
                            j2 = j0;
                        }
                    }

                    nearestPointsForPoint.Add(points[i], new Edge(points[j1], points[j2]));
                }

                var fromPlane = sign > 0 ? plane1 : plane2;
                var toPlane = sign > 0 ? plane2 : plane1;
                var trianglePaths = pathPart.Value.OrderBy(v => Math.Min(fromPlane.GetDistance(v.edge.p1), fromPlane.GetDistance(v.edge.p2))).ToList();
                var tp11111111112 = pathPart.Value.OrderBy(v => Math.Max(toPlane.GetDistance(v.edge.p1), toPlane.GetDistance(v.edge.p2))).ToList();

                var dfromto = toPlane.GetDistance(fromPlane.GetSomePoint());
                var dtofrom = fromPlane.GetDistance(toPlane.GetSomePoint());
                Debug.Assert(Math.Abs(dfromto - dtofrom) < 1e-4);
                for (var i = 0; i < trianglePaths.Count; ++i) {
                    var e1 = trianglePaths[i].edge;
                    var e2 = tp11111111112[i].edge;
                    var d1 = Math.Min(fromPlane.GetDistance(e1.p1), fromPlane.GetDistance(e1.p2));
                    var d11 = Math.Min(fromPlane.GetDistance(e2.p1), fromPlane.GetDistance(e2.p2));
                    var d2 = Math.Max(toPlane.GetDistance(e2.p1), toPlane.GetDistance(e2.p2));
                    var d21 = Math.Max(toPlane.GetDistance(e1.p1), toPlane.GetDistance(e1.p2));
                    Debug.Assert(e1 == e2);
                }

                var firstEdge = trianglePaths.First().edge;
                var firstPoint = fromPlane.GetDistance(firstEdge.p1) < fromPlane.GetDistance(firstEdge.p2) ? firstEdge.p1 : firstEdge.p2;

                var lastEdge = trianglePaths.Last().edge;
                var lastPoint = toPlane.GetDistance(lastEdge.p1) < toPlane.GetDistance(lastEdge.p2) ? lastEdge.p1 : lastEdge.p2;

                // var usedEdges = new Dictionary<Edge, bool>();

                var subResult1 = GetSubResult(true, firstPoint, lastPoint, points, fromPlane, triangleByPoint);
                var subResult2 = GetSubResult(false, firstPoint, lastPoint, points, fromPlane, triangleByPoint);
                // var subResult1 = GetSubResult(true, edgesByPoint, firstPoint, lastPoint, triangleByEdge);
                // var subResult2 = GetSubResult(false, edgesByPoint, firstPoint, lastPoint, triangleByEdge);

                var maxY1 = -100000.0f;
                foreach (var item in subResult1) {
                    maxY1 = Mathf.Max(item.surfacePosition.y);
                }

                var maxY2 = -100000.0f;
                foreach (var item in subResult2) {
                    maxY2 = Mathf.Max(item.surfacePosition.y);
                }

                var subResult = new List<Position>();
                if (maxY1 > maxY2) {
                    subResult = subResult1;
                }
                else {
                    subResult = subResult2;
                }

                if (subResult.Count == 0) {
                    // TODO Разобраться почему так может происходить.
                    continue;
                }

                // while (subResult.Last().surfacePosition != lastPoint) {
                //     var position = subResult.Last();
                //     var ok = false;
                //     foreach (var edge in edgesByPoint[PreparePoint(position.surfacePosition)]) {
                //         if (!usedEdges.ContainsKey(edge)) {
                //             var nextPoint = position.surfacePosition == edge.p1 ? edge.p2 : edge.p1;
                //             var N = triangleByEdge[edge].GetPlane().GetNormal();
                //             subResult.Add(new Position(position.surfacePosition + N, -N, position.surfacePosition));
                //             subResult.Add(new Position(nextPoint + N, -N, nextPoint));
                //             usedEdges.Add(edge, true);
                //             ok = true;
                //         }
                //     }
                //
                //     if (!ok) {
                //         break;
                //     }
                // }
                //
                {
                    var newDirection = new Point(0, -1, 0);
                    var first = subResult.First().surfacePosition;
                    var onPlane = first - sign * new Point(fromPlane.A, fromPlane.B, fromPlane.C).normalized * (float)fromPlane.GetDistance(first);
                    result.Add(new Position(onPlane - newDirection * paintHeight, newDirection, onPlane));
                }

                result.AddRange(subResult);

                {
                    var newDirection = new Point(0, -1, 0);
                    var last = subResult.Last().surfacePosition;
                    var onPlane = last + sign * new Point(toPlane.A, toPlane.B, toPlane.C).normalized * (float)toPlane.GetDistance(last);
                    result.Add(new Position(onPlane - newDirection * paintHeight, newDirection, onPlane));
                }

                sign *= -1;

                // 1. пофиксить этот алгоритм
                // 2. попробовать смоделировать качество покрытия объекта
                // 3. моделирование покрытие объекта
                break;
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

                foreach (var p in triangle.GetPoints()) {
                    var d = basePlane.GetDistance(p);
                    if (Math.Abs(d - d1) < 1e-3 && Math.Abs(d - d2) < 1e-3) {
                        newVertices.Add(p);
                    }
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

        void AddEdge(ref Dictionary<Point, List<Edge>> d, Point p, Edge e) {
            var p0 = PreparePoint(p);
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

        List<Position> GetSubResult(bool isForwardDirection, Point firstPoint, Point lastPoint, List<Point> points, Plane fromPlane, Dictionary<Point, Triangle> triangleByPoint) {

        // List<Position> GetSubResult(bool isForwardDirection, Dictionary<Point, List<Edge>> edgesByPoint, Point firstPoint, Point lastPoint, Dictionary<Edge, Triangle> triangleByEdge) {
            var subResult = new List<Position>();
            // var usedEdges = new Dictionary<Edge, bool>();

            // points.Sort((p1, p2) => fromPlane.GetDistance(p1) > fromPlane.GetDistance(p2) ? 1 : -1);
            // var pointToId = new Dictionary<Point, int>();
            // var id = 0;
            // foreach (var p in points) {
            //     ++id;
            //     pointToId.Add(p, id);
            // }

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

            // List<Point> forwardPoints = new List<Point>(), backwardPoints = new List<Point>();
            // foreach (var p in points) {
            //     var t = triangleByPoint[p];
            //     if (p == firstPoint) {
            //         forwardPoints.Add(p);
            //         backwardPoints.Add(p);
            //     }
            //
            //     if (t.GetPlane().GetNormal().y >= 0) {
            //         forwardPoints.Add(p);
            //     }
            //     else {
            //         backwardPoints.Add(p);
            //     }
            // }

            {
                // var edges = edgesByPoint[PreparePoint(firstPoint)];
                // Debug.Assert(edges.Count == 2);
                // if (edges.Count == 1) {
                //     Debug.Assert(false);
                //     return subResult; // Магия. Такого не должно быть, но оно есть :(
                // }
                // var e1 = edges[0];
                // var e2 = edges[1];

                // var p1 = e1.p1 == firstPoint ? e1.p2 : e1.p1;
                // var p2 = e2.p1 == firstPoint ? e2.p2 : e2.p1;
                // var p1 = nearestPointsToPoint[firstPoint].p1;
                // var p2 = nearestPointsToPoint[firstPoint].p2;
                // var p1 = forwardPoints.First();
                // var p2 = backwardPoints.First();
                //
                // List<Point> targetPoints;
                //
                // var nextPoint = p1;
                // // var targetEdge = e1;
                // if (isForwardDirection) {
                //     nextPoint = p2;
                //     targetPoints = forwardPoints;
                //     // targetEdge = e2;
                // }
                //
                // else {
                //     // targetEdge = e2;
                //     targetPoints = backwardPoints;
                //     nextPoint = p2;
                // }
                // // usedEdges.Add(targetEdge, true);
                //
                // for (var i = 0; i < targetPoints.Count - 1; ++i) {
                //     var triangle = triangleByPoint[targetPoints[i]];
                //     var n = triangle.GetPlane().GetNormal();
                //     subResult.Add(new Position(targetPoints[i] + n, -n, targetPoints[i]));
                //     subResult.Add(new Position(targetPoints[i + 1] + n, -n, targetPoints[i + 1]));
                // }

                // // var triangle = triangleByEdge[targetEdge];
                // var triangle = triangleByPoint[firstPoint];
                // var n = triangle.GetPlane().GetNormal();
                // subResult.Add(new Position(firstPoint + n, -n, firstPoint));
                //
                // // var targetPoint = targetEdge.p1 == firstPoint ? targetEdge.p2 : targetEdge.p1;
                // subResult.Add(new Position(nextPoint + n, -n, nextPoint));
            }

            // var usedPoints = new Dictionary<Point, bool>();
            // while (subResult.Last().surfacePosition != lastPoint) {
            //     var position = subResult.Last();
            //     var ok = false;
            //     foreach (var point in nearestPointsToPoint[position.surfacePosition].GetPoints()) {
            //         if (!usedPoints.ContainsKey(point)) {
            //             var N = triangleByPoint[point].GetPlane().GetNormal();
            //             subResult.Add(new Position(position.surfacePosition + N * paintHeight, -N, position.surfacePosition));
            //             subResult.Add(new Position(point + N * paintHeight, -N, point));
            //             usedPoints.Add(point, true);
            //             ok = true;
            //         }
            //     }
            //     // foreach (var edge in edgesByPoint[PreparePoint(position.surfacePosition)]) {
            //     //     if (!usedEdges.ContainsKey(edge)) {
            //     //         var nextPoint = position.surfacePosition == edge.p1 ? edge.p2 : edge.p1;
            //     //         var N = triangleByEdge[edge].GetPlane().GetNormal();
            //     //         subResult.Add(new Position(position.surfacePosition + N, -N, position.surfacePosition));
            //     //         subResult.Add(new Position(nextPoint + N, -N, nextPoint));
            //     //         usedEdges.Add(edge, true);
            //     //         ok = true;
            //     //     }
            //     // }
            //
            //     if (!ok) {
            //         break;
            //     }
            // }

            return subResult;
        }

        List<Point> GetSequence(Point firstPoint, List<Point> points, Dictionary<Point, Triangle> triangleByPoint) {
            var result = new List<Point>();
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

                result.Add(currentPoint);
            }

            for (var i = 0; i < result.Count - 1; ++i) {
                Debug.Assert(result[i] != result[i + 1]);
            }

            return result;
        }
    }
}
