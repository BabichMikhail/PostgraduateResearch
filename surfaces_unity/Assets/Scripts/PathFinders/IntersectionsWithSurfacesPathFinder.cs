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

        public IntersectionsWithSurfacesPathFinder(float aPaintRadius, float aPaintHeight) {
            paintHeight = aPaintHeight;
            paintRadius = aPaintRadius;
        }

        public List<Position> GetPath(ref List<Triangle> triangles) {
            var a = new Vector3(1, 0, 0);
            var b = new Vector3(1, 1, 0);
            var c = new Vector3(1, 1, 1);

            var basePlane = new Plane(a, b, c);

            var a1 = new Vector3(a.x * Mathf.Cos(Mathf.PI / 2) - a.z * Mathf.Sin(Mathf.PI / 2), a.y, a.x * Mathf.Sin(Mathf.PI / 2) + a.z * Mathf.Cos(Mathf.PI / 2));
            var b1 = new Vector3(b.x * Mathf.Cos(Mathf.PI / 2) - b.z * Mathf.Sin(Mathf.PI / 2), b.y, b.x * Mathf.Sin(Mathf.PI / 2) + b.z * Mathf.Cos(Mathf.PI / 2));
            var c1 = new Vector3(c.x * Mathf.Cos(Mathf.PI / 2) - c.z * Mathf.Sin(Mathf.PI / 2), c.y, c.x * Mathf.Sin(Mathf.PI / 2) + c.z * Mathf.Cos(Mathf.PI / 2));
            var plane1 = new Plane(a1, b1, c1);
            var plane2 = new Plane(a1, b1, c1);

            var maxPlaneD = 0.0f;
            foreach (var t in triangles) {
                foreach (var p in t.GetPoints()) {
                    maxPlaneD = Mathf.Max(p.magnitude, maxPlaneD);
                }
            }

            var basePlaneD = 2 * maxPlaneD * basePlane.GetDenominator();
            var plane1D = 2 * maxPlaneD * plane1.GetDenominator();
            var plane2D = 2 * maxPlaneD * plane2.GetDenominator();

            basePlane.D = basePlaneD;
            plane1.D = plane1D;
            plane2.D = -plane2D;

            foreach (var t in triangles) {
                foreach (var p in t.GetPoints()) {
                    basePlaneD = Mathf.Min(basePlane.GetDistance(p), basePlaneD);
                    plane1D = Mathf.Min(plane1.GetDistance(p), plane1D);
                    plane2D = Mathf.Min(plane2.GetDistance(p), plane2D);
                }
            }

            basePlane.D -= (basePlaneD - 10) * basePlane.GetDenominator();
            plane1.D -= (plane1D - 10) * plane1.GetDenominator();
            plane2.D += (plane2D - 10) * plane2.GetDenominator();

            var baseDistance = 1000000.0f;
            var distance1 = 1000000.0f;
            var distance2 = 1000000.0f;
            foreach (var t in triangles) {
                foreach (var p in t.GetPoints()) {
                    // For debug.
                    baseDistance = basePlane.GetDistance(p);
                    distance1 = plane1.GetDistance(p);
                    distance2 = plane2.GetDistance(p);
                }
            }

            var lineWidth = paintRadius;
            var pathParts = new List<TrianglePath>();
            foreach (var t in triangles) {
                var minDistance = basePlane.GetDistance(t.p1);
                var maxDistance = basePlane.GetDistance(t.p1);

                foreach (var v in new List<Vector3>{t.p2, t.p3}) {
                    var distance = basePlane.GetDistance(v);
                    minDistance = Mathf.Min(minDistance, distance);
                    maxDistance = Mathf.Max(maxDistance, distance);
                }

                var edges = t.GetEdges();
                for (var distance = Mathf.Ceil(minDistance / lineWidth) * lineWidth; distance <= maxDistance; distance += lineWidth) {
                    var vertices = new List<Vector3>();
                    var firstEdge = edges.First();
                    var minTDistance = basePlane.GetDistance(firstEdge.p1);
                    var maxTDistance = basePlane.GetDistance(firstEdge.p1);

                    foreach (var e in edges) {
                        foreach (var p in e.GetPoints()) {
                            var d = basePlane.GetDistance(p);
                            maxTDistance = Mathf.Max(d, maxTDistance);
                            minTDistance = Mathf.Min(d, minTDistance);
                        }
                    }

                    if (maxTDistance - minTDistance > 1e-4) {
                        foreach (var e in edges) {
                            var d1 = basePlane.GetDistance(e.p1);
                            var d2 = basePlane.GetDistance(e.p2);

                            var minD = Mathf.Min(d1, d2);
                            var maxD = Mathf.Max(d1, d2);
                            if (maxD - minD > 1e-8 && minD - distance <= 0.0 && distance - maxD <= 0.0) {
                                vertices.Add(e.p1 + (e.p2 - e.p1) * (float)((distance - d1) / (d2 - d1)));
                            }
                        }
                        var dict = new Dictionary<Vector3, bool>();
                        foreach (var v in vertices) {
                            if (!dict.ContainsKey(v)) {
                                dict.Add(v, true);
                            }
                        }
                        vertices = dict.Keys.ToList();

                        if (vertices.Count > 2) {
                            Debug.Assert(false);
                        }

                        if (vertices.Count == 2) {
                            foreach (var e in edges) {
                                var d1 = basePlane.GetDistance(e.p1);
                                var d2 = basePlane.GetDistance(e.p2);

                                var minD = Mathf.Min(d1, d2);
                                var maxD = Mathf.Max(d1, d2);
                                if (vertices.Count != 2 && maxD - minD > 1e-8 && minD - distance <= 0.0 && distance - maxD <= 0.0) {
                                    Debug.Assert(false);
                                    // vertices.Add(e.p1 + (e.p2 - e.p1) * ((distance - d1) / (d2 - d1)));
                                }
                            }

                            Debug.Assert(vertices.Count == 2);
                            Debug.Assert(Mathf.Abs(basePlane.GetDistance(vertices[0]) - basePlane.GetDistance(vertices[1])) <= 1e-4f);

                            var v1 = vertices[0];
                            var v2 = vertices[1];
                            if (v2.x < v1.x) {
                                var v = v1;
                                v1 = v2;
                                v2 = v;
                            }

                            if ((v1 - v2).sqrMagnitude  > 1e-4) {
                                pathParts.Add(new TrianglePath(t, new Edge(v1, v2)));
                            }
                        }
                    }
                }
            }

            var pathPartsByDistance = new SortedDictionary<double, List<TrianglePath>>();
            foreach (var pp in pathParts) {
                var distance = Math.Ceiling(basePlane.GetDistance(pp.edge.p1) * 1e4) / 1e4;
                if (pathPartsByDistance.ContainsKey(distance)) {
                    pathPartsByDistance[distance].Add(pp);
                }
                else {
                    pathPartsByDistance.Add(distance, new List<TrianglePath>{pp});
                }
            }

            var sign = 1;
            var result = new List<Position>();
            foreach (var p in pathPartsByDistance) {
                var edgesByPoint = new Dictionary<Vector3, List<Edge>>();
                var triangleByEdge = new Dictionary<Edge, Triangle>();
                foreach (var tp in p.Value) {
                    AddEdge(ref edgesByPoint, tp.edge.p1, tp.edge);
                    AddEdge(ref edgesByPoint, tp.edge.p2, tp.edge);

                    AddTriangle(ref triangleByEdge, tp.edge, tp.triangle);
                }

                var fromPlane = sign > 0 ? plane1 : plane2;
                var toPlane = sign > 0 ? plane2 : plane1;
                var trianglePaths = p.Value.OrderBy(v => Mathf.Min(fromPlane.GetDistance(v.edge.p1), fromPlane.GetDistance(v.edge.p2)));

                var firstEdge = trianglePaths.First().edge;
                var firstPoint = fromPlane.GetDistance(firstEdge.p1) < fromPlane.GetDistance(firstEdge.p2) ? firstEdge.p1 : firstEdge.p2;

                var lastEdge = trianglePaths.Last().edge;
                var lastPoint = toPlane.GetDistance(lastEdge.p1) < toPlane.GetDistance(lastEdge.p2) ? lastEdge.p1 : lastEdge.p2;

                var usedEdges = new Dictionary<Edge, bool>();
                var subResult = new List<Position>();

                var subResult1 = GetSubResult(true, edgesByPoint, firstPoint, lastPoint, triangleByEdge);
                var subResult2 = GetSubResult(false, edgesByPoint, firstPoint, lastPoint, triangleByEdge);

                var maxY1 = -100000.0f;
                foreach (var item in subResult1) {
                    maxY1 = Mathf.Max(item.surfacePosition.y);
                }

                var maxY2 = -100000.0f;
                foreach (var item in subResult2) {
                    maxY2 = Mathf.Max(item.surfacePosition.y);
                }

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
                // {
                //     var edges = edgesByPoint[PreparePoint(firstPoint)];
                //     Debug.Assert(edges.Count == 2);
                //     if (edges.Count == 1) {
                //         continue; // Магия. Такого не должно быть, но оно есть :(
                //     }
                //     var e1 = edges[0];
                //     var e2 = edges[1];
                //
                //     var p1 = e1.p1 == firstPoint ? e1.p2 : e1.p1;
                //     var p2 = e2.p1 == firstPoint ? e2.p2 : e2.p1;
                //
                //     var targetEdge = e1;
                //     if (p2.y > p1.y) {
                //         targetEdge = e2;
                //     }
                //     usedEdges.Add(targetEdge, true);
                //
                //     var triangle = triangleByEdge[targetEdge];
                //     var N = triangle.GetPlane().GetNormal();
                //     subResult.Add(new Position(firstPoint + N, -N, firstPoint));
                //
                //     var secondPoint = targetEdge.p1 == firstPoint ? targetEdge.p2 : targetEdge.p1;
                //     subResult.Add(new Position(secondPoint + N, -N, secondPoint));
                // }

                while (subResult.Last().surfacePosition != lastPoint) {
                    var position = subResult.Last();
                    var ok = false;
                    foreach (var edge in edgesByPoint[PreparePoint(position.surfacePosition)]) {
                        if (!usedEdges.ContainsKey(edge)) {
                            var nextPoint = position.surfacePosition == edge.p1 ? edge.p2 : edge.p1;
                            var N = triangleByEdge[edge].GetPlane().GetNormal();
                            subResult.Add(new Position(position.surfacePosition + N, -N, position.surfacePosition));
                            subResult.Add(new Position(nextPoint + N, -N, nextPoint));
                            usedEdges.Add(edge, true);
                            ok = true;
                        }
                    }

                    if (!ok) {
                        break;
                    }
                }

                {
                    // var firstPosition = subResult[0];
                    // var secondPosition = subResult[1];
                    // // var firstOrigin = firstPosition.originPosition - new Vector3(0, 0, paintRadius);
                    // // var firstDirection = new Vector3(0, -1, 0);
                    // // result.Add(new Position(firstOrigin, firstDirection, firstOrigin + firstDirection * paintHeight));
                    //
                    // var newOriginDirection = firstPosition.originPosition - secondPosition.originPosition;
                    // var newSurfaceDirection = firstPosition.surfacePosition - secondPosition.surfacePosition;
                    // newOriginDirection.y = 0;
                    // newSurfaceDirection.y = 0;
                    //
                    // var newOrigin = MyMath.Intersect(plane1, new Line(firstPosition.originPosition, secondPosition.originPosition));
                    // var newSurface = MyMath.Intersect(plane1, new Line(firstPosition.surfacePosition, secondPosition.surfacePosition));
                    // var newDirection = new Vector3(0, -1, 0);
                    // // var newOrigin = firstPosition.originPosition + newOriginDirection.normalized * 10;
                    // // var newSurface = firstPosition.surfacePosition + newSurfaceDirection.normalized * 10;
                    // result.Add(new Position(newOrigin, newDirection, newOrigin + newDirection));

                    var newDirection = new Vector3(0, -1, 0);
                    var first = subResult.First().surfacePosition;
                    var onPlane = first - sign * new Vector3(fromPlane.A, fromPlane.B, fromPlane.C).normalized * fromPlane.GetDistance(first);
                    result.Add(new Position(onPlane - newDirection * paintHeight, newDirection, onPlane));
                }

                result.AddRange(subResult);

                {
                    // var lastPosition = subResult[subResult.Count - 1];
                    // var predLastPosition = subResult[subResult.Count - 2];
                    // // var lastOrigin = lastPosition.originPosition + new Vector3(0, 0, paintRadius);
                    // // var lastDirection = new Vector3(0, -1, 0);
                    // // result.Add(new Position(lastOrigin, lastDirection, lastOrigin + lastDirection * paintHeight));
                    //
                    // var newOriginDirection = lastPosition.originPosition - predLastPosition.originPosition;
                    // var newSurfaceDirection = lastPosition.surfacePosition - predLastPosition.surfacePosition;
                    // newOriginDirection.y = 0;
                    // newSurfaceDirection.y = 0;
                    //
                    // var newOrigin = MyMath.Intersect(plane2, new Line(lastPosition.originPosition, predLastPosition.originPosition));
                    // var newSurface = MyMath.Intersect(plane2, new Line(lastPosition.surfacePosition, predLastPosition.surfacePosition));
                    // var newDirection = new Vector3(0, -1, 0);
                    // // var newOrigin = lastPosition.originPosition + newOriginDirection.normalized * 10;
                    // // var newSurface = lastPosition.surfacePosition + newSurfaceDirection.normalized * 10;
                    // result.Add(new Position(newOrigin, newDirection, newOrigin + newDirection));

                    var newDirection = new Vector3(0, -1, 0);
                    var last = subResult.Last().surfacePosition;
                    var onPlane = last + sign * new Vector3(toPlane.A, toPlane.B, toPlane.C).normalized * toPlane.GetDistance(last);
                    result.Add(new Position(onPlane - newDirection * paintHeight, newDirection, onPlane));
                }

                sign *= -1;
                // break;
                ////////////////////

                // 1. пофиксить этот алгоритм
                // 2. попробовать смоделировать качество покрытия объекта
                // 3. моделирование покрытие объекта

                // var trianglePaths = p.Value.OrderBy(v => Mathf.Min(v.edge.p1.x, v.edge.p2.x)).ToList();
                // var reverseTrianglePaths = new List<TrianglePath>();
                // var subResult = new List<Position>();
                // foreach (var tp in trianglePaths) {
                //     var N = tp.triangle.GetPlane().GetNormal() * paintHeight;
                //     if (N.y < 0) { // BUG!
                //         reverseTrianglePaths.Add(tp);
                //     }
                //     else {
                //         subResult.Add(new Position(tp.edge.p1 + N, -N, tp.edge.p1));
                //         subResult.Add(new Position(tp.edge.p2 + N, -N, tp.edge.p2));
                //     }
                // }
                //
                // reverseTrianglePaths.Reverse();
                // foreach (var tp in reverseTrianglePaths) {
                //     var N = tp.triangle.GetPlane().GetNormal() * paintHeight;
                //     subResult.Add(new Position(tp.edge.p2 + N, -N, tp.edge.p2));
                //     subResult.Add(new Position(tp.edge.p1 + N, -N, tp.edge.p1));
                // }
                //
                // var firstPosition = subResult.First();
                // var firstOrigin = firstPosition.originPosition - new Vector3(paintRadius, 0, 0);
                // var firstDirection = new Vector3(0, -1, 0);
                // result.Add(new Position(firstOrigin, firstDirection, firstOrigin + firstDirection * paintHeight));
                //
                // result.AddRange(subResult);
                // result.Add(new Position(firstOrigin, firstDirection, firstOrigin + firstDirection * paintHeight));
            }

            return result;
        }

        Vector3 PreparePoint(Vector3 v) {
            var precision = 1e2f;
            return new Vector3(Mathf.Round(v.x * precision) / precision, Mathf.Round(v.y * precision) / precision, Mathf.Round(v.z * precision) / precision);
        }

        void AddEdge(ref Dictionary<Vector3, List<Edge>> d, Vector3 v, Edge e) {
            var v0 = PreparePoint(v);
            if (d.ContainsKey(v0)) {
                d[v0].Add(e);
            }
            else {
                d.Add(v0, new List<Edge>{e});
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

        List<Edge> GetSequences() {
            return new List<Edge>();
        }

        List<Position> GetSubResult(bool isForwardDirection, Dictionary<Vector3, List<Edge>> edgesByPoint, Vector3 firstPoint, Vector3 lastPoint, Dictionary<Edge, Triangle> triangleByEdge) {
            var subResult = new List<Position>();
            var usedEdges = new Dictionary<Edge, bool>();

            {
                var edges = edgesByPoint[PreparePoint(firstPoint)];
                Debug.Assert(edges.Count == 2);
                if (edges.Count == 1) {
                    return subResult; // Магия. Такого не должно быть, но оно есть :(
                }
                var e1 = edges[0];
                var e2 = edges[1];

                var p1 = e1.p1 == firstPoint ? e1.p2 : e1.p1;
                var p2 = e2.p1 == firstPoint ? e2.p2 : e2.p1;

                var targetEdge = e1;
                if (isForwardDirection && p2.y > p1.y) {
                    targetEdge = e2;
                }

                if (!isForwardDirection && p2.y <= p1.y) {
                    targetEdge = e2;
                }
                usedEdges.Add(targetEdge, true);

                var triangle = triangleByEdge[targetEdge];
                var N = triangle.GetPlane().GetNormal();
                subResult.Add(new Position(firstPoint + N, -N, firstPoint));

                var secondPoint = targetEdge.p1 == firstPoint ? targetEdge.p2 : targetEdge.p1;
                subResult.Add(new Position(secondPoint + N, -N, secondPoint));
            }

            while (subResult.Last().surfacePosition != lastPoint) {
                var position = subResult.Last();
                var ok = false;
                foreach (var edge in edgesByPoint[PreparePoint(position.surfacePosition)]) {
                    if (!usedEdges.ContainsKey(edge)) {
                        var nextPoint = position.surfacePosition == edge.p1 ? edge.p2 : edge.p1;
                        var N = triangleByEdge[edge].GetPlane().GetNormal();
                        subResult.Add(new Position(position.surfacePosition + N, -N, position.surfacePosition));
                        subResult.Add(new Position(nextPoint + N, -N, nextPoint));
                        usedEdges.Add(edge, true);
                        ok = true;
                    }
                }

                if (!ok) {
                    break;
                }
            }

            return subResult;
        }
    }
}
