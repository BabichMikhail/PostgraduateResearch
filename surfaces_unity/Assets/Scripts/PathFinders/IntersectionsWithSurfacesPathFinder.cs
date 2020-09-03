using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices.ComTypes;
using Generic;
using UnityEngine;

namespace PathFinders
{
    public class IntersectionsWithSurfacesPathFinder : IPathFinder {
        private class TrianglePath {
            public readonly VertexHelper.Triangle triangle;
            public readonly VertexHelper.Edge edge;

            public TrianglePath(VertexHelper.Triangle aTriangle, VertexHelper.Edge aEdge) {
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

        public List<Position> GetPath(ref List<VertexHelper.Triangle> triangles) {
            var basePlane = new VertexHelper.Plane(new Vector3(0, 0, 0), new Vector3(0, 1, 0), new Vector3(1, 0, 0));

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
                for (var d = Mathf.Ceil(minDistance / lineWidth) * lineWidth; d <= maxDistance; d += lineWidth) {
                    var vertices = new List<Vector3>();
                    foreach (var e in edges) {
                        var d1 = basePlane.GetDistance(e.p1);
                        var d2 = basePlane.GetDistance(e.p2);

                        var minD = Mathf.Min(d1, d2);
                        var maxD = Mathf.Max(d1, d2);
                        if (minD <= d && d <= maxD) {
                            vertices.Add(e.p1 + (e.p2 - e.p1) * ((d - d1) / (d2 - d1)));
                        }
                    }

                    Debug.Assert(vertices.Count == 2);
                    Debug.Assert(Mathf.Abs(basePlane.GetDistance(vertices[0]) - basePlane.GetDistance(vertices[1])) <= 10e-4);

                    var v1 = vertices[0];
                    var v2 = vertices[1];
                    if (v2.x < v1.x) {
                        var v = v1;
                        v1 = v2;
                        v2 = v;
                    }
                    pathParts.Add(new TrianglePath(t, new VertexHelper.Edge(v1, v2)));
                }
            }

            var pathPartsByDistance = new SortedDictionary<float, List<TrianglePath>>();
            foreach (var pp in pathParts) {
                var distance = Mathf.Ceil(basePlane.GetDistance(pp.edge.p1) * 10000) / 10000;
                if (pathPartsByDistance.ContainsKey(distance)) {
                    pathPartsByDistance[distance].Add(pp);
                }
                else {
                    pathPartsByDistance.Add(distance, new List<TrianglePath>{pp});
                }
            }

            var result = new List<Position>();
            foreach (var p in pathPartsByDistance) {
                var trianglePaths = p.Value.OrderBy(v => Mathf.Min(v.edge.p1.x, v.edge.p2.x)).ToList();
                var reverseTrianglePaths = new List<TrianglePath>();
                var subResult = new List<Position>();
                foreach (var tp in trianglePaths) {
                    var N = tp.triangle.GetPlane().GetNormal();
                    if (N.y < 0) {
                        reverseTrianglePaths.Add(tp);
                    }
                    else {
                        subResult.Add(new Position(tp.edge.p1 + N * paintHeight, -N, tp.edge.p1));
                        subResult.Add(new Position(tp.edge.p2 + N * paintHeight, -N, tp.edge.p2));
                    }
                }

                reverseTrianglePaths.Reverse();
                foreach (var tp in reverseTrianglePaths) {
                    var N = tp.triangle.GetPlane().GetNormal() * paintHeight;
                    subResult.Add(new Position(tp.edge.p2 + N * paintHeight, -N, tp.edge.p2));
                    subResult.Add(new Position(tp.edge.p1 + N * paintHeight, -N, tp.edge.p1));
                }

                var firstPosition = subResult.First();
                var firstOrigin = firstPosition.originPosition - new Vector3(-paintRadius, 0, 0);
                var firstDirection = new Vector3(0, -1, 0);
                result.Add(new Position(firstOrigin, firstDirection, firstOrigin + firstDirection * paintHeight));

                result.AddRange(subResult);
                result.Add(new Position(firstOrigin, firstDirection, firstOrigin + firstDirection * paintHeight));
            }

            return result;
        }
    }
}
