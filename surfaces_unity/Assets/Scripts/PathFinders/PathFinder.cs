using System.Collections.Generic;
using System.Linq;
using Generic;
using UnityEngine;

namespace PathFinders
{
    public class Position {
        public readonly Vector3 originPosition;
        public readonly Vector3 paintDirection;
        public readonly Vector3 surfacePosition;

        public Position(Vector3 aOriginPosition, Vector3 aPaintDirection, Vector3 aSurfacePosition) {
            originPosition = aOriginPosition;
            paintDirection = aPaintDirection;
            surfacePosition = aSurfacePosition;
        }
    }

    public class PathFinder {
        public float paintHeight;
        public float paintRadius;

        private static PathFinder instance = null;

        public static PathFinder GetInstance() {
            return instance ?? (instance = new PathFinder(0.2f, 0.3f));
        }

        private PathFinder(float aPaintRadius, float aPaintHeight) {
            paintHeight = aPaintHeight;
            paintRadius = aPaintRadius;
        }

        private void ProcessBigTriangleArea(ref List<Position> result, float triangleHeight, float remind, Vector3 p1, Vector3 p2, Vector3 p3, Vector3 N) {
            // var remind = r1 / 2;
            // var k = (t.l2 * t.l2 - t.l3 * t.l3 + t.l1 * t.l1) / (2 * t.l1 * t.l1);
            // var H = t.p3 + (t.p2 - t.p3) * k;
            // var tn = H - t.p1;
            for (var i = 0; i < Mathf.Floor(triangleHeight / paintRadius); ++i) {
                var step = paintRadius;
                if (i == 0) {
                    step -= remind;
                }

                var surfacePointA = p2 + (p1 - p2) * ((i + 1) * step - remind) / (p1 - p2).magnitude;
                result.Add(new Position(surfacePointA + N * paintHeight, -N, surfacePointA));

                var surfacePointB = p3 + (p1 - p3) * ((i + 1) * step - remind) / (p1 - p3).magnitude;
                result.Add(new Position(surfacePointB + N * paintHeight, -N, surfacePointB));
            }
        }

        private void ProcessTriangle(ref List<Position> result, VertexHelper.Triangle t) {
            var N = t.GetPlane().GetNormal();
            var R = t.GetRadiusOfTheCircumscribedCircle();
            if (R > paintRadius) {
                var s = t.GetSquare();
                var h1 = 2 * s / t.l1;
                var h2 = 2 * s / t.l2;
                var h3 = 2 * s / t.l3;

                var r1 = paintRadius * (1 + Mathf.Floor(h1 / paintRadius)) - h1;
                var r2 = paintRadius * (1 + Mathf.Floor(h2 / paintRadius)) - h2;
                var r3 = paintRadius * (1 + Mathf.Floor(h3 / paintRadius)) - h3;
                if (r1 > r2 && r1 > r3) {
                    ProcessBigTriangleArea(ref result, h1, r1 / 2, t.p1, t.p2, t.p3, N);
                    // var remind = r1 / 2;
                    // // var k = (t.l2 * t.l2 - t.l3 * t.l3 + t.l1 * t.l1) / (2 * t.l1 * t.l1);
                    // // var H = t.p3 + (t.p2 - t.p3) * k;
                    // // var tn = H - t.p1;
                    // for (var i = 0; i < Mathf.Floor(h1 / paintRadius); ++i) {
                    //     var step = paintRadius;
                    //     if (i == 0) {
                    //         step -= remind;
                    //     }
                    //
                    //     var surfacePointA = t.p2 + (t.p1 - t.p2) * ((i + 1) * step - remind);
                    //     result.Add(new Position(surfacePointA + N * paintHeight, -N, surfacePointA));
                    //
                    //     var surfacePointB = t.p3 + (t.p1 - t.p3) * ((i + 1) * step - remind);
                    //     result.Add(new Position(surfacePointB + N * paintHeight, -N, surfacePointB));
                    // }
                }
                else if (r2 > r3 && r2 > r1) {
                    ProcessBigTriangleArea(ref result, h2, r2 / 2, t.p2, t.p3, t.p1, N);
                }
                else { // r3 > r2 && r3 > r1;
                    ProcessBigTriangleArea(ref result, h3, r3 / 2, t.p3, t.p1, t.p2, N);
                }
            }
            else {
                result.Add(new Position(t.O + N * paintHeight, -N, t.O));
            }
        }

        public List<Position> GetPath(ref List<VertexHelper.Triangle> triangles) {
            var watch = System.Diagnostics.Stopwatch.StartNew();

            if (triangles.Count == 0) {
                return new List<Position>();
            }

            var trianglesByVertex = VertexHelper.GetTrianglesByVertex(triangles);
            var trianglesByEdge = VertexHelper.GetTrianglesByEdge(triangles);

            var processedTriangles = new Dictionary<VertexHelper.Triangle, bool>();
            var trianglesSequence = new List<VertexHelper.Triangle>();

            {
                var initialVertex = triangles[Random.Range(0, triangles.Count)].p1;
                var trianglesInVertex = trianglesByVertex[initialVertex];
                var initialTriangle = trianglesInVertex[Random.Range(0, trianglesInVertex.Count)];

                trianglesSequence.Add(initialTriangle);
                processedTriangles.Add(initialTriangle, true);

                while (true) {
                    var startCount = trianglesSequence.Count;
                    foreach (var edge in trianglesSequence.Last().GetEdges()) {
                        var ok = false;
                        if (edge.HasVertex(initialVertex)) {
                            foreach (var otherTriangle in trianglesByEdge[edge]) {
                                if (!processedTriangles.ContainsKey(otherTriangle)) {
                                    trianglesSequence.Add(otherTriangle);
                                    processedTriangles.Add(otherTriangle, true);
                                    ok = true;
                                    break;
                                }
                            }
                        }

                        if (ok) {
                            break;
                        }
                    }

                    if (startCount == trianglesSequence.Count) {
                        break;
                    }
                }
            }

            var newTriangleCount = trianglesSequence.Count;
            while (trianglesSequence.Count < triangles.Count && newTriangleCount > 0) {
                newTriangleCount = 0;

                while (true) {
                    var initialVertex = Vector3.zero;
                    foreach (var edge in trianglesSequence.Last().GetEdges()) {
                        var ok = false;
                        foreach (var t in trianglesByEdge[edge]) {
                            if (!processedTriangles.ContainsKey(t)) {
                                initialVertex = edge.p1;
                                trianglesSequence.Add(t);
                                processedTriangles.Add(t, true);
                                ok = true;
                                break;
                            }
                        }

                        if (ok) {
                            break;
                        }
                    }

                    if (initialVertex == Vector3.zero) {
                        var nearestTriangleIndex = -1;
                        var nearestVertexIndex = -1;
                        var nearestDistance = 0.0f;
                        for (var i = 0; i < triangles.Count; ++i) {
                            if (!processedTriangles.ContainsKey(triangles[i])) {
                                var vertices = triangles[i].GetVertices();
                                for (var j = 0; j < vertices.Count; ++j) {
                                    var hasProcessedTriangles = false;
                                    var hasUnprocessedTriangles = false;

                                    foreach (var t in trianglesByVertex[vertices[j]]) {
                                        hasProcessedTriangles = hasProcessedTriangles || processedTriangles.ContainsKey(t);
                                        hasUnprocessedTriangles = hasUnprocessedTriangles || !processedTriangles.ContainsKey(t);
                                    }

                                    if (hasProcessedTriangles && hasUnprocessedTriangles) {
                                        var distance = nearestTriangleIndex != -1
                                            ? (triangles[nearestTriangleIndex].GetVertices()[nearestVertexIndex] - vertices[j]).sqrMagnitude
                                            : 0;
                                        if (nearestVertexIndex == -1 || distance < nearestDistance) {
                                            nearestTriangleIndex = i;
                                            nearestVertexIndex = j;
                                            nearestDistance = distance;
                                        }
                                    }
                                }
                            }
                        }

                        if (nearestVertexIndex == -1) {
                            break;
                        }

                        initialVertex = triangles[nearestTriangleIndex].GetVertices()[nearestVertexIndex];
                        trianglesSequence.Add(triangles[nearestTriangleIndex]);
                        processedTriangles.Add(triangles[nearestTriangleIndex], true);
                    }

                    while (true) {
                        var startCount = trianglesSequence.Count;
                        foreach (var edge in trianglesSequence.Last().GetEdges()) {
                            if (edge.HasVertex(initialVertex)) {
                                foreach (var t in trianglesByEdge[edge]) {
                                    if (!processedTriangles.ContainsKey(t)) {
                                        trianglesSequence.Add(t);
                                        processedTriangles.Add(t, true);
                                        break;
                                    }
                                }
                            }
                        }

                        if (startCount == trianglesSequence.Count) {
                            break;
                        }
                    }
                }
            }

            watch.Stop();
            Debug.Log(watch.ElapsedMilliseconds + " ms. Time for calculation of triangles");
            watch.Restart();

            Debug.Log(triangles.Count);
            Debug.Log(trianglesSequence.Count);

            foreach (var t in triangles.Where(t => processedTriangles.ContainsKey(t))) {
                t.DebugColor = Color.red;
            }

            var result = new List<Position>();
            foreach (var t in trianglesSequence) {
                ProcessTriangle(ref result, t);
            }

            Debug.Log(result.Count);
            watch.Stop();
            Debug.Log(watch.ElapsedMilliseconds + " ms. Time for calculation of paths");
            
            return result;
        }
    }
}
