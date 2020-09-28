using System;
using System.Collections.Generic;
using UnityEngine;

namespace Generic
{
    public class VertexHelper {
        public static Dictionary<Point, List<Triangle>> GetTrianglesByVertex(List<Triangle> triangles) {
            var result = new Dictionary<Point, List<Triangle>>();
            foreach (var triangle in triangles) {
                var keys = new List<Point>{
                    triangle.p1,
                    triangle.p2,
                    triangle.p3,
                };

                foreach (var key in keys) {
                    if (result.ContainsKey(key)) {
                        result[key].Add(triangle);
                    }
                    else {
                        result.Add(key, new List<Triangle>{ triangle });
                    }
                }
            }

            return result;
        }

        public static Dictionary<Edge, List<Triangle>> GetTrianglesByEdge(List<Triangle> triangles) {
            var result = new Dictionary<Edge, List<Triangle>>();
            foreach (var triangle in triangles) {
                var keys = new List<Edge>{
                    new Edge(triangle.p1, triangle.p2),
                    new Edge(triangle.p1, triangle.p3),
                    new Edge(triangle.p2, triangle.p1),
                    new Edge(triangle.p2, triangle.p3),
                    new Edge(triangle.p3, triangle.p1),
                    new Edge(triangle.p3, triangle.p2),
                };

                foreach (var key in keys) {
                    if (result.ContainsKey(key)) {
                        result[key].Add(triangle);
                    }
                    else {
                        result.Add(key, new List<Triangle>{ triangle });
                    }
                }
            }

            return result;
        }

        public static List<Triangle> GetAllRawSubTriangles(List<Triangle> triangles, float h) {
            var subTriangles = new List<Triangle>();
            foreach (var t in triangles) {
                var plane = t.GetPlane();
                var step = plane.GetNormal() * h;
                var subPlane1 = new Plane(t.p1 + step, t.p2 + step, t.p3 + step);
                // subTriangles.Add(subPlane1.GetRawTriangle());
            }

            return subTriangles;
        }

        public static List<Triangle> NormalizeTriangles(List<Triangle> triangles) {
            Triangle baseTriangle = null;
            for (var i = 0; i < triangles.Count; ++i) {
                var bt = triangles[i];

                {
                    var t = new Triangle(bt.p1, bt.p2, bt.p3);
                    var n = t.GetPlane().GetNormal();

                    var st = new Triangle(t.p1 + n, t.p2 + n, t.p3 + n);
                    var ok = true;
                    foreach (var tj in triangles) {
                        ok = (tj.p1 - t.p1).sqrMagnitude <= (tj.p1 - st.p1).sqrMagnitude
                             && (tj.p2 - t.p2).sqrMagnitude <= (tj.p2 - st.p2).sqrMagnitude
                             && (tj.p3 - t.p3).sqrMagnitude <= (tj.p3 - st.p3).sqrMagnitude;
                        if (!ok) {
                            break;
                        }
                    }

                    if (ok) {
                        baseTriangle = new Triangle(t.p1, t.p2, t.p3);
                        triangles[i] = baseTriangle;
                        break;
                    }
                }

                {
                    var t = new Triangle(bt.p3, bt.p1, bt.p2);
                    var n = t.GetPlane().GetNormal();

                    var st = new Triangle(t.p1 + n, t.p2 + n, t.p3 + n);
                    var ok = true;
                    foreach (var tj in triangles) {
                        ok = (tj.p1 - t.p1).sqrMagnitude <= (tj.p1 - st.p1).sqrMagnitude
                             && (tj.p2 - t.p2).sqrMagnitude <= (tj.p2 - st.p2).sqrMagnitude
                             && (tj.p3 - t.p3).sqrMagnitude <= (tj.p3 - st.p3).sqrMagnitude;
                        if (!ok) {
                            break;
                        }
                    }

                    if (ok) {
                        baseTriangle = new Triangle(t.p1, t.p2, t.p3);
                        triangles[i] = baseTriangle;
                        break;
                    }
                }
            }

            Debug.Assert(baseTriangle != null);

            var trianglesByEdge = GetTrianglesByEdge(triangles);
            var result = new List<Triangle>{ baseTriangle };
            var processedTriangles = new Dictionary<Triangle, bool>{ [baseTriangle] = true };

            var q = new Queue<Edge>();
            foreach (var edge in baseTriangle.GetEdges()) {
                q.Enqueue(edge);
            }
            while (processedTriangles.Count != triangles.Count) {
                var key = q.Dequeue();

                foreach (var t in trianglesByEdge[key]) {
                    if (processedTriangles.ContainsKey(t)) {
                        continue;
                    }

                    var other = t.p1;
                    if (other == key.p1 || other == key.p2) {
                        other = t.p2;
                    }
                    if (other == key.p1 || other == key.p2) {
                        other = t.p3;
                    }
                    Debug.Assert(other != key.p1 && other != key.p2);

                    var tNorm = new Triangle(other, key.p2, key.p1);
                    result.Add(tNorm);

                    q.Enqueue(new Edge(tNorm.p1, tNorm.p2));
                    q.Enqueue(new Edge(tNorm.p2, tNorm.p3));
                    q.Enqueue(new Edge(tNorm.p3, tNorm.p1));

                    processedTriangles[t] = true;
                }
            }

            Debug.Assert(result.Count == triangles.Count);
            return result;
        }

        public static List<Triangle> GetAllMergedTriangles(List<Triangle> triangles, float h) {
            var subTriangles = new List<Triangle>();
            foreach (var t in triangles) {
                var plane = t.GetPlane();
                var step = plane.GetNormal() * h;
                var subPlane1 = new Plane(t.p1 + step, t.p2 + step, t.p3 + step);
                var subPlane2 = new Plane(t.p1 - step, t.p2 - step, t.p3 - step);
                // subTriangles.Add(subPlane1.GetRawTriangle());
                // subTriangles.Add(subPlane2.GetRawTriangle());
            }

            return subTriangles;
        }

        public static List<Triangle> MoveSurface(List<Triangle> triangles, float h) {
            var result = new List<Triangle>();

            var subTriangles = new List<Triangle>();
            foreach (var triangle in triangles) {
                subTriangles.Add(new Triangle(triangle.p1, triangle.p2, triangle.p3));
            }

            return result;
        }

        public static void RunMathTests() {
            {
                var p1 = new Point(1, 2, -2);
                var p2 = new Point(3, -2, 1);
                var p3 = new Point(5, 1, -4);

                var plane = new Plane(p1, p2, p3);
                Debug.Assert(Mathf.Abs(plane.A - 11) < 1e-3);
                Debug.Assert(Mathf.Abs(plane.B - 16) < 1e-3);
                Debug.Assert(Mathf.Abs(plane.C - 14) < 1e-3);
                Debug.Assert(Mathf.Abs(plane.D + 15) < 1e-3);
            }
        }
    }
}
