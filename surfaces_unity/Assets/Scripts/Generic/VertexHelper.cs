using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Generic
{
    public class VertexHelper {
        public static Dictionary<Point, List<Triangle>> GetTrianglesByVertex(List<Triangle> triangles) {
            var result = new Dictionary<Point, List<Triangle>>();
            foreach (var triangle in triangles) {
                var keys = new List<Point>{
                    triangle.P1,
                    triangle.P2,
                    triangle.P3,
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
                    new Edge(triangle.P1, triangle.P2),
                    new Edge(triangle.P1, triangle.P3),
                    new Edge(triangle.P2, triangle.P1),
                    new Edge(triangle.P2, triangle.P3),
                    new Edge(triangle.P3, triangle.P1),
                    new Edge(triangle.P3, triangle.P2),
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
                var subPlane1 = new Plane(t.P1 + step, t.P2 + step, t.P3 + step);
                // subTriangles.Add(subPlane1.GetRawTriangle());
            }

            return subTriangles;
        }

        public static List<Triangle> NormalizeTriangles(List<Triangle> triangles) {
            Triangle baseTriangle = null;
            for (var i = 0; i < triangles.Count; ++i) {
                var bt = triangles[i];

                {
                    var t = new Triangle(bt.P1, bt.P2, bt.P3);
                    var n = t.GetPlane().GetNormal();

                    var st = new Triangle(t.P1 + n, t.P2 + n, t.P3 + n);
                    var ok = true;
                    foreach (var tj in triangles) {
                        ok = (tj.P1 - t.P1).SqrMagnitude <= (tj.P1 - st.P1).SqrMagnitude
                             && (tj.P2 - t.P2).SqrMagnitude <= (tj.P2 - st.P2).SqrMagnitude
                             && (tj.P3 - t.P3).SqrMagnitude <= (tj.P3 - st.P3).SqrMagnitude;
                        if (!ok) {
                            break;
                        }
                    }

                    if (ok) {
                        baseTriangle = new Triangle(t.P1, t.P2, t.P3);
                        triangles[i] = baseTriangle;
                        break;
                    }
                }

                {
                    var t = new Triangle(bt.P3, bt.P1, bt.P2);
                    var n = t.GetPlane().GetNormal();

                    var st = new Triangle(t.P1 + n, t.P2 + n, t.P3 + n);
                    var ok = true;
                    foreach (var tj in triangles) {
                        ok = (tj.P1 - t.P1).SqrMagnitude <= (tj.P1 - st.P1).SqrMagnitude
                             && (tj.P2 - t.P2).SqrMagnitude <= (tj.P2 - st.P2).SqrMagnitude
                             && (tj.P3 - t.P3).SqrMagnitude <= (tj.P3 - st.P3).SqrMagnitude;
                        if (!ok) {
                            break;
                        }
                    }

                    if (ok) {
                        baseTriangle = new Triangle(t.P1, t.P2, t.P3);
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

                    var other = t.P1;
                    if (other == key.P1 || other == key.P2) {
                        other = t.P2;
                    }
                    if (other == key.P1 || other == key.P2) {
                        other = t.P3;
                    }
                    Debug.Assert(other != key.P1 && other != key.P2);

                    var tNorm = new Triangle(other, key.P2, key.P1);
                    result.Add(tNorm);

                    q.Enqueue(new Edge(tNorm.P1, tNorm.P2));
                    q.Enqueue(new Edge(tNorm.P2, tNorm.P3));
                    q.Enqueue(new Edge(tNorm.P3, tNorm.P1));

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
                var subPlane1 = new Plane(t.P1 + step, t.P2 + step, t.P3 + step);
                var subPlane2 = new Plane(t.P1 - step, t.P2 - step, t.P3 - step);
                // subTriangles.Add(subPlane1.GetRawTriangle());
                // subTriangles.Add(subPlane2.GetRawTriangle());
            }

            return subTriangles;
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
