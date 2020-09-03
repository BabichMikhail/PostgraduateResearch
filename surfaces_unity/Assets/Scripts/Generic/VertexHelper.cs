using System;
using System.Collections.Generic;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

namespace Generic
{
    public class VertexHelper {
        public class Plane {
            public float A;
            public float B;
            public float C;
            public float D;

            private Vector3 p1;
            private Vector3 p2;
            private Vector3 p3;

            public Plane(Vector3 aP1, Vector3 aP2, Vector3 aP3) {
                p1 = aP1;
                p2 = aP2;
                p3 = aP3;

                A = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
                B = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
                C = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
                D = -(p1.x * A + p1.y * B + p1.z * C);
            }

            public Vector3 GetNormal() {
                return new Vector3(A, B, C).normalized;
            }

            public Triangle GetRawTriangle() {
                return new Triangle(p1, p2, p3);
            }

            public float GetDistance(Vector3 p) {
                return Mathf.Sqrt(Mathf.Pow(A * p.x + B * p.y + C * p.z + D, 2) / (A * A + B * B + C * C));
            }
        }

        public class Edge {
            public readonly Vector3 p1;
            public readonly Vector3 p2;

            public Edge(Vector3 aP1, Vector3 aP2) {
                p1 = aP1;
                p2 = aP2;
            }

            public override int GetHashCode() => new KeyValuePair<Vector3, Vector3>(p1, p2).GetHashCode();

            public override bool Equals(object obj)
            {
                var other = (Edge)obj;
                return other != null && p1 == other.p1 && p2 == other.p2;
            }

            public bool HasVertex(Vector3 p) => p1 == p || p2 == p;
        }

        public class Triangle {
            public readonly Vector3 p1;
            public readonly Vector3 p2;
            public readonly Vector3 p3;
            public readonly Vector3 O;

            public readonly float l1;
            public readonly float l2;
            public readonly float l3;

            public Color DebugColor = Color.green;

            public Triangle(Vector3 aP1, Vector3 aP2, Vector3 aP3) {
                p1 = aP1;
                p2 = aP2;
                p3 = aP3;

                l1 = (p2 - p3).magnitude;
                l2 = (p1 - p3).magnitude;
                l3 = (p1 - p2).magnitude;

                O = new Vector3(
                    (l3 * p3.x + l2 * p2.x + l1 * p1.x) / (l3 + l2 + l1),
                    (l3 * p3.y + l2 * p2.y + l1 * p1.y) / (l3 + l2 + l1),
                    (l3 * p3.z + l2 * p2.z + l1 * p1.z) / (l3 + l2 + l1)
                );
            }

            public Plane GetPlane() => new Plane(p1, p2, p3);

            public List<Vector3> GetVertices() => new List<Vector3>{ p1, p2, p3 };

            public float GetMinX() => Mathf.Min(p1.x, Mathf.Min(p2.x, p3.x));

            public bool HasPoint(Vector3 p) =>
                Mathf.Abs(p1.sqrMagnitude - p.sqrMagnitude) < 0e-6 ||
                Mathf.Abs(p2.sqrMagnitude - p.sqrMagnitude) < 0e-6 ||
                Mathf.Abs(p3.sqrMagnitude - p.sqrMagnitude) < 0e-6;

            public List<Edge> GetEdges() => new List<Edge> {
                new Edge(p1, p2),
                new Edge(p1, p3),
                new Edge(p2, p3),
            };

            public float GetPerimeter() => l1 + l2 + l3;

            public float GetSemiPerimeter() => GetPerimeter() / 2;

            public float GetRadiusOfTheCircumscribedCircle() => l1 * l2 * l3 / 4 / GetSquare();

            public float GetSquare() {
                var p = GetSemiPerimeter();
                return Mathf.Sqrt(p * (p - l1) * (p - l2) * (p - l3));
            }

            public override bool Equals(object obj) {
                var other = (Triangle)obj;
                System.Diagnostics.Debug.Assert(other != null, nameof(other) + " != null");
                return p1 == other.p1 && p2 == other.p2 && p3 == other.p3;
            }
        }

        public static Dictionary<Vector3, List<Triangle>> GetTrianglesByVertex(List<Triangle> triangles) {
            var result = new Dictionary<Vector3, List<Triangle>>();
            foreach (var triangle in triangles) {
                var keys = new List<Vector3>{
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
                subTriangles.Add(subPlane1.GetRawTriangle());
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
                subTriangles.Add(subPlane1.GetRawTriangle());
                subTriangles.Add(subPlane2.GetRawTriangle());
            }

            return subTriangles;
        }

        public static List<List<Vector3>> MoveSurface(List<List<Vector3>> triangles, float h) {
            var result = new List<List<Vector3>>();

            var subTriangles = new List<Triangle>();
            foreach (var triangle in triangles) {
                subTriangles.Add(new Triangle(triangle[0], triangle[1], triangle[2]));
            }

            return result;
        }

        public static void RunMathTests() {
            {
                var p1 = new Vector3(1, 2, -2);
                var p2 = new Vector3(3, -2, 1);
                var p3 = new Vector3(5, 1, -4);

                var plane = new Plane(p1, p2, p3);
                Debug.Assert(Math.Abs(plane.A - 11) < 0.001);
                Debug.Assert(Math.Abs(plane.B - 16) < 0.001);
                Debug.Assert(Math.Abs(plane.C - 14) < 0.001);
                Debug.Assert(Math.Abs(plane.D + 15) < 0.001);
            }
        }
    }
}
