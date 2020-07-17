using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Security.Cryptography;
using TreeEditor;
using Unity.Profiling;
using UnityEditor;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

namespace Generic
{
    public class VertexHelper
    {
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
        }

        public class Triangle {
            public Vector3 p1;
            public Vector3 p2;
            public Vector3 p3;
            public Vector3 O;

            public Triangle(Vector3 aP1, Vector3 aP2, Vector3 aP3) {
                p1 = aP1;
                p2 = aP2;
                p3 = aP3;

                var AB = (p1 - p2).magnitude;
                var AC = (p1 - p3).magnitude;
                var BC = (p2 - p3).magnitude;
                O = new Vector3(
                    (AB * p3.x + AC * p2.x + BC * p1.x) / (AB + AC + BC),
                    (AB * p3.y + AC * p2.y + BC * p1.y) / (AB + AC + BC),
                    (AB * p3.z + AC * p2.z + BC * p1.z) / (AB + AC + BC)
                );
            }

            public Plane GetPlane() {
                return new Plane(p1, p2, p3);
            }

            public List<Vector3> GetPoints() {
                return new List<Vector3>{ p1, p2, p3 };
            }
        }

        public static List<Triangle> GetAllRawSubTriangles(List<Triangle> triangles, float h) {
            var subTriangles = new List<Triangle>();
            foreach (var t in triangles) {
                var plane = t.GetPlane();
                var step = plane.GetNormal() * h;
                var subPlane1 = new Plane(t.p1 + step, t.p2 + step, t.p3 + step);
                // var subPlane2 = new Plane(t.p1 - step, t.p2 - step, t.p3 - step);
                subTriangles.Add(subPlane1.GetRawTriangle());
                // subTriangles.Add(subPlane2.GetRawTriangle());
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

            var trianglesHash = new Dictionary<KeyValuePair<Vector3, Vector3>, List<Triangle>>();
            foreach (var triangle in triangles) {
                var keys = new List<KeyValuePair<Vector3, Vector3>>{
                    new KeyValuePair<Vector3, Vector3>(triangle.p1, triangle.p2),
                    new KeyValuePair<Vector3, Vector3>(triangle.p1, triangle.p3),
                    new KeyValuePair<Vector3, Vector3>(triangle.p2, triangle.p1),
                    new KeyValuePair<Vector3, Vector3>(triangle.p2, triangle.p3),
                    new KeyValuePair<Vector3, Vector3>(triangle.p3, triangle.p1),
                    new KeyValuePair<Vector3, Vector3>(triangle.p3, triangle.p2),
                };

                foreach (var key in keys) {
                    if (trianglesHash.ContainsKey(key)) {
                        trianglesHash[key].Add(triangle);
                    }
                    else {
                        trianglesHash.Add(key, new List<Triangle>{ triangle });
                    }
                }
            }

            var result = new List<Triangle>{ baseTriangle };
            var processedTriangles = new Dictionary<Triangle, bool>{ [baseTriangle] = true };

            var q = new Queue<KeyValuePair<Vector3, Vector3>>();
            q.Enqueue(new KeyValuePair<Vector3, Vector3>(baseTriangle.p1, baseTriangle.p2));
            q.Enqueue(new KeyValuePair<Vector3, Vector3>(baseTriangle.p2, baseTriangle.p3));
            q.Enqueue(new KeyValuePair<Vector3, Vector3>(baseTriangle.p3, baseTriangle.p1));
            while (processedTriangles.Count != triangles.Count) {
                var key = q.Dequeue();

                foreach (var t in trianglesHash[key]) {
                    if (processedTriangles.ContainsKey(t)) {
                        continue;
                    }

                    var other = t.p1;
                    if (other == key.Key || other == key.Value) {
                        other = t.p2;
                    }
                    if (other == key.Key || other == key.Value) {
                        other = t.p3;
                    }
                    Debug.Assert(other != key.Key && other != key.Value);

                    var tNorm = new Triangle(other, key.Value, key.Key);
                    result.Add(tNorm);

                    q.Enqueue(new KeyValuePair<Vector3, Vector3>(tNorm.p1, tNorm.p2));
                    q.Enqueue(new KeyValuePair<Vector3, Vector3>(tNorm.p2, tNorm.p3));
                    q.Enqueue(new KeyValuePair<Vector3, Vector3>(tNorm.p3, tNorm.p1));

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
