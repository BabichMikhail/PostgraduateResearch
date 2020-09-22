using System.Collections.Generic;
using UnityEngine;

namespace Generic
{
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

        public List<Vector3> GetPoints() => new List<Vector3>{ p1, p2, p3 };

        public float GetMinX() => Mathf.Min(p1.x, Mathf.Min(p2.x, p3.x));

        public bool HasPoint(Vector3 p) =>
            Mathf.Abs(p1.sqrMagnitude - p.sqrMagnitude) < 1e-6 ||
            Mathf.Abs(p2.sqrMagnitude - p.sqrMagnitude) < 1e-6 ||
            Mathf.Abs(p3.sqrMagnitude - p.sqrMagnitude) < 1e-6;

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
}
