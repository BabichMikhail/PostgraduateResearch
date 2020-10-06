using System;
using UnityEngine;

namespace Generic
{
    public class MyMath
    {
        public static Vector3 Intersect(Plane plane, Line line) {
            var t = -(plane.A * line.x0 + plane.B * line.y0 + plane.C * line.z0 + plane.D) / (plane.A * line.p0x + plane.B * line.p0y + plane.C * line.p0z);
            return new Vector3(line.p0x * t + line.x0, line.p0y * t + line.y0, line.p0z * t + line.z0);
        }

        public static double GetDistance(Plane plane, Edge edge) {
            return Math.Min(plane.GetDistance(edge.p1), plane.GetDistance(edge.p2));
        }

        // For disjoint edges only!
        public static double GetDistance(Edge edge1, Edge edge2) {
            return Math.Min(
                Math.Min((edge1.p1 - edge2.p1).magnitude, (edge1.p1 - edge2.p2).magnitude),
                Math.Min((edge1.p2 - edge2.p1).magnitude, (edge1.p2 - edge2.p2).magnitude)
            );
        }

        public static bool isEqual(float a, float b) => Math.Abs(a - b) < 1e-6;
    }
}
