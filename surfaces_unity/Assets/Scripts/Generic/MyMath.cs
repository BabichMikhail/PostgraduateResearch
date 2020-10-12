using System;
using UnityEngine;

namespace Generic
{
    public static class MyMath {
        public static Vector3 Intersect(Plane plane, Line line) {
            var t = -(plane.A * line.X0 + plane.B * line.Y0 + plane.C * line.Z0 + plane.D) / (plane.A * line.P0X + plane.B * line.P0Y + plane.C * line.P0Z);
            return new Vector3(line.P0X * t + line.X0, line.P0Y * t + line.Y0, line.P0Z * t + line.Z0);
        }

        public static double GetDistance(Point p1, Point p2) {
            return (p1 - p2).Magnitude;
        }

        public static double GetDistance(Plane plane, Edge edge) {
            return Math.Min(plane.GetDistance(edge.P1), plane.GetDistance(edge.P2));
        }

        public static double GetDistance(Edge edge, Point point) {
            return Math.Min((edge.P1 - point).Magnitude, (edge.P2 - point).Magnitude);
        }

        // For disjoint edges only!
        public static double GetDistance(Edge edge1, Edge edge2) {
            return Math.Min(GetDistance(edge1, edge2.P1), GetDistance(edge1, edge2.P2));
        }

        public static double Dot(Point p1, Point p2) => p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
    }
}
