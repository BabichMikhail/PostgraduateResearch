using System;

namespace Generic
{
    public static class MyMath {
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

        public static double Dot(Point p1, Point p2) {
            return p1.X * p2.X + p1.Y * p2.Y + p1.Z * p2.Z;
        }
    }
}
