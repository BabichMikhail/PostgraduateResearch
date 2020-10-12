using System;
using System.Collections.Generic;

namespace Generic
{
    public class Triangle {
        public readonly Point P1;
        public readonly Point P2;
        public readonly Point P3;
        public readonly Point O;
        private readonly Point rawN;

        public readonly float L1;
        public readonly float L2;
        public readonly float L3;

        public Triangle(Point aP1, Point aP2, Point aP3) {
            P1 = aP1;
            P2 = aP2;
            P3 = aP3;

            L1 = (P2 - P3).Magnitude;
            L2 = (P1 - P3).Magnitude;
            L3 = (P1 - P2).Magnitude;

            O = new Point(
                (L3 * P3.X + L2 * P2.X + L1 * P1.X) / (L3 + L2 + L1),
                (L3 * P3.Y + L2 * P2.Y + L1 * P1.Y) / (L3 + L2 + L1),
                (L3 * P3.Z + L2 * P2.Z + L1 * P1.Z) / (L3 + L2 + L1)
            );
        }

        public Triangle(Point aP1, Point aP2, Point aP3, Point aRawN) : this(aP1, aP2, aP3) {
            rawN = aRawN;
        }

        public Point GetNormal() => Settings.GetInstance().UseRawNormals ? rawN : GetPlane().GetNormal();

        public Plane GetPlane() => new Plane(P1, P2, P3);

        public List<Point> GetPoints() => new List<Point>{ P1, P2, P3 };

        public List<Edge> GetEdges() => new List<Edge> {
            new Edge(P1, P2),
            new Edge(P1, P3),
            new Edge(P2, P3),
        };

        public float GetPerimeter() => L1 + L2 + L3;

        public float GetSemiPerimeter() => GetPerimeter() / 2;

        public float GetRadiusOfTheCircumscribedCircle() => L1 * L2 * L3 / 4 / GetSquare();

        public float GetSquare() {
            var p = GetSemiPerimeter();
            return (float)Math.Sqrt(p * (p - L1) * (p - L2) * (p - L3));
        }

        public static bool operator ==(Triangle a, Triangle b) {
            return a.GetPlane().GetNormal() == b.GetPlane().GetNormal() && (
                a.P1 == b.P1 && a.P2 == b.P2 && a.P3 == b.P3 ||
                a.P1 == b.P1 && a.P2 == b.P3 && a.P3 == b.P2 ||
                a.P1 == b.P2 && a.P2 == b.P1 && a.P3 == b.P3 ||
                a.P1 == b.P2 && a.P2 == b.P3 && a.P3 == b.P1 ||
                a.P1 == b.P3 && a.P2 == b.P1 && a.P3 == b.P2 ||
                a.P1 == b.P3 && a.P2 == b.P2 && a.P3 == b.P1
            );
        }

        public static bool operator !=(Triangle a, Triangle b) => !(a == b);
        public override int GetHashCode() => P1.GetHashCode() + P2.GetHashCode() + P3.GetHashCode();
    }
}
