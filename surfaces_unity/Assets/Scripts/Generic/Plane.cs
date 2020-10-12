using System;

namespace Generic
{
    public struct Plane {
        public readonly float A;
        public readonly float B;
        public readonly float C;
        public float D;

        private readonly Point p1;
        private readonly Point p2;
        private readonly Point p3;

        public Plane(Point aP1, Point aP2, Point aP3) {
            p1 = aP1;
            p2 = aP2;
            p3 = aP3;

            A = (p2.Y - p1.Y) * (p3.Z - p1.Z) - (p2.Z - p1.Z) * (p3.Y - p1.Y);
            B = (p2.Z - p1.Z) * (p3.X - p1.X) - (p2.X - p1.X) * (p3.Z - p1.Z);
            C = (p2.X - p1.X) * (p3.Y - p1.Y) - (p2.Y - p1.Y) * (p3.X - p1.X);
            D = -(p1.X * A + p1.Y * B + p1.Z * C);
        }

        public Point GetNormal() {
            return new Point(A, B, C).Normalized;
        }

        public Point GetSomePoint() {
            return new Point(0, 0, -D / C);
        }

        public double GetDistance(Point p) {
            return Math.Abs((double)A * (double)p.X + (double)B * (double)p.Y + (double)C * (double)p.Z + (double)D) /
                Math.Sqrt((double)A * (double)A + (double)B * (double)B + (double)C * (double)C);
        }

        public double GetDenominator() {
            return Math.Sqrt((double)A * (double)A + (double)B * (double)B + (double)C * (double)C);
        }

        public override int GetHashCode() {
            return new Point(A, B, C).GetHashCode() + new Point(A, B, D).GetHashCode();
        }
    }
}
