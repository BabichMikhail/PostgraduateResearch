using System;

namespace Generic
{
    public class Point : IEquatable<Point>, IFormattable {
        public readonly float X;
        public readonly float Y;
        public readonly float Z;

        public Point(float ax, float ay, float az) {
            X = ax;
            Y = ay;
            Z = az;
        }

        public float Magnitude => (float)Math.Sqrt(SqrMagnitude);
        public float SqrMagnitude => X * X + Y * Y + Z * Z;
        public Point Normalized => new Point(X, Y, Z) / Magnitude;
        public static Point Zero => new Point(0, 0, 0);

        public static Point operator +(Point a, Point b) => new Point(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Point operator -(Point a, Point b) => new Point(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Point operator /(Point a, float b) => new Point(a.X / b, a.Y / b, a.Z / b);
        public static Point operator *(Point a, float b) => new Point(a.X * b, a.Y * b, a.Z * b);
        public static Point operator *(float a, Point b) => b * a;
        public static Point operator -(Point a) => new Point(-a.X, -a.Y, -a.Z);
        public static bool operator ==(Point a, Point b) {
            var num1 = a.X - b.X;
            var num2 = a.Y - b.Y;
            var num3 = a.Z - b.Z;
            return num1 * num1 + num2 * num2 + num3 * num3 < 9.99999943962493E-11;
        }

        public static bool operator !=(Point a, Point b) => !(a == b);
        public bool Equals(Point other) => this == other;
        public override int GetHashCode() => X.GetHashCode() ^ Y.GetHashCode() << 2 ^ Z.GetHashCode() >> 2;
        public override string ToString() => $"({X} {Y} {Z})";
        public string ToString(string format, IFormatProvider formatProvider) => ToString();
    }
}
