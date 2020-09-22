using UnityEngine;

namespace Generic
{
    public class Point { // TODO use Point instead Vector3
        public float x;
        public float y;
        public float z;

        public Point(float ax, float ay, float az) {
            x = ax;
            y = ay;
            z = az;
        }

        public Point(Vector3 vertex) {
            x = vertex.x;
            y = vertex.y;
            z = vertex.z;
        }

        public float Magnitude() => Mathf.Sqrt(SqrMagnitude());

        public float SqrMagnitude() => x * x + y * y + z * z;

        public Point Normalized() => new Point(x, y, z) / Magnitude();

        public static Point operator+ (Point a, Point b) => new Point(a.x + b.x, a.y + b.y, a.z + b.z);
        public static Point operator -(Point a, Point b) => new Point(a.x - b.x, a.y - b.y, a.z - b.z);
        public static Point operator/ (Point a, float b) => new Point(a.x / b, a.y / b, a.z / b);
        public static Point operator*(Point a, float b) => new Point(a.x * b, a.y * b, a.z * b);
        public static Point operator -(Point a) => new Point(-a.x, -a.y, -a.z);

        public override bool Equals(object obj) {
            var other = (Point)obj;
            var d = 1e-4f;
            return other != null && Mathf.Abs(other.x - x) < d && Mathf.Abs(other.y - y) < d && Mathf.Abs(other.z - z) < d;
        }

        public Vector3 ToVector3() {
            return new Vector3(x, y, z);
        }

        public override int GetHashCode() => ToVector3().GetHashCode();
    }
}
