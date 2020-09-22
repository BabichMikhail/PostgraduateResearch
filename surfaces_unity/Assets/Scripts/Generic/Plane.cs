using UnityEngine;

namespace Generic
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

        public float GetDistance(Vector3 p) {
            return Mathf.Abs(A * p.x + B * p.y + C * p.z + D) / GetDenominator();
        }

        public float GetDenominator() {
            return Mathf.Sqrt(A * A + B * B + C * C);
        }
    }
}
