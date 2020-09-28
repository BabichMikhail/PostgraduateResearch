using System;
using UnityEngine;

namespace Generic
{
    public struct Plane {
        public float A;
        public float B;
        public float C;
        public float D;

        private Point p1;
        private Point p2;
        private Point p3;

        public Plane(Point aP1, Point aP2, Point aP3) {
            p1 = aP1;
            p2 = aP2;
            p3 = aP3;

            A = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
            B = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
            C = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
            D = -(p1.x * A + p1.y * B + p1.z * C);
        }

        public Point GetNormal() {
            return new Point(A, B, C).normalized;
        }

        // public Triangle GetRawTriangle() {
        //     return new Triangle(p1, p2, p3);
        // }

        public Point GetSomePoint() {
            return new Point(0, 0, -D / C);
        }

        public double GetDistance(Point p) {
            return Math.Abs((double)A * (double)p.x + (double)B * (double)p.y + (double)C * (double)p.z + (double)D) /
                   Math.Sqrt((double)A * (double)A + (double)B * (double)B + (double)C * (double)C);
            // return Math.Sqrt(Math.Pow((double)A * (double)p.x + (double)B * (double)p.y + (double)C * (double)p.z + (double)D, 2) /
            //        Math.Abs((double)A * (double)A + (double)B * (double)B + (double)C * (double)C));
        }

        public double GetDenominator() {
            return Math.Sqrt((double)A * (double)A + (double)B * (double)B + (double)C * (double)C);
        }

        public override int GetHashCode() => new Point(A, B, C).GetHashCode() + new Point(A, B, D).GetHashCode();
    }
}
