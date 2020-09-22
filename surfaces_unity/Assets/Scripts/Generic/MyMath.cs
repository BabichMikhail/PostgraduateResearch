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
    }
}
