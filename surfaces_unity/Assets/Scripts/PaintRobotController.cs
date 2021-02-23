using System;
using System.Collections.Generic;
using System.Linq;
using Library.Generic;
using UnityEditor;
using UnityEngine;
using Plane = Library.Generic.Plane;
using Random = UnityEngine.Random;

public class PaintRobotController : MonoBehaviour {
    private GameObject canvas;
    private float maxSpeed;
    private float speed;
    private float maxAcceleration;
    private float acceleration;

    private void Start() {
        canvas = GameObject.Find("Canvas");
        Debug.Assert(canvas);
    }

    public void SetMaxSpeed(float aSpeed) {
        maxSpeed = aSpeed;
        UpdateColor();
    }

    public void SetCurrentSpeed(float aSpeed) {
        speed = aSpeed;
        UpdateColor();
    }

    public void SetMaxAcceleration(float aAcceleration) {
        maxAcceleration = aAcceleration;
        UpdateColor();
    }

    public void SetCurrentAcceleration(float aAcceleration) {
        acceleration = aAcceleration;
        UpdateColor();
    }

    private void UpdateColor() {
        var r = gameObject.GetComponentInChildren<Renderer>();
        r.material.color = (GetSpeedColor(speed, maxSpeed) + GetAccelerationColor(acceleration, maxAcceleration)) / 2.0f;
    }

    public static Color GetAccelerationColor(float acceleration, float maxAcceleration) {
        var color = Color.white;
        if (acceleration <= 0.75f * maxAcceleration) {
            color = Color.green;
        }
        else if (acceleration <= maxAcceleration) {
            color = Color.blue;
        }
        else if (acceleration <= 1.25f * maxAcceleration) {
            color = Color.magenta;
        }
        else if (acceleration <= 1.5f * maxAcceleration) {
            color = Color.red;
        }
        else {
            color = (Color.black + Color.red) / 2.0f;
        }

        return color;
    }

    public static Color GetSpeedColor(float speed, float maxSpeed) {
        var color = Color.white;
        if (speed <= 0.95f * maxSpeed) {
            color = Color.green;
        }
        else if (speed <= maxSpeed) {
            color = Color.blue;
        }
        else if (speed <= 1.25f * maxSpeed) {
            color = Color.magenta;
        }
        else if (speed <= 1.5f * maxSpeed) {
            color = Color.red;
        }
        else {
            color = (Color.black + Color.red) / 2.0f;
        }

        return color;
    }

    private float paintHeight = 1.0f;
    private float paintRadius = 0.0f;
    private int pointsPerSecond = 0;
    private const float pointRadius = 0.1f;

    private List<Triangle> objectTriangles = new List<Triangle>();

    public void SetPaintHeight(float height) {
        paintHeight = height;
    }

    public void SetPaintRadius(float radius) {
        paintRadius = radius;
    }

    public void SetPointGenerationSpeed(int aPointsPerSecond) {
        pointsPerSecond = aPointsPerSecond;
    }

    public void ResetGeneratedPoints() {
        trianglePoints.Clear();
    }

    public void SetObjectTriangles(List<Triangle> aObjectTriangles) {
        objectTriangles = aObjectTriangles;
    }

    private int firstNewIndex = 0;
    private readonly List<Vector3> trianglePoints = new List<Vector3>();
    private readonly List<Position> drawPositions = new List<Position>();

    private Position GetPaintPosition() {
        var t = transform;
        var p = t.position;
        var f = t.forward;
        return new Position(Utils.VtoP(p), Utils.VtoP(f), Utils.VtoP(p + f * paintHeight), Position.PositionType.Middle);
    }

    public static float NextGaussian() {
        float v1, v2, s;
        do {
            v1 = 2.0f * Random.Range(0.0f, 1.0f) - 1.0f;
            v2 = 2.0f * Random.Range(0.0f, 1.0f) - 1.0f;
            s = v1 * v1 + v2 * v2;
        } while (s >= 1.0f || s == 0.0f);

        s = Mathf.Sqrt(-2.0f * Mathf.Log(s) / s);

        return v1 * s;
    }

    private static float NextGaussian(float mean, float standardDeviation) {
        return mean + NextGaussian() * standardDeviation;
    }

    private Point TransformPoint(Point n1, Point n2, Point n3, Point point) {
        var x = n1.x * point.x + n2.x * point.y + n3.x * point.z;
        var y = n1.y * point.x + n2.y * point.y + n3.y * point.z;
        var z = n1.z * point.x + n2.z * point.y + n3.z * point.z;

        return new Point(x, y, z);
    }

    private Position TryGetPositionOnSurface(Position drawPosition) {
        var candidates = new List<Position>();

        var line = new Line(drawPosition.originPoint, drawPosition.surfacePoint);
        foreach (var t in objectTriangles) {
            var plane = t.GetPlane();

            var a11 = plane.a;
            var a12 = plane.b;
            var a13 = plane.c;
            var a21 = line.p0Y;
            var a22 = -line.p0X;
            var a23 = 0;
            var a31 = line.p0Z;
            var a32 = 0;
            var a33 = -line.p0X;

            var b1 = -plane.d;
            var b2 = line.p0Y * line.x0 - line.p0X * line.y0;
            var b3 = line.p0Z * line.x0 - line.p0X * line.z0;

            var d = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{a11, a12, a13}, new List<double>{a21, a22, a23}, new List<double>{a31, a32, a33}}
            );
            var d1 = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{b1, a12, a13}, new List<double>{b2, a22, a23}, new List<double>{b3, a32, a33}}
            );
            var d2 = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{a11, b1, a13}, new List<double>{a21, b2, a23}, new List<double>{a31, b3, a33}}
            );
            var d3 = MMath.GetDeterminant(new List<List<double>>
                {new List<double>{a11, a12, b1}, new List<double>{a21, a22, b2}, new List<double>{a31, a32, b3}}
            );

            var point = new Point((float) (d1 / d), (float)(d2 / d), (float)(d3 / d));

            var t1 = new Triangle(t.p1, t.p2, point);
            var t2 = new Triangle(t.p1, point, t.p3);
            var t3 = new Triangle(point, t.p2, t.p3);

            if (Math.Abs(t1.GetSquare() + t2.GetSquare() + t3.GetSquare() - t.GetSquare()) < 1e-1) {
                candidates.Add(new Position(drawPosition.originPoint, drawPosition.paintDirection, point, drawPosition.type));
            }
        }

        candidates.Sort(delegate(Position a, Position b) {
            var d1 = MMath.GetDistance(a.originPoint, a.surfacePoint);
            var d2 = MMath.GetDistance(b.originPoint, b.surfacePoint);
            if (d1 > d2) return 1;
            if (d1 < d2) return -1;
            return 0;
        });

        return candidates.Count > 0 ? candidates.First() : null;
    }

    private void FixedUpdate() {
        var count = Mathf.RoundToInt(pointsPerSecond * Time.deltaTime);

        var paintPosition = GetPaintPosition();

        var mean = Utils.PtoV(paintPosition.surfacePoint);
        var p = new Plane(paintPosition.paintDirection, paintPosition.surfacePoint);
        var n1 = (paintPosition.surfacePoint - p.GetSomePoint()).Normalized;
        var n2 = -paintPosition.paintDirection.Normalized;
        var n3 = new Point(n1.y * n2.z - n1.z * n2.y, n1.z * n2.x - n1.x * n2.z, n1.x * n2.y - n1.y * n2.x).Normalized;
        var standardDeviation = paintRadius / 3;
        for (var i = 0; i < count; ++i) {
            var point = new Vector3(NextGaussian(0, standardDeviation), 0, NextGaussian(0, standardDeviation));

            var drawDirection = (Utils.VtoP(mean + point) - paintPosition.originPoint).Normalized;
            var drawPosition = new Position(paintPosition.originPoint, drawDirection, Utils.VtoP(mean + point), Position.PositionType.Middle);
            var positionOnSurface = TryGetPositionOnSurface(drawPosition);
            if (!(positionOnSurface is null)) {
                drawPositions.Add(positionOnSurface);
            }

            const int vCount = 3;
            var scale = transform.localScale;
            for (var j = 0; j < vCount; ++j) {
                var offset = new Vector3(
                    scale.x * pointRadius * Mathf.Cos(2 * Mathf.PI / vCount * j),
                    scale.y * 0,
                    scale.z * pointRadius * Mathf.Sin(2 * Mathf.PI / vCount * j)
                );
                trianglePoints.Add(mean + Utils.PtoV(TransformPoint(n1, n2, n3, Utils.VtoP(point + offset))));
            }
        }
    }

    public List<Vector3> GetNewPoints() {
        var i = firstNewIndex;
        firstNewIndex = trianglePoints.Count;
        return trianglePoints.GetRange(i, firstNewIndex - i);
    }

    private void OnDrawGizmos() {
        Gizmos.color = Color.red;
        var t = transform;
        var position = t.position;
        var direction = t.forward;
        var paintPosition = GetPaintPosition();
        Gizmos.DrawLine(position, Utils.PtoV(paintPosition.surfacePoint));

        Handles.DrawWireDisc(Utils.PtoV(paintPosition.surfacePoint), -direction, paintRadius);

        Gizmos.DrawSphere(position + direction * paintHeight, 0.3f);

        foreach (var p in drawPositions.OrderByDescending(x => Vector3.Distance(Camera.current.transform.position, Utils.PtoV(x.surfacePoint)))) {
            Gizmos.DrawSphere(Utils.PtoV(p.surfacePoint), pointRadius);
        }
    }
}
