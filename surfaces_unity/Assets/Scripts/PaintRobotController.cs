using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
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
        points.Clear();
    }

    private List<Vector3> points = new List<Vector3>();

    private Vector3 GetPaintPoint() {
        return transform.position + transform.forward * paintHeight;
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

    public static float NextGaussian(float mean, float standardDeviation) {
        return mean + NextGaussian() * standardDeviation;
    }

    private void Update() {
        var count = Mathf.RoundToInt(pointsPerSecond * Time.deltaTime);

        var mean = GetPaintPoint();
        var standardDeviation = paintRadius / 3;
        for (var i = 0; i < count; ++i) {
            var point = new Vector3(NextGaussian(mean.x, standardDeviation), mean.y, NextGaussian(mean.z, standardDeviation));
            points.Add(point);
        }
    }

    private void OnDrawGizmos() {
        Gizmos.color = Color.red;
        var position = transform.position;
        var direction = transform.forward;
        var paintPoint = GetPaintPoint();
        Gizmos.DrawLine(position, paintPoint);

        UnityEditor.Handles.DrawWireDisc(paintPoint, -direction, paintRadius);

        Gizmos.DrawSphere(position + direction * paintHeight, 0.3f);

        foreach (var p in points) {
            Gizmos.DrawSphere(p, pointRadius);
        }
    }
}
