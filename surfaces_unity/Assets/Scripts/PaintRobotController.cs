using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
}
