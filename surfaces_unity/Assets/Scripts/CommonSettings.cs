using UnityEngine;

public class CommonSettings : MonoBehaviour {
    [Header("Paint settings")]
    public float standardGramPerSquareMeter = 50f;
    public float toleranceFromTheStandardPart = 0.2f;
    public float paintAdhesionPart = 0.5f;
    public float paintDensityGramPerCubicMeter = 2.7e6f;
    public float paintConsumptionRateKgPerHour = 1.0f;

    [Header("Draw settings")]
    public bool drawSurfacePath = true;
    public bool drawOriginPath = false;
    public bool drawFromOriginToSurfacePath = true;
    public bool drawFoundPath = false;
    public bool drawApproximatedPath = true;
    public bool drawPathStepByStep = false;
    public bool drawDiffWithLinearPath = false;
    public bool drawLinearPath = false;
    public bool drawApproximatedPathWithSpeed = true;
    public bool drawApproximatedPathWithAcceleration = false;
}
