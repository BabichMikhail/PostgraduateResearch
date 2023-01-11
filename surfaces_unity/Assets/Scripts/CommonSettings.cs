using UnityEngine;

public class CommonSettings : MonoBehaviour {
    [Header("Paint settings")]
    public float standardGramPerSquareMeter = 50f;
    public float toleranceFromTheStandardPart = 0.2f;
    public float paintAdhesionPart = 0.5f;
    public float paintDensityGramPerCubicMeter = 2.7e6f;
    public float paintConsumptionRateKgPerHour = 1.0f;
    public float paintPorosityRate = 0.1f;

    [Header("Draw settings")]
    public bool drawSurfacePath = true;
    public bool drawOriginPath = false;
    public bool drawFromOriginToSurfacePath = true;
    public bool drawFoundPath = false;
    public bool drawApproximatedPath = true;
    public bool drawLinearPath = false;
    public bool drawApproximatedPathWithSpeed = true;
    public bool drawApproximatedPathWithAcceleration = false;
    public bool drawApproximatedPathWithCustomColors = false;

    public const float SCALE_IN_GAME = 1000.0f;

    public float GetPaintConsumptionRateInGameScale() {
        return paintConsumptionRateKgPerHour * 1000 / 3600 /
               paintDensityGramPerCubicMeter *
               paintAdhesionPart * Mathf.Pow(SCALE_IN_GAME, 3) /
               (1.0f - paintPorosityRate); // M^3 -> MM^3 (scale in game);
    }
}
