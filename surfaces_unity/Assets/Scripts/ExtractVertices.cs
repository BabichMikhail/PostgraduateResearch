using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Windows.Forms;
using Library.Algorithm;
using Library.Generic;
using Library.PathApproximation;
using Library.RobotPathBuilder;
using PathFinders;
using TriangleHandler;
using UnityEngine;
using UnityEngine.Rendering;
using Button = UnityEngine.UI.Button;
using Color = UnityEngine.Color;
using MColor = Library.Algorithm.Color;
using Debug = UnityEngine.Debug;
using Plane = Library.Generic.Plane;
using Vector3 = UnityEngine.Vector3;
using Point = Library.Generic.Point;
using Position = Library.Generic.Position;
using Triangle = Library.Generic.Triangle;

[RequireComponent(typeof(MeshFilter))]
public class ExtractVertices : MonoBehaviour {
    public string sampleName;
    public Material material;
    public Material colorMaterial;
    public GameObject paintRobotPrefab;
    public float scaleGameToWorldMeter = 1.0f;
    public float sampleScaleToWorldMeter = 0.001f;

    private const float SCALE_IN_GAME = 1000.0f;

    private List<GameObject> paintRobots = new List<GameObject>();

    [Header("Draw settings")]
    public float paintRobotScale = 1.0f;

    [Header("Paint parameters")]
    public PathFinderType pathFinderType;
    public float paintSpeed = 1.0f;
    public float paintRadius;
    public float paintHeight;
    public float paintLateralAllowance;
    public float paintLongitudinalAllowance;
    public float paintLineWidth;
    public float maxPaintRobotSpeed;
    public float maxPaintRobotAcceleration;

    [Header("Paint speed and performance")]
    public int pointPerSecondDrawingSpeed = 0;
    public int maxPointCount = 0;
    public float timeScale = 1.0f;
    public int maxPaintRobotPathSimplifyIterations = 10;

    [Header("Other")]
    public float maxTriangleSquare = 1000000;
    public float linearPathStep = 0.005f;
    public float yRotation = 0.0f;
    public bool needRunExperiment = false;

    private CommonSettings commonSettings = null;

    private GameObject rotationCube = null;
    private List<Triangle> baseTriangles = null;
    private List<RobotPathProcessor> baseRobotPathProcessors = new List<RobotPathProcessor>();
    private List<RobotPathProcessor> simplifiedRobotPathProcessors = new List<RobotPathProcessor>();
    private List<RobotPathProcessor> linearRobotPathProcessors = new List<RobotPathProcessor>();
    private GameObject paintPointsHolder = null;

    private TexturePaintResult texturePaintResult = null;

    private struct ScaledVariables {
        public float paintSpeed;
        public float paintRadius;
        public float paintHeight;
        public float paintLateralAllowance;
        public float paintLongitudinalAllowance;
        public float paintLineWidth;
        public float maxPaintRobotSpeed;
        public float maxPaintRobotAcceleration;

        public float maxTriangleSquare;
        public float linearPathStep;
    }

    private ScaledVariables v;

    private void SetVertices(List<Triangle> triangles) {
        var verticesDict = new Dictionary<Vector3, int>();
        var meshVertices = new List<Vector3>();
        var meshTriangles = new List<int>();
        foreach (var t in triangles) {
            foreach (var p in t.GetPoints()) {
                if (!verticesDict.ContainsKey(Utils.PtoV(p))) {
                    verticesDict[Utils.PtoV(p)] = meshVertices.Count;
                    meshVertices.Add(Utils.PtoV(p));
                }

                meshTriangles.Add(verticesDict[Utils.PtoV(p)]);
            }
        }
        Debug.Log("Triangle count: " + triangles.Count);

        var mesh = gameObject.GetComponent<MeshFilter>().mesh;
        mesh.Clear();
        mesh.vertices = meshVertices.ToArray();
        mesh.triangles = meshTriangles.ToArray();
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        mesh.RecalculateTangents();

        gameObject.GetComponent<MeshRenderer>().material = material;
    }

    private void SetColoredVertices(List<Triangle> triangles, Dictionary<Triangle, Dictionary<Point, double>> paintAmount) {
        var maxAmount = 0.0;
        foreach (var amountItem in paintAmount) {
            foreach (var amount in amountItem.Value) {
                maxAmount = Math.Max(maxAmount, amount.Value);
            }
        }

        var meshTriangles = new List<int>();
        var meshVertices = new List<Vector3>();
        var meshColors = new List<Color>();
        foreach (var t in triangles) {
            foreach (var p in t.GetPoints()) {
                meshTriangles.Add(meshVertices.Count);
                meshVertices.Add(Utils.PtoV(p));
                var color = Color.white;
                if (paintAmount.ContainsKey(t) && paintAmount[t].ContainsKey(p)) {
                    var amount = paintAmount[t][p];
                    color = new Color((float)(amount / maxAmount), 0, 0, 1);
                }
                meshColors.Add(color);
            }
        }
        Debug.Log("Triangle count: " + triangles.Count);

        var mesh = gameObject.GetComponent<MeshFilter>().mesh;
        mesh.Clear();
        mesh.indexFormat = IndexFormat.UInt32;
        mesh.vertices = meshVertices.ToArray();
        mesh.triangles = meshTriangles.ToArray();
        mesh.colors = meshColors.ToArray();
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        mesh.RecalculateTangents();

        gameObject.GetComponent<MeshRenderer>().material = colorMaterial;
    }

    private void UpdateScaledVariables() {
        var k = SCALE_IN_GAME;

        v.paintSpeed = k * paintSpeed;
        v.paintRadius = k * paintRadius;
        v.paintHeight = k * paintHeight;
        v.paintLateralAllowance = k * paintLateralAllowance;
        v.paintLongitudinalAllowance = k * paintLongitudinalAllowance;
        v.paintLineWidth = k * paintLineWidth;
        v.maxPaintRobotSpeed = k * maxPaintRobotSpeed;
        v.maxPaintRobotAcceleration = k * maxPaintRobotAcceleration;
        v.maxTriangleSquare = k * k * maxTriangleSquare;
        v.linearPathStep = k * linearPathStep;
    }

    private List<Triangle> LoadTriangles() {
        var watch = Stopwatch.StartNew();
        var result = new StlTriangleHandler(Path.Combine(Utils.GetDataFolder(), sampleName, "data.stl")).GetTriangles();
        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of reading figure");

        return result;
    }

    public void Awake() {
        UpdateScaledVariables();

        var exportPathDataButton = GameObject.Find("ExportPathDataButton").GetComponent<Button>();
        exportPathDataButton.onClick.AddListener(ExportPathData);

        var drawFigureButton = GameObject.Find("DrawFigureButton").GetComponent<Button>();
        drawFigureButton.onClick.AddListener(InitializeFigure);

        var initializePathButton = GameObject.Find("InitializePathButton").GetComponent<Button>();
        initializePathButton.onClick.AddListener(InitializePath);

        var simplifyPathButton = GameObject.Find("SimplifyPathButton").GetComponent<Button>();
        simplifyPathButton.onClick.AddListener(SimplifyPath);

        var createPaintRobotsButton = GameObject.Find("CreatePaintRobotsButton").GetComponent<Button>();
        createPaintRobotsButton.onClick.AddListener(CreatePaintRobots);

        var resetGeneratedPointsButton = GameObject.Find("ResetGeneratedPointsButton").GetComponent<Button>();
        resetGeneratedPointsButton.onClick.AddListener(ResetGeneratedPoints);

        var saveDataButton = GameObject.Find("SaveDataButton").GetComponent<Button>();
        saveDataButton.onClick.AddListener(delegate { SaveData(); });

        var loadDataButton = GameObject.Find("LoadDataButton").GetComponent<Button>();
        loadDataButton.onClick.AddListener(LoadData);

        var calculateTexturePaintButton = GameObject.Find("CalculateTexturePaintButton").GetComponent<Button>();
        calculateTexturePaintButton.onClick.AddListener(CalculateTexturePaint);

        var drawTexturePaintButton = GameObject.Find("DrawTexturePaintButton").GetComponent<Button>();
        drawTexturePaintButton.onClick.AddListener(DrawTexturePaint);

        var createPlaneForExportPaintDataButton = GameObject.Find("CreatePlaneForExportPaintDataButton").GetComponent<Button>();
        createPlaneForExportPaintDataButton.onClick.AddListener(delegate { CreatePlaneForExportPaintData(); });

        var exportPaintDataButton = GameObject.Find("ExportPaintDataButton").GetComponent<Button>();
        exportPaintDataButton.onClick.AddListener(delegate { ExportPaintData(); });

        rotationCube = GameObject.Find("RotationCube");

        commonSettings = GameObject.Find("Settings").GetComponent<CommonSettings>();

        baseTriangles = LoadTriangles();

        paintPointsHolder = new GameObject {
            name = "PaintPointsHolder"
        };
        paintPointsHolder.AddComponent<MeshFilter>();
        paintPointsHolder.AddComponent<MeshRenderer>();

        paintPointsHolder.GetComponent<MeshRenderer>().material = material;
    }

    private void RunExperiment() {
        var folder = "3";

        InitializePath();
        SimplifyPath();
        CalculateTexturePaint();
        SaveData(Path.Combine(Utils.GetStoreFolder(), "generated", folder, "data", $"{sampleName}_{name}.txt"));

        for (var i = 0; i < simplifiedRobotPathProcessors.Count; ++i) {
            CreatePlaneForExportPaintData(i);
            ExportPaintData(Path.Combine(Utils.GetStoreFolder(), "generated", folder, "experiments", $"{sampleName}_{name}_path_{i}.txt"));
            DestroyPlaneForExportPaintData();
        }
    }

    private List<Triangle> GetFigureTriangles() {
        var angles = Quaternion.identity.eulerAngles;
        angles.y += yRotation;
        var extraRotation = Quaternion.Euler(angles);

        var rotatedTriangles = new List<Triangle>();
        var rotation = rotationCube.transform.rotation;
        var k = sampleScaleToWorldMeter * SCALE_IN_GAME;
        foreach (var triangle in baseTriangles) {
            rotatedTriangles.Add(new Triangle(
                Utils.VtoP(extraRotation * rotation * Utils.PtoV(triangle.p1) * k),
                Utils.VtoP(extraRotation * rotation * Utils.PtoV(triangle.p2) * k),
                Utils.VtoP(extraRotation * rotation * Utils.PtoV(triangle.p3) * k)
            ));
        }

        return rotatedTriangles;
    }

    private void DrawFigure(List<Triangle> triangles) {
        SetVertices(triangles);
    }

    private void InitializeFigure() {
        DrawFigure(GetFigureTriangles());
    }

    public void LogFuncTime(System.Action f, string msg) {
        var watch = Stopwatch.StartNew();
        f();
        watch.Stop();
        Debug.Log($"{watch.ElapsedMilliseconds} ms. {msg}");
    }

    public void InitializePath() {
        Debug.Log("InitializePath");

        var triangles = GetFigureTriangles();
        DrawFigure(triangles);

        Debug.Log($"Summary figure square: {triangles.Sum(x => x.GetSquare())}. Triangle count: {triangles.Count}.");

        var watch = Stopwatch.StartNew();
        var pathFinder = PathFinderFactory.Create(
            pathFinderType, v.paintRadius, v.paintHeight, v.paintLateralAllowance, v.paintLongitudinalAllowance, v.paintLineWidth
        );
        var basePaths = pathFinder.GetPaths(triangles);
        baseRobotPathProcessors = RobotPathProcessorBuilder.Build(basePaths, v.paintSpeed, float.NaN);

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of path calculation");
        watch.Restart();

        // var app = new Bezier2Approximation();
        // var app = new Bezier3Approximation();
        // var app = new Bezier4Approximation();
        // var app = new Bezier5Approximation();
        // var app = new Bezier20Approximation();
        // var app = new LinearApproximation(true);
        // var app = new SecondOrderApproximation();
        // var app = new ThirdOrderApproximation();
        // aPaths = app.Approximate(basePaths, 30.0f, triangles);

        // var app2 = new Bezier5Approximation(false);
        // aPaths = app2.Approximate(aPaths, 5.0f, triangles);

        var app3 = new LinearApproximation(false);
        var linearPaths = new List<List<Position>>();
        basePaths.ForEach(x => linearPaths.Add(app3.Approximate(x, v.linearPathStep, triangles)));

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of path approximation");
        watch.Restart();

        linearRobotPathProcessors = RobotPathProcessorBuilder.Build(linearPaths, v.paintSpeed, float.NaN);
        simplifiedRobotPathProcessors = new List<RobotPathProcessor>(linearRobotPathProcessors);

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of robot path processors building");
    }

    private List<int> GetBadRobotPathItemIndexes(List<RobotPathItem> robotPathItems) {
        var result = new List<int>();
        for (var j = 0; j < robotPathItems.Count; ++j) {
            var item = robotPathItems[j];
            if (item.GetSpeedMultiplier() * v.paintSpeed > v.maxPaintRobotSpeed) {
                result.Add(j);
            }
        }

        return result;
    }

    private void SimplifyPath() {
        var triangles = GetFigureTriangles();

        simplifiedRobotPathProcessors = new List<RobotPathProcessor>(linearRobotPathProcessors);
        for (var i = 0; i < simplifiedRobotPathProcessors.Count; ++i) {
            var rp = simplifiedRobotPathProcessors[i];

            var attempts = 0;
            var maxAttempts = 200;
            while (true) {
                var robotPathItems = rp.GetRobotPathItems();
                Debug.Assert(robotPathItems.First().a.type == Position.PositionType.Start);
                Debug.Assert(robotPathItems.Last().b.type == Position.PositionType.Finish);

                var badItemIndexes = GetBadRobotPathItemIndexes(robotPathItems);
                var r = attempts + 1;
                var segments = badItemIndexes.Select(idx => new KeyValuePair<int, int>(Math.Max(1, idx - r), Math.Min(robotPathItems.Count - 2, idx + r))).ToList();

                var mergedSegments = new List<KeyValuePair<int, int>>();
                for (var j = 0; j < segments.Count; ++j) {
                    var k = j + 1;
                    while (k < segments.Count && segments[k - 1].Value >= segments[k].Key) {
                        ++k;
                    }

                    mergedSegments.Add(new KeyValuePair<int, int>(segments[j].Key, segments[k - 1].Value));
                    j = k - 1;
                }

                var newRobotPathItems = new List<RobotPathItem>();
                var currentIndex = 0;
                foreach (var segment in mergedSegments) {
                    var from = segment.Key;
                    var count = segment.Value - segment.Key + 1;

                    newRobotPathItems.AddRange(robotPathItems.GetRange(currentIndex, from - currentIndex));
                    var bezierApp = new BezierNApproximation(false, Math.Max(count - 2, 1));

                    {
                        if (attempts == 2) {
                            Debug.Log("qwe");
                        }

                        var subItems = robotPathItems.GetRange(from, count);
                        var badPositions = new List<Position>();
                        subItems.ForEach(x => badPositions.Add(x.a));
                        badPositions.Add(subItems.Last().b);

                        var subPositions = bezierApp.Approximate(badPositions, v.linearPathStep, triangles);

                        var a1 = subPositions.First();
                        var a2 = subItems.First().a;
                        if (MMath.GetDistance(a1.originPoint, a2.originPoint) > 1e-4 || MMath.GetDistance(a1.surfacePoint, a2.surfacePoint) > 1e-4) {
                            throw new Exception("Magic");
                        }

                        var b1 = subPositions.Last();
                        var b2 = subItems.Last().b;
                        if (MMath.GetDistance(b1.originPoint, b2.originPoint) > 1e-4 || MMath.GetDistance(b1.surfacePoint, b2.surfacePoint) > 1e-4) {
                            throw new Exception("Magic");
                        }

                        subPositions[0] = a2;
                        subPositions[subPositions.Count - 1] = b2;

                        var newSubItems = RobotPathProcessorBuilder.BuildRobotPathItems(subPositions);
                        newRobotPathItems.AddRange(newSubItems);
                    }
                    currentIndex = from + count;
                }
                var newRange = robotPathItems.GetRange(currentIndex, robotPathItems.Count - currentIndex);
                newRobotPathItems.AddRange(newRange);

                var newRpp = RobotPathProcessorBuilder.Build(newRobotPathItems, v.paintSpeed, float.NaN);
                if (GetBadRobotPathItemIndexes(newRpp.GetRobotPathItems()).Count == 0) {
                    simplifiedRobotPathProcessors[i] = newRpp;
                    break;
                }

                if (attempts == maxAttempts) {
                    simplifiedRobotPathProcessors[i] = RobotPathProcessorBuilder.Build(newRobotPathItems, v.paintSpeed, v.maxPaintRobotSpeed);
                    break;
                }

                ++attempts;
            }

            Debug.Log("Attempts: " + attempts);
        }

        if (true) {
            var paintConsumptionRateGameSizeUnitsCubicMeterPerSecond =
                commonSettings.paintConsumptionRateKgPerHour * 1000 / 3600 /
                commonSettings.paintDensityGramPerCubicMeter *
                commonSettings.paintAdhesionPart * Mathf.Pow(SCALE_IN_GAME, 3); // M^3 -> MM^3 (scale in game);

            var newRpps = new List<RobotPathProcessor>();
            for (var i = 0; i < simplifiedRobotPathProcessors.Count; ++i) {
                var rpp = simplifiedRobotPathProcessors[i];

                var planeGameObject = CreatePlaneGameObject(i);
                var position = planeGameObject.transform.position;
                var n = planeGameObject.transform.up;
                Destroy(planeGameObject);

                var currentTriangles = new Dictionary<Triangle, bool>();
                var plane = new Plane(Utils.VtoP(n), Utils.VtoP(position));
                foreach (var t in triangles) {
                    foreach (var e in t.GetEdges()) {
                        if (MMath.Intersect(plane, new Segment(e.p1, e.p2)) != null && !currentTriangles.ContainsKey(t)) {
                            currentTriangles.Add(t, true);
                            break;
                        }
                    }
                }

                // TODO clear debug calculations
                var paintResult = new DrawingSimulator().ProcessPath(
                    new List<RobotPathProcessor>{rpp},
                    currentTriangles.Keys.ToList(),
                    20 * v.paintHeight,
                    v.paintRadius,
                    v.paintHeight,
                    v.maxTriangleSquare,
                    paintConsumptionRateGameSizeUnitsCubicMeterPerSecond
                );

                var sum1 = 0.0;
                foreach (var q in paintResult.paintAmount) {
                    foreach (var w in q.Value) {
                        sum1 += w.Value;
                    }
                }

                var sum2 = 0.0;
                foreach (var q in paintResult.detailedPaintAmount) {
                    foreach (var w in q.Value) {
                        foreach (var e in w.Value) {
                            sum2 += e.Value;
                        }
                    }
                }

                var newPaintAmountByPoint = new Dictionary<Point, double>();
                foreach (var q in paintResult.paintAmount) {
                    foreach (var w in q.Value) {
                        if (!newPaintAmountByPoint.ContainsKey(w.Key)) {
                            newPaintAmountByPoint.Add(w.Key, 0.0);
                        }

                        newPaintAmountByPoint[w.Key] += w.Value;
                    }
                }

                var newMaxPaintAmount = newPaintAmountByPoint.Max(x => x.Value);
                var newMinPaintAmount = newPaintAmountByPoint.Min(x => x.Value);

                var newDetailedPaintAmount = new Dictionary<RobotPathItem, Dictionary<Point, double>>();
                foreach (var items in paintResult.detailedPaintAmount) {
                    var item = items.Key;

                    if (!newDetailedPaintAmount.ContainsKey(item)) {
                        newDetailedPaintAmount.Add(item, new Dictionary<Point, double>());
                    }

                    foreach (var triangleAmount in items.Value) {
                        var paintAmounts = MMath.GetIntersectionPaintAmount(triangleAmount.Value, plane, triangleAmount.Key);
                        if (paintAmounts != null) {
                            foreach (var t in paintAmounts.amountItems) {
                                var amount = t.amount;
                                // if (amount > 1e-6) {
                                    var point = t.point;
                                    if (!newDetailedPaintAmount[items.Key].ContainsKey(point)) {
                                        newDetailedPaintAmount[items.Key].Add(point, 0.0);
                                    }

                                    newDetailedPaintAmount[items.Key][point] += amount;
                                // }
                            }
                        }
                    }
                }

                var accelerationMultipliers = new List<double>();
                var idxByPathItem = new Dictionary<RobotPathItem, int>();
                foreach (var pathItem in rpp.GetRobotPathItems()) {
                    accelerationMultipliers.Add(1.0);
                    idxByPathItem.Add(pathItem, idxByPathItem.Count);
                }

                var iterationsNumber = 0;
                var currentError = double.MaxValue;
                var targetAmount = double.NaN;
                while (true) {
                    var paintAmountByPoint = new Dictionary<Point, double>();
                    var paintAmountByPathItem = new Dictionary<RobotPathItem, double>();
                    foreach (var items in newDetailedPaintAmount) {
                        foreach (var amounts in items.Value) {
                            if (!paintAmountByPoint.ContainsKey(amounts.Key)) {
                                paintAmountByPoint.Add(amounts.Key, 0.0);
                            }

                            paintAmountByPoint[amounts.Key] += amounts.Value / accelerationMultipliers[idxByPathItem[items.Key]];
                        }
                    }

                    var minAmount = paintAmountByPoint.Min(x => x.Value);
                    var maxAmount = paintAmountByPoint.Max(x => x.Value);
                    if (double.IsNaN(targetAmount)) {
                        targetAmount = minAmount;
                    }

                    var newError = (maxAmount - minAmount) / maxAmount;
                    if (newError >= currentError || newError > 0.95 || Math.Abs(newError - currentError) < 1e-6) {
                        break;
                    }

                    currentError = newError;

                    var newKByPoint = new Dictionary<Point, double>();
                    foreach (var q in paintAmountByPoint) {
                        newKByPoint.Add(q.Key, q.Value / targetAmount);
                    }

                    var sumAmountByPoint = new Dictionary<Point, double>();
                    foreach (var items in newDetailedPaintAmount) {
                        foreach (var pointAmount in items.Value) {
                            var point = pointAmount.Key;
                            var amount = pointAmount.Value;
                            if (!sumAmountByPoint.ContainsKey(point)) {
                                sumAmountByPoint.Add(point, 0.0);
                            }
                            sumAmountByPoint[point] += amount;
                        }
                    }

                    var newAccelerationMultipliers = new List<double>();
                    accelerationMultipliers.ForEach(x => newAccelerationMultipliers.Add(0));
                    foreach (var items in newDetailedPaintAmount) {
                        foreach (var amounts in items.Value) {
                            newAccelerationMultipliers[idxByPathItem[items.Key]] += newKByPoint[amounts.Key] * amounts.Value / sumAmountByPoint[amounts.Key];
                        }
                    }

                    // var maxMultiplier = newAccelerationMultipliers.Max();
                    // for (var j = 0; j < newAccelerationMultipliers.Count; ++j) {
                    //     if (newAccelerationMultipliers[j] == 0) {
                    //         newAccelerationMultipliers[j] = 1;
                    //     }
                    //     else {
                    //         newAccelerationMultipliers[j] /= maxMultiplier;
                    //     }
                    // }

                    accelerationMultipliers = newAccelerationMultipliers;

                    ++iterationsNumber;
                }
                Debug.Log($"Iterations number: {iterationsNumber}");

                var pathItems = rpp.GetRobotPathItems();
                var newPathItems = new List<RobotPathItem>();
                foreach (var pathItem in pathItems) {
                    newPathItems.Add(new RobotPathItem(
                        pathItem.a,
                        pathItem.b,
                        pathItem.extraAccelerationMultiplier * (float)accelerationMultipliers[idxByPathItem[pathItem]]
                    ));
                }

                var newRpp = new RobotPathProcessor(newPathItems);
                newRpp.SetSurfaceSpeed(rpp.GetSurfaceSpeed());
                newRpp.SetMaxOriginSpeed(rpp.GetMaxOriginSpeed());
                newRpps.Add(newRpp);
            }

            // var newRpps = new List<RobotPathProcessor>();
            // foreach (var rpp in simplifiedRobotPathProcessors) {
            //     var pathItems = rpp.GetRobotPathItems();
            //     var newPathItems = new List<RobotPathItem>();
            //     foreach (var pathItem in pathItems) {
            //         newPathItems.Add(new RobotPathItem(pathItem.a, pathItem.b, pathItem.extraAccelerationMultiplier * (float)decelerationMultipliers[idxByPathItem[pathItem]]));
            //     }
            //     var newRpp = new RobotPathProcessor(newPathItems);
            //     newRpp.SetSurfaceSpeed(rpp.GetSurfaceSpeed());
            //     newRpp.SetMaxOriginSpeed(rpp.GetMaxOriginSpeed());
            //     newRpps.Add(newRpp);
            // }
            simplifiedRobotPathProcessors = newRpps;
        }

        for (var g = 0; g < 1 && false; ++g) {
            var newRPPs = new List<RobotPathProcessor>();
            for (var i = 0; i < simplifiedRobotPathProcessors.Count; ++i) {
                var rpp = simplifiedRobotPathProcessors[i];
                var robotPathItems = rpp.GetRobotPathItems();

                if (GetBadRobotPathItemIndexes(robotPathItems).Count > 0) {
                    var planeGameObject = CreatePlaneGameObject(i);
                    var position = planeGameObject.transform.position;
                    var n = planeGameObject.transform.up;
                    Destroy(planeGameObject);

                    var plane = new Plane(Utils.VtoP(n), Utils.VtoP(position));
                    var currentTriangles = new List<Triangle>();
                    foreach (var t in triangles) {
                        foreach (var e in t.GetEdges()) {
                            if (MMath.Intersect(plane, new Segment(e.p1, e.p2)) != null) {
                                currentTriangles.Add(t);
                                break;
                            }
                        }
                    }

                    var robotSurfaceDistance = -2.0 * v.paintLongitudinalAllowance;
                    robotPathItems.ForEach(x => robotSurfaceDistance += MMath.GetDistance(x.a.surfacePoint, x.b.surfacePoint));

                    var paintConsumptionRateGameSizeUnitsCubicMeterPerSecond =
                        commonSettings.paintConsumptionRateKgPerHour * 1000 / 3600 /
                        commonSettings.paintDensityGramPerCubicMeter *
                        commonSettings.paintAdhesionPart * Mathf.Pow(SCALE_IN_GAME, 3); // M^3 -> MM^3 (scale in game);

                    var paintResult = new DrawingSimulator().ProcessPath(
                        simplifiedRobotPathProcessors,
                        currentTriangles,
                        20 * v.paintHeight,
                        v.paintRadius,
                        v.paintHeight,
                        v.maxTriangleSquare,
                        paintConsumptionRateGameSizeUnitsCubicMeterPerSecond
                    );
                    var exportData = GetPaintDataForExport(paintResult, plane);

                    // paintResult.detailedPaintAmount

                    var paintSurfaceDistance = 0.0;
                    exportData.ForEach(x => paintSurfaceDistance += x.Key.GetLength());
                    var maxValue = exportData.Max(x => x.Value);

                    var k = paintSurfaceDistance / robotSurfaceDistance;

                    var j = 0;
                    var p = 0;
                    var distanceJ = 0.0;
                    var distanceP = 0.0;
                    var minQ = double.MaxValue;
                    var maxQ = double.MinValue;
                    var newRobotPathItems = new List<RobotPathItem>();
                    while (j < exportData.Count && p < robotPathItems.Count) {
                        distanceP += MMath.GetDistance(robotPathItems[p].a.surfacePoint, robotPathItems[p].b.surfacePoint);

                        var subDistance = 0.0;
                        var subJ = j;
                        while (distanceJ + subDistance < (distanceP - v.paintLongitudinalAllowance) * k && subJ < exportData.Count) {
                            subDistance += exportData[subJ].Key.GetLength();
                            ++subJ;
                        }

                        var sumQ = 0.0;
                        for (var w = j; w < subJ; ++w) {
                            sumQ += exportData[w].Value * (float)(exportData[w].Key.GetLength() / subDistance);
                        }

                        var q = 1.0;
                        if (subJ > j) {
                            q = sumQ / maxValue;
                            minQ = Math.Min(q, minQ);
                            maxQ = Math.Max(q, maxQ);
                        }

                        newRobotPathItems.Add(new RobotPathItem(robotPathItems[p].a, robotPathItems[p].b, (float)q));

                        j = subJ;
                        ++p;
                        distanceJ += subDistance;
                    }

                    for (var u = newRobotPathItems.Count; u < robotPathItems.Count; ++u) {
                        var item = robotPathItems[u];
                        newRobotPathItems.Add(new RobotPathItem(item.a, item.b));
                    }

                    var newRpp = new RobotPathProcessor(newRobotPathItems);
                    newRpp.SetSurfaceSpeed(rpp.GetSurfaceSpeed());
                    newRpp.SetMaxOriginSpeed(rpp.GetMaxOriginSpeed());
                    newRPPs.Add(newRpp);
                    // TODO recalculate deceleration multiplier
                }
                else {
                    newRPPs.Add(rpp);
                }
            }

            simplifiedRobotPathProcessors = newRPPs;
        }
    }

    private void CreatePaintRobots() {
        var objectTriangles = GetFigureTriangles();
        paintRobots = new List<GameObject>();
        foreach (var rpp in simplifiedRobotPathProcessors) {
            var pos = rpp.Move(0.0f);
            var robot = Instantiate(paintRobotPrefab, Utils.PtoV(pos.a.originPoint), Quaternion.LookRotation(Utils.PtoV(pos.a.paintDirection), Vector3.up));
            var controller = robot.GetComponent<PaintRobotController>();
            controller.SetMaxSpeed(v.maxPaintRobotSpeed);
            controller.SetPaintHeight(v.paintHeight);
            controller.SetPaintRadius(v.paintRadius);
            controller.SetPointGenerationSpeed(pointPerSecondDrawingSpeed);
            controller.SetObjectTriangles(objectTriangles);
            robot.transform.localScale *= paintRobotScale;
            paintRobots.Add(robot);
        }
    }

    private void ResetGeneratedPoints() {
        foreach (var pr in paintRobots) {
            var controller = pr.GetComponent<PaintRobotController>();
            controller.ResetGeneratedPoints();
        }
    }

    private void SaveData(string filename = "") {
        if (filename.Length == 0) {
            var dialog = new SaveFileDialog {
                InitialDirectory = Utils.GetStoreFolder(),
                Filter = "text files (*.txt)|*.txt",
                RestoreDirectory = false
            };

            if (dialog.ShowDialog() == DialogResult.OK) {
                filename = dialog.FileName;
            }
        }

        if (filename.Length > 0) {
            DataExport.TexturePaintResultData texturePaintResultData = null;
            if (texturePaintResult != null) {
                texturePaintResultData = new DataExport.TexturePaintResultData(texturePaintResult);
            }

            var data = new DataExport.PathData {
                settings = new DataExport.SceneSettings {
                    sampleName = sampleName,
                    scaleGameToWorldMeter = scaleGameToWorldMeter,
                    sampleScaleToWorldMeter = sampleScaleToWorldMeter,

                    drawSurfacePath = commonSettings.drawSurfacePath,
                    drawOriginPath = commonSettings.drawOriginPath,
                    drawFromOriginToSurfacePath = commonSettings.drawFromOriginToSurfacePath,
                    drawFoundPath = commonSettings.drawFoundPath,
                    drawApproximatedPath = commonSettings.drawApproximatedPath,
                    drawLinearPath = commonSettings.drawLinearPath,
                    drawApproximatedPathWithSpeed = commonSettings.drawApproximatedPathWithSpeed,
                    drawApproximatedPathWithAcceleration = commonSettings.drawApproximatedPathWithAcceleration,
                    paintRobotScale = paintRobotScale,

                    paintSpeed = paintSpeed,
                    paintRadius = paintRadius,
                    paintHeight = paintHeight,
                    paintLateralAllowance = paintLateralAllowance,
                    paintLongitudinalAllowance = paintLongitudinalAllowance,
                    paintLineWidth = paintLineWidth,
                    maxPaintRobotSpeed = maxPaintRobotSpeed,
                    maxPaintRobotAcceleration = maxPaintRobotAcceleration,

                    pointPerSecondDrawingSpeed = pointPerSecondDrawingSpeed,
                    maxPointCount = maxPointCount,
                    timeScale = timeScale,
                    maxPaintRobotPathSimplifyIterations = maxPaintRobotPathSimplifyIterations,

                    maxTriangleSquare = maxTriangleSquare,
                    linearPathStep = linearPathStep,
                    yRotation = yRotation,
                },
                state = new DataExport.SceneState {
                    isPaintRobotsCreated = paintRobots.Count > 0,
                },
                rotationDataCube = new DataExport.ObjectRotationData(rotationCube.transform.rotation),
                baseRobots = baseRobotPathProcessors.Select(rpp => new DataExport.RobotPathProcessorData(rpp)).ToList(),
                linearRobots = linearRobotPathProcessors.Select(rpp => new DataExport.RobotPathProcessorData(rpp)).ToList(),
                simplifiedRobots = simplifiedRobotPathProcessors.Select(rpp => new DataExport.RobotPathProcessorData(rpp)).ToList(),
                texturePaintResult = texturePaintResultData,
                baseTriangles = baseTriangles.Select(t => new DataExport.TriangleData(t)).ToList(),
            };

            File.WriteAllText(filename, JsonUtility.ToJson(data));
            Debug.Log(filename);
        }
    }

    private void ApplyLoadedData(DataExport.PathData data) {
        var settings = data.settings;
        Debug.Assert(settings.sampleName == sampleName);

        scaleGameToWorldMeter = settings.scaleGameToWorldMeter;
        sampleScaleToWorldMeter = settings.sampleScaleToWorldMeter;

        commonSettings.drawSurfacePath = settings.drawSurfacePath;
        commonSettings.drawOriginPath = settings.drawOriginPath;
        commonSettings.drawFromOriginToSurfacePath = settings.drawFromOriginToSurfacePath;
        commonSettings.drawFoundPath = settings.drawFoundPath;
        commonSettings.drawApproximatedPath = settings.drawApproximatedPath;
        commonSettings.drawLinearPath = settings.drawLinearPath;
        commonSettings.drawApproximatedPathWithSpeed = settings.drawApproximatedPathWithSpeed;
        commonSettings.drawApproximatedPathWithAcceleration = settings.drawApproximatedPathWithAcceleration;

        paintRobotScale = settings.paintRobotScale;

        paintSpeed = settings.paintSpeed;
        paintRadius = settings.paintRadius;
        paintHeight = settings.paintHeight;
        paintLateralAllowance = settings.paintLateralAllowance;
        paintLongitudinalAllowance = settings.paintLongitudinalAllowance;
        paintLineWidth = settings.paintLineWidth;
        maxPaintRobotSpeed = settings.maxPaintRobotSpeed;
        maxPaintRobotAcceleration = settings.maxPaintRobotAcceleration;

        pointPerSecondDrawingSpeed = settings.pointPerSecondDrawingSpeed;
        maxPointCount = settings.maxPointCount;
        timeScale = settings.timeScale;
        maxPaintRobotPathSimplifyIterations = settings.maxPaintRobotPathSimplifyIterations;

        maxTriangleSquare = settings.maxTriangleSquare;
        linearPathStep = settings.linearPathStep;
        yRotation = settings.yRotation;

        rotationCube.transform.rotation = data.rotationDataCube.GetQuaternion();

        baseTriangles = data.baseTriangles.Select(x => x.GetTriangle()).ToList();

        if (!(data.texturePaintResult is null)) {
            texturePaintResult = data.texturePaintResult.GetTexturePaintResult();
        }

        if (!(data.texturePlaneData is null)) {
            CreatePlaneForExportPaintData();

            var planeData = data.texturePlaneData;
            paintTexturePlaneGameObject.transform.position = Utils.PtoV(planeData.position.GetPoint());
            paintTexturePlaneGameObject.transform.localScale = Utils.PtoV(planeData.scale.GetPoint());
            paintTexturePlaneGameObject.transform.rotation = planeData.rotationData.GetQuaternion();
        }

        UpdateScaledVariables();
        DrawFigure(GetFigureTriangles());

        baseRobotPathProcessors = data.baseRobots.Select(x => x.GetRobotPathProcessor()).ToList();
        linearRobotPathProcessors = data.linearRobots.Select(x => x.GetRobotPathProcessor()).ToList();
        simplifiedRobotPathProcessors = data.simplifiedRobots.Select(x => x.GetRobotPathProcessor()).ToList();

        paintRobots.Clear();
        if (data.state.isPaintRobotsCreated) {
            CreatePaintRobots();

            var i = 0;
            foreach (var rd in data.simplifiedRobots) {
                // TODO
                // paintRobots[i].GetComponent<PaintRobotController>().SetDrawingPositions(rd.GetDrawingPositions());
                // ++i;
            }
        }
    }

    private void LoadData() {
        var dialog = new OpenFileDialog {
            InitialDirectory = Utils.GetStoreFolder(),
            Filter = "text files (*.txt)|*.txt",
            RestoreDirectory = false
        };

        if (dialog.ShowDialog() == DialogResult.OK) {
            var pathData = JsonUtility.FromJson<DataExport.PathData>(File.ReadAllText(dialog.FileName));
            Debug.Log(dialog.FileName);

            var root = GameObject.Find("Objects");
            var settings = pathData.settings;
            for (var i = 0; i < root.transform.childCount; ++i) {
                var child = root.transform.GetChild(i).gameObject;
                var controller = child.GetComponent<ExtractVertices>();
                var active = controller.sampleName == settings.sampleName;
                child.SetActive(active);
                if (active) {
                    controller.ApplyLoadedData(pathData);
                }
            }
        }
    }

    private void CalculateTexturePaint() {
        if (baseRobotPathProcessors.Count == 0) {
            Debug.Log("Unable to CalculateTexturePaint. Path is null");
            return;
        }

        Debug.Log("CalculateTexturePaint");
        var triangles = GetFigureTriangles();

        var watch = Stopwatch.StartNew();

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of calculate robot path with speeds");
        watch.Restart();

        var paintConsumptionRateGameSizeUnitsCubicMeterPerSecond =
            commonSettings.paintConsumptionRateKgPerHour * 1000 / 3600 /
            commonSettings.paintDensityGramPerCubicMeter *
            commonSettings.paintAdhesionPart * Mathf.Pow(SCALE_IN_GAME, 3); // M^3 -> MM^3 (scale in game);
        texturePaintResult = new DrawingSimulator().ProcessPath(
            simplifiedRobotPathProcessors, triangles, 20 * v.paintHeight, v.paintRadius, v.paintHeight, v.maxTriangleSquare,
            paintConsumptionRateGameSizeUnitsCubicMeterPerSecond
        );
        Debug.Log($"Triangles: {triangles.Count}; {texturePaintResult.triangles.Count}.");
        SetColoredVertices(texturePaintResult.triangles, texturePaintResult.paintAmount);

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of path processing");
    }

    private void DrawTexturePaint() {
        if (texturePaintResult is null) {
            Debug.Log("Unable DrawTexturePaint: texturePaintResult is null");
            return;
        }

        Debug.Log("DrawTexturePaint");

        var watch = Stopwatch.StartNew();
        SetColoredVertices(texturePaintResult.triangles, texturePaintResult.paintAmount);
        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of drawing texture paint result");
    }

    private GameObject paintTexturePlaneGameObject = null;

    private GameObject CreatePlaneGameObject(int pathIdx = -1) {
        var triangles = GetFigureTriangles();
        var position = triangles.Aggregate(Point.Zero, (current, t) => current + t.o) / triangles.Count;
        if (pathIdx != -1) {
            var items = simplifiedRobotPathProcessors[pathIdx].GetRobotPathItems();
            position = items.Aggregate(Point.Zero, (current, t) => current + t.a.surfacePoint + t.b.surfacePoint) / (2 * items.Count);
        }

        var paintTexturePlanePosition = position;
        var paintTexturePlane = new Plane(
            position,
            position + Point.Up * SCALE_IN_GAME,
            position + Point.Forward * SCALE_IN_GAME
        );

        var n = Utils.PtoV(paintTexturePlane.GetNormal());
        var rotation = Quaternion.LookRotation(n);
        rotation = Quaternion.Euler(rotation.eulerAngles + new Vector3(90, 0, 0));

        var planeGameObject = GameObject.CreatePrimitive(PrimitiveType.Plane);
        planeGameObject.transform.position = Utils.PtoV(paintTexturePlanePosition);
        planeGameObject.transform.rotation = rotation;

        return planeGameObject;
    }

    private void CreatePlaneForExportPaintData(int pathIdx = -1) {
        if (texturePaintResult is null) {
            Debug.Log("Unable CreatePlaneForExportPaintData: texturePaintResult is null");
            return;
        }

        Debug.Log("CreatePlaneForExportPaintData");

        var watch = Stopwatch.StartNew();

        DestroyPlaneForExportPaintData();
        paintTexturePlaneGameObject = CreatePlaneGameObject(pathIdx);

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of creating plane for export paint data");
    }

    private void DestroyPlaneForExportPaintData() {
        Destroy(paintTexturePlaneGameObject);
    }

    private List<KeyValuePair<Segment, double>> GetPaintDataForExport(TexturePaintResult paintResult, Plane paintTexturePlane) {
        var segments = new List<Segment>();
        var values = new List<double>();
        var intersectionResults = new List<MMath.IntersectionPaintResult>();
        foreach (var t in paintResult.triangles) {
            var paintAmountResult = MMath.GetIntersectionPaintAmount(paintResult.paintAmount[t], paintTexturePlane, t);
            if (paintAmountResult != null) {
                intersectionResults.Add(paintAmountResult);
                segments.Add(paintAmountResult.GetSegment());
                values.Add(paintAmountResult.GetAvgAmount() / SCALE_IN_GAME);
            }
        }

        var result = new List<KeyValuePair<Segment, double>>();
        if (intersectionResults.Count > 0) {
            var idxBySegment = new Dictionary<Segment, int>();
            var segmentsByPoint = new Dictionary<Point, List<Segment>>();
            foreach (var segment in segments) {
                idxBySegment.Add(segment, idxBySegment.Count);

                foreach (var p in segment.GetPoints()) {
                    if (!segmentsByPoint.ContainsKey(p)) {
                        segmentsByPoint.Add(p, new List<Segment>());
                    }
                    segmentsByPoint[p].Add(segment);
                }
            }

            var newSegments = new List<Segment>();
            Point lastPoint = null;
            var processedSegments = new Dictionary<Segment, bool>();
            var processedPoints = new Dictionary<Point, bool>();
            while (processedSegments.Count != segments.Count) {
                var ok = false;
                if (!(lastPoint is null)) {
                    foreach (var segment in segmentsByPoint[lastPoint]) {
                        if (!processedSegments.ContainsKey(segment)) {
                            processedSegments.Add(segment, true);
                            newSegments.Add(segment);
                            ok = true;
                            break;
                        }
                    }

                    if (!ok) {
                        processedPoints.Add(lastPoint, true);
                        var lastSegment = newSegments.Last();
                        lastPoint = lastPoint == lastSegment.p1 ? lastSegment.p2 : lastSegment.p1;
                        if (processedPoints.ContainsKey(lastPoint)) {
                            lastPoint = null;
                        }
                    }
                }

                if (!ok && lastPoint is null) {
                    Edge e = null;
                    if (newSegments.Count > 0) {
                        e = new Edge(newSegments.Last().p1, newSegments.Last().p2);
                    }
                    foreach (var line in segments) {
                        if (!processedSegments.ContainsKey(line)) {
                            foreach (var p in line.GetPoints()) {
                                if (!(e is null) && (lastPoint is null || MMath.GetDistance(e, p) < MMath.GetDistance(e, lastPoint))) {
                                    lastPoint = p;
                                }

                                // TODO Bad condition for first point
                                if (e is null && (lastPoint is null || p.z < lastPoint.z)) {
                                    lastPoint = p;
                                }
                                // if (lastPoint is null || p.x < lastPoint.x) {
                                //     lastPoint = p;
                                //     break;
                                // }
                            }
                        }
                    }
                }
            }

            Debug.Assert(newSegments.Count == segments.Count);
            result.AddRange(newSegments.Select(segment => new KeyValuePair<Segment, double>(segment, values[idxBySegment[segment]])));
        }

        return result;
    }

    private void ExportPaintData(string filename = "") {
        if (texturePaintResult is null) {
            Debug.Log("Unable ExportPaintData: texturePaintResult is null");
            return;
        }

        if (paintTexturePlaneGameObject is null) {
            Debug.Log("Unable ExportPaintData: paintTexturePlane is null");
            return;
        }

        Debug.Log("ExportPaintData");

        if (filename.Length == 0) {
            var dialog = new SaveFileDialog {
                InitialDirectory = Utils.GetStoreFolder(),
                Filter = "text files (*.txt)|*.txt",
                RestoreDirectory = false
            };

            if (dialog.ShowDialog() == DialogResult.OK) {
                filename = dialog.FileName;
            }
        }

        if (filename.Length > 0) {
            var position = paintTexturePlaneGameObject.transform.position;
            var n = paintTexturePlaneGameObject.transform.up;

            var watch = Stopwatch.StartNew();
            var exportData = GetPaintDataForExport(texturePaintResult, new Plane(Utils.VtoP(n), Utils.VtoP(position)));
            watch.Stop();
            Debug.Log(watch.ElapsedMilliseconds + " ms. Time of export paint data");

            var strLines = new List<string>();
            foreach (var exportItem in exportData) {
                var p1 = exportItem.Key.p1;
                var p2 = exportItem.Key.p2;
                var value = exportItem.Value;
                strLines.Add($"({p1.x};{p1.y};{p1.z})-({p2.x};{p2.y};{p2.z})\t{value}");
            }

            File.WriteAllLines(filename, strLines);
        }
    }

    private void MoveRobot(float time) {
        Debug.Assert(simplifiedRobotPathProcessors.Count == paintRobots.Count || paintRobots.Count == 0);
        if (simplifiedRobotPathProcessors.Count == paintRobots.Count) {
            var i = 0;
            var isFinished = true;
            foreach (var rp in simplifiedRobotPathProcessors) {
                isFinished = isFinished && rp.IsFinished();

                var pos = rp.Move(time);

                var robot = paintRobots[i];
                robot.transform.position = Utils.PtoV(pos.a.originPoint);
                robot.transform.rotation = Quaternion.LookRotation(Utils.PtoV(pos.a.paintDirection), Vector3.up);

                var controller = robot.GetComponent<PaintRobotController>();
                controller.SetMaxSpeed(v.maxPaintRobotSpeed);
                controller.SetCurrentSpeed(rp.GetCurrentOriginSpeed());
                controller.SetMaxAcceleration(v.maxPaintRobotAcceleration);
                controller.SetCurrentAcceleration(rp.GetCurrentAcceleration());

                ++i;
            }

            if (isFinished) {
                foreach (var rp in simplifiedRobotPathProcessors) {
                    rp.Reset();
                }
            }
        }
    }

    private string StoreState(Point positionA, Point positionB, float speed, float deceleration, double distance) {
        return string.Format("({0} {1} {2})-({3} {4} {5}) {6} {7} {8}",
            positionA.x.ToString(new CultureInfo("en-US")),
            positionA.y.ToString(new CultureInfo("en-US")),
            positionA.z.ToString(new CultureInfo("en-US")),
            positionB.x.ToString(new CultureInfo("en-US")),
            positionB.y.ToString(new CultureInfo("en-US")),
            positionB.z.ToString(new CultureInfo("en-US")),
            speed.ToString(new CultureInfo("en-US")),
            deceleration.ToString(new CultureInfo("en-US")),
            distance.ToString(new CultureInfo("en-US"))
        );
    }

    private void ExportPathData() {
        if (!(simplifiedRobotPathProcessors is null)) {
            foreach (var rp in simplifiedRobotPathProcessors) {
                rp.SetSurfaceSpeed(v.paintSpeed);
            }

            var lines = new List<string>{$"{simplifiedRobotPathProcessors.Count}"};
            foreach (var rp in simplifiedRobotPathProcessors) {
                var pathLines = new List<string>();

                foreach (var item in rp.GetRobotPathItems()) {
                    pathLines.Add(StoreState(
                        item.a.originPoint,
                        item.b.originPoint,
                        item.GetSpeedMultiplier(),
                        item.GetSpeedDecelerationMultiplier(1.0f, float.NaN),
                        MMath.GetDistance(item.a.originPoint, item.b.originPoint)
                    ));
                }

                lines.Add($"{pathLines.Count}");
                lines.AddRange(pathLines);
            }

            var filePath = Path.Combine(Utils.GetStoreFolder(), "pathData.txt");
            File.WriteAllLines(filePath, lines);
        }
    }

    private void FixedUpdate() {
        UpdateScaledVariables();

        simplifiedRobotPathProcessors.ForEach(x => x.SetSurfaceSpeed(v.paintSpeed));
        MoveRobot(Time.deltaTime * timeScale);

        if (needRunExperiment) {
            RunExperiment();
            needRunExperiment = false;
            gameObject.SetActive(false);
        }
    }

    private void OnDrawGizmos() {
        if (commonSettings is null) {
            return;
        }

        if (commonSettings.drawApproximatedPathWithSpeed) {
            foreach (var rpp in simplifiedRobotPathProcessors) {
                foreach (var item in rpp.GetRobotPathItems()) {
                    Gizmos.color = PaintRobotController.GetSpeedColor(item.GetSpeed(v.paintSpeed, rpp.GetMaxOriginSpeed()), v.maxPaintRobotSpeed);
                    Gizmos.DrawLine(Utils.PtoV(item.a.originPoint), Utils.PtoV(item.b.originPoint));
                }
            }
        }

        if (commonSettings.drawApproximatedPathWithAcceleration) {
            foreach (var rpp in simplifiedRobotPathProcessors) {
                foreach (var item in rpp.GetRobotPathItems()) {
                    // TODO acceleration;
                    Gizmos.color = PaintRobotController.GetSpeedColor(item.GetSpeed(v.paintSpeed, rpp.GetMaxOriginSpeed()), v.maxPaintRobotSpeed);
                    Gizmos.DrawLine(Utils.PtoV(item.a.originPoint), Utils.PtoV(item.b.originPoint));
                }
            }
        }

        if (commonSettings.drawLinearPath) {
            if (commonSettings.drawOriginPath) {
                Gizmos.color = Color.blue;
                foreach (var rpp in simplifiedRobotPathProcessors) {
                    foreach (var item in rpp.GetRobotPathItems()) {
                        Gizmos.DrawLine(Utils.PtoV(item.a.originPoint), Utils.PtoV(item.b.originPoint));
                    }
                }
            }
        }

        if (commonSettings.drawFromOriginToSurfacePath) {
            if (commonSettings.drawFoundPath) {
                Gizmos.color = Color.magenta;
                foreach (var rpp in baseRobotPathProcessors) {
                    foreach (var item in rpp.GetRobotPathItems()) {
                        Gizmos.DrawLine(Utils.PtoV(item.a.originPoint), Utils.PtoV(item.a.surfacePoint));
                        if (item.b.type == Position.PositionType.Finish) {
                            Gizmos.DrawLine(Utils.PtoV(item.b.originPoint), Utils.PtoV(item.b.surfacePoint));
                        }
                    }
                }
            }

            if (commonSettings.drawApproximatedPath) {
                Gizmos.color = Color.cyan;
                foreach (var rpp in simplifiedRobotPathProcessors) {
                    foreach (var item in rpp.GetRobotPathItems()) {
                        Gizmos.DrawLine(Utils.PtoV(item.a.originPoint), Utils.PtoV(item.a.surfacePoint));
                        if (item.b.type == Position.PositionType.Finish) {
                            Gizmos.DrawLine(Utils.PtoV(item.b.originPoint), Utils.PtoV(item.b.surfacePoint));
                        }
                    }
                }
            }
        }

        if (commonSettings.drawOriginPath) {
            if (commonSettings.drawFoundPath) {
                Gizmos.color = Color.black;
                foreach (var rpp in baseRobotPathProcessors) {
                    foreach (var item in rpp.GetRobotPathItems()) {
                        Gizmos.DrawLine(Utils.PtoV(item.a.originPoint), Utils.PtoV(item.b.originPoint));
                    }
                }
            }

            if (commonSettings.drawApproximatedPath) {
                Gizmos.color = Color.black;
                foreach (var rpp in simplifiedRobotPathProcessors) {
                    foreach (var item in rpp.GetRobotPathItems()) {
                        Gizmos.DrawLine(Utils.PtoV(item.a.originPoint), Utils.PtoV(item.b.originPoint));
                    }
                }
            }
        }

        if (commonSettings.drawSurfacePath) {
            if (commonSettings.drawFoundPath) {
                Gizmos.color = Color.blue;
                foreach (var rpp in baseRobotPathProcessors) {
                    foreach (var item in rpp.GetRobotPathItems()) {
                        Gizmos.DrawLine(Utils.PtoV(item.a.surfacePoint), Utils.PtoV(item.b.surfacePoint));
                    }
                }
            }

            if (commonSettings.drawApproximatedPath) {
                Gizmos.color = Color.blue;
                foreach (var rpp in simplifiedRobotPathProcessors) {
                    foreach (var item in rpp.GetRobotPathItems()) {
                        Gizmos.DrawLine(Utils.PtoV(item.a.surfacePoint), Utils.PtoV(item.b.surfacePoint));
                    }
                }
            }
        }
    }
}
