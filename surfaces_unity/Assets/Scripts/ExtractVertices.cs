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
    public float minPaintRobotSpeed;
    public float maxPaintRobotSpeed;
    public float maxPaintRobotAcceleration;
    public bool addExtraParallelPaths;

    [Header("Paint speed and performance")]
    public int pointPerSecondDrawingSpeed = 0;
    public int maxPointCount = 0;
    public float timeScale = 1.0f;
    public int maxPaintRobotPathSimplifyIterations = 10;
    public bool useBasePathSimplification;
    public bool useBaseSpeedCorrection;
    public bool useAdvancedPathSimplification;
    public float targetPaintThickness = 100 * 1e-6f;

    [Header("Other")]
    public float maxTriangleSquare = 1000000;
    public float linearPathStep = 0.005f;
    public float yRotation = 0.0f;
    public bool needRunExperiment = false;
    public string experimentStoreFolder;
    public string experimentFileName;
    public bool isOrt = false;

    private CommonSettings commonSettings = null;

    private GameObject rotationCube = null;
    private List<Triangle> baseTriangles = null;
    private List<RobotPathProcessor> baseRobotPathProcessors = new List<RobotPathProcessor>();
    private List<RobotPathProcessor> simplifiedRobotPathProcessors = new List<RobotPathProcessor>();
    private List<RobotPathProcessor> linearRobotPathProcessors = new List<RobotPathProcessor>();
    private GameObject paintPointsHolder = null;

    private TexturePaintResult texturePaintResult = null;
    private readonly Color heatMapMinColor = new Color(0.0f, 0.5f, 1.0f, 1.0f);
    private readonly Color heatMapMaxColor = new Color(0.0f, 1.0f, 0.5f, 1.0f);

    private struct ScaledVariables {
        public float paintSpeed;
        public float paintRadius;
        public float paintHeight;
        public float paintLateralAllowance;
        public float paintLongitudinalAllowance;
        public float paintLineWidth;
        public float minPaintRobotSpeed;
        public float maxPaintRobotSpeed;
        public float maxPaintRobotAcceleration;

        public float maxTriangleSquare;
        public float linearPathStep;

        public float targetPaintThickness;
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
        var maxAmount = double.MinValue;
        var minAmount = double.MaxValue;
        foreach (var amountItem in paintAmount) {
            foreach (var amount in amountItem.Value) {
                maxAmount = Math.Max(maxAmount, amount.Value);
                minAmount = Math.Min(minAmount, amount.Value);
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
                    color = heatMapMinColor + (heatMapMaxColor - heatMapMinColor) * (float)((amount - minAmount) / (maxAmount - minAmount));
                    // color = new Color((float)(amount / maxAmount), 0, 0, 1);
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
        v.minPaintRobotSpeed = k * minPaintRobotSpeed;
        v.maxPaintRobotSpeed = k * maxPaintRobotSpeed;
        v.maxPaintRobotAcceleration = k * maxPaintRobotAcceleration;
        v.maxTriangleSquare = k * k * maxTriangleSquare;
        v.linearPathStep = k * linearPathStep;
        v.targetPaintThickness = k * targetPaintThickness;
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
        exportPathDataButton.onClick.AddListener(delegate { ExportPathData(); });

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

        var runExperimentButton = GameObject.Find("RunExperimentButton").GetComponent<Button>();
        runExperimentButton.onClick.AddListener(delegate {
            var filename = Path.Combine(experimentStoreFolder, experimentFileName);
            if (filename.Length == 0) {
                var dialog = new SaveFileDialog {
                    InitialDirectory = Utils.GetStoreFolder(),
                    Filter = "all files (*)|*",
                    RestoreDirectory = false
                };
                if (dialog.ShowDialog() == DialogResult.OK) {
                    filename = dialog.FileName;
                }
            }

            if (filename.Length > 0) {
                InitializePath();
                SimplifyPath();
                CreatePlaneForExportPaintData();
                CalculateTexturePaint();

                ExportHeatMapSettings(filename + "_heat_map.txt");
                ExportPaintData(filename + ".txt");
                ExportPathSpeeds(filename + "_speeds.txt");
                ExportTexturePaintData(filename + "_texture.txt");
                SaveData(filename + "_data.txt");
            }
        });

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
        var baseExportPath = Path.Combine(Utils.GetStoreFolder(), "generated", experimentStoreFolder);
        Directory.CreateDirectory(baseExportPath);

        InitializePath();
        SimplifyPath();
        CalculateTexturePaint();
        SaveData(Path.Combine(baseExportPath, "data", $"{sampleName}_{name}.txt"));
        ExportPathData(Path.Combine(baseExportPath, "pathData", $"{sampleName}_{name}.txt"));

        for (var i = 0; i < simplifiedRobotPathProcessors.Count; ++i) {
            CreatePlaneForExportPaintData(i);
            ExportPaintData(Path.Combine(baseExportPath, "experiments", $"{sampleName}_{name}_path_{i}.txt"));
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

    private void LogFuncTime(System.Action f, string msg) {
        var watch = Stopwatch.StartNew();
        f();
        watch.Stop();
        Debug.Log($"{watch.ElapsedMilliseconds} ms. {msg}");
    }

    private void InitializePath() {
        Debug.Log("InitializePath");

        var triangles = GetFigureTriangles();
        DrawFigure(triangles);

        Debug.Log($"Summary figure square: {triangles.Sum(x => x.GetSquare())}. Triangle count: {triangles.Count}.");

        var watch = Stopwatch.StartNew();
        var pathFinder = PathFinderFactory.Create(
            pathFinderType, v.paintRadius, v.paintHeight, v.paintLateralAllowance, v.paintLongitudinalAllowance, v.paintLineWidth, addExtraParallelPaths
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

    private List<int> GetBadRobotPathItemIndexes(IReadOnlyList<RobotPathItem> robotPathItems) {
        var result = new List<int>();
        for (var j = 0; j < robotPathItems.Count; ++j) {
            var item = robotPathItems[j];
            if (item.GetSpeed(v.paintSpeed) > v.maxPaintRobotSpeed) {
                result.Add(j);
            }
        }

        return result;
    }

    private void SimplifyPath() {
        var triangles = GetFigureTriangles();
        simplifiedRobotPathProcessors = new List<RobotPathProcessor>(linearRobotPathProcessors);

        if (useBasePathSimplification) {
            for (var i = 0; i < simplifiedRobotPathProcessors.Count; ++i) {
                var rp = simplifiedRobotPathProcessors[i];

                var attempts = 0;
                var maxAttempts = 10000;
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
                            var subItems = robotPathItems.GetRange(from, count);
                            var badPositions = new List<Position>();
                            subItems.ForEach(x => badPositions.Add(x.a));
                            badPositions.Add(subItems.Last().b);

                            var subPositions = bezierApp.Approximate(badPositions, v.linearPathStep, triangles);

                            var a1 = subPositions.First();
                            var a2 = subItems.First().a;
                            if (MMath.GetDistance(a1.originPoint, a2.originPoint) > 1 || MMath.GetDistance(a1.surfacePoint, a2.surfacePoint) > 1) {
                                throw new Exception("Magic");
                            }

                            var b1 = subPositions.Last();
                            var b2 = subItems.Last().b;
                            if (MMath.GetDistance(b1.originPoint, b2.originPoint) > 1 || MMath.GetDistance(b1.surfacePoint, b2.surfacePoint) > 1) {
                                throw new Exception("Magic");
                            }

                            subPositions[0] = a2;
                            subPositions[subPositions.Count - 1] = b2;

                            var newSubItems = RobotPathProcessorBuilder.BuildRobotPathItems(subPositions);
                            newRobotPathItems.AddRange(newSubItems);
                        }
                        currentIndex = from + count;
                    }
                    newRobotPathItems.AddRange(robotPathItems.GetRange(currentIndex, robotPathItems.Count - currentIndex));

                    var newRpp = RobotPathProcessorBuilder.Build(newRobotPathItems, v.paintSpeed, float.NaN);
                    if (GetBadRobotPathItemIndexes(newRpp.GetRobotPathItems()).Count == 0) {
                        simplifiedRobotPathProcessors[i] = newRpp;
                        break;
                    }

                    if (attempts == maxAttempts) {
                        for (var j = 0; j < newRobotPathItems.Count; ++j) {
                            var pathItem = newRobotPathItems[j];
                            // var speed = pathItem.GetSpeed(v.paintSpeed);
                            var k = 1.0f;
                            // var k = Math.Min(speed, v.maxPaintRobotSpeed) / speed;
                            newRobotPathItems[j] = new RobotPathItem(pathItem.a, pathItem.b, k);
                        }
                        simplifiedRobotPathProcessors[i] = RobotPathProcessorBuilder.Build(newRobotPathItems, v.paintSpeed, v.maxPaintRobotSpeed);
                        break;
                    }

                    attempts += 10;
                }

                Debug.Log("Attempts: " + attempts);
            }
        }

        if (useBaseSpeedCorrection) {
            var paintConsumptionRateGameSizeUnitsCubicMeterPerSecond =
                commonSettings.paintConsumptionRateKgPerHour * 1000 / 3600 /
                commonSettings.paintDensityGramPerCubicMeter *
                commonSettings.paintAdhesionPart * Mathf.Pow(SCALE_IN_GAME, 3); // M^3 -> MM^3 (scale in game);

            for (var i = 0; i < simplifiedRobotPathProcessors.Count; ++i) {
                var rpp = simplifiedRobotPathProcessors[i];
                var pathItems = rpp.GetRobotPathItems();

                var planeGameObject = CreatePlaneGameObject(i);
                var planePosition = planeGameObject.transform.position;
                var n = planeGameObject.transform.up;
                Destroy(planeGameObject);

                var currentTriangles = new Dictionary<Triangle, bool>();
                var plane = new Plane(Utils.VtoP(n), Utils.VtoP(planePosition));
                foreach (var t in triangles) {
                    foreach (var e in t.GetEdges()) {
                        if (MMath.Intersect(plane, new Segment(e.p1, e.p2)) != null && !currentTriangles.ContainsKey(t)) {
                            currentTriangles.Add(t, true);
                            break;
                        }
                    }
                }

                var paintResult = new DrawingSimulator().ProcessPath(
                    new List<RobotPathProcessor>{rpp},
                    currentTriangles.Keys.ToList(),
                    20 * v.paintHeight,
                    v.paintRadius,
                    v.paintHeight,
                    v.maxTriangleSquare,
                    paintConsumptionRateGameSizeUnitsCubicMeterPerSecond
                );

                var data = GetPaintDataForExport(paintResult, plane);
                var sumDistance = data.Last().Item2;
                var sumValue = 0.0;
                for (var j = 0; j < data.Count; ++j) {
                    var distance = 0.0;
                    if (j > 0) {
                        distance += MMath.GetDistance(data[j].Item1, data[j - 1].Item1);
                    }

                    if (j + 1 < data.Count) {
                        distance += MMath.GetDistance(data[j].Item1, data[j + 1].Item1);
                    }

                    if (j > 0) {
                        sumValue += data[j].Item3 * MMath.GetDistance(data[j].Item1, data[j - 1].Item1) / distance;
                    }

                    if (j + 1 < data.Count) {
                        sumValue += data[j].Item3 * MMath.GetDistance(data[j].Item1, data[j + 1].Item1) / distance;
                    }
                }

                var avgValue = sumValue / sumDistance;
                var extraMultiplier = avgValue / v.targetPaintThickness;

                var newPathItems = new List<RobotPathItem>();
                foreach (var pathItem in pathItems) {
                    newPathItems.Add(new RobotPathItem(pathItem.a, pathItem.b, pathItem.extraAccelerationMultiplier * (float)extraMultiplier));
                }

                var newRpp = new RobotPathProcessor(newPathItems);
                newRpp.SetSurfaceSpeed(rpp.GetSurfaceSpeed());
                newRpp.SetMaxOriginSpeed(rpp.GetMaxOriginSpeed());
                simplifiedRobotPathProcessors[i] = newRpp;
            }
        }

        if (useAdvancedPathSimplification) {
            var paintConsumptionRateGameSizeUnitsCubicMeterPerSecond =
                commonSettings.paintConsumptionRateKgPerHour * 1000 / 3600 /
                commonSettings.paintDensityGramPerCubicMeter *
                commonSettings.paintAdhesionPart * Mathf.Pow(SCALE_IN_GAME, 3); // M^3 -> MM^3 (scale in game);



            Dictionary<Position, List<RobotPathItem>> pathItemsByPosition = null;
            List<Point> points = null;
            List<Position> positions = null;
            Dictionary<Point, Dictionary<RobotPathItem, double>> paintAmountByPointAndPathItem = null;

            var ok = false;
            for (var i = 0; i < simplifiedRobotPathProcessors.Count; ++i) {
                var iterationCount = 0;
                double CalculateLimitV2(double v1, double w1, double v2, double w2, double acceleration) {
                    var c = 1.0 - 0.01 * Math.Log10(iterationCount + 1);
                    return Math.Abs(w1 / v1 - w2 / v2) * (c * v.maxPaintRobotAcceleration / acceleration);
                }

                var limits = new List<double>();
                var usedLimitsIndexes = new Dictionary<int, bool>();
                var aLog = new List<Tuple<double, double, int>>();
                var weights = new List<double>();

                var rpp = simplifiedRobotPathProcessors[i];
                var pathItems = rpp.GetRobotPathItems();
                var basePathItems = new List<RobotPathItem>();
                foreach (var pathItem in pathItems) {
                    basePathItems.Add(new RobotPathItem(pathItem.a, pathItem.b, 1.0f));
                }

                var allTimes = new List<List<float>>();
                while (!ok) {
                    var times = new List<float>();
                    var watch = Stopwatch.StartNew();

                    if (iterationCount == 0) {
                        for (var j = 0; j < pathItems.Count; ++j) {
                            weights.Add(basePathItems[j].GetSpeed(v.paintSpeed) / pathItems[j].GetSpeed(v.paintSpeed));
                        }

                        for (var j = 0; j < pathItems.Count - 1; ++j) {
                            var acceleration = RobotPathItem.GetAcceleration(pathItems[j], pathItems[j + 1], v.paintSpeed);
                            var v1 = basePathItems[j].GetSpeed(v.paintSpeed);
                            var v2 = basePathItems[j + 1].GetSpeed(v.paintSpeed);
                            var limit = CalculateLimitV2(v1, weights[j], v2, weights[j + 1], acceleration);
                            limits.Add(limit);
                        }
                    }

                    watch.Stop();
                    times.Add(watch.ElapsedMilliseconds); // 0
                    watch.Restart();

                    var planeGameObject = CreatePlaneGameObject(i);
                    var planePosition = planeGameObject.transform.position;
                    var n = planeGameObject.transform.up;
                    Destroy(planeGameObject);

                    var plane = new Plane(Utils.VtoP(n), Utils.VtoP(planePosition));

                    watch.Stop();
                    times.Add(watch.ElapsedMilliseconds); // 1
                    watch.Restart();

                    if (iterationCount == 0) {
                        var currentTriangles = new Dictionary<Triangle, bool>();
                        foreach (var t in triangles) {
                            foreach (var e in t.GetEdges()) {
                                if (MMath.Intersect(plane, new Segment(e.p1, e.p2)) != null && !currentTriangles.ContainsKey(t)) {
                                    currentTriangles.Add(t, true);
                                    break;
                                }
                            }
                        }

                        var paintResult = new DrawingSimulator().ProcessPath(
                            new List<RobotPathProcessor>{rpp},
                            currentTriangles.Keys.ToList(),
                            20 * v.paintHeight,
                            v.paintRadius,
                            v.paintHeight,
                            v.maxTriangleSquare,
                            paintConsumptionRateGameSizeUnitsCubicMeterPerSecond
                        );

                        var newDetailedPaintAmountForPositions = new Dictionary<Position, Dictionary<Point, double>>();
                        foreach (var items in paintResult.detailedPaintAmountForPositions) {
                            DictUtils.FillValueIfNotExists(newDetailedPaintAmountForPositions, items.Key, new Dictionary<Point, double>());
                            foreach (var triangleAmount in items.Value) {
                                var paintAmounts = MMath.GetIntersectionPaintAmount(triangleAmount.Value, plane, triangleAmount.Key);
                                if (paintAmounts != null) {
                                    DictUtils.SumValue(newDetailedPaintAmountForPositions[items.Key], paintAmounts.GetAvgPoint(), paintAmounts.GetAvgAmount());
                                }
                            }
                        }

                        paintAmountByPointAndPathItem = GetPaintAmountByPointAndPathItem(paintResult, plane);

                        var idxByPoint = new Dictionary<Point, int>();
                        var idxByPosition = new Dictionary<Position, int>();
                        foreach (var paintAmountsForPosition in newDetailedPaintAmountForPositions) {
                            var position = paintAmountsForPosition.Key;
                            DictUtils.FillValueIfNotExists(idxByPosition, position, idxByPosition.Count);

                            foreach (var paintAmountByPoint in paintAmountsForPosition.Value) {
                                var point = paintAmountByPoint.Key;
                                DictUtils.FillValueIfNotExists(idxByPoint, point, idxByPoint.Count);
                            }
                        }

                        points = idxByPoint.Keys.OrderBy(x => idxByPoint[x]).ToList();
                        positions = idxByPosition.Keys.OrderBy(x => idxByPosition[x]).ToList();

                        pathItemsByPosition = new Dictionary<Position, List<RobotPathItem>>();
                        foreach (var pathItem in pathItems) {
                            DictUtils.FillValueIfNotExists(pathItemsByPosition, pathItem.a, new List<RobotPathItem>());
                            DictUtils.FillValueIfNotExists(pathItemsByPosition, pathItem.b, new List<RobotPathItem>());
                            pathItemsByPosition[pathItem.a].Add(pathItem);
                            pathItemsByPosition[pathItem.b].Add(pathItem);
                        }

                        var oneCount = 0;
                        foreach (var q in pathItemsByPosition) {
                            if (q.Value.Count == 1) {
                                ++oneCount;
                            }
                        }
                        if (oneCount != 2) {
                            throw new Exception("Invalid path items");
                        }
                    }
                    ++iterationCount;

                    watch.Stop();
                    times.Add(watch.ElapsedMilliseconds); // 2
                    watch.Restart();

                    var rRows = new List<List<double>>();
                    var sRows = new List<List<double>>();
                    var gRows = new List<List<double>>();
                    var aRows = new List<List<double>>();
                    var hList = new List<List<double>>();
                    var bList = new List<List<double>>();
                    var wRows = new List<List<double>>();

                    foreach (var point in points) {
                        var newRow = new List<double>();
                        foreach (var pathItem in pathItems) {
                            newRow.Add(paintAmountByPointAndPathItem[point][pathItem]);
                        }

                        rRows.Add(newRow);
                        sRows.Add(new List<double>{v.targetPaintThickness});
                    }

                    foreach (var position in positions) {
                        var currentPathItems = pathItemsByPosition[position];
                        if (currentPathItems.Count == 2) {
                            var newRRow = new List<double>();
                            var newGRow = new List<double>();
                            var currentLimit = double.NaN;
                            for (var j = 0; j < pathItems.Count; ++j) {
                                var pathItem = pathItems[j];
                                if (currentPathItems[0] == pathItem) {
                                    var baseItem1 = basePathItems[j];
                                    var baseItem2 = basePathItems[j + 1];
                                    if (usedLimitsIndexes.ContainsKey(j)) {
                                        var v1 = baseItem1.GetSpeed(v.paintSpeed);
                                        var v2 = baseItem2.GetSpeed(v.paintSpeed);
                                        var k = v1 / weights[j] - v2 / weights[j + 1] >= 0 ? -1.0 : 1.0;
                                        newGRow.Add(k / v1);
                                        newGRow.Add(-k / v2);
                                        currentLimit = Math.Abs(limits[j]);
                                        DictUtils.FillValueIfNotExists(usedLimitsIndexes, j, true);
                                    }

                                    var alpha = 0.01;
                                    newRRow.Add(alpha);
                                    newRRow.Add(-alpha);
                                }
                                else if (currentPathItems[1] == pathItem) {
                                    Debug.Assert(pathItems[j - 1] == currentPathItems[0]);
                                }
                                else {
                                    newGRow.Add(0.0);
                                    newRRow.Add(0.0);
                                }
                            }

                            if (!double.IsNaN(currentLimit)) {
                                gRows.Add(newGRow);
                                hList.Add(new List<double>{Math.Abs(currentLimit)});
                            }

                            rRows.Add(newRRow);
                            sRows.Add(new List<double>{0.0});
                        }
                    }

                    for (var j = 0; j < basePathItems.Count; ++j) {
                        var newGRow = new List<double>();
                        for (var k = 0; k < j; ++k) {
                            newGRow.Add(0.0);
                        }

                        newGRow.Add(1.0);

                        for (var k = j + 1; k < basePathItems.Count; ++k) {
                            newGRow.Add(0.0);
                        }

                        gRows.Add(newGRow);
                        hList.Add(new List<double>{basePathItems[j].GetSpeed(v.paintSpeed) / v.minPaintRobotSpeed});
                    }

                    for (var j = 0; j < basePathItems.Count; ++j) {
                        var newGRow = new List<double>();
                        for (var k = 0; k < j; ++k) {
                            newGRow.Add(0.0);
                        }

                        newGRow.Add(-1.0);

                        for (var k = j + 1; k < basePathItems.Count; ++k) {
                            newGRow.Add(0.0);
                        }

                        gRows.Add(newGRow);
                        hList.Add(new List<double>{-basePathItems[j].GetSpeed(v.paintSpeed) / v.maxPaintRobotSpeed});
                    }

                    for (var j = 0; j < rRows.Count; ++j) {
                        var newWRow = new List<double>();
                        for (var k = 0; k < rRows.Count; ++k) {
                            newWRow.Add(j == k ? 1.0 : 0.0);
                        }
                        wRows.Add(newWRow);
                    }

                    watch.Stop();
                    times.Add(watch.ElapsedMilliseconds); // 3
                    watch.Restart();

                    var rm = new Matrix(rRows);
                    var sm = new Matrix(sRows);
                    var gm = new Matrix(gRows);
                    var hm = new Matrix(hList);
                    var am = new Matrix(aRows); // not used
                    var bm = new Matrix(bList); // not used
                    var wm = new Matrix(wRows);
                    var result = Equations.QuadraticProgrammingASLE(rm, sm, gm, hm, am, bm, wm);

                    watch.Stop();
                    times.Add(watch.ElapsedMilliseconds); // 4
                    watch.Restart();

                    Debug.Assert(result.ColumnCount == 1);

                    var newPathItems = new List<RobotPathItem>();
                    for (var j = 0; j < result.RowCount; ++j) {
                        var basePathItem = basePathItems[j];
                        weights[j] = result.Get(j, 0);
                        newPathItems.Add(new RobotPathItem(basePathItem.a, basePathItem.b, (float)(basePathItem.extraAccelerationMultiplier / result.Get(j, 0))));
                    }

                    ok = true;
                    var aList = new List<double>();
                    for (var j = 1; j < newPathItems.Count; ++j) {
                        var a = RobotPathItem.GetAcceleration(newPathItems[j - 1], newPathItems[j], v.paintSpeed);
                        ok = ok && Math.Abs(a) < v.maxPaintRobotAcceleration;
                        aList.Add(a);
                    }

                    if (!ok) {
                        for (var j = 0; j < aList.Count; ++j) {
                            if (Math.Abs(aList[j]) > v.maxPaintRobotAcceleration || usedLimitsIndexes.ContainsKey(j)) {
                                var newV1 = basePathItems[j].GetSpeed(v.paintSpeed);
                                var newV2 = basePathItems[j + 1].GetSpeed(v.paintSpeed);
                                var newLimit = CalculateLimitV2(newV1, weights[j], newV2, weights[j + 1], aList[j]);
                                limits[j] = newLimit;
                                DictUtils.FillValueIfNotExists(usedLimitsIndexes, j, true);
                            }
                        }
                    }

                    var aMax = aList.Max();
                    var aMin = aList.Min();
                    aLog.Add(new Tuple<double, double, int>(aMax, aMin, usedLimitsIndexes.Count));

                    watch.Stop();
                    times.Add(watch.ElapsedMilliseconds); // 5
                    allTimes.Add(times);

                    if (ok) {
                        var newRpp = new RobotPathProcessor(newPathItems);
                        newRpp.SetSurfaceSpeed(rpp.GetSurfaceSpeed());
                        newRpp.SetMaxOriginSpeed(rpp.GetMaxOriginSpeed());
                        simplifiedRobotPathProcessors[i] = newRpp;
                    }

                    // var testPaintResult = new DrawingSimulator().ProcessPath(
                    //     new List<RobotPathProcessor>{newRpp},
                    //     currentTriangles.Keys.ToList(),
                    //     20 * v.paintHeight,
                    //     v.paintRadius,
                    //     v.paintHeight,
                    //     v.maxTriangleSquare,
                    //     paintConsumptionRateGameSizeUnitsCubicMeterPerSecond
                    // );
                    // var testPaintAmountByPointAndPathItem = GetPaintAmountByPointAndPathItem(testPaintResult, plane);
                    // var testMinAmount = double.MaxValue;
                    // var testMaxAmount = double.MinValue;
                    // foreach (var q in testPaintAmountByPointAndPathItem) {
                    //     var sumAmount = q.Value.Sum(x => x.Value);
                    //     testMinAmount = Math.Min(testMinAmount, sumAmount);
                    //     testMaxAmount = Math.Max(testMaxAmount, sumAmount);
                    // }
                }
                Debug.Log(aLog);
                Debug.Log($"Optimization iterations: {iterationCount}");
            }
        }
    }

    private Dictionary<Point, Dictionary<RobotPathItem, double>> GetPaintAmountByPointAndPathItem(TexturePaintResult paintResult, Plane plane) {
        var paintAmountByPointAndPathItem = new Dictionary<Point, Dictionary<RobotPathItem, double>>();
        foreach (var pathItemAmount in paintResult.detailedPaintAmountForItems) {
            var pathItem = pathItemAmount.Key;

            foreach (var triangleAmount in pathItemAmount.Value) {
                var paintAmounts = MMath.GetIntersectionPaintAmount(triangleAmount.Value, plane, triangleAmount.Key);
                if (paintAmounts != null) {
                    DictUtils.FillValueIfNotExists(paintAmountByPointAndPathItem, paintAmounts.GetAvgPoint(), new Dictionary<RobotPathItem, double>());
                    DictUtils.SumValue(paintAmountByPointAndPathItem[paintAmounts.GetAvgPoint()], pathItem, paintAmounts.GetAvgAmount());
                }
            }
        }

        return paintAmountByPointAndPathItem;
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
            simplifiedRobotPathProcessors,
            triangles,
            20 * v.paintHeight,
            v.paintRadius,
            v.paintHeight,
            v.maxTriangleSquare,
            paintConsumptionRateGameSizeUnitsCubicMeterPerSecond
        );
        Debug.Log($"Triangles: {triangles.Count}; {texturePaintResult.triangles.Count}.");

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
        if (!isOrt) {
            rotation = Quaternion.Euler(rotation.eulerAngles + new Vector3(90, 0, 0));
        }
        else {
            rotation = Quaternion.Euler(rotation.eulerAngles + new Vector3(-90, -90, 0));
        }

        var planeGameObject = GameObject.CreatePrimitive(PrimitiveType.Plane);
        planeGameObject.transform.position = Utils.PtoV(paintTexturePlanePosition);
        planeGameObject.transform.rotation = rotation;

        return planeGameObject;
    }

    private void CreatePlaneForExportPaintData(int pathIdx = -1) {
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

    private List<Tuple<Point, double, double>> GetPaintDataForExport(TexturePaintResult paintResult, Plane paintTexturePlane) {
        var segments = new List<Segment>();
        var values = new List<double>();
        var intersectionResults = new List<MMath.IntersectionPaintResult>();
        foreach (var t in paintResult.triangles) {
            var paintAmountResult = MMath.GetIntersectionPaintAmount(paintResult.paintAmount[t], paintTexturePlane, t);
            if (paintAmountResult != null) {
                intersectionResults.Add(paintAmountResult);
                segments.Add(paintAmountResult.GetSegment());
                values.Add(paintAmountResult.GetAvgAmount());
            }
        }

        var result = new List<Tuple<Point, double, double>>();
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
                                if (e is null && (lastPoint is null || !isOrt && p.z < lastPoint.z || isOrt && p.x < lastPoint.x)) {
                                    lastPoint = p;
                                }
                                // if (e is null && (lastPoint is null || p.x < lastPoint.x)) {
                                //     lastPoint = p;
                                // }
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
            var distance = 0.0;
            var lastAmount = values[idxBySegment[newSegments.First()]];
            foreach (var segment in newSegments) {
                var amount = values[idxBySegment[segment]];
                result.Add(new Tuple<Point, double, double>(segment.p1, distance, 0.5 * (lastAmount + amount)));
                distance += segment.GetLength();
                lastAmount = amount;
            }
            result.Add(new Tuple<Point, double, double>(newSegments.Last().p2, distance + newSegments.Last().GetLength(), values[idxBySegment[newSegments.Last()]]));
        }

        return result;
    }

    private void ExportHeatMapSettings(string filename = "") {
        if (texturePaintResult is null) {
            Debug.Log("Unable ExportPaintData: texturePaintResult is null");
            return;
        }

        if (paintTexturePlaneGameObject is null) {
            Debug.Log("Unable ExportPaintData: paintTexturePlane is null");
            return;
        }

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
            var maxAmount = double.MinValue;
            var minAmount = double.MaxValue;
            foreach (var amountItem in texturePaintResult.paintAmount) {
                foreach (var amount in amountItem.Value) {
                    maxAmount = Math.Max(maxAmount, amount.Value);
                    minAmount = Math.Min(minAmount, amount.Value);
                }
            }

            var minColor = heatMapMinColor;
            var maxColor = heatMapMaxColor;
            var strLines = new List<string>{
                "\tR\tG\tB\tA\tValue",
                $"Min\t{minColor.r}\t{minColor.g}\t{minColor.b}\t{minColor.a}\t{minAmount}",
                $"Max\t{maxColor.r}\t{maxColor.g}\t{maxColor.b}\t{maxColor.a}\t{maxAmount}",
            };

            File.WriteAllLines(filename, strLines);
        }
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
                var p = exportItem.Item1;
                var distance = exportItem.Item2;
                var value = exportItem.Item3;
                strLines.Add($"({p.x};{p.y};{p.z})\t{distance}\t{value / SCALE_IN_GAME}");
            }

            File.WriteAllLines(filename, strLines);
        }
    }

    private void ExportPathSpeeds(string filename = "") {
        Debug.Log("ExportPathSpeeds");

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
            var strLines = new List<string>();
            foreach (var rpp in simplifiedRobotPathProcessors) {
                var sumDistance = 0.0;
                var pathItems = rpp.GetRobotPathItems();
                for (var j = 0; j < pathItems.Count; ++j) {
                    var pathItem = pathItems[j];

                    var speed = pathItem.GetSpeed(v.paintSpeed);
                    var acceleration = j < pathItems.Count - 2 ? RobotPathItem.GetAcceleration(pathItems[j], pathItems[j + 1], v.paintSpeed) : 0.0;
                    var distance = MMath.GetDistance(pathItem.a.originPoint, pathItem.b.originPoint);

                    sumDistance += distance;
                    strLines.Add($"{speed}\t{sumDistance}\t{acceleration}\t{pathItem.GetSpeedMultiplier()}\t{pathItem.GetSpeedAccelerationMultiplier()}");
                }
            }

            File.WriteAllLines(filename, strLines);
        }
    }

    private void ExportTexturePaintData(string filename = "") {
        if (texturePaintResult is null) {
            Debug.Log("Unable ExportPaintData: texturePaintResult is null");
            return;
        }

        Debug.Log("ExportTexturePaintData");

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
            var strLines = new List<string>();
            foreach (var item in texturePaintResult.paintAmount) {
                var str = "";
                foreach (var amountItem in item.Value) {
                    str += $"({amountItem.Key.x};{amountItem.Key.y};{amountItem.Key.z})\t{amountItem.Value}\t";
                }

                strLines.Add(str.TrimEnd());
            }

            File.WriteAllLines(filename, strLines);
        }
    }

    struct ExportPathItem {
        public float x;
        public float y;
        public float time;
    }

    private void ExportPathData(string filename = "") {
        if (simplifiedRobotPathProcessors.Count == 0) {
            Debug.Log("Unable ExportPathData: simplifiedRobotPathProcessors are empty");
            return;
        }

        Debug.Log("ExportPathData");

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
            var watch = Stopwatch.StartNew();
            var exportData = new List<List<ExportPathItem>>();
            foreach (var rpp in simplifiedRobotPathProcessors) {
                var pathItems = rpp.GetRobotPathItems();
                var first = pathItems.First().a.originPoint;
                var items = new List<ExportPathItem>();
                var time = 0.0f;
                items.Add(new ExportPathItem {
                    x = 0.0f,
                    y = 0.0f,
                    time = time,
                });
                foreach (var pathItem in pathItems) {
                    var origin = pathItem.b.originPoint;

                    var xz = origin.x + origin.z - first.x - first.z;
                    var y = origin.y - first.y;
                    time += pathItem.GetTime(v.paintSpeed);

                    items.Add(new ExportPathItem {
                        x = xz,
                        y = y,
                        time = time,
                    });
                }

                exportData.Add(items);
            }

            var strLines = new List<string>();
            var maxCount = simplifiedRobotPathProcessors.Max(x => x.GetRobotPathItems().Count);
            for (var j = 0; j < maxCount; ++j) {
                var str = "";
                foreach (var items in exportData) {
                    if (items.Count > j) {
                        str += $"{items[j].x}\t{items[j].y}\t{items[j].time}\t\t";
                    }
                    else {
                        str += "\t\t\t\t";
                    }
                }

                strLines.Add(str.Substring(0, str.Length - 2));
            }

            watch.Stop();
            Debug.Log(watch.ElapsedMilliseconds + " ms. Time of export path data");

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
                    Gizmos.color = PaintRobotController.GetSpeedColor(item.GetSpeed(v.paintSpeed), v.maxPaintRobotSpeed);
                    Gizmos.DrawLine(Utils.PtoV(item.a.originPoint), Utils.PtoV(item.b.originPoint));
                }
            }
        }

        if (commonSettings.drawApproximatedPathWithAcceleration) {
            foreach (var rpp in simplifiedRobotPathProcessors) {
                foreach (var item in rpp.GetRobotPathItems()) {
                    // TODO acceleration;
                    Gizmos.color = PaintRobotController.GetSpeedColor(item.GetSpeed(v.paintSpeed), v.maxPaintRobotSpeed);
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
