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

    private List<GameObject> paintRobots = new List<GameObject>();

    [Header("Draw settings")]
    public float paintRobotScale = 1.0f;

    [Header("Paint parameters")]
    public PathFinderType pathFinderType;
    public float paintSpeed = 1.0f;
    public float paintRadius;
    public float paintHeight;
    public float paintAngle;
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
    public int baseSimplificationPointCount = 12;
    public bool useBaseSpeedCorrection;
    public bool useAdvancedPathSimplification;
    public bool dontUseAdvancedPathSimplificationV2;
    public float targetPaintThicknessOnSingleLine = 10.0f * 1e-6f;
    public float targetPaintThicknessOnSurface = 16.6f * 1e-6f;
    public List<float> defaultOffsets = new List<float> {
        0.0f,
    };
    public List<float> ortOffsets = new List<float> {
        0.0f,
    };

    [Header("Other")]
    public float maxTriangleSquare = 1000000;
    public float linearPathStep = 0.005f;
    public float yRotation = 0.0f;
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
    private readonly Color heatMapNormColor = new Color(0.0f, 1.0f, 0.5f, 1.0f);
    private readonly Color heatMapMaxColor = new Color(1.0f, 0.0f, 0.5f, 1.0f);

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

        public float targetPaintThicknessOnSingleLine;
        public float targetPaintThicknessOnSurface;
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
        var _maxAmount = double.MinValue;
        var _minAmount = double.MaxValue;
        foreach (var amountItem in paintAmount) {
            foreach (var amount in amountItem.Value) {
                _maxAmount = Math.Max(_maxAmount, amount.Value);
                _minAmount = Math.Min(_minAmount, amount.Value);
            }
        }
        var t2 = v.targetPaintThicknessOnSurface;
        var t1 = v.targetPaintThicknessOnSingleLine;
        var k = simplifiedRobotPathProcessors.Count > 1 ? t2 / t1 : 1.0f;
        var target = t1 * k;
        var minAmount = target * 0.89006395f;
        var maxAmount = target * 2.0f;
        // var minAmount = _minAmount;
        // var maxAmount = target * 2.0f;

        var meshTriangles = new List<int>();
        var meshVertices = new List<Vector3>();
        var meshColors = new List<Color>();
        foreach (var t in triangles) {
            foreach (var p in t.GetPoints()) {
                meshTriangles.Add(meshVertices.Count);
                meshVertices.Add(Utils.PtoV(p));
                var color = heatMapMinColor;
                if (paintAmount.ContainsKey(t) && paintAmount[t].ContainsKey(p)) {
                    var amount = paintAmount[t][p];
                    var c1 = heatMapMinColor;
                    var c2 = heatMapNormColor;
                    var a = minAmount;
                    var b = target;
                    if (amount > target) {
                        c1 = heatMapNormColor;
                        c2 = heatMapMaxColor;
                        a = target;
                        b = maxAmount;
                    }
                    amount = Math.Min(amount, b);
                    color = c1 + (c2 - c1) * (float)((amount - a) / (b - a));
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
        var k = CommonSettings.SCALE_IN_GAME;

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
        v.targetPaintThicknessOnSingleLine = k * targetPaintThicknessOnSingleLine;
        v.targetPaintThicknessOnSurface = k * targetPaintThicknessOnSurface;
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
                CalculateTexturePaint();

                ExportHeatMapSettings(filename + "_heat_map.txt");

                isOrt = false;
                foreach (var sigmaCount in defaultOffsets) {
                    CreatePlaneForExportPaintData(-1, new Point(v.paintRadius / 3.0f * sigmaCount, 0.0f, 0.0f));
                    ExportPaintData($"{filename}_{sigmaCount.ToString(CultureInfo.InvariantCulture)}_def.txt");
                }

                isOrt = true;
                foreach (var sigmaCount in ortOffsets) {
                    CreatePlaneForExportPaintData(-1, new Point(0.0f, 0.0f, v.paintRadius / 3.0f * sigmaCount));
                    ExportPaintData($"{filename}_{sigmaCount.ToString(CultureInfo.InvariantCulture)}_ort.txt");
                }

                isOrt = false;
                ExportPathSpeeds(filename + "_speeds.txt");
                ExportPathNormalAngles(filename + "_angles.txt");
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

    private List<Triangle> GetFigureTriangles() {
        var angles = Quaternion.identity.eulerAngles;
        angles.y += yRotation;
        var extraRotation = Quaternion.Euler(angles);

        var rotatedTriangles = new List<Triangle>();
        var rotation = rotationCube.transform.rotation;
        var k = sampleScaleToWorldMeter * CommonSettings.SCALE_IN_GAME;
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
            pathFinderType, v.paintRadius, v.paintHeight, paintAngle, v.paintLateralAllowance, v.paintLongitudinalAllowance, v.paintLineWidth, addExtraParallelPaths
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

        var app3 = new LinearApproximation(false, pathFinderType == PathFinderType.IntersectionsWithSurfacesPathFinder);
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
        var d = new Dictionary<int, bool>();
        for (var j = 0; j < robotPathItems.Count; ++j) {
            var item = robotPathItems[j];
            if (item.GetSpeed(v.paintSpeed) > v.maxPaintRobotSpeed) {
                DictUtils.FillValueIfNotExists(d, j, true);
            }

            if (j > 0) {
                var a = RobotPathItem.GetAcceleration(robotPathItems[j - 1], robotPathItems[j], v.paintSpeed);
                if (a > v.maxPaintRobotAcceleration) {
                    DictUtils.FillValueIfNotExists(d, j, true);
                    DictUtils.FillValueIfNotExists(d, j + 1, true);
                }
            }
        }

        return d.Keys.ToList().OrderBy(x => x).ToList();
    }

    private void SimplifyPath() {
        var triangles = GetFigureTriangles();
        simplifiedRobotPathProcessors = new List<RobotPathProcessor>(linearRobotPathProcessors);

        if (useBasePathSimplification) {
            for (var i = 0; i < simplifiedRobotPathProcessors.Count; ++i) {
                var rpp = simplifiedRobotPathProcessors[i];

                var attempts = 498;
                var maxAttempts = 500;
                var maxPositionCount = baseSimplificationPointCount;
                while (true) {
                    var robotPathItems = rpp.GetRobotPathItems();
                    Debug.Assert(robotPathItems.First().a.type == Position.PositionType.Start);
                    Debug.Assert(robotPathItems.Last().b.type == Position.PositionType.Finish);

                    var badItemIndexes = GetBadRobotPathItemIndexes(robotPathItems);
                    var r = attempts + 1;
                    var segments = badItemIndexes.Select(idx => new KeyValuePair<int, int>(Math.Max(0, idx - r), Math.Min(robotPathItems.Count - 1, idx + r))).ToList();

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
                        var bezierApp = new BezierNApproximation(false, pathFinderType == PathFinderType.IntersectionsWithSurfacesPathFinder, Math.Max(count - 2, 1));

                        {
                            var subItems = robotPathItems.GetRange(from, count);
                            var badPositions = new List<Position>();
                            subItems.ForEach(x => badPositions.Add(x.a));
                            badPositions.Add(subItems.Last().b);

                            var addMiddlePosition = false;
                            var posCount = Math.Min(badPositions.Count, maxPositionCount) - 2;
                            if (posCount % 2 == 1) {
                                --posCount;
                                addMiddlePosition = true;
                            }

                            var idxs = new List<int>();
                            idxs.Add(0);
                            if (posCount > 0) {
                                var distance = (badPositions.Count - 2) / (posCount + 1);
                                for (var j = 1; j <= posCount / 2; ++j) {
                                    idxs.Add(j * distance);
                                }
                            }

                            if (addMiddlePosition) {
                                idxs.Add(badPositions.Count / 2);
                            }

                            if (posCount > 0) {
                                var distance = (badPositions.Count - 2) / (posCount + 1);
                                for (var j = posCount / 2; j >= 1; --j) {
                                    idxs.Add(badPositions.Count - 1 - j * distance);
                                }
                            }

                            idxs.Add(badPositions.Count - 1);

                            var badPositionsForApproximation = new List<Position>();
                            foreach (var idx in idxs) {
                                badPositionsForApproximation.Add(badPositions[idx]);
                            }

                            var subPositions = bezierApp.Approximate(badPositionsForApproximation, v.linearPathStep, triangles);

                            var a1 = subPositions.First();
                            var a2 = subItems.First().a;
                            if (MMath.GetDistance(a1.originPoint, a2.originPoint) > 1 || MMath.GetDistance(a1.surfacePoint, a2.surfacePoint) > 1) {
                                //throw new Exception("Magic");
                            }

                            var b1 = subPositions.Last();
                            var b2 = subItems.Last().b;
                            if (MMath.GetDistance(b1.originPoint, b2.originPoint) > 1 || MMath.GetDistance(b1.surfacePoint, b2.surfacePoint) > 1) {
                                //throw new Exception("Magic");
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

                        // var newPositions = new List<Position>();
                        // foreach (var item in newRobotPathItems) {
                        //     newPositions.Add(item.a);
                        // }
                        // newPositions.Add(newRobotPathItems.Last().b);
                        //
                        // var linearApp = new LinearApproximation(false, pathFinderType == PathFinderType.IntersectionsWithSurfacesPathFinder);
                        // var linearPathItems = linearApp.Approximate(newPositions, v.linearPathStep, triangles);
                        //
                        // var newLinearPathItems = RobotPathProcessorBuilder.BuildRobotPathItems(newPositions);
                        simplifiedRobotPathProcessors[i] = RobotPathProcessorBuilder.Build(newRobotPathItems, v.paintSpeed, v.maxPaintRobotSpeed);
                        break;
                    }

                    attempts += 2;
                }

                Debug.Log("Attempts: " + attempts);
            }
        }

        if (addExtraParallelPaths) {
            var anchorPath = simplifiedRobotPathProcessors[simplifiedRobotPathProcessors.Count / 2];
            for (var i = 0; i < 10; ++i) {
                {
                    var j = simplifiedRobotPathProcessors.Count - 1 - i;

                    var rpp = simplifiedRobotPathProcessors[j];
                    var x = rpp.GetRobotPathItems().First().a.surfacePoint.x;
                    var newRobotPathItems = new List<RobotPathItem>();
                    foreach (var pathItem in anchorPath.GetRobotPathItems()) {
                        var a = new Position(
                            new Point(x, pathItem.a.originPoint.y, pathItem.a.originPoint.z),
                            new Point(pathItem.a.paintDirection.x, pathItem.a.paintDirection.y, pathItem.a.paintDirection.z),
                            new Point(x, pathItem.a.surfacePoint.y, pathItem.a.surfacePoint.z),
                            pathItem.a.type
                        );
                        var b = new Position(
                            new Point(x, pathItem.b.originPoint.y, pathItem.b.originPoint.z),
                            new Point(pathItem.b.paintDirection.x, pathItem.b.paintDirection.y, pathItem.b.paintDirection.z),
                            new Point(x, pathItem.b.surfacePoint.y, pathItem.b.surfacePoint.z),
                            pathItem.b.type
                        );
                        newRobotPathItems.Add(new RobotPathItem(a, b, pathItem.extraAccelerationMultiplier));
                    }

                    var newRpp = new RobotPathProcessor(newRobotPathItems);
                    newRpp.SetSurfaceSpeed(rpp.GetSurfaceSpeed());
                    newRpp.SetMaxOriginSpeed(rpp.GetMaxOriginSpeed());
                    simplifiedRobotPathProcessors[j] = newRpp;
                }
            }
        }

        if (useBaseSpeedCorrection) {
            for (var i = 0; i < simplifiedRobotPathProcessors.Count; ++i) {
                var rpp = simplifiedRobotPathProcessors[i];
                var pathItems = rpp.GetRobotPathItems();

                var planeGameObject = CreatePlaneGameObject(i);
                var planePosition = planeGameObject.transform.position;
                var n = planeGameObject.transform.up;
                Destroy(planeGameObject);

                var data = GetPaintDataForExport(new Plane(Utils.VtoP(n), Utils.VtoP(planePosition)));
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
                var extraMultiplier = avgValue / v.targetPaintThicknessOnSingleLine;

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
            var _isOrt = isOrt;
            isOrt = false;

            var notOptimizedRobotIndexes = new List<int>();
            var optimizedRobotIndexes = new List<int>();
            for (var i = 0; i < simplifiedRobotPathProcessors.Count; ++i) {
                var rpp = simplifiedRobotPathProcessors[i];
                var pathItems = rpp.GetRobotPathItems();

                var baseSpeeds = new List<double>();
                var baseDistances = new List<double>();
                foreach (var pathItem in pathItems) {
                    baseSpeeds.Add(pathItem.GetSpeed(v.paintSpeed));
                    baseDistances.Add(pathItem.GetDistance());
                }

                var planeGameObject = CreatePlaneGameObject(i);
                var planePosition = planeGameObject.transform.position;
                var n = planeGameObject.transform.up;
                Destroy(planeGameObject);

                var plane = new Plane(Utils.VtoP(n), Utils.VtoP(planePosition));

                var trianglePoints = new Dictionary<Triangle, List<Point>>();
                foreach (var t in triangles) {
                    foreach (var e in t.GetEdges()) {
                        var point = MMath.Intersect(plane, new Segment(e.p1, e.p2));
                        if (point != null) {
                            DictUtils.FillValueIfNotExists(trianglePoints, t, new List<Point>());
                            trianglePoints[t].Add(point);
                        }
                    }

                    Debug.Assert(!trianglePoints.ContainsKey(t) || trianglePoints[t].Count <= 2);
                    if (trianglePoints.ContainsKey(t) && trianglePoints[t].Count == 2) {
                        var a = trianglePoints[t].First();
                        var b = trianglePoints[t].Last();
                        var d = MMath.GetDistance(a, b);

                        var count = d / v.linearPathStep;
                        var step = d / count;
                        var direction = (b - a).Normalized;
                        Debug.Assert(MMath.GetDistance(a + (float)d * direction, b) < 1);
                        for (var j = 1.0f; j < count; ++j) {
                            trianglePoints[t].Add(a + direction * (float)step * j);
                        }
                    }
                }

                if (trianglePoints.Count == 0 || optimizedRobotIndexes.Count > 0) {
                    notOptimizedRobotIndexes.Add(i);
                    continue;
                }

                var paintResult = new DrawingSimulator().ProcessPoints(
                    new List<RobotPathProcessor>{rpp},
                    trianglePoints.Keys.ToList(),
                    trianglePoints,
                    10 * v.paintHeight,
                    v.paintRadius,
                    v.paintHeight,
                    commonSettings.GetPaintConsumptionRateInGameScale()
                );

                var idxByPoint = new Dictionary<Point, int>();
                foreach (var triangleAmount in paintResult.paintAmount) {
                    foreach (var pointAmount in triangleAmount.Value) {
                        DictUtils.FillValueIfNotExists(idxByPoint, pointAmount.Key, idxByPoint.Count);
                    }
                }

                var rRows = new List<List<double>>();
                var sRows = new List<List<double>>();
                var wRows = new List<List<double>>();

                var paintAmountByPointAndPathItem = GetPaintAmountByPointAndPathItem(paintResult);
                paintResult.triangles.Clear();
                paintResult.paintAmount.Clear();
                paintResult.detailedPaintAmountForItems.Clear();

                var points = idxByPoint.Keys.OrderBy(x => idxByPoint[x]).ToList();
                foreach (var point in points) {
                    var newRow = new List<double>();
                    foreach (var pathItem in pathItems) {
                        newRow.Add(paintAmountByPointAndPathItem[point][pathItem]);
                    }

                    rRows.Add(newRow);
                    sRows.Add(new List<double>{v.targetPaintThicknessOnSingleLine});
                }

                for (var j = 0; j < baseSpeeds.Count - 1; ++j) {
                    var newRRow = new List<double>();
                    for (var k = 0; k < j; ++k) {
                        newRRow.Add(0.0);
                    }

                    var alpha = 0.01;
                    newRRow.Add(alpha);
                    newRRow.Add(-alpha);

                    for (var k = j + 2; k < baseSpeeds.Count; ++k) {
                        newRRow.Add(0.0);
                    }

                    rRows.Add(newRRow);
                    sRows.Add(new List<double>{0.0});
                }

                for (var j = 0; j < rRows.Count; ++j) {
                    var newWRow = new List<double>();
                    for (var k = 0; k < rRows.Count; ++k) {
                        newWRow.Add(j == k ? 1.0 : 0.0);
                    }
                    wRows.Add(newWRow);
                }

                var rm = new Matrix(rRows);
                var sm = new Matrix(sRows);
                var wm = new Matrix(wRows);

                // var weights = GetWeights(rm, sm, wm, baseSpeeds, baseDistances);
                var result = Equations.QuadraticProgrammingASLEV2(
                    rm, sm, wm, baseSpeeds, baseDistances,
                    v.minPaintRobotSpeed, v.maxPaintRobotSpeed, v.maxPaintRobotAcceleration, !dontUseAdvancedPathSimplificationV2
                );

                var weights = new List<double>();
                for (var j = 0; j < result.RowCount; ++j) {
                    weights.Add(result.Get(j, 0));
                }

                var newPathItems = new List<RobotPathItem>();
                for (var j = 0; j < weights.Count; ++j) {
                    var pathItem = pathItems[j];
                    newPathItems.Add(new RobotPathItem(pathItem.a, pathItem.b, pathItem.extraAccelerationMultiplier / (float)weights[j]));
                }

                var newRpp = new RobotPathProcessor(newPathItems);
                newRpp.SetSurfaceSpeed(rpp.GetSurfaceSpeed());
                newRpp.SetMaxOriginSpeed(rpp.GetMaxOriginSpeed());
                simplifiedRobotPathProcessors[i] = newRpp;

                optimizedRobotIndexes.Add(i);
            }

            foreach (var i in notOptimizedRobotIndexes) {
                var nearestRobotId = -1;
                var distance = double.MaxValue;
                foreach (var j in optimizedRobotIndexes) {
                    var startI = simplifiedRobotPathProcessors[j].GetRobotPathItems().First().a;
                    var startJ = simplifiedRobotPathProcessors[j].GetRobotPathItems().First().a;
                    var newDistance = MMath.GetDistance(startI.originPoint, startJ.originPoint);
                    if (newDistance < distance) {
                        nearestRobotId = j;
                        distance = newDistance;
                    }
                }

                var pathItemsI = simplifiedRobotPathProcessors[i].GetRobotPathItems();
                var pathItemsJ = simplifiedRobotPathProcessors[nearestRobotId].GetRobotPathItems();
                for (var j = 0; j < pathItemsJ.Count; ++j) {
                    var pathItem = pathItemsI[j];
                    pathItemsI[j] = new RobotPathItem(pathItem.a, pathItem.b, pathItemsJ[j].extraAccelerationMultiplier);
                }

                var newRpp = new RobotPathProcessor(pathItemsI);
                newRpp.SetSurfaceSpeed(simplifiedRobotPathProcessors[nearestRobotId].GetSurfaceSpeed());
                newRpp.SetMaxOriginSpeed(simplifiedRobotPathProcessors[nearestRobotId].GetMaxOriginSpeed());
                simplifiedRobotPathProcessors[i] = newRpp;
            }

            isOrt = _isOrt;
        }
    }

    private double CalculateLimit(double v1, double w1, double v2, double w2, double acceleration, int iterationNumber) {
        var c = 1.0 - 0.01 * Math.Log10(iterationNumber + 1);
        return Math.Abs(w1 / v1 - w2 / v2) * (c * v.maxPaintRobotAcceleration / acceleration);
    }

    private List<double> GetWeights(Matrix rm, Matrix sm, Matrix wm, List<double> speeds, List<double> distances) {
        var weights = new List<double>();
        for (var j = 0; j < speeds.Count; ++j) {
            weights.Add(1.0);
        }

        var iterationCount = 0;
        var limits = new List<double>();
        for (var j = 0; j < speeds.Count - 1; ++j) {
            var acceleration = RobotPathItem.GetAcceleration(speeds[j], distances[j], speeds[j + 1], distances[j + 1]);
            var limit = CalculateLimit(speeds[j], weights[j], speeds[j + 1], weights[j + 1], acceleration, iterationCount);
            limits.Add(limit);
        }

        var usedLimitsIndexes = new Dictionary<int, bool>();
        var allTimes = new List<List<float>>();
        var ok = false;
        var aLog = new List<Tuple<double, double, int>>();
        while (!ok) {
            ++iterationCount;
            var times = new List<float>();
            var watch = Stopwatch.StartNew();

            var gRows = new List<List<double>>();
            var hList = new List<List<double>>();

            for (var j = 0; j < speeds.Count - 1; ++j) {
                if (usedLimitsIndexes.ContainsKey(j)) {
                    var newGRow = new List<double>();
                    for (var k = 0; k < j; ++k) {
                        newGRow.Add(0.0);
                    }

                    var v1 = speeds[j];
                    var v2 = speeds[j + 1];
                    var c = v1 / weights[j] - v2 / weights[j + 1] >= 0 ? -1.0 : 1.0;
                    newGRow.Add(c / v1);
                    newGRow.Add(-c / v2);
                    var currentLimit = Math.Abs(limits[j]);

                    for (var k = j + 2; k < speeds.Count; ++k) {
                        newGRow.Add(0.0);
                    }

                    gRows.Add(newGRow);
                    hList.Add(new List<double>{Math.Abs(currentLimit)});
                }
            }

            for (var j = 0; j < speeds.Count; ++j) {
                var newGRow = new List<double>();
                for (var k = 0; k < j; ++k) {
                    newGRow.Add(0.0);
                }

                newGRow.Add(1.0);

                for (var k = j + 1; k < speeds.Count; ++k) {
                    newGRow.Add(0.0);
                }

                gRows.Add(newGRow);
                hList.Add(new List<double>{speeds[j] / v.minPaintRobotSpeed});
            }

            for (var j = 0; j < speeds.Count; ++j) {
                var newGRow = new List<double>();
                for (var k = 0; k < j; ++k) {
                    newGRow.Add(0.0);
                }

                newGRow.Add(-1.0);

                for (var k = j + 1; k < speeds.Count; ++k) {
                    newGRow.Add(0.0);
                }

                gRows.Add(newGRow);
                hList.Add(new List<double>{-speeds[j] / v.maxPaintRobotSpeed});
            }

            watch.Stop();
            times.Add(watch.ElapsedMilliseconds); // 0
            watch.Restart();

            var gm = new Matrix(gRows);
            var hm = new Matrix(hList);
            var result = Equations.QuadraticProgrammingASLE(rm, sm, gm, hm, wm);

            watch.Stop();
            times.Add(watch.ElapsedMilliseconds); // 1
            watch.Restart();

            Debug.Assert(result.ColumnCount == 1);

            for (var j = 0; j < result.RowCount; ++j) {
                weights[j] = result.Get(j, 0);
            }

            ok = true;
            var aList = new List<double>();
            for (var j = 0; j < speeds.Count - 1; ++j) {
                var a = RobotPathItem.GetAcceleration(speeds[j] / weights[j], distances[j], speeds[j + 1] / weights[j + 1], distances[j + 1]);
                ok = ok && Math.Abs(a) < v.maxPaintRobotAcceleration;
                aList.Add(a);
            }

            if (!ok) {
                for (var j = 0; j < aList.Count; ++j) {
                    if (Math.Abs(aList[j]) > v.maxPaintRobotAcceleration || usedLimitsIndexes.ContainsKey(j)) {
                        limits[j] = CalculateLimit(speeds[j], weights[j], speeds[j + 1], weights[j + 1], aList[j], iterationCount);
                        DictUtils.FillValueIfNotExists(usedLimitsIndexes, j, true);
                    }
                }
            }

            var aMax = aList.Max();
            var aMin = aList.Min();
            aLog.Add(new Tuple<double, double, int>(aMax, aMin, usedLimitsIndexes.Count));

            watch.Stop();
            times.Add(watch.ElapsedMilliseconds); // 2
            allTimes.Add(times);
        }

        return weights;
    }

    private Dictionary<Point, Dictionary<RobotPathItem, double>> GetPaintAmountByPointAndPathItem(TexturePaintResult paintResult) {
        var paintAmountByPointAndPathItem = new Dictionary<Point, Dictionary<RobotPathItem, double>>();
        foreach (var pathItemAmount in paintResult.detailedPaintAmountForItems) {
            var pathItem = pathItemAmount.Key;

            foreach (var triangleAmount in pathItemAmount.Value) {
                foreach (var pointAmount in triangleAmount.Value) {
                    var point = pointAmount.Key;
                    DictUtils.FillValueIfNotExists(paintAmountByPointAndPathItem, point, new Dictionary<RobotPathItem, double>());
                    if (paintAmountByPointAndPathItem[point].ContainsKey(pathItem)) {
                        paintAmountByPointAndPathItem[point][pathItem] = (paintAmountByPointAndPathItem[point][pathItem] + pointAmount.Value) / 2.0f;
                    }
                    else {
                        paintAmountByPointAndPathItem[point].Add(pathItem, pointAmount.Value);
                    }
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
                    useBasePathSimplification = useBasePathSimplification,
                    baseSimplificationPointCount = baseSimplificationPointCount,
                    useBaseSpeedCorrection = useBaseSpeedCorrection,
                    useAdvancedPathSimplification = useAdvancedPathSimplification,
                    dontUseAdvancedPathSimplificationV2 = dontUseAdvancedPathSimplificationV2,
                    targetPaintThicknessOnSingleLine = targetPaintThicknessOnSingleLine,
                    targetPaintThicknessOnSurface = targetPaintThicknessOnSurface,
                    defaultOffsets = defaultOffsets,
                    ortOffsets = ortOffsets,

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
        useBasePathSimplification = settings.useBasePathSimplification;
        baseSimplificationPointCount = settings.baseSimplificationPointCount;
        useBaseSpeedCorrection = settings.useBaseSpeedCorrection;
        useAdvancedPathSimplification = settings.useAdvancedPathSimplification;
        dontUseAdvancedPathSimplificationV2 = settings.dontUseAdvancedPathSimplificationV2;
        targetPaintThicknessOnSingleLine = settings.targetPaintThicknessOnSingleLine;
        targetPaintThicknessOnSurface = settings.targetPaintThicknessOnSurface;
        defaultOffsets = settings.defaultOffsets;
        ortOffsets = settings.ortOffsets;

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

            var root = GameObject.Find("Objects-Samples-1Path (2)");
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

        texturePaintResult = new DrawingSimulator().ProcessPath(
            simplifiedRobotPathProcessors,
            triangles,
            10 * v.paintHeight,
            v.paintRadius,
            v.paintHeight,
            v.maxTriangleSquare,
            commonSettings.GetPaintConsumptionRateInGameScale()
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

    private GameObject CreatePlaneGameObject(int pathIdx = -1, Point centerOffset = null) {
        var triangles = GetFigureTriangles();
        var position = triangles.Aggregate(Point.Zero, (current, t) => current + t.o) / triangles.Count;
        if (pathIdx != -1) {
            var items = simplifiedRobotPathProcessors[pathIdx].GetRobotPathItems();
            position = items.Aggregate(Point.Zero, (current, t) => current + t.a.surfacePoint + t.b.surfacePoint) / (2 * items.Count);
        }

        if (centerOffset != null) {
            position += centerOffset;
        }

        var paintTexturePlanePosition = position;
        var paintTexturePlane = new Plane(
            position,
            position + Point.Up * CommonSettings.SCALE_IN_GAME,
            position + Point.Forward * CommonSettings.SCALE_IN_GAME
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

    private void CreatePlaneForExportPaintData(int pathIdx = -1, Point centerOffset = null) {
        Debug.Log("CreatePlaneForExportPaintData");

        var watch = Stopwatch.StartNew();

        DestroyPlaneForExportPaintData();
        paintTexturePlaneGameObject = CreatePlaneGameObject(pathIdx, centerOffset);

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of creating plane for export paint data");
    }

    private void DestroyPlaneForExportPaintData() {
        Destroy(paintTexturePlaneGameObject);
    }

    private List<Tuple<Point, double, double>> GetPaintDataForExport(Plane paintTexturePlane) {
        var triangles = GetFigureTriangles();

        var pointsDict = new Dictionary<Point, bool>();
        var trianglePoints = new Dictionary<Triangle, List<Point>>();
        foreach (var t in triangles) {
            foreach (var e in t.GetEdges()) {
                var point = MMath.Intersect(paintTexturePlane, new Segment(e.p1, e.p2));
                if (point != null) {
                    DictUtils.FillValueIfNotExists(trianglePoints, t, new List<Point>());
                    trianglePoints[t].Add(point);
                    DictUtils.FillValueIfNotExists(pointsDict, point, true);
                }
            }

            Debug.Assert(!trianglePoints.ContainsKey(t) || trianglePoints[t].Count <= 2);
            if (trianglePoints.ContainsKey(t) && trianglePoints[t].Count >= 2) {
                var tPoints = trianglePoints[t];
                tPoints = tPoints.OrderBy(p => isOrt ? p.x : p.z).ToList();

                var a = tPoints.First();
                var b = tPoints.Last();
                var d = MMath.GetDistance(a, b);

                var count = d / v.linearPathStep;
                var step = d / count;
                var direction = (b - a).Normalized;
                Debug.Assert(MMath.GetDistance(a + (float)d * direction, b) < 1);
                for (var j = 1.0f; j < count; ++j) {
                    var point = a + direction * (float)step * j;
                    trianglePoints[t].Add(point);
                    DictUtils.FillValueIfNotExists(pointsDict, point, true);
                }
            }
        }

        var paintResult = new DrawingSimulator().ProcessPoints(
            simplifiedRobotPathProcessors,
            trianglePoints.Keys.ToList(),
            trianglePoints,
            10 * v.paintHeight,
            v.paintRadius,
            v.paintHeight,
            commonSettings.GetPaintConsumptionRateInGameScale()
        );

        var points = pointsDict.Keys.ToList();
        if (!isOrt) {
            points = points.OrderBy(x => x.z).ToList();
        }
        else {
            points = points.OrderBy(x => x.x).ToList();
        }

        var valueByPoint = new Dictionary<Point, double>();
        foreach (var triangleAmount in paintResult.paintAmount) {
            foreach (var pointAmount in triangleAmount.Value) {
                if (valueByPoint.ContainsKey(pointAmount.Key)) {
                    valueByPoint[pointAmount.Key] = 0.5 * (valueByPoint[pointAmount.Key] + pointAmount.Value);
                }
                else {
                    valueByPoint.Add(pointAmount.Key, pointAmount.Value);
                }
            }
        }

        paintResult.triangles.Clear();
        paintResult.paintAmount.Clear();
        paintResult.detailedPaintAmountForItems.Clear();

        var result = new List<Tuple<Point, double, double>>();
        var distance = 0.0;
        for (var i = 0; i < points.Count; ++i) {
            if (i > 0) {
                distance += MMath.GetDistance(points[i - 1], points[i]);
            }
            result.Add(new Tuple<Point, double, double>(points[i], distance, valueByPoint[points[i]]));
        }

        return result;
    }

    private void ExportHeatMapSettings(string filename = "") {
        if (texturePaintResult is null) {
            Debug.Log("Unable ExportPaintData: texturePaintResult is null");
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
            var exportData = GetPaintDataForExport(new Plane(Utils.VtoP(n), Utils.VtoP(position)));
            watch.Stop();
            Debug.Log(watch.ElapsedMilliseconds + " ms. Time of export paint data");

            var strLines = new List<string>();
            foreach (var exportItem in exportData) {
                var p = exportItem.Item1;
                var distance = exportItem.Item2;
                var value = exportItem.Item3;
                strLines.Add($"({p.x};{p.y};{p.z})\t{distance}\t{value / CommonSettings.SCALE_IN_GAME}");
            }

            exportData.Clear();

            File.WriteAllLines(filename, strLines);
        }
    }

    private List<double> GetRppAnglesToFigure(List<Position> pathItems) {
        var triangles = GetFigureTriangles();

        var planes = new List<Plane>();
        foreach (var t in GetFigureTriangles()) {
            planes.Add(t.GetPlane());
        }

        var result = new List<double>();
        // Normal denormalization and redirection;
        for (var j = 0; j < pathItems.Count; ++j) {
            var item = pathItems[j];
            var ok = false;
            for (var k = 0; k < planes.Count; ++k) {
                var point = MMath.Intersect(
                    planes[k],
                    new Segment(
                        item.originPoint,
                        item.surfacePoint + item.paintDirection * (float)MMath.GetDistance(item.originPoint, item.surfacePoint) * 0.1f
                    )
                );
                if (!(point is null)) {
                    var t = triangles[k];

                    var s0 = t.GetSquare();
                    var s1 = new Triangle(point, t.p1, t.p2).GetSquare();
                    var s2 = new Triangle(point, t.p1, t.p3).GetSquare();
                    var s3 = new Triangle(point, t.p2, t.p3).GetSquare();

                    if (Math.Abs(s0 - s1 - s2 - s3) < 50) {
                        var dot = MMath.Dot(planes[k].GetNormal(), -item.paintDirection);
                        var angle = Math.Acos(Math.Min(dot, 1.0));
                        result.Add(angle / Math.PI * 180.0);
                        ok = true;
                        break;
                    }
                }
            }

            if (!ok) {
                result.Add(0.0);
            }
        }

        return result;
    }

    private void ExportPathNormalAngles(string filename = "") {
        Debug.Log("ExportPathNormalAngles");

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
                var pathItems = rpp.GetRobotPathItems();

                var positions = new List<Position>();
                for (var j = 0; j < pathItems.Count; ++j) {
                    positions.Add(pathItems[j].a);
                    if (j + 1 == pathItems.Count) {
                        positions.Add(pathItems[j].b);
                    }
                }

                var angles = GetRppAnglesToFigure(positions);
                for (var j = 0; j < positions.Count; ++j) {
                    var p = positions[j].surfacePoint;
                    strLines.Add($"({p.x};{p.y};{p.z})\t{angles[j]}");
                }
            }

            strLines.Add("\t");

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
                var sumOriginDistance = 0.0;
                var sumSurfaceDistance = 0.0;
                var pathItems = rpp.GetRobotPathItems();
                for (var j = 0; j < pathItems.Count; ++j) {
                    var pathItem = pathItems[j];

                    var speed = pathItem.GetSpeed(v.paintSpeed);
                    var acceleration = j + 2 < pathItems.Count ? RobotPathItem.GetAcceleration(pathItems[j], pathItems[j + 1], v.paintSpeed) : 0.0;
                    var originDistance = MMath.GetDistance(pathItem.a.originPoint, pathItem.b.originPoint);
                    var surfaceDistance = MMath.GetDistance(pathItem.a.surfacePoint, pathItem.b.surfacePoint);
                    var time = pathItem.GetTime(v.paintSpeed);

                    sumOriginDistance += originDistance;
                    sumSurfaceDistance += surfaceDistance;
                    strLines.Add($"{speed}\t{sumOriginDistance}\t{sumSurfaceDistance}\t{acceleration}\t{pathItem.GetSpeedMultiplier()}\t{pathItem.GetSpeedAccelerationMultiplier()}\t{time}");
                }

                strLines.Add("\t");
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
