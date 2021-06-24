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

    private GameObject paintRobot;
    private float paintTime;

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
    private List<Position> path = null;
    private List<Position> linearPath = null;
    private List<RobotPathProcessor> robotPathProcessors = new List<RobotPathProcessor>();
    private GameObject paintPointsHolder = null;

    private readonly List<RobotPath> robotPaths = new List<RobotPath>();

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

    private void SetColoredVertices(List<Triangle> triangles, Dictionary<Triangle, Dictionary<Point, float>> paintAmount) {
        var maxAmount = 0.0f;
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
                    color = new Color(amount / maxAmount, 0, 0, 1);
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
        createPlaneForExportPaintDataButton.onClick.AddListener(delegate { CreatePlaneForExportPaintData(0); });

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
        var folder = "1";

        InitializePath();
        SimplifyPath();
        CalculateTexturePaint();
        SaveData(Path.Combine(Utils.GetStoreFolder(), "generated", folder, "data", $"{sampleName}_{name}.txt"));

        for (var i = 0; i < robotPathProcessors.Count; ++i) {
            CreatePlaneForExportPaintData(i);
            ExportPaintData(Path.Combine(Utils.GetStoreFolder(), "generated", folder, "experiments", $"{sampleName}_{name}_path_{i}.txt"));
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

    public void InitializePath() {
        Debug.Log("InitializePath");

        var triangles = GetFigureTriangles();
        DrawFigure(triangles);

        Debug.Log($"Summary figure square: {triangles.Sum(x => x.GetSquare())}. Triangle count: {triangles.Count}.");

        var watch = Stopwatch.StartNew();
        var pathFinder = PathFinderFactory.Create(
            pathFinderType, v.paintRadius, v.paintHeight, v.paintLateralAllowance, v.paintLongitudinalAllowance, v.paintLineWidth
        );
        path = pathFinder.GetPath(triangles);

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
        // aPath = app.Approximate(path, 30.0f, triangles);

        // var app2 = new Bezier5Approximation(false);
        // aPath = app2.Approximate(aPath, 5.0f, triangles);

        var app3 = new LinearApproximation(false);
        linearPath = app3.Approximate(path, v.linearPathStep, triangles);

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of path approximation");
        watch.Restart();

        var pathBuilder = new RobotPathBuilder();
        robotPathProcessors = pathBuilder.Build(linearPath, v.paintSpeed, float.NaN);

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of robot path processors building");
    }

    private List<int> GetBadRobotPathItemIndexes(RobotPath robotPath) {
        var result = new List<int>();
        for (var j = 1; j < robotPath.items.Count - 1; ++j) {
            var item = robotPath.items[j];
            if (item.speed > v.maxPaintRobotSpeed || Math.Abs(item.acceleration) > v.maxPaintRobotAcceleration) {
                result.Add(j);
            }
        }

        return result;
    }

    private void SimplifyPath() {
        if (path is null) {
            return;
        }

        var triangles = GetFigureTriangles();
        var app = new LinearApproximation(true);
        var basePath = app.Approximate(path, v.linearPathStep, triangles);

        var pathBuilder = new RobotPathBuilder();
        robotPathProcessors = pathBuilder.Build(basePath, v.paintSpeed, float.NaN);
        for (var i = 0; i < robotPathProcessors.Count; ++i) {
            var rp = robotPathProcessors[i];

            var attempts = 0;
            var maxAttempts = 200;
            while (attempts <= maxAttempts) {
                var robotPath = rp.GetRobotPathData();
                Debug.Assert(robotPath.items.First().position.type == Position.PositionType.Start);
                Debug.Assert(robotPath.items.Last().position.type == Position.PositionType.Finish);

                var badItemIndexes = GetBadRobotPathItemIndexes(robotPath);
                var r = attempts + 1;
                var segments = new List<KeyValuePair<int, int>>();
                for (var j = 0; j < badItemIndexes.Count; ++j) {
                    var idx = badItemIndexes[j];
                    segments.Add(new KeyValuePair<int, int>(Math.Max(1, idx - r), Math.Min(robotPath.items.Count - 2, idx + r)));
                }

                var mergedSegments = new List<KeyValuePair<int, int>>();
                for (var j = 0; j < segments.Count; ++j) {
                    var k = j + 1;
                    while (k < segments.Count && segments[k - 1].Value >= segments[k].Key) {
                        ++k;
                    }

                    mergedSegments.Add(new KeyValuePair<int, int>(segments[j].Key, segments[k - 1].Value));
                    j = k - 1;
                }

                var positions = new List<Position>();
                foreach (var robotPosition in rp.GetRobotPositions()) {
                    positions.Add(robotPosition.position);
                }

                var newPositions = new List<Position>();
                var currentIndex = 0;
                foreach (var segment in mergedSegments) {
                    var from = segment.Key;
                    var count = segment.Value - segment.Key + 1;

                    newPositions.AddRange(positions.GetRange(currentIndex, from - currentIndex));
                    var bezierApp = new BezierNApproximation(false, Math.Max(count - 2, 1));
                    var subPositions = bezierApp.Approximate(positions.GetRange(from, count), 5.0f, triangles);
                    newPositions.AddRange(subPositions);
                    currentIndex = from + count;
                }
                newPositions.AddRange(positions.GetRange(currentIndex, positions.Count - currentIndex));

                var newRps = pathBuilder.Build(newPositions, v.paintSpeed, float.NaN);
                Debug.Assert(newRps.Count == 1);
                var newRp = newRps.First();
                var newRpBadItemIndexes = GetBadRobotPathItemIndexes(newRp.GetRobotPathData());
                if (newRpBadItemIndexes.Count == 0) {
                    robotPathProcessors[i] = newRp;
                    break;
                }
                if (attempts == maxAttempts) {
                    robotPathProcessors[i] = pathBuilder.Build(newPositions, v.paintSpeed, v.maxPaintRobotSpeed).First();
                    break;
                }

                ++attempts;
            }

            Debug.Log("Attempts: " + attempts);
        }

        robotPaths.Clear();
        UpdateRobotPathWithSpeed();
    }

    private void CreatePaintRobots() {
        var objectTriangles = GetFigureTriangles();
        paintRobots = new List<GameObject>();
        foreach (var rpp in robotPathProcessors) {
            var pos = rpp.Move(0.0f);
            var robot = Instantiate(paintRobotPrefab, Utils.PtoV(pos.point), Quaternion.LookRotation(Utils.PtoV(pos.direction), Vector3.up));
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
            var robotsData = new List<DataExport.RobotPathProcessorData>();
            var id = 0;
            foreach (var rpp in robotPathProcessors) {
                var drawingPositions = new List<Position>();
                if (paintRobots.Count == robotPathProcessors.Count) {
                    drawingPositions = paintRobots[id].GetComponent<PaintRobotController>().GetDrawingPositions();
                }
                robotsData.Add(new DataExport.RobotPathProcessorData(id, rpp, drawingPositions));
                ++id;
            }

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
                    drawPathStepByStep = commonSettings.drawPathStepByStep,
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
                robots = robotsData,
                path = path.Select(position => new DataExport.PositionData(position)).ToList(),
                linearPath = linearPath.Select(position => new DataExport.PositionData(position)).ToList(),
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
        commonSettings.drawPathStepByStep = settings.drawPathStepByStep;
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

        path = new List<Position>();
        foreach (var positionData in data.path) {
            path.Add(positionData.GetPosition());
        }

        linearPath = new List<Position>();
        foreach (var positionData in data.linearPath) {
            linearPath.Add(positionData.GetPosition());
        }

        robotPathProcessors = new List<RobotPathProcessor>();
        foreach (var rd in data.robots) {
            robotPathProcessors.Add(rd.GetRobotPathProcessor());
        }

        UpdateRobotPathWithSpeed();
        paintRobots.Clear();
        if (data.state.isPaintRobotsCreated) {
            CreatePaintRobots();

            var i = 0;
            foreach (var rd in data.robots) {
                paintRobots[i].GetComponent<PaintRobotController>().SetDrawingPositions(rd.GetDrawingPositions());
                ++i;
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
        if (path is null) {
            Debug.Log("Unable to CalculateTexturePaint. Path is null");
            return;
        }

        Debug.Log("CalculateTexturePaint");
        var triangles = GetFigureTriangles();

        var watch = Stopwatch.StartNew();

        var app = new LinearApproximation(false);
        var tmpPath = app.Approximate(path, v.linearPathStep, triangles);
        Debug.Log("Position count: " + tmpPath.Count);

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of prepare path");
        watch.Restart();

        var pathBuilder = new RobotPathBuilder();
        var fakeRobotPathProcessors = pathBuilder.Build(linearPath, v.paintSpeed, v.maxPaintRobotSpeed);
        var fakeRobotPaths = new List<RobotPath>();
        fakeRobotPathProcessors.ForEach(x => fakeRobotPaths.Add(x.GetRobotPathData()));
        // TODO simplify path;

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of calculate robot path with speeds");
        watch.Restart();

        var simulator = new DrawingSimulator();
        var paintConsumptionRateGameSizeUnitsCubicMeterPerSecond =
            commonSettings.paintConsumptionRateKgPerHour * 1000 / 3600 /
            commonSettings.paintDensityGramPerCubicMeter *
            commonSettings.paintAdhesionPart * Mathf.Pow(SCALE_IN_GAME, 3); // M^3 -> MM^3 (scale in game);
        texturePaintResult = simulator.ProcessPath(
            fakeRobotPaths, triangles, 20 * v.paintHeight, v.paintRadius, v.paintHeight, v.maxTriangleSquare,
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
            var items = robotPathProcessors[pathIdx].GetRobotPathData().items;
            position = items.Aggregate(Point.Zero, (current, t) => current + t.position.surfacePoint) / items.Count;
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

        if (!(paintTexturePlaneGameObject is null)) {
            Destroy(paintTexturePlaneGameObject);
        }

        paintTexturePlaneGameObject = CreatePlaneGameObject(pathIdx);

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of creating plane for export paint data");
    }

    private List<string> GetExportPaintData(Plane paintTexturePlane) {
        var watch = Stopwatch.StartNew();

        var lines = new List<Line>();
        var values = new List<float>();
        foreach (var t in texturePaintResult.triangles) {
            var points = new List<Point>();
            var originPoints = new List<Point>();
            var q = new List<bool>();
            var pp = new List<Point>();
            foreach (var e in t.GetEdges()) {
                var ok = false;
                var point = MMath.Intersect(paintTexturePlane, new Line(e.p1, e.p2), true);

                pp.Add(point);

                if (!(point is null)) {
                    ok = true;
                    points.Add(point);
                    originPoints.Add(e.p1);
                    originPoints.Add(e.p2);
                }

                q.Add(ok);
            }

            if (points.Count != 2 && points.Count != 0) {
                Debug.Log("Illegal magic");
            }

            if (points.Count == 2) {
                var line = new Line(points[0], points[1]);
                lines.Add(line);
                var info = texturePaintResult.paintAmount[t];
                values.Add((info[originPoints[0]] + info[originPoints[1]] + info[originPoints[2]] + info[originPoints[3]]) / 4.0f / SCALE_IN_GAME);
            }
        }

        if (lines.Count > 0) {
            var idxByLine = new Dictionary<Line, int>();
            var linesByPoint = new Dictionary<Point, List<Line>>();
            foreach (var line in lines) {
                idxByLine.Add(line, idxByLine.Count);

                foreach (var p in line.GetPoints()) {
                    if (!linesByPoint.ContainsKey(p)) {
                        linesByPoint.Add(p, new List<Line>());
                    }
                    linesByPoint[p].Add(line);
                }
            }

            var newLines = new List<Line>();
            Point lastPoint = null;
            var processedLines = new Dictionary<Line, bool>();
            var processedPoints = new Dictionary<Point, bool>();
            while (processedLines.Count != lines.Count) {
                var ok = false;
                if (!(lastPoint is null)) {
                    foreach (var line in linesByPoint[lastPoint]) {
                        if (!processedLines.ContainsKey(line)) {
                            processedLines.Add(line, true);
                            newLines.Add(line);
                            ok = true;
                            break;
                        }
                    }

                    if (!ok) {
                        processedPoints.Add(lastPoint, true);
                        var lastLine = newLines.Last();
                        lastPoint = lastPoint == lastLine.p1 ? lastLine.p2 : lastLine.p1;
                        if (processedPoints.ContainsKey(lastPoint)) {
                            lastPoint = null;
                        }
                    }
                }

                if (!ok && lastPoint is null) {
                    Edge e = null;
                    if (newLines.Count > 0) {
                        e = new Edge(newLines.Last().p1, newLines.Last().p2);
                    }
                    foreach (var line in lines) {
                        if (!processedLines.ContainsKey(line)) {
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

            Debug.Assert(newLines.Count == lines.Count);

            var newValues = new List<float>();
            foreach (var line in newLines) {
                newValues.Add(values[idxByLine[line]]);
            }

            lines = newLines;
            values = newValues;
        }

        var strLines = new List<string>();
        for (var i = 0; i < lines.Count; ++i) {
            var p1 = lines[i].p1;
            var p2 = lines[i].p2;
            strLines.Add($"({p1.x};{p1.y};{p1.z})-({p2.x};{p2.y};{p2.z})\t{values[i]}");
        }

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of export paint data");

        return strLines;
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
            File.WriteAllLines(filename, GetExportPaintData(new Plane(Utils.VtoP(n), Utils.VtoP(position))));
        }
    }

    private void MoveRobot(float time) {
        Debug.Assert(robotPathProcessors.Count == paintRobots.Count || paintRobots.Count == 0);
        if (robotPathProcessors.Count == paintRobots.Count) {
            var i = 0;
            var isFinished = true;
            foreach (var rp in robotPathProcessors) {
                isFinished = isFinished && rp.IsFinished();

                rp.SetBaseSpeed(v.paintSpeed);
                var pos = rp.Move(time);

                var robot = paintRobots[i];
                robot.transform.position = Utils.PtoV(pos.point);
                robot.transform.rotation = Quaternion.LookRotation(Utils.PtoV(pos.direction), Vector3.up);

                var controller = robot.GetComponent<PaintRobotController>();
                controller.SetMaxSpeed(v.maxPaintRobotSpeed);
                controller.SetCurrentSpeed(rp.GetCurrentSpeed());
                controller.SetMaxAcceleration(v.maxPaintRobotAcceleration);
                controller.SetCurrentAcceleration(rp.GetCurrentAcceleration());

                ++i;
            }

            if (isFinished) {
                foreach (var rp in robotPathProcessors) {
                    rp.Reset();
                }
            }
        }
    }

    private string StoreState(Point position, float speed, float acceleration, float distance) {
        return string.Format("{0} {1} {2} {3} {4} {5}",
            position.x.ToString(new CultureInfo("en-US")),
            position.y.ToString(new CultureInfo("en-US")),
            position.z.ToString(new CultureInfo("en-US")),
            speed.ToString(new CultureInfo("en-US")),
            acceleration.ToString(new CultureInfo("en-US")),
            distance.ToString(new CultureInfo("en-US"))
        );
    }

    private void ExportPathData() {
        if (!(robotPathProcessors is null)) {
            foreach (var rp in robotPathProcessors) {
                rp.SetBaseSpeed(v.paintSpeed);
            }

            var lines = new List<string>{$"{robotPathProcessors.Count}"};
            foreach (var rp in robotPathProcessors) {
                var pathLines = new List<string>();

                var pathData = rp.GetRobotPathData();
                foreach (var item in pathData.items) {
                    pathLines.Add(StoreState(item.position.originPoint, item.speed, item.acceleration, 0.0f));
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

        MoveRobot(Time.deltaTime * timeScale);

        if (needRunExperiment) {
            RunExperiment();
            needRunExperiment = false;
            gameObject.SetActive(false);
        }
    }

    private void UpdateRobotPathWithSpeed() {
        if (!(robotPathProcessors is null)) {
            var needUpdate = robotPathProcessors.Count != robotPaths.Count;
            foreach (var robotPath in robotPaths) {
                needUpdate = Math.Abs(robotPath.paintSpeed - v.paintSpeed) > 10e-8f;
                if (needUpdate) {
                    break;
                }
            }

            if (needUpdate) {
                robotPaths.Clear();
                foreach (var robotPathProcessor in robotPathProcessors) {
                    robotPathProcessor.SetBaseSpeed(v.paintSpeed);
                    robotPaths.Add(robotPathProcessor.GetRobotPathData());
                }
            }
        }
    }

    private void OnDrawGizmos() {
        if (commonSettings is null) {
            return;
        }

        UpdateRobotPathWithSpeed();

        var maxCount = commonSettings.drawPathStepByStep ? Math.Floor(Time.time * 3) : 1e9;
        if (commonSettings.drawFromOriginToSurfacePath) {
            if (commonSettings.drawFoundPath && !(path is null)) {
                Gizmos.color = Color.magenta;
                for (var i = 0; i < (int)Math.Min(path.Count, maxCount); ++i) {
                    var pos = path[i];
                    Gizmos.DrawLine(Utils.PtoV(pos.originPoint), Utils.PtoV(pos.surfacePoint));
                }
            }

            if (commonSettings.drawApproximatedPath && !(robotPathProcessors is null)) {
                Gizmos.color = Color.cyan;
                foreach (var robotPath in robotPaths) {
                    foreach (var item in robotPath.items) {
                        var pos = item.position;
                        Gizmos.DrawLine(Utils.PtoV(pos.originPoint), Utils.PtoV(pos.surfacePoint));
                    }
                }
            }
        }

        if (commonSettings.drawApproximatedPathWithSpeed) {
            foreach (var robotPath in robotPaths) {
                for (var i = 0; i < robotPath.items.Count - 1; ++i) {
                    Gizmos.color = PaintRobotController.GetSpeedColor(robotPath.items[i].speed, v.maxPaintRobotSpeed);
                    var pos1 = robotPath.items[i].position;
                    var pos2 = robotPath.items[i + 1].position;
                    if (pos1.type != Position.PositionType.Finish) {
                        Gizmos.DrawLine(Utils.PtoV(pos1.originPoint), Utils.PtoV(pos2.originPoint));
                    }
                }
            }
        }

        if (commonSettings.drawApproximatedPathWithAcceleration) {
            foreach (var robotPath in robotPaths) {
                for (var i = 0; i < robotPath.items.Count - 1; ++i) {
                    Gizmos.color = PaintRobotController.GetSpeedColor(robotPath.items[i].acceleration, v.maxPaintRobotAcceleration);
                    var pos1 = robotPath.items[i].position;
                    var pos2 = robotPath.items[i + 1].position;
                    if (pos1.type != Position.PositionType.Finish) {
                        Gizmos.DrawLine(Utils.PtoV(pos1.originPoint), Utils.PtoV(pos2.originPoint));
                    }
                }
            }
        }

        if (commonSettings.drawLinearPath && !(linearPath is null)) {
            if (commonSettings.drawOriginPath) {
                Gizmos.color = Color.blue;
                for (var i = 0; i < linearPath.Count - 1; ++i) {
                    var pos1 = linearPath[i];
                    var pos2 = linearPath[i + 1];
                    if (pos1.type != Position.PositionType.Finish) {
                        Gizmos.DrawLine(Utils.PtoV(pos1.originPoint), Utils.PtoV(pos2.originPoint));
                    }
                }
            }
        }

        if (commonSettings.drawOriginPath) {
            if (commonSettings.drawFoundPath && !(path is null)) {
                Gizmos.color = Color.black;
                for (var i = 0; i < (int)Math.Min(path.Count - 1, maxCount); ++i) {
                    var pos1 = path[i];
                    var pos2 = path[i + 1];
                    if (pos1.type != Position.PositionType.Finish) {
                        Gizmos.DrawLine(Utils.PtoV(pos1.originPoint), Utils.PtoV(pos2.originPoint));
                    }
                }
            }

            if (commonSettings.drawApproximatedPath) {
                Gizmos.color = Color.black;
                foreach (var robotPath in robotPaths) {
                    for (var i = 0; i < (int)Math.Min(robotPath.items.Count - 1, maxCount); ++i) {
                        var pos1 = robotPath.items[i].position;
                        var pos2 = robotPath.items[i + 1].position;
                        Gizmos.DrawLine(Utils.PtoV(pos1.originPoint), Utils.PtoV(pos2.originPoint));
                    }
                }
            }
        }

        if (commonSettings.drawSurfacePath) {
            if (commonSettings.drawFoundPath && !(path is null)) {
                Gizmos.color = Color.blue;
                for (var i = 0; i < (int)Math.Min(path.Count - 1, maxCount); ++i) {
                    var pos1 = path[i];
                    var pos2 = path[i + 1];
                    if (pos1.type != Position.PositionType.Finish) {
                        Gizmos.DrawLine(Utils.PtoV(pos1.surfacePoint), Utils.PtoV(pos2.surfacePoint));
                    }
                }
            }

            if (commonSettings.drawApproximatedPath) {
                Gizmos.color = Color.blue;
                foreach (var robotPath in robotPaths) {
                    for (var i = 0; i < (int)Math.Min(robotPath.items.Count - 1, maxCount); ++i) {
                        var pos1 = robotPath.items[i].position;
                        var pos2 = robotPath.items[i + 1].position;
                        Gizmos.DrawLine(Utils.PtoV(pos1.surfacePoint), Utils.PtoV(pos2.surfacePoint));
                    }
                }
            }
        }
    }
}
