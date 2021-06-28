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
        var folder = "2";

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
            while (attempts <= maxAttempts) {
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
                        badPositions.Add(robotPathItems.Last().b);

                        var subPositions = bezierApp.Approximate(badPositions, v.linearPathStep, triangles);

                        newRobotPathItems.AddRange(RobotPathProcessorBuilder.BuildRobotPathItems(subPositions));
                    }
                    currentIndex = from + count;
                }
                newRobotPathItems.AddRange(robotPathItems.GetRange(currentIndex, robotPathItems.Count - currentIndex));

                var newRp = RobotPathProcessorBuilder.Build(newRobotPathItems, v.paintSpeed, float.NaN);
                var newRpBadItemIndexes = GetBadRobotPathItemIndexes(newRp.GetRobotPathItems());
                if (newRpBadItemIndexes.Count == 0) {
                    simplifiedRobotPathProcessors[i] = newRp;
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

    }

    private void CreatePaintRobots() {
        var objectTriangles = GetFigureTriangles();
        paintRobots = new List<GameObject>();
        // TODO
        // foreach (var rpp in simplifiedRobotPathProcessors) {
        //     var pos = rpp.Move(0.0f);
        //     var robot = Instantiate(paintRobotPrefab, Utils.PtoV(pos.point), Quaternion.LookRotation(Utils.PtoV(pos.direction), Vector3.up));
        //     var controller = robot.GetComponent<PaintRobotController>();
        //     controller.SetMaxSpeed(v.maxPaintRobotSpeed);
        //     controller.SetPaintHeight(v.paintHeight);
        //     controller.SetPaintRadius(v.paintRadius);
        //     controller.SetPointGenerationSpeed(pointPerSecondDrawingSpeed);
        //     controller.SetObjectTriangles(objectTriangles);
        //     robot.transform.localScale *= paintRobotScale;
        //     paintRobots.Add(robot);
        // }
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
            foreach (var rpp in simplifiedRobotPathProcessors) {
                var drawingPositions = new List<Position>();
                if (paintRobots.Count == simplifiedRobotPathProcessors.Count) {
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
                // TODO fix save data
                // path = path.Select(position => new DataExport.PositionData(position)).ToList(),
                // linearPath = linearPath.Select(position => new DataExport.PositionData(position)).ToList(),
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

        // TODO
        // path = new List<Position>();
        // foreach (var positionData in data.path) {
        //     path.Add(positionData.GetPosition());
        // }
        //
        // linearPath = new List<Position>();
        // foreach (var positionData in data.linearPath) {
        //     linearPath.Add(positionData.GetPosition());
        // }

        simplifiedRobotPathProcessors = new List<RobotPathProcessor>();
        foreach (var rd in data.robots) {
            var rpp = rd.GetRobotPathProcessor();
            rpp.SetSurfaceSpeed(v.paintSpeed);
            simplifiedRobotPathProcessors.Add(rpp);
        }

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

    private List<string> GetPaintDataForExport(TexturePaintResult paintResult, Plane paintTexturePlane) {
        var lines = new List<Line>();
        var values = new List<float>();
        foreach (var t in paintResult.triangles) {
            var points = new List<Point>();
            var originPoints = new List<Point>();
            foreach (var e in t.GetEdges()) {
                var point = MMath.Intersect(paintTexturePlane, new Line(e.p1, e.p2), true);

                if (!(point is null)) {
                    points.Add(point);
                    originPoints.Add(e.p1);
                    originPoints.Add(e.p2);
                }
            }

            if (points.Count != 2 && points.Count != 0) {
                Debug.Log("Illegal magic");
            }

            if (points.Count == 2) {
                var line = new Line(points[0], points[1]);
                lines.Add(line);
                var info = paintResult.paintAmount[t];
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

            var watch = Stopwatch.StartNew();
            var exportData = GetPaintDataForExport(texturePaintResult, new Plane(Utils.VtoP(n), Utils.VtoP(position)));
            watch.Stop();
            Debug.Log(watch.ElapsedMilliseconds + " ms. Time of export paint data");

            File.WriteAllLines(filename, exportData);
        }
    }

    private void MoveRobot(float time) {
        Debug.Assert(simplifiedRobotPathProcessors.Count == paintRobots.Count || paintRobots.Count == 0);
        // TODO fix move robot
        // if (simplifiedRobotPathProcessors.Count == paintRobots.Count) {
        //     var i = 0;
        //     var isFinished = true;
        //     foreach (var rp in simplifiedRobotPathProcessors) {
        //         isFinished = isFinished && rp.IsFinished();
        //
        //         var pos = rp.Move(time);
        //
        //         var robot = paintRobots[i];
        //         robot.transform.position = Utils.PtoV(pos.point);
        //         robot.transform.rotation = Quaternion.LookRotation(Utils.PtoV(pos.direction), Vector3.up);
        //
        //         var controller = robot.GetComponent<PaintRobotController>();
        //         controller.SetMaxSpeed(v.maxPaintRobotSpeed);
        //         controller.SetCurrentSpeed(rp.GetCurrentOriginSpeed());
        //         controller.SetMaxAcceleration(v.maxPaintRobotAcceleration);
        //         controller.SetCurrentAcceleration(rp.GetCurrentAcceleration());
        //
        //         ++i;
        //     }
        //
        //     if (isFinished) {
        //         foreach (var rp in simplifiedRobotPathProcessors) {
        //             rp.Reset();
        //         }
        //     }
        // }
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
