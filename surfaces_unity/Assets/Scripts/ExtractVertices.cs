using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using JetBrains.Annotations;
using Library.PathApproximation;
using Library.PathFinders;
using Library.RobotPathBuilder;
using PathFinders;
using TriangleHandler;
using Unity.Profiling;
using UnityEditor.Build;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.PlayerLoop;
using UnityEngine.UI;
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
    public GameObject paintRobotPrefab;

    private GameObject paintRobot;
    private float paintTime;

    private List<GameObject> paintRobots;

    public bool drawSurfacePath = false;
    public bool drawOriginPath = false;
    public bool drawFromOriginToSurfacePath = false;
    public bool drawFoundPath = false;
    public bool drawApproximatedPath = false;
    public bool drawPathStepByStep = false;
    public bool drawDiffWithLinearPath = false;
    public bool drawLinearPath = false;
    public bool drawApproximatedPathWithSpeed = false;
    public bool drawApproximatedPathWithAcceleration = false;

    public PathFinderType pathFinderType;
    public float paintRadius;
    public float paintHeight;
    public float paintLateralAllowance;
    public float paintLongitudinalAllowance;
    public float paintSpeed = 1.0f;
    public float maxPaintRobotSpeed = 0.0f;
    public float maxPaintRobotAcceleration = 0.0f;
    public int maxPaintRobotPathSimplifyIterations = 10;
    public float scaleGameToWorld = 1.0f;
    public float timeScale = 1.0f;

    private Button savePathDataButton = null;
    private Button drawFigureButton = null;
    private Button calculatePathButton = null;
    private Button simplifyPathButton = null;
    private Button createPaintRobotsButtons = null;
    private GameObject rotationCube = null;
    private List<Triangle> baseTriangles = null;
    private List<Position> path = null;
    private List<Position> linearPath = null;
    private List<RobotPathProcessor> robotPathProcessors = null;

    readonly struct TriangleChunk {
        public readonly List<Triangle> Triangles;

        public TriangleChunk(List<Triangle> aTriangles) {
            Triangles = aTriangles;
        }
    }

    private Point VtoP(Vector3 v) => new Point(v.x, v.y, v.z);
    private Vector3 PtoV(Point p) => new Vector3(p.x, p.y, p.z);

    private void SetVertices(List<TriangleChunk> chunks) {
        var mf = gameObject.GetComponent<MeshFilter>();
        mf.mesh.Clear();

        var verticesDict = new Dictionary<Vector3, int>();
        var vertices = new List<Vector3>();
        var trianglesDescriptions = new List<int>();
        foreach (var chunk in chunks) {
            foreach (var triangle in chunk.Triangles) {
                foreach (var point in triangle.GetPoints()) {
                    if (!verticesDict.ContainsKey(PtoV(point))) {
                        verticesDict[PtoV(point)] = vertices.Count;
                        vertices.Add(PtoV(point));
                    }

                    trianglesDescriptions.Add(verticesDict[PtoV(point)]);
                }
            }
        }

        var count = chunks.Sum(chunk => chunk.Triangles.Count);
        Debug.Log(count);

        var mesh = mf.mesh;
        mesh.Clear();
        mesh.vertices = vertices.ToArray();
        mesh.triangles = trianglesDescriptions.ToArray();
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        mesh.RecalculateTangents();

        var mr = gameObject.GetComponent<MeshRenderer>();
        mr.material = material;
    }

    private List<Triangle> GetTriangles() {
        var watch = Stopwatch.StartNew();
        var result = new StlTriangleHandler(Path.Combine(Utils.GetDataFolder(), sampleName, "data.stl")).GetTriangles();
        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of reading figure");

        return result;
    }

    public void Start() {
        savePathDataButton = GameObject.Find("SavePathDataButton").GetComponent<Button>();
        savePathDataButton.onClick.AddListener(SavePathData);

        drawFigureButton = GameObject.Find("DrawFigureButton").GetComponent<Button>();
        drawFigureButton.onClick.AddListener(InitializeFigure);

        calculatePathButton = GameObject.Find("CalculateButton").GetComponent<Button>();
        calculatePathButton.onClick.AddListener(InitializePath);

        simplifyPathButton = GameObject.Find("SimplifyPathButton").GetComponent<Button>();
        simplifyPathButton.onClick.AddListener(SimplifyPath);

        createPaintRobotsButtons = GameObject.Find("CreatePaintRobotsButton").GetComponent<Button>();
        createPaintRobotsButtons.onClick.AddListener(CreatePaintRobots);

        rotationCube = GameObject.Find("RotationCube");

        baseTriangles = GetTriangles();
    }

    private List<Triangle> GetFigureTriangles() {
        var rotatedTriangles = new List<Triangle>();
        var rotation = rotationCube.transform.rotation;
        foreach (var triangle in baseTriangles) {
            rotatedTriangles.Add(new Triangle(VtoP(rotation * PtoV(triangle.p1)), VtoP(rotation * PtoV(triangle.p2)), VtoP(rotation * PtoV(triangle.p3))));
        }

        return rotatedTriangles;
    }

    private void DrawFigure(List<Triangle> triangles) {
        var chunks = new List<TriangleChunk>{
            new TriangleChunk(triangles),
        };

        SetVertices(chunks);
    }

    private void InitializeFigure() {
        DrawFigure(GetFigureTriangles());
    }

    private void InitializePath() {
        var triangles = GetFigureTriangles();
        DrawFigure(triangles);

        var watch = Stopwatch.StartNew();
        var pathFinder = PathFinderFactory.Create(pathFinderType, paintRadius, paintHeight, paintLateralAllowance, paintLongitudinalAllowance);
        path = pathFinder.GetPath(ref triangles);

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
        linearPath = app3.Approximate(path, 5.0f, triangles);

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of path approximation");
        watch.Restart();

        var pathBuilder = new RobotPathBuilder();
        robotPathProcessors = pathBuilder.Build(linearPath, paintSpeed * scaleGameToWorld);

        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of robot paths calculation");
    }

    private List<int> GetBadRobotPathItemIndexes(RobotPath robotPath) {
        var result = new List<int>();
        for (var j = 1; j < robotPath.items.Count - 1; ++j) {
            if (j < 120 || j > 350) {
                //continue;
            }
            var item = robotPath.items[j];
            if (item.speed > maxPaintRobotSpeed * scaleGameToWorld || Math.Abs(item.acceleration) > maxPaintRobotAcceleration * scaleGameToWorld) {
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
        var basePath = app.Approximate(path, 5.0f, triangles);

        var pathBuilder = new RobotPathBuilder();
        robotPathProcessors = pathBuilder.Build(basePath, paintSpeed * scaleGameToWorld);
        for (var i = 0; i < robotPathProcessors.Count; ++i) {
            var rp = robotPathProcessors[i];

            var attempts = 0;
            var maxAttempts = 600;
            while (attempts <= maxAttempts) {
                var robotPath = rp.GetRobotPathData(paintSpeed * scaleGameToWorld);
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

                var newRps = pathBuilder.Build(newPositions, paintSpeed * scaleGameToWorld);
                Debug.Assert(newRps.Count == 1);
                var newRp = newRps.First();
                var newRpBadItemIndexes = GetBadRobotPathItemIndexes(newRp.GetRobotPathData(paintSpeed * scaleGameToWorld));
                if (newRpBadItemIndexes.Count == 0 || attempts == maxAttempts) {
                    robotPathProcessors[i] = newRp;
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
        if (robotPathProcessors is null) {
            return;
        }

        paintRobots = new List<GameObject>();
        foreach (var rpp in robotPathProcessors) {
            var pos = rpp.Move(0.0f);
            var robot = Instantiate(paintRobotPrefab, PtoV(pos.point), Quaternion.LookRotation(PtoV(pos.direction), Vector3.up));
            var controller = robot.GetComponent<PaintRobotController>();
            controller.SetMaxSpeed(maxPaintRobotSpeed * scaleGameToWorld);
            paintRobots.Add(robot);
        }
    }

    private void MoveRobot(float time) {
        if (!(robotPathProcessors is null) && !(paintRobots is null) && (robotPathProcessors.Count == paintRobots.Count)) {
            var i = 0;
            var isFinished = true;
            foreach (var rp in robotPathProcessors) {
                isFinished = isFinished && rp.IsFinished();

                rp.SetBaseSpeed(paintSpeed * scaleGameToWorld);
                var pos = rp.Move(time);

                var robot = paintRobots[i];
                robot.transform.position = PtoV(pos.point);
                robot.transform.rotation = Quaternion.LookRotation(PtoV(pos.direction), Vector3.up);

                var controller = robot.GetComponent<PaintRobotController>();
                controller.SetMaxSpeed(maxPaintRobotSpeed * scaleGameToWorld);
                controller.SetCurrentSpeed(rp.GetCurrentSpeed());
                controller.SetMaxAcceleration(maxPaintRobotAcceleration * scaleGameToWorld);
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

    private void SavePathData() {
        if (!(robotPathProcessors is null)) {
            foreach (var rp in robotPathProcessors) {
                rp.SetBaseSpeed(1.0f);
            }

            var lines = new List<string>{$"{robotPathProcessors.Count}"};
            foreach (var rp in robotPathProcessors) {
                var pathLines = new List<string>();

                var pathData = rp.GetRobotPathData(paintSpeed * scaleGameToWorld);
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
        MoveRobot(Time.deltaTime * timeScale);
    }

    private List<RobotPath> robotPaths = new List<RobotPath>();

    private void UpdateRobotPathWithSpeed() {
        if (!(robotPathProcessors is null)) {
            var needUpdate = robotPathProcessors.Count != robotPaths.Count;
            foreach (var robotPath in robotPaths) {
                needUpdate = Math.Abs(robotPath.paintSpeed - paintSpeed * scaleGameToWorld) > 10e-8f;
                if (needUpdate) {
                    break;
                }
            }

            if (needUpdate) {
                robotPaths.Clear();
                foreach (var robotPathProcessor in robotPathProcessors) {
                    robotPaths.Add(robotPathProcessor.GetRobotPathData(paintSpeed * scaleGameToWorld));
                }
            }
        }
    }

    private void OnDrawGizmos() {
        // TODO Improve draw
        UpdateRobotPathWithSpeed();

        var maxCount = drawPathStepByStep ? Math.Floor(Time.time * 3) : 1e9;
        if (drawFromOriginToSurfacePath) {
            if (drawFoundPath && !(path is null)) {
                Gizmos.color = Color.magenta;
                for (var i = 0; i < (int)Math.Min(path.Count, maxCount); ++i) {
                    var pos = path[i];
                    Gizmos.DrawLine(PtoV(pos.originPoint), PtoV(pos.surfacePoint));
                }
            }

            if (drawApproximatedPath && !(robotPathProcessors is null)) {
                Gizmos.color = Color.cyan;
                foreach (var robotPath in robotPaths) {
                    foreach (var item in robotPath.items) {
                        var pos = item.position;
                        Gizmos.DrawLine(PtoV(pos.originPoint), PtoV(pos.surfacePoint));
                    }
                }
                // for (var i = 0; i < (int)Math.Min(aPath.Count, maxCount); ++i) {
                //     var pos = aPath[i];
                //     Gizmos.DrawLine(PtoV(pos.originPoint), PtoV(pos.surfacePoint));
                // }
            }
        }

        if (drawApproximatedPathWithSpeed) {
            foreach (var robotPath in robotPaths) {
                for (var i = 0; i < robotPath.items.Count - 1; ++i) {
                    Gizmos.color = PaintRobotController.GetSpeedColor(robotPath.items[i].speed, maxPaintRobotSpeed * scaleGameToWorld);
                    var pos1 = robotPath.items[i].position;
                    var pos2 = robotPath.items[i + 1].position;
                    if (pos1.type != Position.PositionType.Finish) {
                        Gizmos.DrawLine(PtoV(pos1.originPoint), PtoV(pos2.originPoint));
                    }
                }
            }
        }

        if (drawApproximatedPathWithAcceleration) {
            foreach (var robotPath in robotPaths) {
                for (var i = 0; i < robotPath.items.Count - 1; ++i) {
                    Gizmos.color = PaintRobotController.GetSpeedColor(robotPath.items[i].acceleration, maxPaintRobotAcceleration * scaleGameToWorld);
                    var pos1 = robotPath.items[i].position;
                    var pos2 = robotPath.items[i + 1].position;
                    if (pos1.type != Position.PositionType.Finish) {
                        Gizmos.DrawLine(PtoV(pos1.originPoint), PtoV(pos2.originPoint));
                    }
                }
            }
        }

        if (drawLinearPath && !(linearPath is null)) {
            // Gizmos.color = Color.grey;
            // for (var i = 0; drawSurfacePath && i < linearPath.Count - 1; ++i) {
            //     var pos1 = linearPath[i];
            //     var pos2 = linearPath[i + 1];
            //     if (pos1.type != Position.PositionType.Finish) {
            //         Gizmos.DrawLine(PtoV(pos1.surfacePoint), PtoV(pos2.surfacePoint));
            //     }
            // }
            //
            // Gizmos.color = Color.gray;
            // for (var i = 0; drawFromOriginToSurfacePath && i < linearPath.Count; ++i) {
            //     var pos = linearPath[i];
            //     Gizmos.DrawLine(PtoV(pos.originPoint), PtoV(pos.surfacePoint));
            // }

            Gizmos.color = Color.blue;
            for (var i = 0; drawOriginPath && i < linearPath.Count - 1; ++i) {
                var pos1 = linearPath[i];
                var pos2 = linearPath[i + 1];
                if (pos1.type != Position.PositionType.Finish) {
                    Gizmos.DrawLine(PtoV(pos1.originPoint), PtoV(pos2.originPoint));
                }
            }
        }

        if (drawOriginPath) {
            if (drawFoundPath && !(path is null)) {
                Gizmos.color = Color.black;
                for (var i = 0; i < (int)Math.Min(path.Count - 1, maxCount); ++i) {
                    var pos1 = path[i];
                    var pos2 = path[i + 1];
                    // if (pos1.type != Position.PositionType.Finish) {
                    Gizmos.DrawLine(PtoV(pos1.originPoint), PtoV(pos2.originPoint));
                    // }
                }
            }

            if (drawApproximatedPath) {
                Gizmos.color = Color.black;
                foreach (var robotPath in robotPaths) {
                    for (var i = 0; i < (int)Math.Min(robotPath.items.Count - 1, maxCount); ++i) {
                        var pos1 = robotPath.items[i].position;
                        var pos2 = robotPath.items[i + 1].position;
                        Gizmos.DrawLine(PtoV(pos1.originPoint), PtoV(pos2.originPoint));
                    }
                }
                // for (var i = 0; i < (int)Math.Min(aPath.Count - 1, maxCount); ++i) {
                //     var pos1 = aPath[i];
                //     var pos2 = aPath[i + 1];
                //     if (pos1.type != Position.PositionType.Finish) {
                //         Gizmos.DrawLine(PtoV(pos1.originPoint), PtoV(pos2.originPoint));
                //     }
                // }

                if (drawDiffWithLinearPath) {
                    Gizmos.color = Color.blue;
                    foreach (var robotPath in robotPaths) {
                        for (var i = 0; i < (int) Math.Min(robotPath.items.Count - 1, maxCount); ++i) {
                            var pos1 = robotPath.items[i].position;
                            var pos2 = robotPath.items[i + 1].position;
                            Gizmos.DrawLine(PtoV(pos1.originPoint), PtoV(pos2.originPoint));
                        }
                    }
                    // for (var i = 0; i < (int)Math.Min(aPath.Count - 1, maxCount); ++i) {
                    //     var pos1 = aPath[i];
                    //     var pos2 = aPath[i + 1];
                    //     if (pos1.type != Position.PositionType.Finish) {
                    //         Gizmos.DrawLine(PtoV(pos1.originPoint), PtoV(pos2.originPoint));
                    //     }
                    // }
                }

                // if (drawDiffWithLinearPath && !(linearPath is null)) {
                //     {
                //         Gizmos.color = Color.yellow;
                //         var i = 0;
                //         var j = 0;
                //         while (i < aPath.Count && j < linearPath.Count) {
                //             var i0 = i;
                //             for (; i < aPath.Count && aPath[i].type != Position.PositionType.Finish; ++i) {}
                //
                //             var j0 = j;
                //             for (; j < linearPath.Count && linearPath[j].type != Position.PositionType.Finish; ++j) {}
                //
                //             for (var k = 0; k < Math.Min(j - j0, i - i0) + 1; ++k) {
                //                 var pos1 = aPath[i0 + k].originPoint;
                //                 var pos2 = linearPath[j0 + k].originPoint;
                //                 Gizmos.DrawLine(PtoV(pos1), PtoV(pos2));
                //             }
                //
                //             ++i;
                //             ++j;
                //         }
                //     }
                //
                // if (drawLinearPath && !(linearPath is null)) {
                //     Gizmos.color = Color.white;
                //
                //     for (var i = 0; i < linearPath.Count - 1; ++i) {
                //         var pos1 = linearPath[i];
                //         var pos2 = linearPath[i + 1];
                //         if (pos1.type != Position.PositionType.Finish) {
                //             Gizmos.DrawLine(PtoV(pos1.surfacePoint), PtoV(pos2.surfacePoint));
                //         }
                //     }
                //
                //     for (var i = 0; i < linearPath.Count; ++i) {
                //         var pos = linearPath[i];
                //         Gizmos.DrawLine(PtoV(pos.originPoint), PtoV(pos.surfacePoint));
                //     }
                //
                //     for (var i = 0; i < linearPath.Count - 1; ++i) {
                //         var pos1 = linearPath[i];
                //         var pos2 = linearPath[i + 1];
                //         if (pos1.type != Position.PositionType.Finish) {
                //             Gizmos.DrawLine(PtoV(pos1.originPoint), PtoV(pos2.originPoint));
                //         }
                //     }
                // }
                // }
            }
        }

        if (drawSurfacePath) {
            if (drawFoundPath && !(path is null)) {
                Gizmos.color = Color.blue;
                for (var i = 0; i < (int)Math.Min(path.Count - 1, maxCount); ++i) {
                    var pos1 = path[i];
                    var pos2 = path[i + 1];
                    if (pos1.type != Position.PositionType.Finish) {
                        Gizmos.DrawLine(PtoV(pos1.surfacePoint) - PtoV(pos1.paintDirection) * 1e-4f, PtoV(pos2.surfacePoint) - PtoV(pos2.paintDirection) * 1e-4f);
                    }
                }
            }

            if (drawApproximatedPath) {
                // Gizmos.color = Color.green;
                Gizmos.color = Color.blue;
                foreach (var robotPath in robotPaths) {
                    for (var i = 0; i < (int)Math.Min(robotPath.items.Count - 1, maxCount); ++i) {
                        var pos1 = robotPath.items[i].position;
                        var pos2 = robotPath.items[i + 1].position;
                        Gizmos.DrawLine(PtoV(pos1.surfacePoint) - PtoV(pos1.paintDirection) * 5 * 1e-4f, PtoV(pos2.surfacePoint) - PtoV(pos2.paintDirection) * 5 * 1e-4f);
                    }
                }
            }
        }

        if (!(paintRobots is null)) {
            Gizmos.color = Color.red;
            foreach (var pr in paintRobots) {
                var position = pr.transform.position;
                Gizmos.DrawLine(position, position + pr.transform.forward * 30);
            }
        }
    }
}
