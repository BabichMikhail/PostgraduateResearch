using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using PathFinders;
using TriangleHandler;
using UnityEngine;
using UnityEngine.UI;
using Debug = UnityEngine.Debug;
using Vector3 = UnityEngine.Vector3;
using Point = Library.Generic.Point;
using Triangle = Library.Generic.Triangle;

[RequireComponent(typeof(MeshFilter))]
public class ExtractVertices : MonoBehaviour {
    public String sampleName;
    public Material material;
    public GameObject paintRobotPrefab;
    public float paintSpeed;

    private GameObject paintRobot;
    private float paintTime;
    private int currentPathIndex = -1;

    public bool drawSurfacePath = false;
    public bool drawOriginPath = false;
    public bool drawFromOriginToSurfacePath = false;
    public bool drawPathStepByStep = false;

    public PathFinderType pathFinderType;
    public float paintRadius;
    public float paintHeight;
    public float paintLateralAllowance;
    public float paintLongitudinalAllowance;

    private Button calculatePathButton = null;
    private Button drawFigureButton = null;
    private GameObject rotationCube = null;
    private List<Triangle> baseTriangles = null;
    private List<Position> path = null;

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
        calculatePathButton = GameObject.Find("CalculateButton").GetComponent<Button>();
        calculatePathButton.onClick.AddListener(InitializePath);

        drawFigureButton = GameObject.Find("DrawFigureButton").GetComponent<Button>();
        drawFigureButton.onClick.AddListener(InitializeFigure);

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

    private void InitializeFigure() {
        var triangles = GetFigureTriangles();
        var chunks = new List<TriangleChunk>{
            new TriangleChunk(triangles),
        };

        SetVertices(chunks);
    }

    private void InitializePath() {
        var triangles = GetFigureTriangles();
        var chunks = new List<TriangleChunk>{
            new TriangleChunk(triangles),
        };

        SetVertices(chunks);

        var watch = Stopwatch.StartNew();
        var pathFinder = PathFinderFactory.Create(pathFinderType, paintRadius, paintHeight, paintLateralAllowance, paintLongitudinalAllowance);
        path = pathFinder.GetPath(ref triangles);
        watch.Stop();
        Debug.Log(watch.ElapsedMilliseconds + " ms. Time of path calculation");
    }

    private void MoveRobot(float time) {}

    private void FixedUpdate() {
        if (currentPathIndex >= 0 && path.Count > currentPathIndex) {
            MoveRobot(Time.deltaTime);
        }
    }

    private void OnDrawGizmos() {
        var maxCount = drawPathStepByStep ? Math.Floor(Time.time * 3) : 1e9;
        if (path != null) {
            if (drawFromOriginToSurfacePath) {
                Gizmos.color = Color.magenta;
                for (var i = 0; i < (int) Math.Min(path.Count, maxCount); ++i) {
                    var pos = path[i];
                    Gizmos.DrawLine(PtoV(pos.originPosition), PtoV(pos.surfacePosition));
                }
            }

            Gizmos.color = Color.black;
            for (var i = 0; i < (int)Math.Min(path.Count - 1, maxCount); ++i) {
                var pos1 = path[i];
                var pos2 = path[i + 1];
                if (drawOriginPath && pos1.type != Position.PositionType.Finish) {
                    Gizmos.DrawLine(PtoV(pos1.originPosition), PtoV(pos2.originPosition));
                }
            }

            Gizmos.color = Color.blue;
            for (var i = 0; i < (int)Math.Min(path.Count - 1, maxCount); ++i) {
                var pos1 = path[i];
                var pos2 = path[i + 1];
                if (drawSurfacePath && pos1.type != Position.PositionType.Finish) {
                    Gizmos.DrawLine(PtoV(pos1.surfacePosition) - PtoV(pos1.paintDirection) * 1e-4f, PtoV(pos2.surfacePosition) - PtoV(pos1.paintDirection) * 1e-4f);
                }
            }
        }
    }
}
