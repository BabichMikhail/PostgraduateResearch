using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using Generic;
using PathFinders;
using TriangleHandler;
using UnityEngine;
using VertexHandler;
using Debug = UnityEngine.Debug;
using Vector3 = UnityEngine.Vector3;

public class ExtractVertices : MonoBehaviour {
    public Material material;
    public float h;
    public bool drawSubTriangles;
    public bool drawNormals;

    public bool drawSurfacePath = false;
    public bool drawOriginPath = false;
    public bool drawFromOriginToSurfacePath = false;

    private List<Position> path = null;

    [DllImport("msvcrt.dll", CallingConvention = CallingConvention.Cdecl)]
    public static extern int printf(string format, __arglist);

    [DllImport("msvcrt.dll", CallingConvention = CallingConvention.Cdecl)]
    public static extern int scanf(string format, __arglist);

    private IVertexHandler vertexHandler;

    private class VectorComparer : IComparer<Vector3> {
        private Func<Vector3, float> getValueFunc;
        private float accuracy;

        public VectorComparer(Func<Vector3, float> aGetValueFunc, float aAccuracy) {
            getValueFunc = aGetValueFunc;
            accuracy = aAccuracy;
        }

        public int Compare(Vector3 a, Vector3 b) {
            var result = (int)((getValueFunc(a) - getValueFunc(b)) / accuracy);
            if (result == 0) {
                result = (int)((a.x - b.x) / accuracy);
            }
            if (result == 0) {
                result = (int)((a.y - b.y) / accuracy);
            }
            if (result == 0) {
                result = (int)((a.z - b.z) / accuracy);
            }

            return result;
        }
    }

    private interface ITriangleBuilder {
        List<VertexHelper.Triangle> getTriangles(List<Vector3> vertices);
    }

    private class MyTriangleBuilder : ITriangleBuilder {
        private List<Vector3> getNearestVertices(Vector3 currentVertex, List<Vector3> vertices, Func<Vector3, float> getValueFunc) {
            var sortedVertices = vertices.ToList();
            sortedVertices.Sort(new VectorComparer(getValueFunc, ACCURACY));

            var items = new List<Vector3>();
            print(currentVertex);
            print(vertices.Count);
            var index = 0;
            for (var i = 0; i < vertices.Count; ++i) {
                if (currentVertex == vertices[i]) {
                    index = i;
                    break;
                }
            }
            // var index = sortedVertices.BinarySearch(currentVertex, new VectorComparer(getValueFunc, ACCURACY));
            Debug.Log(index);
            // Debug.Log("Begin");
            // Debug.Log("Search: " + currentVertex.x + " " + currentVertex.y + " " + currentVertex.z);

            // Debug.Log("Found : " + index + " " + sortedVertices[index].x + " " + sortedVertices[index].y + " " + sortedVertices[index].z);
            for (var i = index - 1; i >= 0 && Math.Abs(getValueFunc(sortedVertices[index - 1]) - getValueFunc(sortedVertices[i])) < ACCURACY; --i) {
                items.Add(sortedVertices[i]);
                // Debug.Log("Add: " + i + " " + sortedVertices[i].x + " " + sortedVertices[i].y + " " + sortedVertices[i].z);
            }

            for (var i = index + 1; i < sortedVertices.Count && Math.Abs(getValueFunc(sortedVertices[index + 1]) - getValueFunc(sortedVertices[i])) < ACCURACY; ++i) {
                items.Add(sortedVertices[i]);
                // Debug.Log("Add: " + i + " " + sortedVertices[i].x + " " + sortedVertices[i].y + " " + sortedVertices[i].z);
            }

            // Debug.Log(items.Count);
            return items;
        }

        public List<VertexHelper.Triangle> getTriangles(List<Vector3> vertices) {
            // Debug.Log("Initialize");
            // foreach (var vertex in vertices) {
            //     // Debug.Log("Vertex: " + vertex.x + " " + vertex.y + " " + vertex.z);
            // }

            // Debug.Log("Initialized");
            var triangles = new List<VertexHelper.Triangle>();

            var indexValue = 0;
            var indexByVertex = new Dictionary<Vector3, int>();
            foreach (var vertex in vertices) {
                indexByVertex[vertex] = indexValue;
                ++indexValue;
            }

            if (vertices.Count >= 3) {
                // var parents = new Dictionary<int, int>();
                var processed = new Dictionary<int, bool>();
                var queue = new Queue<int>();
                var watchIndex = 0;

                while (watchIndex < vertices.Count) {
                    if (!processed.ContainsKey(watchIndex)) {
                        queue.Enqueue(watchIndex);
                        // Debug.Log("Watch: " + watchIndex);
                    }

                    ++watchIndex;
                    while (queue.Count > 0) {
                        var index = queue.Dequeue();
                        if (processed.ContainsKey(index)) {
                            continue;
                        }

                        // Debug.Log("Process: " + index);
                        processed[index] = true;
                        // Debug.Log(index);
                        var vertex = vertices[index];
                        // Debug.Log("Process Vertex: " + vertex.x + " " + vertex.y + " " + vertex.z);

                        // Debug.Log("Process x");
                        var items = getNearestVertices(vertex, vertices, item => item.x);
                        // Debug.Log("Process y");
                        items.AddRange(getNearestVertices(vertex, vertices, item => item.y));
                        // Debug.Log("Process z");
                        items.AddRange(getNearestVertices(vertex, vertices, item => item.z));
                        // Debug.Log(items.Count);
                        var uniqueItems = items.Distinct().ToList();
                        // Debug.Log(uniqueItems.Count);
                        var log = "Items:";
                        foreach (var item in items) {
                            log += item.x + " " + item.y + " " + item.z + "\n";
                        }
                        // Debug.Log(log);

                        uniqueItems.Sort(new VectorComparer(x => Vector3.SqrMagnitude(x - vertex), ACCURACY));

                        var maxI = 1;
                        while (maxI < uniqueItems.Count && Math.Abs(Vector3.SqrMagnitude(vertex - uniqueItems[1]) - Vector3.SqrMagnitude(vertex - uniqueItems[maxI])) < ACCURACY) {
                            ++maxI;
                        }

                        // Debug.Log("MaxI: " + maxI);

                        for (var i = 0; i < maxI; ++i) {
                            for (var j = i + 1; j < Math.Max(2, maxI); ++j) {
                                var usedI = processed.ContainsKey(indexByVertex[uniqueItems[i]]);
                                var usedJ = processed.ContainsKey(indexByVertex[uniqueItems[j]]);
                                if (!(processed.ContainsKey(indexByVertex[uniqueItems[i]]) ^ processed.ContainsKey(indexByVertex[uniqueItems[j]]))) {
                                    var triangle = new VertexHelper.Triangle(
                                        vertex,
                                        uniqueItems[i],
                                        uniqueItems[j]
                                    );
                                    // var triangle = new List<Vector3>();
                                    // triangle.Add(vertex);
                                    // triangle.Add(uniqueItems[i]);
                                    // triangle.Add(uniqueItems[j]);
                                    triangles.Add(triangle);

                                    var line = "Add Triangle: ";
                                    foreach (var v in triangle.GetVertices()) {
                                        line += "(" + v.x + "  " + " " + v.y + " " + v.z + ") ";
                                    }

                                    // Debug.Log(line);
                                }
                            }

                            if (!processed.ContainsKey(indexByVertex[uniqueItems[i]])) {
                                queue.Enqueue(indexByVertex[uniqueItems[i]]);
                            }
                        }
                    }
                }
            }

            return triangles;
        }
    }

    private class GreedyBuilder : ITriangleBuilder {
        public List<VertexHelper.Triangle> getTriangles(List<Vector3> vertices) {
            var triangles = new List<VertexHelper.Triangle>();

            var usedVertices = new Dictionary<int, bool>();

            var i1 = 0;
            var i2 = 1;
            var distance = Vector3.SqrMagnitude(vertices[0] - vertices[1]);

            for (var i = 0; i < vertices.Count; ++i) {
                for (var j = i + 1; j < vertices.Count; ++j) {
                    var newDistance = Vector3.SqrMagnitude(vertices[0] - vertices[1]);
                    if (newDistance < distance) {
                        i1 = i;
                        i2 = j;
                        distance = newDistance;
                    }
                }
            }

            return triangles;
        }
    }

    private class DelaunayTriangulation : ITriangleBuilder {
        public List<VertexHelper.Triangle> getTriangles(List<Vector3> vertices) {
            var lines = new List<string> {vertices.Count.ToString()};
            for (var i = 0; i < vertices.Count; ++i) {
                lines.Add(vertices[i].x + " " + vertices[i].y + " " + vertices[i].z);
            }
            var root = Utils.GetPyCoreProjectPath();
            File.WriteAllLines(Path.Combine(root, "vertices.txt"), lines);

            var triangles = new List<VertexHelper.Triangle>();
            var start = new ProcessStartInfo();

            start.FileName = Path.Combine(root, "venv", "Scripts", "python.exe");
            start.Arguments = Path.Combine(root, "restoreSurface.py") + " >" + Path.Combine(root, "triangles.txt");
            start.UseShellExecute = false;
            start.RedirectStandardOutput = false;
            start.CreateNoWindow = false;
            start.RedirectStandardError = true;
            var process = Process.Start(start);
            process.WaitForExit(30000);
            Debug.Log("process");
            Debug.Log(start.FileName);
            Debug.Log(start.Arguments);
            Debug.Log(process.ExitCode);
            // Debug.Log(process.StandardError.ReadToEnd());
            Debug.Log(process.StandardError.ReadToEnd());
            if (process.ExitCode == 0) {
                // var reader = File.ReadAllText("G:\\Projects\\3d-scan\\triangles.txt");
                ;//process.StandardOutput;
                // var output = reader.ReadToEnd();
                Debug.Log(Path.Combine(root, "triangles.txt"));
                var output = File.ReadAllText(Path.Combine(root, "triangles.txt"));
                Debug.Log(output);
                var triangleLines = output.Split('\n');
                for (var i = 1; i <= int.Parse(triangleLines[0]); ++i) {
                    Debug.Log(triangleLines[i]);
                    var parts = triangleLines[i].Replace(".", ",").Split(' ');
                    Debug.Log(parts[0]);
                    Debug.Log(parts[1]);
                    Debug.Log(parts[2]);
                    float x1 = (float)double.Parse(parts[0]), y1 = (float)double.Parse(parts[1]), z1 = (float)double.Parse(parts[2]);
                    float x2 = (float)double.Parse(parts[3]), y2 = (float)double.Parse(parts[4]), z2 = (float)double.Parse(parts[5]);
                    float x3 = (float)double.Parse(parts[6]), y3 = (float)double.Parse(parts[7]), z3 = (float)double.Parse(parts[8]);

                    triangles.Add(new VertexHelper.Triangle(new Vector3(x1, y1, z1), new Vector3(x2, y2, z2), new Vector3(x3, y3, z3)));
                }
            }

            return triangles;
        }
    }

    private const float ACCURACY = 0.001f;

    private List<Color> myColors = new List<Color>() {
        Color.red,
        Color.blue,
        Color.green,
        Color.cyan,
        Color.yellow,
        Color.magenta,
        Color.grey,
        (Color.blue + Color.grey + Color.red) / 3,
    };

    enum ColorType {
        Random,
        Red,
        White,
        Debug,
        Default,
    }

    struct TriangleChunk {
        public ColorType colorType;
        public List<VertexHelper.Triangle> triangles;
        public float colorAlpha;
        public bool drawTwoSide;

        public TriangleChunk(List<VertexHelper.Triangle> aTriangles, ColorType aColorType, float aColorAlpha, bool aDrawTwoSide) {
            triangles = aTriangles;
            colorType = aColorType;
            colorAlpha = aColorAlpha;
            drawTwoSide = aDrawTwoSide;
        }
    }

    private List<Vector3> GetVertices()  {
        var vertices = new List<Vector3>();
        foreach (var mf in gameObject.GetComponentsInChildren<MeshFilter>()) {
            // foreach (var v in mf.mesh.vertices) {
            //     vertices.Add(new Vector3(v.x * transform.localScale.x, v.y * transform.localScale.y));
            // }

            vertices.AddRange(mf.mesh.vertices);
            Debug.Log(mf.mesh.vertices.Length);
            Debug.Log(mf.mesh.subMeshCount);
            Debug.Log(mf.mesh.GetTopology(0) == MeshTopology.Triangles);
            Debug.Log(mf.mesh.colors.Length);
            Debug.Log(mf.mesh.GetTriangles(0).Length);
        }

        return vertices.Distinct().ToList();
    }

    private void SetVertices(List<TriangleChunk> chunks) {
        var mf = gameObject.GetComponentsInChildren<MeshFilter>().First();
        mf.mesh.Clear();

        float minCoord = 1000000;
        float maxCoord = -1000000;
        foreach (var chunk in chunks) {
            foreach (var triangle in chunk.triangles) {
                foreach (var vertex in triangle.GetVertices()) {
                    minCoord = Math.Min(minCoord, vertex.x);
                    minCoord = Math.Min(minCoord, vertex.y);
                    minCoord = Math.Min(minCoord, vertex.z);

                    maxCoord = Math.Max(maxCoord, vertex.x);
                    maxCoord = Math.Max(maxCoord, vertex.y);
                    maxCoord = Math.Max(maxCoord, vertex.z);
                }
            }
        }

        var index = 0;
        var verticesDict = new Dictionary<Vector3, int>();
        var vertices = new List<Vector3>();
        var uv = new List<Vector2>();
        var trianglesDescriptions = new List<int>();
        var colors = new List<Color>();
        foreach (var chunk in chunks) {
            foreach (var triangle in chunk.triangles) {
                foreach (var vertex in triangle.GetVertices()) {
                    if (!verticesDict.ContainsKey(vertex)) {
                        verticesDict[vertex] = vertices.Count;
                        vertices.Add(vertexHandler.PrepareVertex(vertex));
                        uv.Add(new Vector2(0.7f, 0.3f));

                        var newColor = Color.white;
                        if (chunk.colorType == ColorType.Default) {
                            float elem = (3 * maxCoord - vertex.x - vertex.y - vertex.z) / 3.0f / (maxCoord - minCoord);
                            newColor = new Color(0, elem, 0, chunk.colorAlpha);
                        }
                        else if (chunk.colorType == ColorType.Random) {
                            newColor = myColors[index % myColors.Count];
                        }
                        else if (chunk.colorType == ColorType.Red) {
                            newColor = Color.red;
                        }
                        else if (chunk.colorType == ColorType.White) {
                            newColor = Color.white;
                        }
                        else if (chunk.colorType == ColorType.Debug) {
                            newColor = triangle.DebugColor;
                            Debug.Log(newColor);
                        }

                        newColor.a = chunk.colorAlpha;
                        colors.Add(newColor);
                        ++index;
                    }

                    // Debug.Log(vertex);
                    trianglesDescriptions.Add(verticesDict[vertex]);
                }

                if (chunk.drawTwoSide) {
                    var points = triangle.GetVertices();
                    points.Reverse();
                    foreach (var vertex in points) {
                        trianglesDescriptions.Add(verticesDict[vertex]);
                    }
                }

                //
                // var line = "Triangle: ";
                // foreach (var v in points)  {
                //     line += "(" + v.x + "  "  + " " + v.y + " " + v.z + ") ";
                // }
                // Debug.Log(line);
            }
        }

        var count = 0;
        foreach (var chunk in chunks) {
            count += chunk.triangles.Count;
        }
        Debug.Log(count);

        mf.mesh.SetVertices(vertices);
        mf.mesh.SetTriangles(trianglesDescriptions, 0, false);
        // mf.mesh.bounds
        //mf.mesh.SetUVs(0, uv);
        mf.mesh.SetColors(colors);

        var mr = gameObject.GetComponent<MeshRenderer>();
        mr.material = material;
        //mr.material = new Material(Shader.Find("Diffuse"));
    }

    private List<VertexHelper.Triangle> GetTriangles() {
        // // var vertices = GetEllipsoidVertices();
        // var vertices = vertexHandler.GetVertices();
        // Debug.Log(vertices.Count);
        //
        // // var builder = new MyTriangleBuilder();
        // var builder = new DelaunayTriangulation();
        // // return VertexHelper.GetAllRawSubTriangles(builder.getTriangles(vertices), 1.0f);
        // return builder.getTriangles(vertices);
        return new StlTriangleHandler(Path.Combine(Utils.GetDataFolder(), "sample1", "data.stl")).GetTriangles();
    }

    private void Awake() {
        // VertexHelper.RunMathTests();
        vertexHandler = new RawVertexHandler(gameObject);
        // vertexHandler = new EllipsoidVertexHandler(gameObject);
    }

    public void Start() {
        var baseTriangles = GetTriangles();
        var normalizedTriangles = VertexHelper.NormalizeTriangles(baseTriangles);
        var subTriangles = VertexHelper.GetAllRawSubTriangles(normalizedTriangles, h);

        path = PathFinder.GetInstance().GetPath(ref baseTriangles);

        var chunks = new List<TriangleChunk>{
            new TriangleChunk(baseTriangles, ColorType.Default, 0.9f, true),
        };

        if (drawSubTriangles) {
            chunks.Add(new TriangleChunk(subTriangles, ColorType.Red, 0.2f, true));
        }
        
        SetVertices(chunks);

        if (drawNormals) {
            foreach (var t in subTriangles) {
                var obj = Instantiate(new GameObject(), t.O, Quaternion.identity);
                var lineRenderer = obj.AddComponent<LineRenderer>();
                lineRenderer.SetPositions(new Vector3[] { t.O, t.O + t.GetPlane().GetNormal() * 4 });
            }
        }
    }

    private void OnDrawGizmos() {
        if (path != null) {
            if (drawFromOriginToSurfacePath) {
                foreach (var pos in path) {
                    Gizmos.DrawLine(pos.originPosition, pos.surfacePosition);
                }
            }

            for (var i = 0; i < path.Count - 1; ++i) {
                var pos1 = path[i];
                var pos2 = path[i + 1];

                if (drawOriginPath) {
                    Gizmos.DrawLine(pos1.originPosition, pos2.originPosition);
                }

                if (drawSurfacePath) {
                    Gizmos.DrawLine(pos1.surfacePosition, pos2.surfacePosition);
                }
            }
        }
    }
}
