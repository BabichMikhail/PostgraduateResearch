using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Generic;
using UnityEngine;

namespace TriangleHandler
{
    public class StlTriangleHandler : ITriangleHandler {
        private const int BINARY_HEADER_LENGTH = 80;

        private readonly string filePath;

        public StlTriangleHandler(string aFilePath) {
            filePath = aFilePath;
        }

        private Point LoadBinaryVertex(byte[] bytes, int firstByteIndex) {
            return new Point(
                BitConverter.ToSingle(bytes, firstByteIndex),
                BitConverter.ToSingle(bytes, firstByteIndex + 8),
                BitConverter.ToSingle(bytes, firstByteIndex + 4)
            );
        }

        private Point LoadASCIIVertex(string line, string linePrefix) {
            line = line.Trim();
            Debug.Assert(line.Substring(0, linePrefix.Length) == linePrefix);

            var numbers = line.Split().Where(item => item.Length > 0).ToList();

            var count = numbers.Count;
            Debug.Assert(numbers.Count == 4 || numbers.Count == 5);
            return new Point(
                float.Parse(numbers[count - 3], CultureInfo.InvariantCulture),
                float.Parse(numbers[count - 2], CultureInfo.InvariantCulture),
                float.Parse(numbers[count - 1], CultureInfo.InvariantCulture));
        }

        private List<Triangle> ParseBinaryFormat(string aFilePath) {
            Debug.Log("Open: " + aFilePath);
            var bytes = File.ReadAllBytes(filePath);

            var header = new List<byte>();
            for (var i = 0; i < BINARY_HEADER_LENGTH; ++i) {
                header.Add(bytes[i]);
            }

            var triangleCount = BitConverter.ToInt32(bytes, BINARY_HEADER_LENGTH);
            Debug.Assert(triangleCount > 0);

            var result = new List<Triangle>();
            for (var i = 0; i < triangleCount; ++i) {
                var firstByteIndex = BINARY_HEADER_LENGTH + 4 + (12 + 12 + 12 + 12 + 2) * i;
                var n = LoadBinaryVertex(bytes, firstByteIndex);
                var p1 = LoadBinaryVertex(bytes, firstByteIndex + 12);
                var p2 = LoadBinaryVertex(bytes, firstByteIndex + 24);
                var p3 = LoadBinaryVertex(bytes, firstByteIndex + 36);
                var byteCount = BitConverter.ToUInt16(bytes, firstByteIndex + 48);

                result.Add(MakeTriangle(p1, p2, p3, n));
            }

            return result;
        }

        private List<Triangle> ParseAsciiFormat(string aFilePath) {
            var result = new List<Triangle>();
            var lines = File.ReadAllLines(aFilePath);

            var filteredLines = lines
                .Where(line => line.Length >= "solid".Length && line.Substring(0, "solid".Length) != "solid" && line.Length >= "endsolid".Length && line.Substring(0, "endsolid".Length) != "endsolid")
                .ToList();

            var vertexCount = filteredLines.Count / 7;
            for (var i = 0; i < vertexCount; ++i) {
                var j = 7 * i;

                var n = LoadASCIIVertex(filteredLines[j], "facet normal");
                var p1 = LoadASCIIVertex(filteredLines[j + 2], "vertex");
                var p2 = LoadASCIIVertex(filteredLines[j + 3], "vertex");
                var p3 = LoadASCIIVertex(filteredLines[j + 4], "vertex");

                result.Add(MakeTriangle(p1, p2, p3, n));
            }

            return result;
        }

        private Triangle MakeTriangle(Point p1, Point p2, Point p3, Point n) {
            var t = new Triangle(p1, p2, p3, n);
            var tn = t.GetNormal();
            const float maxD = 0.1f;
            if ((tn.Normalized - n.Normalized).Magnitude > maxD) {
                t = new Triangle(p2, p1, p3, n);
                tn = t.GetNormal();
            }

            if ((tn.Normalized - n.Normalized).Magnitude >= maxD) {
                Debug.Assert(false);
            }

            Debug.Assert((t.GetPlane().GetNormal().Normalized - n.Normalized).Magnitude < maxD);
            return t;
        }

        public List<Triangle> GetTriangles() {
            var useRawNormals = Settings.GetInstance().UseRawNormals;
            Settings.GetInstance().UseRawNormals = false;

            List<Triangle> result;
            var text = File.ReadAllText(filePath);
            if (text.Substring(0, "solid ".Length) == "solid " && text.Contains("facet normal")) {
                result = ParseAsciiFormat(filePath);
            }
            else {
                result = ParseBinaryFormat(filePath);
            }

            Settings.GetInstance().UseRawNormals = useRawNormals;

            return result;
        }
    }
}
