using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
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

        private Vector3 LoadBinaryVertex(byte[] bytes, int firstByteIndex) {
            return new Vector3(
                BitConverter.ToSingle(bytes, firstByteIndex),
                BitConverter.ToSingle(bytes, firstByteIndex + 8),
                BitConverter.ToSingle(bytes, firstByteIndex + 4)
            );
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
                var firstByteIndex = BINARY_HEADER_LENGTH + 4 + 50 * i;
                var n = LoadBinaryVertex(bytes, firstByteIndex);
                var p1 = LoadBinaryVertex(bytes, firstByteIndex + 12);
                var p2 = LoadBinaryVertex(bytes, firstByteIndex + 24);
                var p3 = LoadBinaryVertex(bytes, firstByteIndex + 36);

                result.Add(new Triangle(p1, p2, p3));
            }

            return result;
        }

        private Vector3 LoadASCIIVertex(string line, string linePrefix) {
            line = line.Trim();
            Debug.Assert(line.Substring(0, "facet normal ".Length) == "facet normal ");

            // Debug.Log(line);
            var numbers = new List<string>();
            foreach (var item in line.Split()) {
                if (item.Length > 0) {
                    numbers.Add(item);
                }
            }

            var count = numbers.Count;
            Debug.Assert(numbers.Count == 4 || numbers.Count == 5);
            return new Vector3(
                float.Parse(numbers[count - 3], CultureInfo.InvariantCulture),
                float.Parse(numbers[count - 2], CultureInfo.InvariantCulture),
                float.Parse(numbers[count - 1], CultureInfo.InvariantCulture));
        }

        private List<Triangle> ParseASCIIFormat(string aFilePath) {
            var result = new List<Triangle>();
            var lines = File.ReadAllLines(aFilePath);

            var filteredLines = new List<string>();
            foreach (var line in lines) {
                if (line.Length >= "solid".Length && line.Substring(0, "solid".Length) != "solid" && line.Length >= "endsolid".Length && line.Substring(0, "endsolid".Length) != "endsolid") {
                    filteredLines.Add(line);
                }
            }

            var vertexCount = filteredLines.Count / 7;
            for (var i = 0; i < vertexCount; ++i) {
                var j = 7 * i;

                var n = LoadASCIIVertex(filteredLines[j], "facet normal");
                var p1 = LoadASCIIVertex(filteredLines[j + 2], "vertex");
                var p2 = LoadASCIIVertex(filteredLines[j + 3], "vertex");
                var p3 = LoadASCIIVertex(filteredLines[j + 4], "vertex");
                var t = new Triangle(p1, p2, p3);
                Debug.Assert(t.GetPlane().GetNormal().normalized == n.normalized);
                result.Add(t);
            }

            return result;
        }

        public List<Triangle> GetTriangles() {
            List<Triangle> result;
            var text = File.ReadAllText(filePath);
            if (text.Substring(0, "solid ".Length) == "solid ") {
                result = ParseASCIIFormat(filePath);
            }
            else {
                result = ParseBinaryFormat(filePath);
            }

            return result;
        }
    }
}
