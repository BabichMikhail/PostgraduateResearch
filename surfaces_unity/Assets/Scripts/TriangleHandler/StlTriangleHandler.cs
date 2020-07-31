using System;
using System.Collections.Generic;
using System.IO;
using Generic;
using UnityEngine;

namespace TriangleHandler
{
    public class StlTriangleHandler : ITriangleHandler {
        private const int HEADER_LENGTH = 80;
        
        private readonly string filePath;
        
        public StlTriangleHandler(string aFilePath) {
            filePath = aFilePath;
        }

        private Vector3 LoadVertex(byte[] bytes, int firstByteIndex) {
            return new Vector3(
                BitConverter.ToSingle(bytes, firstByteIndex),
                BitConverter.ToSingle(bytes, firstByteIndex + 8),
                BitConverter.ToSingle(bytes, firstByteIndex + 4)
            );
        }
        
        public List<VertexHelper.Triangle> GetTriangles() {
            var bytes = File.ReadAllBytes(filePath);
            
            var header = new List<byte>();
            for (var i = 0; i < HEADER_LENGTH; ++i) {
                header.Add(bytes[i]);
            }

            var triangleCount = BitConverter.ToInt32(bytes, HEADER_LENGTH);
            Debug.Assert(triangleCount > 0);

            var result = new List<VertexHelper.Triangle>();
            for (var i = 0; i < triangleCount; ++i) {
                var firstByteIndex = HEADER_LENGTH + 4 + 50 * i;
                var n = LoadVertex(bytes, firstByteIndex);
                var p1 = LoadVertex(bytes, firstByteIndex + 12);
                var p2 = LoadVertex(bytes, firstByteIndex + 24);
                var p3 = LoadVertex(bytes, firstByteIndex + 36);

                // if (float.IsNaN(p1.x) || float.IsNaN(p1.y) || float.IsNaN(p1.z)) {
                //     continue;
                // }
                //
                // if (float.IsNaN(p2.x) || float.IsNaN(p2.y) || float.IsNaN(p2.z)) {
                //     continue;
                // }
                //
                // if (float.IsNaN(p3.x) || float.IsNaN(p3.y) || float.IsNaN(p3.z)) {
                //     continue;
                // }
                //
                // if (Math.Abs(p1.x) > 1000000 || Math.Abs(p2.x) > 1000000 || Math.Abs(p3.x) > 1000000) {
                //     continue;
                // }
                //
                // if (Math.Abs(p1.y) > 1000000 || Math.Abs(p2.y) > 1000000 || Math.Abs(p3.y) > 1000000) {
                //     continue;
                // }
                //
                // if (Math.Abs(p1.z) > 1000000 || Math.Abs(p2.z) > 1000000 || Math.Abs(p3.z) > 1000000) {
                //     continue;
                // }

                result.Add(new VertexHelper.Triangle(p1, p2, p3));
            }

            return result;
        }
    }
}
