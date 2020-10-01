using System.Collections.Generic;
using Generic;
using UnityEngine;

namespace TriangleHandler
{
    public class UnityObjectTriangleHandler : ITriangleHandler {
        private GameObject gameObject;

        public UnityObjectTriangleHandler(GameObject aGameObject) {
            gameObject = aGameObject;
        }

        public List<Triangle> GetTriangles() {
            var meshFilters = gameObject.GetComponentsInChildren<MeshFilter>();
            var result = new List<Triangle>();
            for (var i = 0; i < meshFilters.Length; ++i) {
                var mf = meshFilters[i];
                // List<int> rawTriangles;
                // var triangles = mf.mesh.GetTriangles(0);
                for (var j = 0; j < mf.mesh.subMeshCount; ++j) {
                    var vertices = new List<Vector3>();
                    mf.mesh.GetVertices(vertices);
                    var triangles = mf.mesh.GetTriangles(j);
                    Debug.Assert(true);
                }
                // mf.mesh.GetVertices();
                Debug.Assert(true);
            }
            // for (var i = 0; i < mesh.subMeshCount; ++i) {
            //     var triangles = mesh.GetTriangles(i);
            //     Debug.Assert(true);
            // }

            return result;
        }
    }
}