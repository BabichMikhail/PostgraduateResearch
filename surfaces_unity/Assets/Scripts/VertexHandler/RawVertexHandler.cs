using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace VertexHandler
{
    public class RawVertexHandler : IVertexHandler {
        private GameObject gameObject;

        public RawVertexHandler(GameObject aGameObject) {
            gameObject = aGameObject;
        }

        public List<Vector3> GetVertices() {
            var vertices = new List<Vector3>();
            foreach (var mf in gameObject.GetComponentsInChildren<MeshFilter>()) {
                var ls = gameObject.transform.localScale;
                foreach (var v in mf.mesh.vertices) {
                    vertices.Add(new Vector3(v.x * ls.x, v.y * ls.y, v.z * ls.z));
                }

                Debug.Log(mf.mesh.vertices.Length);
                Debug.Log(mf.mesh.subMeshCount);
                Debug.Log(mf.mesh.GetTopology(0) == MeshTopology.Triangles);
                Debug.Log(mf.mesh.colors.Length);
                Debug.Log(mf.mesh.GetTriangles(0).Length);
            }

            return vertices.Distinct().ToList();
        }

        public Vector3 PrepareVertex(Vector3 vertex) {
            var ls = gameObject.transform.localScale;
            return new Vector3(vertex.x / ls.x, vertex.y / ls.y, vertex.z / ls.z);
        }
    }
}
