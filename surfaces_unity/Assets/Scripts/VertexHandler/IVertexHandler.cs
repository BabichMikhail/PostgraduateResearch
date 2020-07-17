using System.Collections.Generic;
using Generic;
using UnityEngine;

namespace VertexHandler
{
    public interface IVertexHandler {
        List<Vector3> GetVertices();
        Vector3 PrepareVertex(Vector3 vertex);
    }
}
