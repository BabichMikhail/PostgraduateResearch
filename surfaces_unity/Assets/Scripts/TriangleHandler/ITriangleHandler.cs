using System.Collections.Generic;
using Generic;

namespace TriangleHandler
{
    public interface ITriangleHandler {
        List<VertexHelper.Triangle> GetTriangles();
    }
}
