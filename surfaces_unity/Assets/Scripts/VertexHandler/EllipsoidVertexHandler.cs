using System;
using System.Collections.Generic;
using UnityEngine;
using Random = System.Random;

namespace VertexHandler
{
    public class EllipsoidVertexHandler : IVertexHandler {
        private GameObject gameObject;

        public EllipsoidVertexHandler(GameObject aGameObject) {
            gameObject = aGameObject;
        }

        public List<Vector3> GetVertices() {
            var vertices = new List<Vector3>();

            var step = 1.0f;
            var noise = step / 10.0f;

            // Пусть Z > 0
            var localScale = gameObject.transform.localScale;
            var a = (float)(Math.Sqrt(Math.Abs(localScale.x)));
            var b = (float)(Math.Sqrt(Math.Abs(localScale.y)));
            var c = (float)(Math.Sqrt(Math.Abs(localScale.z)));
            // var a = (float)Math.Abs(transform.localScale.x);
            // var b = (float)Math.Abs(transform.localScale.y);
            // var c = (float)Math.Abs(transform.localScale.z);

            var stepX =  (float)((2 * a) / (Math.Ceiling(2 * a / step) + 1));
            var stepY = (float)((2 * b) / (Math.Ceiling(2 * b / step) + 1));

            Debug.Log(a + " " + b + " " + c + " " + stepX + " " + stepY);

            var rand = new Random();
            for (var i = 0; i * stepX <= 2 * a + 0.0001f; ++i) {
                for (var j = 0; j * stepY <= 2 * b + 0.0001f; ++j) {
                    var x = i * stepX - a;
                    var y = j * stepY - b;
                    var val = 1 - x * x / a / a - y * y / b / b;
                    if (val > 0) {
                        var z = c * (float)Math.Sqrt(val);
                        var newX = x / a / 2 + (float)rand.NextDouble() * noise - noise / 2;
                        var newY = y / b / 2 + (float)rand.NextDouble() * noise - noise / 2;
                        var newZ = z / c / 2 + (float)rand.NextDouble() * noise - noise / 2;
                        vertices.Add(new Vector3(newX, newY, newZ));
                        // vertices.Add(new Vector3(
                        //     (float)Math.Sqrt(Math.Abs(newX) * (Math.Abs(newX) < 0.00001 ? Math.Abs(newX) / newX : 1)),
                        //     (float)Math.Sqrt(Math.Abs(newY) * (Math.Abs(newY) < 0.00001 ? Math.Abs(newY) / newY : 1)),
                        //     (float)Math.Sqrt(Math.Abs(newZ) * (Math.Abs(newZ) < 0.00001 ? Math.Abs(newZ) / newZ : 1))
                        // ));
                    }
                    // Debug.Log(1 - x * x / a / a - y * y / b / b);
                    // var z = (float)Math.Sqrt(1 - x * x / a / a - y * y / b / b);
                    // vertices.Add(new Vector3(x, y, z));
                }
            }

            return vertices;
        }

        public Vector3 PrepareVertex(Vector3 vertex) {
            return vertex;
        }
    }
}
