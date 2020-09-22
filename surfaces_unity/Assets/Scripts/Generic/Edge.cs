using System.Collections.Generic;
using UnityEngine;

namespace Generic
{
    public class Edge {
        public readonly Vector3 p1;
        public readonly Vector3 p2;

        public Edge(Vector3 aP1, Vector3 aP2) {
            p1 = aP1;
            p2 = aP2;
        }

        public override int GetHashCode() => new KeyValuePair<Vector3, Vector3>(p1, p2).GetHashCode();

        public override bool Equals(object obj) {
            var other = (Edge)obj;
            return other != null && p1 == other.p1 && p2 == other.p2;
        }

        public List<Vector3> GetPoints() => new List<Vector3> {p1, p2};

        public bool HasPoint(Vector3 p) => p1 == p || p2 == p;
    }
}
