using System.Collections.Generic;

namespace Generic
{
    public class Edge {
        public readonly Point p1;
        public readonly Point p2;

        public Edge(Point aP1, Point aP2) {
            p1 = aP1;
            p2 = aP2;
        }

        public override int GetHashCode() => new KeyValuePair<Point, Point>(p1, p2).GetHashCode();

        public List<Point> GetPoints() => new List<Point> {p1, p2};

        public bool HasPoint(Point p) => p1 == p || p2 == p;

        public static bool operator ==(Edge a, Edge b) {
            return a.p1 == b.p1 && a.p2 == b.p2 || a.p1 == b.p2 && a.p2 == b.p1;
        }

        public static bool operator !=(Edge a, Edge b) {
            return !(a == b);
        }
    }
}
