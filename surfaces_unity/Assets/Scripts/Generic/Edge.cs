using System.Collections.Generic;

namespace Generic
{
    public class Edge {
        public readonly Point P1;
        public readonly Point P2;

        public Edge(Point aP1, Point aP2) {
            P1 = aP1;
            P2 = aP2;
        }

        public override int GetHashCode() => new KeyValuePair<Point, Point>(P1, P2).GetHashCode();

        public List<Point> GetPoints() => new List<Point> {P1, P2};

        public bool HasPoint(Point p) => P1 == p || P2 == p;

        public static bool operator ==(Edge a, Edge b) {
            return a.P1 == b.P1 && a.P2 == b.P2 || a.P1 == b.P2 && a.P2 == b.P1;
        }

        public static bool operator !=(Edge a, Edge b) {
            return !(a == b);
        }
    }
}
