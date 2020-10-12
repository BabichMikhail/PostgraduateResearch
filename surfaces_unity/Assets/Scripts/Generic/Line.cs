
namespace Generic
{
    public class Line {
        public readonly Point P1;
        public readonly Point P2;

        // (x - x_0) / p0_x = (y - y_0) / p0_y = (z - z_0) / p0_z;
        public readonly float P0X;
        public readonly float P0Y;
        public readonly float P0Z;
        public readonly float X0;
        public readonly float Y0;
        public readonly float Z0;
        public Line(Point aP1, Point aP2) {
            P1 = aP1;
            P2 = aP2;

            var direction = P1 - P2;

            P0X = direction.x;
            P0Y = direction.y;
            P0Z = direction.z;

            X0 = P1.x;
            Y0 = P1.y;
            Z0 = P1.z;
        }

        public override int GetHashCode() => new Point(X0, Y0, Z0).GetHashCode() + new Point(P0X, P0Y, P0Z).GetHashCode();
    }
}
