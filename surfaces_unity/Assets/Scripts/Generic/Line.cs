
namespace Generic
{
    public class Line {
        public Point p1;
        public Point p2;

        // (x - x_0) / p0_x = (y - y_0) / p0_y = (z - z_0) / p0_z;
        public float p0x, p0y, p0z;
        public float x0, y0, z0;
        public Line(Point ap1, Point ap2) {
            p1 = ap1;
            p2 = ap2;

            var direction = p1 - p2;

            p0x = direction.x;
            p0y = direction.y;
            p0z = direction.z;

            x0 = p1.x;
            y0 = p1.y;
            z0 = p1.z;
        }

        public override int GetHashCode() => new Point(x0, y0, z0).GetHashCode() + new Point(p0x, p0y, p0z).GetHashCode();
    }
}
