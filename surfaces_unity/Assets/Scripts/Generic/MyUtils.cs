namespace Generic
{
    public static class MyUtils {
        public static void Swap(ref Point p1, ref Point p2) {
            var p0 = p1;
            p1 = p2;
            p2 = p0;
        }

        public static void Swap(ref double a, ref double b) {
            var c = a;
            a = b;
            b = c;
        }
    }
}
