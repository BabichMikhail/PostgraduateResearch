namespace Generic
{
    public class Settings
    {
        private static Settings instance;

        public static Settings GetInstance() {
            return instance ?? (instance = new Settings());
        }

        public bool UseRawNormals { get; set; }
    }
}
