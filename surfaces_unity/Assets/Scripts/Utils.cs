using System.IO;
using System.Net.NetworkInformation;
using UnityEngine;

public class Utils {
    private static string GetRootFolder() {
        return Path.GetDirectoryName(Path.GetDirectoryName(Application.dataPath));
    }

    public static string GetCurrentProjectRootFolder() {
        return Path.GetDirectoryName(Path.GetDirectoryName(Application.dataPath));
    }

    public static string GetPyCoreProjectPath() {
        return Path.Combine(GetRootFolder(), "py_core");
    }
}
