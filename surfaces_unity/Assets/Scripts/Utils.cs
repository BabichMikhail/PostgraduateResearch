using System.IO;
using System.Net.NetworkInformation;
using UnityEngine;

public static class Utils {
    private static string GetRootFolder() {
        return Path.GetDirectoryName(Path.GetDirectoryName(Application.dataPath));
    }

    public static string GetCurrentProjectRootFolder(string getDataFolder) {
        return Path.GetDirectoryName(Path.GetDirectoryName(Application.dataPath));
    }

    public static string GetPyCoreProjectPath() {
        return Path.Combine(GetRootFolder(), "py_core");
    }

    public static string GetDataFolder() {
        return Path.Combine(GetRootFolder(), "data");
    }
}
