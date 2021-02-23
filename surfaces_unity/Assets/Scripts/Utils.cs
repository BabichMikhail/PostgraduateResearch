using System.IO;
using Library.Generic;
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

    public static string GetStoreFolder() {
        return Path.Combine(GetRootFolder(), "store");
    }

    public static Point VtoP(Vector3 v) {
        return new Point(v.x, v.y, v.z);
    }

    public static Vector3 PtoV(Point p) {
        return new Vector3(p.x, p.y, p.z);
    }
}
