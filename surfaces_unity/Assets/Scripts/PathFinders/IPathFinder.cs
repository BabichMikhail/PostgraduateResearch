using System;
using System.Collections.Generic;
using Generic;
using UnityEngine;

namespace PathFinders
{
    public class Position {
        public readonly Vector3 originPosition;
        public readonly Vector3 paintDirection;
        public readonly Vector3 surfacePosition;

        public Position(Vector3 aOriginPosition, Vector3 aPaintDirection, Vector3 aSurfacePosition) {
            originPosition = aOriginPosition;
            paintDirection = aPaintDirection.normalized;
            surfacePosition = aSurfacePosition;
            Debug.Assert((paintDirection - (surfacePosition - originPosition).normalized).magnitude < 10e-4);
        }
    }

    public enum PathFinderType {
        GraphBasedPathFinder,
        IntersectionsWithSurfacesPathFinder,
    }

    public interface IPathFinder {
        List<Position> GetPath(ref List<Triangle> triangles);
    }

    public class PathFinderFactory {
        public static IPathFinder Create(PathFinderType type, float paintRadius, float paintHeight) {
            IPathFinder result = null;
            switch (type) {
                case PathFinderType.GraphBasedPathFinder:
                    result = new GraphBasedPathFinder(paintRadius, paintHeight);
                    break;
                case PathFinderType.IntersectionsWithSurfacesPathFinder:
                    result = new IntersectionsWithSurfacesPathFinder(paintRadius, paintHeight);
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(type), type, null);
            }

            return result;
        }
    }
}
