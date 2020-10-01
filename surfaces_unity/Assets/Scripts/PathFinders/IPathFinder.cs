using System;
using System.Collections.Generic;
using Generic;
using UnityEngine;

namespace PathFinders
{
    public class Position {
        public readonly Point originPosition;
        public readonly Point paintDirection;
        public readonly Point surfacePosition;
        public readonly PointType pointType;

        public enum PointType {
            START,
            MIDDLE,
            FINISH,
        }

        public Position(Point aOriginPosition, Point aPaintDirection, Point aSurfacePosition, PointType aPointType) {
            originPosition = aOriginPosition;
            paintDirection = aPaintDirection.normalized;
            surfacePosition = aSurfacePosition;
            pointType = aPointType;
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
