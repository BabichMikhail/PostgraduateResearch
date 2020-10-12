using System;
using System.Collections.Generic;
using Generic;
using UnityEngine;

namespace PathFinders
{
    public class Position {
        public readonly Point OriginPosition;
        public readonly Point PaintDirection;
        public readonly Point SurfacePosition;
        public readonly PositionType Type;

        public enum PositionType {
            Start,
            Middle,
            Finish,
        }

        public Position(Point aOriginPosition, Point aPaintDirection, Point aSurfacePosition, PositionType aType) {
            OriginPosition = aOriginPosition;
            PaintDirection = aPaintDirection.Normalized;
            SurfacePosition = aSurfacePosition;
            Type = aType;
            Debug.Assert((PaintDirection - (SurfacePosition - OriginPosition).Normalized).Magnitude < 1e-4);
        }
    }

    public enum PathFinderType {
        IntersectionsWithSurfacesPathFinder,
    }

    public interface IPathFinder {
        List<Position> GetPath(ref List<Triangle> triangles);
    }

    public static class PathFinderFactory {
        public static IPathFinder Create(PathFinderType type, float paintRadius, float paintHeight, float paintLateralAllowance, float paintLongitudinalAllowance) {
            IPathFinder result = null;
            switch (type) {
                case PathFinderType.IntersectionsWithSurfacesPathFinder:
                    result = new IntersectionsWithSurfacesPathFinder(paintRadius, paintHeight, paintLateralAllowance, paintLongitudinalAllowance);
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(type), type, null);
            }

            return result;
        }
    }
}
