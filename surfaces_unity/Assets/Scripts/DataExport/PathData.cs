using System;
using System.Collections.Generic;
using Library.Generic;
using Library.RobotPathBuilder;
using UnityEngine;

namespace DataExport {
    [Serializable]
    public class SceneSettings {
        public string sampleName;

        public bool drawSurfacePath;
        public bool drawOriginPath;
        public bool drawFromOriginToSurfacePath;
        public bool drawFoundPath;
        public bool drawApproximatedPath;
        public bool drawPathStepByStep;
        public bool drawDiffWithLinearPath;
        public bool drawLinearPath;
        public bool drawApproximatedPathWithSpeed;
        public bool drawApproximatedPathWithAcceleration;

        public float paintRadius;
        public float paintHeight;
        public float paintLateralAllowance;
        public float paintLongitudinalAllowance;
        public float paintSpeed;
        public float paintRobotScale;
        public int pointPerSecondDrawingSpeed;
        public int maxPointCount;
        public float maxPaintRobotSpeed;
        public float maxPaintRobotAcceleration;
        public int maxPaintRobotPathSimplifyIterations;
        public float scaleGameToWorld;
        public float timeScale;
    }

    [Serializable]
    public class ObjectRotation {
        public float x;
        public float y;
        public float z;
        public float w;

        public ObjectRotation(Quaternion rotation) {
            x = rotation.x;
            y = rotation.y;
            z = rotation.z;
            w = rotation.w;
        }

        public Quaternion GetQuaternion() {
            return new Quaternion(x, y, z, w);
        }
    }

    [Serializable]
    public class PointData {
        public float x;
        public float y;
        public float z;

        public PointData(Point point) {
            x = point.x;
            y = point.y;
            z = point.z;
        }

        public Point GetPoint() {
            return new Point(x, y, z);
        }
    }

    [Serializable]
    public enum PositionTypeData {
        Start,
        Middle,
        Finish,
    }

    [Serializable]
    public class PositionData {
        public PointData originPoint;
        public PointData paintDirection;
        public PointData surfacePoint;
        public PositionTypeData type;

        public PositionData(Position position) {
            originPoint = new PointData(position.originPoint);
            paintDirection = new PointData(position.paintDirection);
            surfacePoint = new PointData(position.surfacePoint);

            switch (position.type) {
                case Position.PositionType.Start:
                    type = PositionTypeData.Start;
                    break;
                case Position.PositionType.Middle:
                    type = PositionTypeData.Middle;
                    break;
                case Position.PositionType.Finish:
                    type = PositionTypeData.Finish;
                    break;
                default:
                    type = PositionTypeData.Finish;
                    Debug.Assert(false);
                    break;
            }
        }

        public Position GetPosition() {
            Position.PositionType positionType;
            switch (type) {
                case PositionTypeData.Start:
                    positionType = Position.PositionType.Start;
                    break;
                case PositionTypeData.Middle:
                    positionType = Position.PositionType.Middle;
                    break;
                case PositionTypeData.Finish:
                    positionType = Position.PositionType.Finish;
                    break;
                default:
                    positionType = Position.PositionType.Finish;
                    Debug.Assert(false);
                    break;
            }

            return new Position(originPoint.GetPoint(), paintDirection.GetPoint(), surfacePoint.GetPoint(), positionType);
        }
    }

    [Serializable]
    public class RobotPositionData {
        public PointData point;
        public PointData direction;
        public PositionData position;
        public float speedMultiplier;

        public RobotPositionData(RobotPosition rp) {
            point = new PointData(rp.point);
            direction = new PointData(rp.direction);
            position = new PositionData(rp.position);
            speedMultiplier = rp.speedMultiplier;
        }

        public RobotPosition GetRobotPosition() {
            return new RobotPosition(point.GetPoint(), direction.GetPoint(), position.GetPosition(), speedMultiplier);
        }
    }

    [Serializable]
    public class RobotPathProcessorData {
        public int id;
        public List<RobotPositionData> pathPositions;
        public List<PositionData> drawingPositions;

        public RobotPathProcessorData(int id, RobotPathProcessor rpp, List<Position> robotDrawingPositions) {
            this.id = id;

            pathPositions = new List<RobotPositionData>();
            foreach (var p in rpp.GetRobotPositions()) {
                pathPositions.Add(new RobotPositionData(p));
            }

            drawingPositions = new List<PositionData>();
            foreach (var p in robotDrawingPositions) {
                drawingPositions.Add(new PositionData(p));
            }
        }

        public RobotPathProcessor GetRobotPathProcessor() {
            var robotPositions = new List<RobotPosition>();
            foreach (var p in pathPositions) {
                robotPositions.Add(p.GetRobotPosition());
            }

            return new RobotPathProcessor(robotPositions);
        }

        public List<Position> GetDrawingPositions() {
            var robotDrawingPositions = new List<Position>();
            foreach (var p in drawingPositions) {
                robotDrawingPositions.Add(p.GetPosition());
            }

            return robotDrawingPositions;
        }
    }

    [Serializable]
    public class PathData {
        public SceneSettings settings;
        public ObjectRotation rotationCube;
        public List<RobotPathProcessorData> robots;
        public List<PositionData> path;
        public List<PositionData> linearPath;
    }
}
