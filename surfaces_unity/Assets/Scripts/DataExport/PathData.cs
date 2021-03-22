using System;
using System.Collections.Generic;
using System.Linq;
using Library.Algorithm;
using Library.Generic;
using Library.RobotPathBuilder;
using UnityEngine;
using MColor = Library.Algorithm.Color;

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
        public float maxTriangleSquare;
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
    public class TriangleData {
        public PointData p1;
        public PointData p2;
        public PointData p3;

        public TriangleData(Point aP1, Point aP2, Point aP3) {
            p1 = new PointData(aP1);
            p2 = new PointData(aP2);
            p3 = new PointData(aP3);
        }

        public TriangleData(Triangle t) : this(t.p1, t.p2, t.p3) {}

        public Triangle GetTriangle() {
            return new Triangle(p1.GetPoint(), p2.GetPoint(), p3.GetPoint());
        }
    }

    [Serializable]
    public class ReplacementData {
        public TriangleData triangle;
        public List<TriangleData> replacements;

        public ReplacementData(Triangle aTriangle, List<Triangle> aReplacements) {
            triangle = new TriangleData(aTriangle);
            replacements = new List<TriangleData>();
            foreach (var r in aReplacements) {
                replacements.Add(new TriangleData(r));
            }
        }
    }

    [Serializable]
    public class ColorData {
        public float r;
        public float g;
        public float b;
        public float a;

        public ColorData(MColor aColor) {
            r = aColor.r;
            g = aColor.g;
            b = aColor.b;
            a = aColor.a;
        }

        public MColor GetColor() {
            return new MColor(r, g, b, a);
        }
    }

    [Serializable]
    public class PointColorInfoData {
        public PointData point;
        public ColorData color;

        public PointColorInfoData(Point p, MColor c) {
            point = new PointData(p);
            color = new ColorData(c);
        }
    }

    [Serializable]
    public class TriangleColorInfoData {
        public TriangleData triangle;
        public List<PointColorInfoData> pointColorInfo;

        public TriangleColorInfoData(Triangle t, Dictionary<Point, MColor> colorInfo) {
            triangle = new TriangleData(t);
            pointColorInfo = new List<PointColorInfoData>();
            foreach (var info in colorInfo) {
                pointColorInfo.Add(new PointColorInfoData(info.Key, info.Value));
            }
        }
    }

    [Serializable]
    public class DrawingResultData {
        // public List<ReplacementData> replacements;
        public List<TriangleColorInfoData> triangleColorInfo;

        public DrawingResultData(DrawingResult result) {
            // replacements = new List<ReplacementData>();
            // foreach (var r in result.replacements) {
            //     replacements.Add(new ReplacementData(r.Key, r.Value));
            // }

            var usedTriangles = new Dictionary<Triangle, bool>();

            triangleColorInfo = new List<TriangleColorInfoData>();
            foreach (var q in result.colorInfo) {
                triangleColorInfo.Add(new TriangleColorInfoData(q.Key, q.Value));
                usedTriangles.Add(q.Key, true);
            }

            foreach (var t in result.triangles) {
                if (!usedTriangles.ContainsKey(t)) {
                    triangleColorInfo.Add(new TriangleColorInfoData(t, new Dictionary<Point, MColor>()));
                }
            }
        }

        public DrawingResult GetDrawingResult() {
            // TODO fix save and load drawing result
            var result = new DrawingResult {
                replacements = new Dictionary<Triangle, List<Triangle>>(),
                triangles = new List<Triangle>(),
                colorInfo = new Dictionary<Triangle, Dictionary<Point, MColor>>()
            };

            // foreach (var r in replacements) {
            //     var items = new List<Triangle>();
            //     r.replacements.ForEach(x => items.Add(x.GetTriangle()));
            //     result.replacements.Add(r.triangle.GetTriangle(), items);
            // }

            foreach (var item in triangleColorInfo) {
                var t = item.triangle.GetTriangle();
                var data = item.pointColorInfo.ToDictionary(x => x.point.GetPoint(), x => x.color.GetColor());
                result.colorInfo.Add(item.triangle.GetTriangle(), data);
                result.triangles.Add(t);
            }

            return result;
        }
    }

    [Serializable]
    public class PathData {
        public SceneSettings settings;
        public ObjectRotation rotationCube;
        public List<RobotPathProcessorData> robots;
        public List<PositionData> path;
        public List<PositionData> linearPath;
        public DrawingResultData drawingResult;
    }
}
