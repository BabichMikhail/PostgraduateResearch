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
        public float scaleGameToWorldMeter;
        public float sampleScaleToWorldMeter;

        public bool drawSurfacePath;
        public bool drawOriginPath;
        public bool drawFromOriginToSurfacePath;
        public bool drawFoundPath;
        public bool drawApproximatedPath;
        public bool drawLinearPath;
        public bool drawApproximatedPathWithSpeed;
        public bool drawApproximatedPathWithAcceleration;
        public float paintRobotScale;

        public float paintSpeed;
        public float paintRadius;
        public float paintHeight;
        public float paintLateralAllowance;
        public float paintLongitudinalAllowance;
        public float paintLineWidth;
        public float maxPaintRobotSpeed;
        public float maxPaintRobotAcceleration;

        public int pointPerSecondDrawingSpeed;
        public int maxPointCount;
        public float timeScale;
        public int maxPaintRobotPathSimplifyIterations;

        public float maxTriangleSquare;
        public float linearPathStep;
        public float yRotation;
    }

    [Serializable]
    public class SceneState {
        public bool isPaintRobotsCreated;
    }

    [Serializable]
    public class ObjectRotationData {
        public float x;
        public float y;
        public float z;
        public float w;

        public ObjectRotationData(Quaternion rotation) {
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
    public class TransformData {
        public PointData position;
        public PointData scale;
        public ObjectRotationData rotationData;
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
        public readonly PositionData a;
        public readonly PositionData b;

        public RobotPositionData(RobotPathItem rpi) {
            a = new PositionData(rpi.a);
            b = new PositionData(rpi.b);
        }

        public RobotPathItem GetRobotPosition() {
            return new RobotPathItem(a.GetPosition(), b.GetPosition());
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
            foreach (var p in rpp.GetRobotPathItems()) {
                pathPositions.Add(new RobotPositionData(p));
            }

            drawingPositions = new List<PositionData>();
            foreach (var p in robotDrawingPositions) {
                drawingPositions.Add(new PositionData(p));
            }
        }

        public RobotPathProcessor GetRobotPathProcessor() {
            var robotPositions = new List<RobotPathItem>();
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
    public class PaintAmountItems {
        public List<float> items = new List<float>();
    }

    [Serializable]
    public class ColorInfoItems {
        public List<ColorData> items = new List<ColorData>();
    }

    [Serializable]
    public class TexturePaintResultData {
        public List<TriangleData> triangles = new List<TriangleData>();
        public List<PaintAmountItems> paintAmountData = new List<PaintAmountItems>();

        public TexturePaintResultData(TexturePaintResult result) {
            foreach (var t in result.triangles) {
                var td = new TriangleData(t);
                var paintAmountItem = new PaintAmountItems();
                foreach (var p in t.GetPoints()) {
                    paintAmountItem.items.Add(result.paintAmount[t][p]);
                }

                triangles.Add(td);
                paintAmountData.Add(paintAmountItem);
            }
        }

        public TexturePaintResult GetTexturePaintResult() {
            var result = new TexturePaintResult {
                paintAmount = new Dictionary<Triangle, Dictionary<Point, float>>(),
                triangles = new List<Triangle>(),
            };

            var i = 0;
            foreach (var td in triangles) {
                var t = td.GetTriangle();

                var j = 0;
                var paintAmounts = new Dictionary<Point, float>();
                foreach (var p in t.GetPoints()) {
                    paintAmounts.Add(p, paintAmountData[i].items[j]);
                    ++j;
                }

                result.triangles.Add(t);
                result.paintAmount.Add(t, paintAmounts);
                ++i;
            }

            return result;
        }
    }

    [Serializable]
    public class PathData {
        public SceneSettings settings;
        public SceneState state;
        public ObjectRotationData rotationDataCube;
        public List<RobotPathProcessorData> robots;
        public List<PositionData> path;
        public List<PositionData> linearPath;
        public TexturePaintResultData texturePaintResult;
        public TransformData texturePlaneData;
        public List<TriangleData> baseTriangles;
    }
}
