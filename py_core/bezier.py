import math
from tkinter import *
from tkinter import font


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Bezier:
    def __init__(self, aBasePoint, aN, aAngle):
        step = 70
        # count = 2 * n + 1
        points = []
        for i in range(aN).__reversed__():
            points.append(Point(aBasePoint.x + math.cos(math.radians(aAngle)) * step * (i + 1), aBasePoint.y + step * (i + 1)))

        points.append(aBasePoint)

        for i in range(aN):
            points.append(Point(aBasePoint.x + step * (i + 1), aBasePoint.y))

        triangle = [1]
        for i in range(len(points) - 1):
            new_triangle = [triangle[0]]
            for j in range(len(triangle) - 1):
                new_triangle.append(triangle[j] + triangle[j + 1])
            new_triangle.append(triangle[-1])
            triangle = new_triangle
        print(triangle)

        self.triangle = triangle
        self.points = points

    def _getValue(self, aT):
        x = 0.0
        y = 0.0
        for k in range(len(self.triangle)):
            x += math.pow(1 - aT, len(self.triangle) - 1 - k) * math.pow(aT, k) * self.triangle[k] * self.points[k].x
            y += math.pow(1 - aT, len(self.triangle) - 1 - k) * math.pow(aT, k) * self.triangle[k] * self.points[k].y
        return Point(x, y)

    def drawFigure(self, aCanvas, aColor):
        path = []

        ct = 0
        for i in range(len(self.points) - 1):
            p1 = self.points[i]
            p2 = self.points[i + 1]
            ct += math.sqrt(math.pow(p2.x - p1.x, 2) + math.pow(p2.y - p1.y, 2))

        r = 1.0
        for j in range(math.floor(ct * round(r))):
            t_val = j / r
            path.append(self._getValue(t_val / ct))
        path.append(self._getValue(ct))

        self._drawPoints(aCanvas, aColor, path, 1)
        pass

    def _drawPoints(self, aCanvas, aColor, aPoints, aWidth):
        width = aWidth
        for point in aPoints:
            aCanvas.create_rectangle(point.x - width, point.y - width, point.x + width, point.y + width, fill=aColor, outline=aColor)

    def drawPoints(self, aCanvas, aColor):
        self._drawPoints(aCanvas, aColor, self.points, 3)


t = Tk()

canvas = Canvas(t, width=900, height=900)
canvas.pack()
canvas.create_rectangle(0, 0, 900, 900, fill='#FFFFFF', outline='#FFFFFF')

basePoint = Point(300, 20)
angle = 45

myFont = font.Font(size=30)

# bezier1 = Bezier(basePoint, 1)
# bezier1.drawPoints(canvas, '#000000')
# bezier1.drawFigure(canvas, '#FF0000')
#
# bezier2 = Bezier(basePoint, 2)
# bezier2.drawPoints(canvas, '#000000')
# bezier2.drawFigure(canvas, '#88FFFF')
#
# bezier3 = Bezier(basePoint, 3)
# bezier3.drawPoints(canvas, '#000000')
# bezier3.drawFigure(canvas, '#FF00FF')
#
# bezier4 = Bezier(basePoint, 4)
# bezier4.drawPoints(canvas, '#000000')
# bezier4.drawFigure(canvas, '#FFFF00')
#
# bezier5 = Bezier(basePoint, 5)
# bezier5.drawPoints(canvas, '#000000')
# bezier5.drawFigure(canvas, '#FF0000')
#
# bezier10000 = Bezier(basePoint, 30)
# bezier10000.drawPoints(canvas, '#000000')
# bezier10000.drawFigure(canvas, '#0000FF')

bezier1 = Bezier(basePoint, 1, angle)
bezier1.drawPoints(canvas, '#000000')
bezier1.drawFigure(canvas, '#888888')

# bezier2 = Bezier(basePoint, 2)
# bezier2.drawPoints(canvas, '#000000')
# bezier2.drawFigure(canvas, '#888888')
#
# bezier3 = Bezier(basePoint, 3)
# bezier3.drawPoints(canvas, '#000000')
# bezier3.drawFigure(canvas, '#888888')
#
# bezier4 = Bezier(basePoint, 4)
# bezier4.drawPoints(canvas, '#000000')
# bezier4.drawFigure(canvas, '#888888')

bezier5 = Bezier(basePoint, 5, angle)
bezier5.drawPoints(canvas, '#000000')
bezier5.drawFigure(canvas, '#888888')

bezier10000 = Bezier(basePoint, 30, angle)
bezier10000.drawPoints(canvas, '#000000')
bezier10000.drawFigure(canvas, '#888888')

# 45
canvas.create_text(336, 120, text="r = 1", font=myFont)
canvas.create_text(432, 60, text="r = 5", font=myFont)
canvas.create_text(562, 120, text="r = 30", font=myFont)

# 90
# canvas.create_text(392, 40, text="r = 1", font=myFont)
# canvas.create_text(392, 86, text="r = 5", font=myFont)
# canvas.create_text(462, 136, text="r = 30", font=myFont)

# 135
# canvas.create_text(242, 40, text="r = 1", font=myFont)
# canvas.create_text(392, 60, text="r = 5", font=myFont)
# canvas.create_text(522, 116, text="r = 30", font=myFont)

mainloop(0)
