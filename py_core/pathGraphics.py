import glob
import os

import math
import matplotlib.pyplot

for x in glob.glob("../store/generated/5/pathdata/*.txt"):
    basename = os.path.basename(x)
    print(basename)
    f = open(x, "r")
    content = f.readlines()
    data = []
    xs = []
    ys = []
    times = []
    speeds = []
    sumTimes = 0
    for line in content:
        parts = line.split('\t')
        if len(parts[0]) == 0:
            break
        x = float(parts[0].replace(',', '.'))
        y = float(parts[1].replace(',', '.'))
        time = float(parts[2].replace(',', '.'))
        data.append({"x": x, "y": y, "time": time})
        xs.append(x)
        ys.append(y)
        x2 = xs[len(xs) - 2]
        y2 = xs[len(ys) - 2]
        times.append(time)
        if sumTimes > 0 and len(xs) >= 2:
            speeds.append(math.sqrt(x * x + y * y - x2 - x2 - y2 * y2) / (time - sumTimes))
        else:
            speeds.append(0)
        sumTimes = time
    matplotlib.pyplot.plot(xs, speeds)
    matplotlib.pyplot.ylabel("speed")
    matplotlib.pyplot.xlabel("x")
    matplotlib.pyplot.title(basename)
    matplotlib.pyplot.show()
    # print(data)

input()
