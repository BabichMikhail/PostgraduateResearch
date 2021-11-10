import turtle
from tkinter import *

file = open("G:\\Projects\\PostgraduateResearch\\store\\articleExperiments9\\plane2_heat_map.txt", "r")
content = file.read()
file.close()
lines = content.split("\n")

data = []
for i in range(2):
    line = lines[i + 1]
    items = line.split("\t")
    r = float(items[1].replace(",", "."))
    g = float(items[2].replace(",", "."))
    b = float(items[3].replace(",", "."))
    a = float(items[4].replace(",", "."))
    print(r, g, b, a)
    print("{:x}".format(int(b * 256)))
    value = float(items[5].replace(",", "."))
    data.append({
        "color": {
            "r": int(r * 255),
            "g": int(g * 255),
            "b": int(b * 255),
            "a": int(a * 255),
            # "r": "{:02x}".format(int(r * 255)),
            # "g": "{:02x}".format(int(g * 255)),
            # "b": "{:02x}".format(int(b * 255)),
            # "a": "{:02x}".format(int(a * 255)),
        },
        "value": value,
    })

root = Tk()
canvas = Canvas(root, width=1000, height=1000)
# canvas.create_rectangle(30, 30, 60, 330, outline="#fb0", fill="#fb0")
# print("#" + data[0]["color"]["r"] + data[0]["color"]["g"] + data[0]["color"]["b"])
color1 = data[1]["color"]
color2 = data[0]["color"]
hexColor1 = "#" + "{:02x}".format(color1["r"]) + "{:02x}".format(color1["g"]) + "{:02x}".format(color1["b"])
hexColor2 = "#" + "{:02x}".format(color2["r"]) + "{:02x}".format(color2["g"]) + "{:02x}".format(color2["b"])

n = 300
for i in range(n + 1):
    r = int(color1["r"] * i / n + color2["r"] * (n - i) / n)
    g = int(color1["g"] * i / n + color2["g"] * (n - i) / n)
    b = int(color1["b"] * i / n + color2["b"] * (n - i) / n)
    print(r, g, b)
    hexColor = "#" + "{:02x}".format(r) + "{:02x}".format(g) + "{:02x}".format(b)
    canvas.create_line(30, 30 + i, 60, 30 + i, width=1, fill=hexColor)
canvas.pack(fill=BOTH, expand=1)
root.mainloop()