import os
import matplotlib.pyplot as plt
import matplotlib.style

# basePath = 'G:\\Projects\\PostgraduateResearch\\store\\articleExperiments34'
# files = [
#     open(os.path.join(basePath, 'plane2sigma_0_ort.txt'), "r"),
#     open(os.path.join(basePath, 'plane1.5sigma_0_ort.txt'), "r"),
#     open(os.path.join(basePath, 'plane1sigma_0_ort.txt'), "r"),
# ]
# names = [
#     'sigma = 2.0',
#     'sigma = 1.5',
#     'sigma = 1.0',
# ]

# basePath = 'G:\\Projects\\PostgraduateResearch\\store\\articleExperiments32'
# files = [
#     open(os.path.join(basePath, 'angleWide12S_0_def.txt'), "r"),
#     open(os.path.join(basePath, 'angleWide12A_0_def.txt'), "r"),
# ]
# names = [
#     'До оптимизации',
#     'После оптимизации',
# ]

basePath = 'G:\\Projects\\PostgraduateResearch\\store\\articleExperiments32'
files = [
    open(os.path.join(basePath, 'angle12S_0_def.txt'), "r"),
    open(os.path.join(basePath, 'angle12S_1_def.txt'), "r"),
    open(os.path.join(basePath, 'angle12S_1.5_def.txt'), "r"),
    open(os.path.join(basePath, 'angle12S_2.5_def.txt'), "r"),
]
names = [
    'σ = 0',
    'σ = 1',
    'σ = 1.5',
    'σ = 2.5',
]


i = 0
coords = []
for f in files:
    lines = f.readlines()
    count = 0
    sum = 0.0
    minValue = 1e+10
    maxValue = 1e-10
    x = []
    y = []
    for line in lines:
        [point, l, value] = line.split('\t')
        count += 1
        v = float(value.replace(',', '.'))
        x.append(count)
        y.append(v)
        minValue = min(v, minValue)
        maxValue = max(v, maxValue)
        sum += v
        # print(value)
    coords.append({'x': x, 'y': y, 'label': names[i], 'avg': sum / count})
    i += 1
    # print(minValue, maxValue, sum / count)

print(matplotlib.style.available)
matplotlib.style.use('tableau-colorblind10')
print(len(coords))

i = 0
linestyles = [
    (0, (1, 0)),
    (0, (1, 1)),
    (0, (2, 2)),
    (0, (4, 4)),
]
for c in coords:
    plt.plot(c['x'], [q / c['avg'] for q in c['y']], label=c['label'], linestyle=linestyles[i], color='gray')
    i += 1

plt.legend()
plt.show()
