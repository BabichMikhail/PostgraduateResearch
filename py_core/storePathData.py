import os
import xlsxwriter

storeFolder = os.path.join('..', 'store')
f = open(os.path.join(storeFolder, 'pathData.txt'), 'r')

workbook = xlsxwriter.Workbook(os.path.join(storeFolder, 'pathData.xlsx'))

lines = f.readlines()
k = 1
for i in range(int(lines[0])):
    n = int(lines[k])
    k += 1

    worksheet = workbook.add_worksheet()
    worksheet.write(0, 0, "x")
    worksheet.write(0, 1, "y")
    worksheet.write(0, 2, "z")
    worksheet.write(0, 3, "speed")
    worksheet.write(0, 4, "acceleration")
    worksheet.write(0, 5, "distance")

    for j in range(n):
        parts = lines[k].split(" ")
        x = float(parts[0])
        y = float(parts[1])
        z = float(parts[2])
        v = float(parts[3])
        a = float(parts[4])
        d = float(parts[5])

        worksheet.write(j + 1, 0, x)
        worksheet.write(j + 1, 1, y)
        worksheet.write(j + 1, 2, z)
        worksheet.write(j + 1, 3, v)
        worksheet.write(j + 1, 4, a)
        worksheet.write(j + 1, 5, d)

        k += 1


workbook.close()
