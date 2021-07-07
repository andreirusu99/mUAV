import csv
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt

CSV_PATH = '/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/out/05Jul21/data.csv'
OUT_PATH = '/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/out/05Jul21'

x = []
y = []
z = []
with open(CSV_PATH, 'r') as csvfile:
    plots = csv.DictReader(csvfile, delimiter=',')
    initial_time = 0
    initial = True
    for row in plots:
        timestamp = int(row['stamp'])
        people = int(row['people'])
        warning = int(row['warning'])
        density = float(row['density'])
        altitude = float(row['altitude[m]'])
        height = float(row['height[m]'])
        temp = float(row['temp[°C]'])
        area = float(row['area[m2]'])
        battery = int(row['battery[%]'])

        if 0.1 < density < 5 and people > 5 and height > 2 and area > 10 and 1625504041 < timestamp < 1625505301:

            if initial:
                initial_time = timestamp
                initial = False

            x.append((timestamp - initial_time))
            y.append(area)
            z.append(density)

    fig, axis = plt.subplots(2, sharex='all')

    axis[0].plot(x, y)
    axis[0].set_title('Area')
    axis[0].set(ylabel='m2')

    axis[1].plot(x, z)
    axis[1].set_title('Density')
    axis[1].set(ylabel='People/10m2')

    plt.xlabel('Time [s]')

    plt.savefig(f'{OUT_PATH}/GRAPH.png')
