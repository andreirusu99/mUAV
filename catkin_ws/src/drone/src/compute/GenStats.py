import csv
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt

CSV_PATH = '/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/out/28Jun21/data.csv'
OUT_PATH = '/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/out/28Jun21'

x = []
y = []
z = []
with open(CSV_PATH, 'r') as csvfile:
    plots = csv.DictReader(csvfile, delimiter=',')
    initial_time = 0
    initial = True
    for row in plots:
        timestamp = int(row['stamp     '])
        people = int(row['people'])
        warning = int(row['warning'])
        density = float(row['density'])
        altitude = float(row['altitude[m]'])
        height = float(row['height[m]'])
        temp = float(row['temp[°C]'])
        area = float(row['area[m2]'])
        battery = int(row['battery[%]'])

        if 0.1 < density < 10 and people > 1 and height > 2 and \
                1624899892 < timestamp < 1624900019:  # 1624900019 crossing

            if initial:
                initial_time = timestamp
                initial = False

            x.append((timestamp - initial_time))
            y.append(density)
            z.append(warning)

    fig, axis = plt.subplots(2, sharex='all')

    axis[0].plot(x, y)
    axis[0].set_title('Density')
    axis[0].set(ylabel='People/10m2')

    axis[1].plot(x, z)
    axis[1].set_title('People closer than 1.5m')

    plt.xlabel('Time [s]')

    plt.savefig(f'{OUT_PATH}/GRAPH.png')
