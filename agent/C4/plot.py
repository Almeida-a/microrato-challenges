import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt1
import pandas as pd


class Main:
    data = pd.read_csv('pos.csv')

    dev_x, dev_y = data.est_x - data.real_x, data.est_y - data.real_y
    stats = tuple((np.min(dv), np.max(dv), np.std(dv), np.average(dv)) for dv in (dev_x, dev_y))
    print(f"X deviation min/max/dev/avg = {stats[0]}")
    print(f"Y deviation min/max/dev/avg = {stats[1]}")

    plt.subplot(2, 2, 1)
    plt.plot(data.real_x, color='g')
    plt.plot(data.est_x, color='r')
    plt.title("X")

    plt.subplot(2, 2, 2)
    plt.plot(data.real_y, color='g')
    plt.plot(data.est_y, color='r')
    plt.title("Y")

    plt.subplot(2, 2, 3)
    plt.plot(dev_x, color='b')
    plt.title("X_dev")

    plt.subplot(2, 2, 4)
    plt.plot(dev_y, color='b')
    plt.title("Y_dev")

    plt.show()



