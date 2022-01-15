import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt1
import pandas as pd


class Main:
    data = pd.read_csv('pos.csv')

    plt.subplot(2, 1, 1)
    plt.plot(data.real_x, color='g')
    plt.plot(data.est_x, color='r')
    plt.title("X")
    plt.subplot(2, 1, 2)
    plt.plot(data.real_y, color='g')
    plt.plot(data.est_y, color='r')
    plt.title("Y")
    plt.show()



