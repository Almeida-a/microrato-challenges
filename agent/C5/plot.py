import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

if __name__ == '__main__':
    data = pd.read_csv('pos.csv')

    # Get the deviations
    dev_x = data.est_x - data.real_x
    dev_y = data.est_y - data.real_y
    dev_ang = data.est_ang - data.real_ang

    stats = tuple((np.min(dv), np.max(dv), np.std(dv), np.average(dv)) for dv in (dev_x, dev_y, dev_ang))
    print(f"X deviation min/max/dev/avg = {stats[0]}")
    print(f"Y deviation min/max/dev/avg = {stats[1]}")
    print(f"Angle deviation min/max/dev/avg = {stats[2]}")

    plt.subplot(2, 2, 1)
    plt.plot(dev_x, color='r')
    plt.legend("RE Diff")
    plt.title("X_dev")

    plt.subplot(2, 2, 2)
    plt.plot(dev_y, color='g')
    plt.legend("RE Diff")
    plt.title("Y_dev")

    plt.subplot(2, 2, 3)
    plt.plot(dev_ang, color='b')
    plt.legend("RE Diff")
    plt.title("Angle_dev")

    plt.show()



