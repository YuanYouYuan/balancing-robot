import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

class Data_plot:

    def __init__(self, max_len=1000, chan_num=1):
        self.data = np.zeros((max_len, chan_num))
        self.max_len = max_len
        self.chan_num = chan_num
        self.fig, self.axes = plt.subplots(chan_num, 1)
        for axis in self.axes:
            axis.set_xlim(0, 100)
            axis.set_ylim(0, 100)
        self.lines  = [axis.plot([], [])[0] for axis in self.axes]

    def input_data(self):
        try:
            new_data = np.random.randn(1, self.chan_num)*100
        except KeyboardInterrupt:
            print('Error while get input data')
        return new_data

    def update_data(self, new_data):
        self.data[0:-1, :] = self.data[1:, :]
        self.data[-1,   :] = new_data

    def update(self, frame_num):
        new_data = self.input_data()
        self.update_data(new_data)
        for i in range(self.chan_num):
            self.lines[i].set_data(range(self.max_len), self.data[:, i])

    def close(self):
        print('close')

def main():
    print('start')
    data_plot = Data_plot(max_len=100, chan_num=4)
    anim = animation.FuncAnimation(data_plot.fig, data_plot.update, interval=50)
    plt.show()
    data_plot.close()

if __name__ == '__main__':
    main()
