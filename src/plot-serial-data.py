import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

class Data_plot:

    def __init__(self, max_len=1000, chan_num=1, port='/dev/ttyUSB0', baudrate='115200'):
        self.data = np.zeros((max_len, chan_num))
        self.time = np.zeros((max_len, 1))
        self.max_len = max_len
        self.chan_num = chan_num
        self.fig, self.axes = plt.subplots(chan_num, 1)
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port=port, baudrate=baudrate)
        for axis in self.axes:
          #  axis.set_xlim(0, 100)
            axis.set_ylim(0, 100)
        self.lines  = [axis.plot([], [])[0] for axis in self.axes]

    def input_data(self):
        try:
            ser_input = self.ser.readline().decode().rstrip().split(',')
            new_data = [float(s) for s in ser_input[0:-1]]
            new_data = np.array(new_data)
            assert len(new_data) is self.chan_num
            time_stamp = float(ser_input[-1])
        except KeyboardInterrupt:
            print('Error while get input data')
        return new_data, time_stamp

    def update_time(self, new_time):
        self.time[0:-1] = self.time[1:]
        self.time[-1] = new_time
        for axis in self.axes:
            axis.set_xlim(self.time[0], self.time[-1])
        

    def update_data(self, new_data):
        self.data[0:-1, :] = self.data[1:, :]
        self.data[-1,   :] = new_data

    def update(self, frame_num):
        new_data, time_stamp = self.input_data()
        self.update_data(new_data)
        self.update_time(time_stamp)
        for i in range(self.chan_num):
            self.lines[i].set_ydata(self.data[:, i])
            self.lines[i].set_xdata(self.time)

    def close(self):
        print('close')

def main():
    print('start')
    data_plot = Data_plot(max_len=100, chan_num=3)
    anim = animation.FuncAnimation(data_plot.fig, data_plot.update, interval=0)
    plt.show()
    data_plot.close()

if __name__ == '__main__':
    main()
