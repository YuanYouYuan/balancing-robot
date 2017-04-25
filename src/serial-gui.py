import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider

port = '/dev/ttyUSB0'
baudrate = 115200
s = serial.Serial(port, baudrate)
datas = [];
datas = np.array(datas)

fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot(datas, 'r--')
plt.subplots_adjust(left=0.25, bottom=0.25)



def data_gen(t=0):
    cnt = 0
    while cnt < 1000:
        cnt += 1
        t += 0.1
        data = s.readline().decode().rstrip()
        yield t, 
while True:
    try:
        data = s.readline().decode().rstrip()
        print(data)
        data = float(data)
        datas = np.append(datas, data)
        line1.set_ydata(datas)
        line1.set_xdata(np.arange(len(datas)))
        fig.canvas.draw()
    except KeyboardInterrupt:
        s.close()
        print(datas)
        exit

    #if data != None:
    #    datas = np.append(datas, data)
    #    plt.plot(data)
    #    plt.show()
    #    

    
