import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

def zoom(a, factor):
    a = np.asarray(a)
    slices = [slice(0, old, 1/factor) for old in a.shape]
    idxs = (np.mgrid[slices]).astype('i')
    return a[tuple(idxs)]

ser = serial.Serial('COM5', 115200, timeout=0)

fig = plt.figure()

data_array = np.random.rand(2, 4)
print(data_array)
data_array_resize = zoom(data_array,2)

im = plt.imshow(data_array, interpolation='hermite', cmap='viridis', animated=True)


def updatefig(*args):
    address = 0
    timeout = 10;
    while address != 1 and timeout > 0:
        timeout = timeout - 1
        try:
            serial_data = ser.readline().rstrip() # read data from serial
                                           # port and strip line endings
            address  = int(serial_data.split(b',')[0]) - 7
        except:
                pass
    if timeout > 0:
        distance = int(serial_data.split(b',')[1])
        if distance == 0:
            distance = 40
        data_array = np.ones((2, 4))
        data_array[0,address - 1] = distance/300

        i = 1;
        while i < 8:
            i = i + 1
            serial_data = ser.readline().rstrip() # read data from serial
                                           # port and strip line endings
            try:
                address  = int(serial_data.split(b',')[0]) - 7
                distance = int(serial_data.split(b',')[1])
                if distance == 0:
                    distance = 40
                if address <= 4:
                    data_array[0, address - 1] = distance/300
                else:
                    data_array[1, address - 1 - 4] = distance/300
            except:
                pass
        #data_array_resize = zoom(data_array,2)
        im.set_array(data_array)
        #print(data_array)
    return im,

ani = animation.FuncAnimation(fig, updatefig, interval=1, blit=False)
plt.show()
