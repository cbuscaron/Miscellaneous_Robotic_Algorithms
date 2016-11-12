#!/usr/bin/env python
# Author: Camilo F. Buscaron
# sudo apt-get install python-pip
# sudo pip install pyserial
# sudo apt-get install python-matplotlib

from __future__ import print_function

import serial
import time, datetime
import matplotlib.pyplot as plt
import csv
import os, sys

def convert(data):
    """ Convert data from list-of-dicts to dict-of-lists, but only use certain keys"""
    return {k: [d[k] for d in data] for k in data[0].keys()}

def save(filename, data, keys):
    """ Save dictionary of lists to csv file.
    Only the dictionary items with 'keys' are save to csv file
    The first row of csv file is the keys value, which will be used when loading the data back in
    """
    with open(filename,'w') as fd:
        csv_out = csv.writer(fd, delimiter=',', quotechar='"')
        csv_out.writerow(keys)
        for row in zip( *[data[k] for k in keys] ):
            csv_out.writerow(row)

def load(filename):
    """ Load csv file into a dictionary of lists.
    The dictionary keys are loaded base on the first line of the data file.
    """
    with open(filename,'r') as fd:
        csv_in = csv.reader(fd, delimiter=',', quotechar='"')
        keys = csv_in.next()
        data = {k:[] for k in keys}
        for row in csv_in:
            for k,v in zip(keys,row):
                data[k].append(float(v))
    return data



def main():
    ser = serial.Serial('/dev/ttyACM0', 115200)

    readings = []
    try:
        print('Started recording...')
        print('ctrl-c to stop')
        while True:
            p_var = ser.readline().strip('\r\n')

            try:
                p_var = float(p_var)
            except ValueError,e:
                continue
            time_stamp = time.time()
            #print(p_var, time_stamp)
            readings.append({'preassure': p_var, 'time_stamp': time_stamp})
    except KeyboardInterrupt:
        #print(readings)
        print('Stopped recording...')
        print('Saving...')
        now = str(datetime.datetime.now().isoformat())
        data = convert(readings)

        save('data_collected_' + now + '.csv', data, ['preassure', 'time_stamp'])
        print('Saved. Closing.')

        plt.plot(data['time_stamp'], data['preassure'], label='Preassure Change')
        plt.xlabel('Time (Secs)')
        plt.ylabel('Preassure (Psi)')

        plt.title("Preassure Reaction Plot")

        plt.legend()

        plt.show()
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

if __name__ == "__main__":
    main()
