from datetime import datetime
from subprocess import Popen

import os
import pigpio

pi = pigpio.pi()

folder = os.path.expanduser("~/") +  datetime.now().strftime("%Y-%m-%d %H:%M:%S")
os.mkdir(folder)
os.mkdir(f"{folder}/images")

# start taking images
Popen(["libcamera-still", "--timelap", "900", "-t", "999999999", "-o", f"{folder}/images/i%04d.png", "-v", "0"])

def write_data(file):
    (c, data) = pi.serial_read(handle)
    if c > 0:
        file.write(data.decode())

with open(f"{folder}/data.txt", "w", buffering=1) as file:

    # serial connection on GPIO 14 and 15
    handle = pi.serial_open("/dev/ttyS0", 9600)

    try:
        while True:
            write_data(file)
    finally:
        file.flush()
        pi.serial_close(handle)
