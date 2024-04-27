from datetime import datetime
from subprocess import Popen

import os
import pigpio

pi = pigpio.pi()

folder = os.path.expanduser("~/") +  datetime.now().strftime("%Y-%m-%d %H:%M:%S")
os.mkdir(folder)

# start taking images
Popen(["libcamera-still", "--timelap", "900", "-o", f"{folder}/images/i%04d.png", "-v", "0"])

def write_data(file):
    (c, data) = pi.serial_read(handle)
    if c > 0:
        file.write(data.decode())


with open(f"{folder}/data.txt", "w", buffering=1) as file:
    try:
        # serial connection on GPIO 14 and 15
        handle = pi.serial_open("/dev/ttyS0", 9600)

        while True:
            write_data(file)
    finally:
        pi.serial_close(handle)

# In crontab:
# @reboot python3 ~/startup.py &
