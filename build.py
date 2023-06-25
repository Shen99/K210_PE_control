import shutil, errno, os

import serial
import serial.tools.list_ports as list_port
from tabulate import tabulate
import argparse

def list_ports():
    ports = list_port.comports()
    ports_info = [[port.device,
                   hex(port.vid) if port.vid else port.vid,
                   hex(port.pid)  if port.pid else port.pid,
                   port.description,
                   port.manufacturer]
                  for port in ports]
    table_header = ["device", "vid", "pid", "description", "manufacturer"]
    ports_info.insert(0, table_header)
    print(tabulate(ports_info, headers="firstrow"))

def copyanything(src, dst):
    try:
        shutil.copytree(src, dst)
    except OSError as exc: # python >2.5
        if exc.errno in (errno.ENOTDIR, errno.EINVAL):
            shutil.copy(src, dst)
        else: raise

parser = argparse.ArgumentParser(prog='serial monitor', description='A serial monitor.')
parser.add_argument('--list', '-l', action='store_true', help='get available port info.')
parser.add_argument('--com', '-c', help='establish communication with given port.')

args = parser.parse_args()

com = "none"

if args.list:
    list_ports()
elif args.com:
    com = args.com
else:
    ports = list_port.comports()
    for port in ports:
        if "USB-SERIAL CH340" in port.description:
            com = port.device
            break

if com != "none":
    source_src = "./src/test"
    source_des = "./kendryte-standalone-sdk/src/test"
    if (os.path.exists(source_des)):
        shutil.rmtree("./kendryte-standalone-sdk/src/test")
    copyanything("./src/test", "./kendryte-standalone-sdk/src/test/")

    os.system('cmake ./kendryte-standalone-sdk -B build -DPROJ=test -G "MinGW Makefiles"')
    if (os.system('cmake --build build') == 0):
        os.system('kflash -p ' + com + ' -b 3000000 --verbose --noansi --terminal build/test.bin')
