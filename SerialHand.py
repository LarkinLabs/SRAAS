import re
import cv2
import time
import evdev
import serial
import re
from SerialHand import *
from Primesense import *
def _readline(self):
    eol = b'\r'
    leneol = len(eol)
    line = bytearray()
    while True:
        c = self.ser.read(1)
        if c:
            line += c
            if line[-leneol:] == eol:
                break
        else:
            break
    return bytes(line)

def doRead(ser,end,tout):
    matcherend = re.compile(end)    #gives you the ability to search for anything
    
    tic     = time.time()
    buff    = ser.read(128)
    # you can use if not ('\n' in buff) too if you don't like re
    
    while ((time.time() - tic) < tout) and (not matcherend.search(buff)):
       buff += ser.read(1)

    return bytes (buff)
                  
def bytes_to_int(bytes):
    result = 0

    for b in bytes:
        result = result * 256 + int(b)

    return result
