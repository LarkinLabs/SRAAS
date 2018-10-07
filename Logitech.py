import evdev
from Queue import Queue
inputs = Queue()
mode = Queue()
def Get_Controller ():
    devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
    for device in devices:
        print device
        if device.name == "Logitech Logitech Cordless RumblePad 2":
            print device
            dev = device
            return dev
    
def HID_Controller(dev):
    MA =0
    MB=0
    LTV = 127
    LTH = 127
    RTV = 127
    RTH = 127
    for event in dev.read_loop():
        if event.type == 3 or event.type == 1 :
            if event.code == 0:
                #print('LH =', str(event.value))
                LTH = int(event.value)
            if event.code == 1:
                #print('LV =', str(event.value))
                LTV = int(event.value)
            if event.code == 2:
                #print('RH =', str(event.value))
                RTH = int(event.value)
            if event.code == 5:
                #print('RV =', str(event.value))
                RTV = int(event.value)
            if event.code == 306:
                if event.value ==1:
                    print('Recording')
                    mode.put(2)
                else:
                    print ('Recording Stopped')
                    mode.put(0)
            if event.code == 305:
                if event.value ==1:
                    print('Auto')
                    mode.put(1)
                else:
                    print ('Manual')
                    mode.put(0)
                
        #print (LTH,LTV )
        V = -1* (LTV - 127)
        T = (LTH - 127)
        #print(V,T)
        MA = V+T/2
        MB = V-T/2
        if event.code == 305:
            MA = 0
            MB = 0
        inputs.queue.clear()
        inputs.put((MA,MB))
        
