
import time
import serial
import smbus

if __name__ == '__main__':

    # serial port inits
    start = time.time()
    cur_line = ""
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=2000000, dsrdtr=True)

    # i2c inits
    bus = smbus.SMBus(0)
    address = 0x40

    while 1:
        ser_var=ser.read(ser.inWaiting())
        data_seg = ser_var.decode('utf-8')
        if data_seg.find('\r') == -1:
            cur_line += data_seg
        else:
            cur_line += data_seg.split('\r')[0]
            result = list(map(float,cur_line.split(',')))
            print(result)
            cur_line = data_seg.split('\r')[1]




        if time.time()-start > 10:
            break

    ser.close()
    print("serial port close")


