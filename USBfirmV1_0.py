from __future__ import division
import matplotlib
import multiprocessing
import time
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from libMPPlot import plot, updateplot, getdata1
import serial
import Adafruit_PCA9685
import math
import pyvesc
import struct
import os
import sys
import datetime
from pyvesc import GetValues, SetRPM, SetCurrent,SetDutyCycle, SetRotorPositionMode, GetRotorPosition

# =====================================================
#   Parallel processing main to show realtime plot
# =====================================================
def main():
    #Create a queue to share data between process
    q = multiprocessing.Queue()

    #Create and start the simulation process
    getdata_proc=multiprocessing.Process(target=getdata,args=(q,))

    #Create the base plot
    plot()

    getdata_proc.start()

    #Call a function to update the plot when there is new data
    status = 0
    while status == 0:
        status = updateplot(q)

    print('Done')

# ======================================================
#                   Helper functions
# ======================================================

# >>>>>>>>>>>>>>>>>
# To set a servo with PWM signals.
# >>>>>>>>>>>>>>>>>
def set_servo_pulse(channel, pulse):
    pulse_length   = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse  *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# >>>>>>>>>>>>>>>>>
# Restrain a signal in range minn-maxn
# >>>>>>>>>>>>>>>>>
def clamp(n,minn,maxn):
    return max(min(maxn,n),minn)

# >>>>>>>>>>>>>>>>>
# Scale signal to new range (from leftRange to rightRange)
# >>>>>>>>>>>>>>>>>
def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan    = leftMax - leftMin
    rightSpan   = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

# >>>>>>>>>>>>>>>>>
# Send PWM signals to PWM commander (PCA9685)
# >>>>>>>>>>>>>>>>>
# TODO: Fix this shit
def send2PCA(pwm,result):
    steerPWM = math.floor(translate(result[0], 1033, 1835 , 262.5, 487.5))
    GearPWM  = math.floor(translate(result[2], 1103, 1906 , 300, 450))
    pwm.set_pwm(0, 0,clamp(steerPWM,262.5, 487.5))
    pwm.set_pwm(1, 0,clamp(GearPWM, 300  , 450))

# >>>>>>>>>>>>>>>>>
# Decode and print VESC status response
# >>>>>>>>>>>>>>>>>
def print_VESC_values(response):
#    _ = os.system('clear')
    print(chr(27) + "[2J")
    print('==========================')
    print('T_VESC:       ',response.temp_fet_filtered)
#    print('T_motor:      ',response.temp_motor_filtered)
    print('I_motor:      ',response.avg_motor_current)
    print('I_battery:    ',response.avg_input_current)
#    print('avg_id:       ',response.avg_id)
#    print('avg_iq:       ',response.avg_iq)
    print('duty_cycle:   ',response.duty_cycle_now)
    print('RPM:          ',response.rpm)
    print('V_battery:    ',response.input_voltage)
    print('mAh_used:     ',response.amp_hours*1000)
    print('mAh_charged:  ',response.amp_hours_charged*1000)
    print('Wh_used:      ',response.watt_hours)
    print('Wh_charged:   ',response.watt_hours_charged)
#    print('tacho:        ',response.tachometer_value)
#    print('tacho_abs:    ',response.tachometer_abs_value)
#    print('fault:        ',response.fault)
#    time.sleep(0.01)
    # =============Calculated values=================
    p_motor = response.input_voltage*response.avg_input_current
    try:
        v_motor = p_motor/response.avg_motor_current
    except:
        v_motor = 0
    # print calculated stuff
    print('P_motor:      ',p_motor)
    print('V_motor:      ',v_motor)
    print('Torque_motor(N*cm):',find_tau(p_motor,response.rpm)*100)
    return p_motor,find_tau(p_motor,response.rpm)*100,v_motor

# >>>>>>>>>>>>>>>>>
# Estimate torque exerted by the motor from its status
# >>>>>>>>>>>>>>>>>
def find_tau(p_motor,RPM):
    if RPM != 0:
        return p_motor*30/(math.pi*RPM)
    else:
        return 0

# >>>>>>>>>>>>>>>>>
# TODO:Find motor commands to achieve torque in specific RPM
# >>>>>>>>>>>>>>>>>
def achieve_tau(tau,RPM):
    p_motor = tau*math.pi*RPM/30

# >>>>>>>>>>>>>>>>>
# Core of management firmware
# >>>>>>>>>>>>>>>>>
def getdata(q):
    # ==============INITS==================
    # serial port inits
    start = time.time()
    cur_line = ""
    ser_ard = serial.Serial(port='/dev/ttyUSB0', baudrate=2000000, dsrdtr=True)

    # VESC comm inits
    ser_vesc = serial.Serial(port='/dev/ttyACM0', baudrate=2000000, timeout=0.05)
        # Optional: Turn on rotor position reading if an encoder is installed
    ser_vesc.write(pyvesc.encode(SetRotorPositionMode(SetRotorPositionMode.DISP_POS_OFF)))

    # i2c inits
    pwm = Adafruit_PCA9685.PCA9685(busnum=1)
    pwm.set_pwm_freq(60)
    pwm.set_pwm(0, 0,375)
    InRPM = 0

    # misc inits
    pmotor = 0
    tau = 0
    Result_Ard = [1500]*4+[0]*9
    ControlSwitch = 1900
    Ctrl_Radio = [0,0,0]

    # Open Result txt to log data
    # TODO: Add date time to filename
    now = datetime.datetime.now()
    currentDateTime = now.strftime("%Y%m%d-%H%M%S")
    f = open("./Results/"+currentDateTime+".txt","w+")
        # Log headers
    f.write("current_time,tau,rpm,steering PWM,Dutycycle,P_motor,\n")
        # 1104, 1506, 1906 for VESC
    ser_vesc.write(pyvesc.encode(SetRPM(1000)))

    # ==============Central Loop==================
    while time.time()-start < 60:
        tic = time.time()

        ##### Read Data #####

        # Read data chuck from Arduino
        ser_var  = ser_ard.read(ser_ard.inWaiting())
        data_seg = ser_var.decode('utf-8')
        if data_seg.find('\r') == -1:
            # If chuck is incomplete, keep reading
            cur_line += data_seg
        else:
            # If chuck is complete, parse chuck
            cur_line += data_seg.split('\r')[0]
            try:
                Result_Ard = list(map(float,cur_line.split(',')))
            except:
                pass
            cur_line = data_seg.split('\r')[1]

        # Request the current state from the vesc
        ser_vesc.write(pyvesc.encode_request(GetValues))
        # Check if there is enough data back for a measurement
        if ser_vesc.in_waiting > 71:
            (response, consumed) = pyvesc.decode(ser_vesc.read(ser_vesc.in_waiting))
            # Print out the values
            if response:
                pmotor,tau,vmotor = print_VESC_values(response)
                #### Update plot ####
                cur_t = time.time()-start
                q.put([cur_t,tau,response.rpm,response.avg_motor_current,vmotor])
                #### Write to log ####
                steering = Ctrl_Radio[0]
                f.write("%f,%f,%f,%f,%f,%f\n" % (cur_t,tau,response.rpm,steering,InRPM*10.0,pmotor))
                # TODO: Write absolute time to log file

        ##### Understand Data #####
        Ctrl_Radio    = Result_Ard[:3]
        ControlSwitch = Result_Ard[3]
        IMUdata       = Result_Ard[4:]
        InRPM         = translate(Ctrl_Radio[1],1100,1900,-10000,10000)
        if ControlSwitch < 1200:
            Ctrl_Radio[0] = (time.time()-start)/20*200.0*math.sin((time.time()-start)*2)+1435
            dummyRPM      = (time.time()-start)/20*2000.0*math.sin((time.time()-start)*10)+4000

        ##### Send Commands #####
        # Set the ERPM of the VESC motor. | Note: ERPM = poles*real_RPM, in this case poles = 2
        send2PCA(pwm,Ctrl_Radio)
        # ser_vesc.write(pyvesc.encode(SetDutyCycle(10000.0)))
        # ser_vesc.write(pyvesc.encode(SetCurrent(2)))
        if ControlSwitch < 1200:
            ser_vesc.write(pyvesc.encode(SetRPM(dummyRPM)))
        else:
            ser_vesc.write(pyvesc.encode(SetDutyCycle(InRPM*10.0)))

        # Show time usage for each cycle
        toc = time.time()-tic
        print(toc)

    # ==============Cleanups==================
    # Close VESC
    # ser_vesc.write(pyvesc.encode(SetCurrent(0)))

    # Close serial ports
    ser_ard.close()
    ser_vesc.close()

    # Close file
    f.close()
    print("serial ports closed")

    # Close parallel queue
    q.put(['Q','Q','Q','Q','Q'])


# ======================================================
#                       MISC
# ======================================================

if __name__ == '__main__':
    main()
