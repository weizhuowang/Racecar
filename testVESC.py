# Note: It is utilizing the pyvesc lib in the folder.
import pyvesc
from pyvesc import GetValues, SetRPM, SetCurrent, SetRotorPositionMode, GetRotorPosition
import serial
import time

# Set your serial port here (either /dev/ttyX or COMX)
serialport = '/dev/ttyACM0'

def print_VESC_values(response):
    print('T_VESC:       ',response.temp_fet_filtered)
#    print('T_motor:      ',response.temp_motor_filtered)
    print('I_motor_avg:  ',response.avg_motor_current)
    print('I_battery_avg:',response.avg_input_current)
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

def get_values_example():
    ERPM = 3000
    with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
        try:
            # Optional: Turn on rotor position reading if an encoder is installed
            ser.write(pyvesc.encode(SetRotorPositionMode(SetRotorPositionMode.DISP_POS_OFF)))
            while True:
                # Set the ERPM of the VESC motor
                #    Note: if you want to set the real RPM you can set a scalar
                #          manually in setters.py
                #          12 poles and 19:1 gearbox would have a scalar of 1/228
#                ERPM = ERPM - 1
                ser.write(pyvesc.encode(SetRPM(ERPM)))

                # Request the current measurement from the vesc
                ser.write(pyvesc.encode_request(GetValues))
                time.sleep(0.05)
                # Check if there is enough data back for a measurement
                if ser.in_waiting > 71:
                    (response, consumed) = pyvesc.decode(ser.read(ser.in_waiting))

                    # Print out the values
                    if response:
                        print_VESC_values(response)
                        print(ERPM)

        except KeyboardInterrupt:
            # Turn Off the VESC
            ser.write(pyvesc.encode(SetCurrent(0)))

if __name__ == "__main__":
    get_values_example()
