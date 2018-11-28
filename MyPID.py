### Step 1: Import/download all necessary modules
import spidev
import math
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import style
import sys
from sys import stdout # This and the import below allow error signal reading to be displayed continuously on command line
import time
from time import sleep
#from datetime import datetime
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BCM) # Set GPIO numbering system using GPIO numbers and not pin numbers
GPIO.setup (18,GPIO.OUT)
GPIO.setup (17,GPIO.OUT)

### PID Control
class PID(object):
    def __init__(self, P=1.0, I=0.0, D=0.0): #PID parameters definition

        P = 1.0
        I = 0.0
        D = 0.0

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()


    def clear(self):
        """Clears PID computations and coefficients"""
        self.set_point = 355.0

        self.PTerm = 1.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback

        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """
        error = self.set_point - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

        if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
        elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

        self.DTerm = 0.0
        if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
        self.last_time = self.current_time
        self.last_error = error

        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
        
    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time
#sys.path.insert(0, '/home/pi/Adafruit-Raspberry-Pi-Python-Code/Adafruit_MAX31865') # Sets the import path to the appropriate folder containing ADC modules
### Step 2: Temperature Measurement
Rref = 430 #Reference resistance
wire = 4 #No. of wires for amplifier

#Configure MAX31865
def configureMAX31865():
    lst = [0x80]
    if (wire == 4):
        lst.append(0xC2)

    spi.writebytes(lst)

#Callendar-Van Dusen Equation
def CallendarVanDusen(R):
    a = 3.9083e-03
    b = -5.7750e-07
    R0 = 100

    return (-R0*a+math.sqrt(R0*R0*a*a - 4*R0*b*(R0-R)))/(2*R0*b);

spi = spidev.SpiDev()

fw = open("testing.txt", "w") #opening new data file
pid = PID(1.0, 0, 0)
pid.set_point = 355.0
#pid.sample_time(0.01)
feedback_value = 0
feedback = feedback_value
feedback_list = []
time_list = []
setpoint_list = []   
for i in range(0,60):
    pid.update(feedback)
    output = pid.output
    spi.open(0,0)
    spi.mode=3
    lst=[0x80,0xC2]
    configureMAX31865()
    reg = spi.readbytes(9)
    del reg[0]
    RTDdata = reg[1] << 8 | reg[2] #register data from RTD
    ADCcode = RTDdata >> 1 #analogue to digital conversion
    print("-----------------------------")
    R = ADCcode*Rref / float(32768)
    temp = round(CallendarVanDusen(R),2) + 273.15 #converting temperature to K
    print("Reading no.:", i)
    print("Resistance:", R, "Ohms")
    print("Temperature:",temp, "K")
### Step 5: Implementation
    set_point = 360 #target temperature
    startTime = time.time()
    p_fact = 5.0
    i_fact = 0.1
    i_total = 1.0
    d_fact = 7.35
    last_e = None
    e = set_point - temp
    if last_e is None:
        last_e = e
    i_total += e
    if (e * p_fact + (e - last_e) * d_fact + i_total * i_fact) > 0.0:
        GPIO.output(17,GPIO.LOW)
        GPIO.output(18,GPIO.HIGH)
        print("Heater On")
    else:
        GPIO.output(17,GPIO.HIGH)
        GPIO.output(18,GPIO.LOW)
        print("Heater Off")
    sleep(1)
    
#logging data
    #fw.write('Reading' + ',' + 'Date' + ',' + 'Time' + ',' + 'Resistance' + ',' + 'Resistance')

    fw.write(str(i+1) + ', '+ str(datetime.datetime.now().strftime('%Y-%m-%d, %H:%M:%S'))+
             ', '+str(R)+ ', ' + str(temp)+ '\n')
for i in range(60,130):
    pid.update(feedback)
    output = pid.output
    spi.open(0,0)
    spi.mode=3
    lst=[0x80,0xC2]
    configureMAX31865()
    reg = spi.readbytes(9)
    del reg[0]
    RTDdata = reg[1] << 8 | reg[2] #register data from RTD
    ADCcode = RTDdata >> 1 #analogue to digital conversion
    print("-----------------------------")
    R = ADCcode*Rref / float(32768)
    temp = round(CallendarVanDusen(R),2) + 273.15 #converting temperature to K
    print("Reading no.:", i)
    print("Resistance:", R, "Ohms")
    print("Temperature:",temp, "K")
### Step 5: Implementation
    set_point = 362
    #target temperature
    startTime = time.time()
    p_fact = 2.5
    i_fact = 0.1
    i_total = 1.0
    d_fact = 7.35
    last_e = None
    e = set_point - temp
    if last_e is None:
        last_e = e
    i_total += e
    if (e * p_fact + (e - last_e) * d_fact + i_total * i_fact) > 0.0:
        GPIO.output(17,GPIO.LOW)
        GPIO.output(18,GPIO.HIGH)
        print("Heater On")
        sleep(0.75)
        GPIO.output(17,GPIO.HIGH)
        GPIO.output(18,GPIO.LOW)
        print("Heater Off")
    else:
        GPIO.output(17,GPIO.HIGH)
        GPIO.output(18,GPIO.LOW)
        print("Heater Off")
    sleep(1)
    
#logging data
    #fw.write('Reading' + ',' + 'Date' + ',' + 'Time' + ',' + 'Resistance' + ',' + 'Resistance')

    fw.write(str(i+1) + ', '+ str(datetime.datetime.now().strftime('%Y-%m-%d, %H:%M:%S'))+
             ', '+str(R)+ ', ' + str(temp)+ '\n')
for i in range(130,600):
    pid.update(feedback)
    output = pid.output
    spi.open(0,0)
    spi.mode=3
    lst=[0x80,0xC2]
    configureMAX31865()
    reg = spi.readbytes(9)
    del reg[0]
    RTDdata = reg[1] << 8 | reg[2] #register data from RTD
    ADCcode = RTDdata >> 1 #analogue to digital conversion
    print("-----------------------------")
    R = ADCcode*Rref / float(32768)
    temp = round(CallendarVanDusen(R),2) + 273.15 #converting temperature to K
    print("Reading no.:", i)
    print("Resistance:", R, "Ohms")
    print("Temperature:",temp, "K")
### Step 5: Implementation
    set_point = 358
    #target temperature
    startTime = time.time()
    p_fact = 2.5
    i_fact = 0.2
    i_total = 1.0
    d_fact = 7.35
    last_e = None
    e = set_point - temp
    if last_e is None:
        last_e = e
    i_total += e
    if (e * p_fact + (e - last_e) * d_fact + i_total * i_fact) > 0.0:
        GPIO.output(17,GPIO.LOW)
        GPIO.output(18,GPIO.HIGH)
        print("Heater On")
        sleep(0.6)
        GPIO.output(17,GPIO.HIGH)
        GPIO.output(18,GPIO.LOW)
        print("Heater Off")
    else:
        GPIO.output(17,GPIO.HIGH)
        GPIO.output(18,GPIO.LOW)
        print("Heater Off")
    sleep(1)
    
#logging data
    #fw.write('Reading' + ',' + 'Date' + ',' + 'Time' + ',' + 'Resistance' + ',' + 'Resistance')

    fw.write(str(i+1) + ', '+ str(datetime.datetime.now().strftime('%Y-%m-%d, %H:%M:%S'))+
             ', '+str(R)+ ', ' + str(temp)+ '\n')
fw.close()
### Step 6: Producing results
style.use('ggplot')

#reading data from file and plotting it
y = np.float32(pd.read_csv("testing.txt",usecols=range(4,5),header=None))
x = np.float32(pd.read_csv("testing.txt",usecols=range(0,1),header=None))
plt.errorbar(x, y, yerr = 0.5, fmt = 'o') #error bars
plt.plot(x, y, label = "PT100 Data")
plt.title("Testing")
plt.ylabel("Temperature (K)")
plt.xlabel("Sample")
##axes = plt.gca()
##axes.set_xlim([x[0]-1,x[-1]+1])
##axes.set_ylim([290,310])
plt.show()
print(y)
print(np.std(y))
print(np.mean(y))

