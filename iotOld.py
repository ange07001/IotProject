# imports
import time
import board
import busio as bus
import pwmio as pwm
import digitalio as io
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219


# functions
def initPin(pin, direction, pull=None):

    dio = io.DigitalInOut(pin)

    if direction == 'output':
        dio.direction = io.Direction.OUTPUT
    elif direction == 'input':
        dio.direction = io.Direction.INPUT


        if pull == 'up':
            dio.pull = io.Pull.UP
        elif pull == 'down':
            dio.pull = io.Pull.DOWN

    return dio

def motor(motor, speed, direction, debug = False):

    pwmSpeed = round((speed/100)*65535)

    if(motor == 1):
        en1.duty_cycle = pwmSpeed
        if(direction == "f"):
            in1.value = True
            in2.value = False
        elif(direction == "r"):
            in1.value = False
            in2.value = True
    elif(motor == 2):
        en2.duty_cycle = pwmSpeed
        if(direction == "f"):
            in3.value = True
            in4.value = False
        elif(direction == "r"):
            in3.value = False
            in4.value = True

    if(bool(debug) == True):
        print(f"""
        en1: {en1.duty_cycle}
        en2: {en2.duty_cycle}
        in1: {in1.value}
        in2: {in2.value}
        in3: {in3.value}
        in4: {in4.value}
        """)
        
def motorStop(motor):
    if (motor == 1):
        en1.duty_cycle = 0
        in1.value = False
        in2.value = False
    elif (motor == 2):
        en2.duty_cycle = 0
        in3.value = False
        in4.value = False
        
def measureINA219(sensor):
    bus_voltage = sensor.bus_voltage  # voltage on V- (load side)
    shunt_voltage = sensor.shunt_voltage  # voltage between V+ and V- across the shunt
    current = sensor.current  # current in mA
    power = sensor.power  # power in watts
    return (bus_voltage,shunt_voltage,current,power)

def servoDeg(deg):
    min_duty = 1638 
    max_duty = 8192 
    return round(min_duty + (deg / 180) * (max_duty - min_duty))

def shift(direction):
    shiftUpDeg = 10
    shiftDownDeg = 150
    shiftMiddleDeg = 75
    if (direction == "up"):
        servo.duty_cycle = (shiftUpDeg)
        time.sleep(0.2)
        servo.duty_cycle = servoDeg(shiftMiddleDeg)
    elif (direction == "down"):
        servo.duty_cycle = servoDeg(shiftDownDeg)
        time.sleep(0.2)
        servo.duty_cycle = servoDeg(shiftMiddleDeg)

        
# define pins and initialize
en1 = pwm.PWMOut(board.GP21, frequency=5000, duty_cycle=0)
en2 = pwm.PWMOut(board.GP20, frequency=5000, duty_cycle=0)

in1 = initPin(board.GP19, "output")
in2 = initPin(board.GP18, "output")
in3 = initPin(board.GP17, "output")
in4 = initPin(board.GP16, "output")

vibrationSensor = initPin(board.GP15, "input")

i2c_bus = bus.I2C(board.GP5,board.GP4)

ina219 = INA219(i2c_bus)

servo = pwm.PWMOut(board.GP14, frequency=50, duty_cycle=servoDeg(75))


# setup INA219
ina219.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
ina219.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
ina219.bus_voltage_range = BusVoltageRange.RANGE_16V

# program loop
while True:
    """sensorReading = measureINA219(ina219)
    
    if vibrationSensor.value:
        print("Vibration Detected")
    
    print("Voltage (VIN+) : {:6.3f}   V".format(sensorReading[0] + sensorReading[1]))
    print("Voltage (VIN-) : {:6.3f}   V".format(sensorReading[0]))
    print("Shunt Voltage  : {:8.5f} V".format(sensorReading[1]))
    print("Shunt Current  : {:7.4f}  A".format(sensorReading[2] / 1000))
    print("Power Register : {:6.3f}   W".format(sensorReading[3]))
    print("")

    
    if ina219.overflow:
        print("Internal Math Overflow Detected!")
        print("")
    """
    shift("up")
    break
    
    """
    if (float(sensorReading[0]) <= 9.6):
        print("VOLTAGE LOW! SHUTTING DOWN")
        motor(1,0,"f",True)
        motor(2,0,"f",True)
        break"""
    