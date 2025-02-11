# ------ Imports ------
import time
import board
import busio as bus
import pwmio as pwm
import digitalio as dio
import analogio as aio
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219

# ------ Functions ------

def initPin(boardPin, direction, pull=None):

    pin = dio.DigitalInOut(boardPin)

    if direction == 'output':
        pin.direction = dio.Direction.OUTPUT
    elif direction == 'input':
        pin.direction = dio.Direction.INPUT


        if pull == 'up':
            pin.pull = dio.Pull.UP
        elif pull == 'down':
            pin.pull = dio.Pull.DOWN

    return pin

def motor(speed, direction, debug = False):

    pwmSpeed = round((speed/100)*65535)

    
        en1.duty_cycle = pwmSpeed
        en2.duty_cycle = pwmSpeed
        if(direction == "f"):
            in1.value = True
            in2.value = False
            in3.value = True
            in4.value = False
        elif(direction == "r"):
            in1.value = False
            in2.value = True
            in3.value = False
            in4.value = True

    if debug:
        print(f"""
        en1: {en1.duty_cycle}
        en2: {en2.duty_cycle}
        in1: {in1.value}
        in2: {in2.value}
        in3: {in3.value}
        in4: {in4.value}
        """)
        
def motorStop(debug = False):
        en1.duty_cycle = 0
        en2.duty_cycle = 0
        in1.value = False
        in2.value = False
        in3.value = False
        in4.value = False
        
        if debug:
            print("Motors are stopped")
            
# ------ Definitions ------

# Time
programStartTime = time.monotonic()

# Motor
en1 = pwm.PWMOut(board.GP21, frequency=5000, duty_cycle=0)
en2 = pwm.PWMOut(board.GP20, frequency=5000, duty_cycle=0)

in1 = initPin(board.GP19, "output")
in2 = initPin(board.GP18, "output")
in3 = initPin(board.GP17, "output")
in4 = initPin(board.GP16, "output")

# Servo
servo = pwm.PWMOut(board.GP14, frequency=50, duty_cycle=servoDeg(75))

# Power sensor
i2c_bus = bus.I2C(board.GP5,board.GP4)
ina219 = INA219(i2c_bus)

ina219.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
ina219.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
ina219.bus_voltage_range = BusVoltageRange.RANGE_16V

# Vibration sensor
vibrationSensor = initPin(board.GP15, "input")

# Hall effect sensor connected to motor
hallMotor = aio.AnalogIn(board.GP28)

# Hall effect sensor connected to output
hallOut = aio.AnalogIn(board.GP27)

# ------ Variables ------

# Hall effect sensor to motor
hallMotorRpm = 0
hallMotorTriggered = False
hallMotorMoved = False
hallMotorStartTime = None
MotorLastRpmUpdate = programStartTime

# Hall effect sensor to output
hallOutRpm = 0
hallOutTriggered = False
hallOutMoved = False
hallOutStartTime = None  # Init
OutlastRpmUpdate = programStartTime  # Track last RPM update



# ------ Loop ------
while True:
        currentTime = time.monotonic()