# Imports
import time
import board
import busio as bus
import pwmio as pwm
import digitalio as dio
import analogio as aio
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219

# Functions
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


# Hall sensors
hallMotor = aio.AnalogIn(board.GP28)
hallOut = aio.AnalogIn(board.GP27)

# Motors
en1 = pwm.PWMOut(board.GP21, frequency=5000, duty_cycle=0)
en2 = pwm.PWMOut(board.GP20, frequency=5000, duty_cycle=0)

in1 = initPin(board.GP19, "output")
in2 = initPin(board.GP18, "output")
in3 = initPin(board.GP17, "output")
in4 = initPin(board.GP16, "output")

# Variables
hallOutRpm = 0
hallOutHit = False
hallOutMoved = False
hallOutStart = None  # Init
lastRpmTime = time.monotonic()  # Track last RPM update
programStart = time.monotonic()

nextRun = 0

while True:
    if hallOut.value <= 20000:
        if not hallOutHit:  # Set start time on first detection
            hallOutStart = time.monotonic_ns()
            hallOutHit = True

        if hallOutHit and hallOutMoved and hallOutStart is not None:
            hallOutStop = time.monotonic_ns()
            elapsed_time = (hallOutStop - hallOutStart) / 1e9  # Convert to seconds

            if elapsed_time > 0:
                hallOutRpm = round(60 / (elapsed_time * 4))
                lastRpmTime = time.monotonic()  # Update last RPM time

            hallOutHit = False
            hallOutMoved = False  # Reset moved
    else:
        hallOutMoved = True  # Indicate sensor saw movement

    # If no new RPM update in 1 second, reset RPM to 0
    if time.monotonic() - lastRpmTime > 1:
        hallOutRpm = 0
        
        
    motor(1,100,"f")
    motor(2,100,"f")
    
    if (time.monotonic() - programStart) >= 10:
        motorStop(1)
        motorStop(2)
        break
    
    
    # Print values after delay
    if time.monotonic() >= nextRun:
        print(f"HallOutRpm: {hallOutRpm}")
        print(f"HallMotor: {hallMotor.value}")
        print(f"HallOut: {hallOut.value}")
        nextRun = time.monotonic() + 1
