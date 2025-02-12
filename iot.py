# ------ Imports ------
import time
import board
import busio as bus
import pwmio as pwm
import digitalio as dio
import analogio as aio
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219

# ------ Functions ------

# Other
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

# RPM measurement
def updateRpmHistory(newValue, historyList):
    historyList.append(newValue)
    
    if len(historyList) > rpmHistoryLength:
        historyList.pop(0)

def getAverageRpm(historyList):
    if len(historyList) == 0:
        return 0
    return round(sum(historyList)/len(historyList))

# Servo
def servoDeg(deg):
    min_duty = 1638 
    max_duty = 8192 
    return round(min_duty + (deg / 180) * (max_duty - min_duty))

def shiftUp():
    global servoTarget, isShifting, shiftStartTime, isShiftDelay
    if not isShiftDelay:
        servoTarget = shiftUpDeg
        isShifting = True
        shiftStartTime = time.monotonic()

def shiftDown():
    global servoTarget, isShifting, shiftStartTime, isShiftDelay
    if not isShiftDelay:
        servoTarget = shiftDownDeg
        isShifting = True
        shiftStartTime = time.monotonic()

# Motor
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
            
# Power sensor
def measureINA219(sensor):
    bus_voltage = sensor.bus_voltage  # voltage on V- (load side)
    shunt_voltage = sensor.shunt_voltage  # voltage between V+ and V- across the shunt
    current = sensor.current  # current in mA
    power = sensor.power  # power in watts
    return (bus_voltage,shunt_voltage,current,power)
            
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
servo = pwm.PWMOut(board.GP14, frequency=50, duty_cycle=servoDeg(83))

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

# Hall effect general
hallTriggerVal = 20000
rpmHistoryLength = 100

# Hall effect sensor to motor
motorRpm = 0
hallMotorTriggered = False
hallMotorMoved = False
hallMotorStartTime = None # Init
motorLastRpmUpdate = programStartTime # Track last RPM update
motorRpmHistory = []

# Hall effect sensor to output
outRpm = 0
hallOutTriggered = False
hallOutMoved = False
hallOutStartTime = None # Init
outLastRpmUpdate = programStartTime  # Track last RPM update
outRpmHistory = []

# Servo
shiftUpDeg = 23
shiftDownDeg = 150
shiftMiddleDeg = 83
servoTarget = shiftMiddleDeg
isShifting = False
isShiftDelay = False
shiftStartTime = None

# Print Values
nextPrintTime = programStartTime
printDelay = 1

# Power Sensor
nextPowerReadTime = programStartTime
powerReadDelay = 1

# Vibration Sensor
nextVibrationReadTime = programStartTime
vibrationReadDelay = 1

servo.duty_cycle = servoDeg(shiftMiddleDeg)
time.sleep(3)
motor(100,"f")

# ------ Loop ------
while True:   
        currentTime = time.monotonic()
        
        
        # Measure RPM
        
        # Hall effect for motor
        if hallMotor.value <= hallTriggerVal:
            if not hallMotorTriggered: # Runs once per measurement to begin measuring time
                hallMotorStartTime = time.monotonic_ns() # Registers time that we hit first magnet
                hallMotorTriggered = True
            
            if hallMotorTriggered and hallMotorMoved and hallMotorStartTime is not None:
                hallMotorStopTime = time.monotonic_ns() # Registers time that we hit second magnet
                motorElapsedTime = (hallMotorStopTime - hallMotorStartTime) / 1e9 # Elapsed time in seconds
                
                if motorElapsedTime > 0:
                    if round(60 / (motorElapsedTime * 4)) < 700: # Filters out unrealistic values
                        motorRpm = round(60 / (motorElapsedTime * 4)) # *4 cause we have 4 magnets per revolution
                        motorLastRpmUpdate = time.monotonic() # Set last time we got a RPM update
                
                hallMotorTriggered = False # Reset flags
                hallMotorMoved = False
        else:
            hallMotorMoved = True
            
        if (currentTime - motorLastRpmUpdate) >= 1:
            motorRpm = 0 # Set RPM to zero if too long since update
            
        updateRpmHistory(motorRpm, motorRpmHistory) # Used to calculate average rpm
        
        # Hall effect for output
        if hallOut.value <= hallTriggerVal:
            if not hallOutTriggered: # Runs once per measurement to begin measuring time
                hallOutStartTime = time.monotonic_ns() # Registers time that we hit first magnet
                hallOutTriggered = True
            
            if hallOutTriggered and hallOutMoved and hallOutStartTime is not None:
                hallOutStopTime = time.monotonic_ns() # Registers time that we hit second magnet
                outElapsedTime = (hallOutStopTime - hallOutStartTime) / 1e9 # Elapsed time in seconds
                
                if outElapsedTime > 0:
                    if round(60 / (outElapsedTime * 4)) < 700: # Filters out unrealistic values
                        outRpm = round(60 / (outElapsedTime * 4)) # *4 cause we have 4 magnets per revolution
                        outLastRpmUpdate = time.monotonic() # Set last time we got a RPM update
                
                hallOutTriggered = False # Reset flags
                hallOutMoved = False
        else:
            hallOutMoved = True
            
        if (currentTime - outLastRpmUpdate) >= 1:
            outRpm = 0 # Set RPM to zero if too long since update
            
        updateRpmHistory(outRpm, outRpmHistory) # Used to calculate average rpm
            
        # Shifting
        servo.duty_cycle = servoDeg(servoTarget)

        if isShifting and (currentTime - shiftStartTime) >= 0.3: # Delay to give the servo time to move
            servoTarget = shiftMiddleDeg
            isShifting = False
            isShiftDelay = True
        elif isShiftDelay and (currentTime - shiftStartTime) >= 0.7: # Cooldown to prevent dubble shifting
            isShiftDelay = False

        # Power
        if currentTime - nextPowerReadTime >= 0: # Runs repetedly with a delay
            powerSensorReading = measureINA219(ina219)
            if powerSensorReading[0] <= 9.6:
                print("--- WARNING ---")
                print(f"Critical low battery voltage: {powerSensorReading[0]}")
                break
            nextPowerReadTime = currentTime + powerReadDelay
        
        
        # Vibration
        if currentTime - nextVibrationReadTime >= 0: # Runs repetedly with a delay
            vibrationVal = vibrationSensor.value
            nextVibrationReadTime = currentTime + vibrationReadDelay
        
        # Motors 
        if currentTime - programStartTime >= 30: # Stops motor and exits
            motorStop()
            break

        # Servo

        
        # Print
        if currentTime - nextPrintTime >= 0:
            motorAverageRpm = getAverageRpm(motorRpmHistory)
            outAverageRpm = getAverageRpm(outRpmHistory)
            print(f"Battery Voltage: {powerSensorReading[0]}")
            print(f"Servo target:  {servoTarget}")
            print(f"Vibration: {vibrationVal}")
            print(f"Motor RPM: {motorRpm}")
            print(f"Motor average RPM: {motorAverageRpm}")
            print(f"Output RPM: {outRpm}")
            print(f"Output average RPM: {outAverageRpm}")
            nextPrintTime = currentTime + printDelay
            
        