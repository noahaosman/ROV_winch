# pyright: reportMissingImports=false
import time
import board
import busio
from digitalio import DigitalInOut, Direction, Pull  # GPIO module
import adafruit_mcp4725
import smbus # i2c package
from threading import Thread


# constants for actuator
RETRACT = 1
EXTEND = 0
false_pulse_delay_actuator = 0.002  # (zero for no debounce delay)
pulses_per_inch = 54
false_pulse_delay_reed_sw = 0.250  # s


class Actuator:

    def __init__(
        self,
        ON_OFF_pin,
        direction_pin,
        feedback_pin,
        rotation_pin,
        cable_diameter=0.375
    ):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.dac = adafruit_mcp4725.MCP4725(self.i2c, address=0x60)
        self.ON = DigitalInOut(eval('board.D'+str(ON_OFF_pin)))
        self.ON.direction = Direction.OUTPUT
        self.direction = DigitalInOut(eval('board.D'+str(direction_pin)))
        self.direction.direction = Direction.OUTPUT
        self.current_direction = EXTEND

        # optical feedback from the actuator:
        self.feedback = DigitalInOut(eval('board.D'+str(feedback_pin)))
        self.feedback.direction = Direction.INPUT
        self.feedback.pull = Pull.UP
        self.last_pulse_time = time.time()-false_pulse_delay_actuator
        self.position = 0
        Thread(daemon=True, target=self.updatePosition).start()

        # rotation tracking for spool
        self.reed_sw = DigitalInOut(eval('board.D'+str(rotation_pin)))
        self.reed_sw.direction = Direction.INPUT
        self.reed_sw.pull = Pull.DOWN
        self.last_reed_time = time.time()-false_pulse_delay_reed_sw
        self.NeedToMoveActuator = False
        self.RotationCounter = 0
        Thread(daemon=True, target=self.rotationTrackingReedSw).start()

        self.ON.value = 0

        self.cable_diameter = cable_diameter

        self.MAspeed = 100

        self.time_init = time.time()        

        with open('stacking_state.txt') as infp:
            self.line_stack_state = int(infp.read())

    # count actuator feedback pulses
    def updatePosition(self):
        prior_feedback_val = self.feedback.value
        while True:
            time.sleep(0.001)
            current_feedback_value = self.feedback.value
            if current_feedback_value == 1 and prior_feedback_val == 0:
                current_pulse_time = time.time()
                if (current_pulse_time - self.last_pulse_time) > false_pulse_delay_actuator:  # debouncing
                    self.position = self.position + 1
                    self.last_pulse_time = current_pulse_time
                    print(self.position)
                    prior_feedback_val = current_feedback_value

    # write speed to actuator. 0<=value<=100
    def writeSpeed(self, value):
        if value == 0:
            self.ON.value = 0
        else:
            self.ON.value = 1

        self.dac.normalized_value = value/100.0

        # value = round(value/100*4095)
        # buf = bytearray(2)
        # buf[0] = (value >> 8) & 0xFF
        # buf[1] = value & 0xFF
        # self.i2c.writeto(0x60, buf)

    # define actuator speed as a function of the winch speed and distance to move
    def getSpeed(self, winch_speed, distance, manual_adjust):
        if manual_adjust:  # if maunally adjusting, base speed off distance to move
            if distance < 0.75:
                speed = 20
            else:
                speed = 100
        else:  # else if winching, base speed off winch speed
            x2 = self.calculateSpeed(distance, winch_speed, 0.33)
            speed = x2
        speed = round(max(25, min(100, speed)))  # bound speed from 20 <-> 100
        return speed

    # move actuator some distance in inches
    def move(self, winch_speed, direction=None, distance=None, manual_adjust=False):  # default to cable diameter
        if direction is None:
            direction = self.current_direction
        if distance is None:
            distance = self.cable_diameter
        self.position = 0   # zero position tracker
        self.direction.value = direction
        speed = self.getSpeed(winch_speed, distance, manual_adjust)
        self.writeSpeed(speed)  # write a speed
        self.time_init = time.time()
        self.last_pulse_time = self.time_init*1000
        target_pulses = (round(pulses_per_inch*distance - self.Overshoot(speed)))  # add some overshoot for time to turn off
        stationary_counter = 0
        # check counted pulses every 50 ms.
        while True:
            prior_position = self.position
            time.sleep(0.05)  # wait for actuator to move
            if self.position >= target_pulses:  # if we've reached our target,
                self.writeSpeed(0)  # turn off actuator
                self.direction.value = self.opposite(direction)  # bounce back
                break
            elif self.position == prior_position:  # if actuator is not moving
                stationary_counter = stationary_counter + 1
                if stationary_counter > 10:
                    self.writeSpeed(0)  # turn off actuator
                    if not manual_adjust:  # reverse direction only if currently winching
                        self.changeDirection()
                    break
            else:
                stationary_counter = 0

    # change actuator direction
    def changeDirection(self):
        self.current_direction = self.opposite(self.current_direction)
        self.direction.value = self.current_direction  # reverse direction
        self.line_stack_state = self.opposite(self.line_stack_state)
        with open('stacking_state.txt') as infp:
            infp.write(str(self.line_stack_state))

    # manually adjust level wind position
    def ManualAdjust(self, in_string):
        if in_string == 'cd':
            print('changing level wind direction')
            self.changeDirection()
        else:
            distance = float(in_string)
            print('level wind adjusting '+in_string+' inches')
            if distance > 0:
                direction = 0
            else:
                direction = 1
            self.move(0, direction, abs(distance), True)

    # reed switch for rotation tracking     
    def rotationTrackingReedSw(self):
        prior_reedsw_value = self.reed_sw.value
        while True:
            time.sleep(0.001)
            current_reedsw_value = self.reed_sw.value
            if current_reedsw_value == 1 and prior_reedsw_value == 0:
                current_reed_time = time.time()
                if (current_reed_time - self.last_reed_time) > false_pulse_delay_reed_sw:
                    time.sleep(0.005)
                    if self.reed_sw.value == 1:
                        self.last_reed_time = current_reed_time
                        self.NeedToMoveActuator = True  # move actuator one cable width
                        self.RotationCounter = self.RotationCounter + 1
                        prior_reedsw_value = current_reedsw_value

    # a useful function to flip booleans, ie 0 --> 1 and 1 --> 0
    def opposite(self, input):
        out = 1 - input
        return out

    # actuator tick overshoot as a function of actuator speed
    def Overshoot(self, x):
        c0 = 0.0157003
        c1 = 0.0705877
        c2 = -0.000215076
        out = c0 + c1*x + c2*x**2
        return out

    # return optimal actuator speed for a given distance, winch speed, % rotation time for actuation
    def calculateSpeed(self, x, winch_speed, perc):
        # actuator speed in in/s = c0 + c1*P + c2*P^2
        c0 = 0.054397
        c1 = 0.0178693
        c2 = -0.0000678376
        SPR = self.secondsPerRotation(winch_speed)
        T = perc*SPR
        P = (-c1 + (c1**2 + 4*c2*(x/T - c0))**0.5) / (2*c2)
        return P

    # seconds per spool rotation as a function of motor speed
    def secondsPerRotation(self, x):
        rpm = -11.8876 + 0.372714*x + 0.00207748*x**2.0
        out = 60/rpm
        return out
