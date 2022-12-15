import time
from machine import Pin, PWM, I2C

# constants for actuator
RETRACT = 1
EXTEND = 0
false_pulse_delay_actuator_feedback = 0 # (zero for no debounce delay)
pulses_per_inch = 48
false_pulse_delay_reed_sw = 250 # ms

class Actuator:

    def __init__(
        self,
        scl_pin,
        sda_pin,
        direction_pin,
        feedback_pin,
        rotation_pin,
        cable_diameter = 0.375
    ):

        self.i2c = I2C(
            1,
            scl=Pin(scl_pin),
            sda=Pin(sda_pin))
        self.direction = Pin(direction_pin, Pin.OUT)
        self.current_direction = EXTEND

        self.feedback = Pin(feedback_pin, Pin.IN, Pin.PULL_UP)
        self.feedback.irq(trigger=Pin.IRQ_RISING, handler=self.updatePosition)
        self.last_pulse_time = time.ticks_ms()-false_pulse_delay_actuator_feedback
        self.position = 0

        self.reed_sw = Pin(rotation_pin, Pin.IN, Pin.PULL_UP)
        self.reed_sw.irq(trigger=Pin.IRQ_RISING, handler=self.rotationTrackingReedSwitchTrigger)
        self.last_reed_time = time.ticks_ms()-false_pulse_delay_reed_sw
        self.NeedToMoveActuator = False

        self.cable_diameter = cable_diameter

        with open('stacking_state.txt') as infp:
            self.line_stack_state = int(infp.read())

# update actuator position
    def updatePosition(self, p):
        current_pulse_time = time.ticks_ms()
        if (current_pulse_time - self.last_pulse_time ) > false_pulse_delay_actuator_feedback: # debouncing
            self.position = self.position + 1
            self.last_pulse_time = current_pulse_time
            print("position: ", self.position," time: ", current_pulse_time*0.001)


# write speed to actuator. 0<=value<=100
    def writeSpeed(self, value):
        value = round(value/100*4095)
        buf=bytearray(2)
        buf[0]=(value >> 8) & 0xFF
        buf[1]=value & 0xFF
        self.i2c.writeto(0x60,buf)


# define actuator speed as a function of the winch speed and distance to move
    def getSpeed(self, winch_speed, distance):
        if winch_speed == 0: # if maunally adjusting, base speed off distance to move
            speed = distance/10*100
        else: # else if winching, base speed off winch speed
            x2 = self.calculateSpeed(distance, winch_speed, 0.33)
            print("Actuator speed: ",x2)
            speed = x2
        speed = round(max(20, min(100, speed))) # bound speed from 20 <-> 100
        return speed


# move actuator some distance in inches
    def move(self, winch_speed, direction = None, distance = None): # default to cable diameter
        if direction is None:
            direction = self.current_direction
        if distance is None:
            distance = self.cable_diameter
        self.position = 0   # zero position tracker
        self.direction.value(direction)
        speed = self.getSpeed(winch_speed, distance)
        self.writeSpeed(speed)  # write a speed
        target_pulses = (round(pulses_per_inch*distance + self.Overshoot(speed))) # add some overshoot to cancel bounceback
        stationary_counter = 0
        start_time = time.ticks_ms()
        # check counted pulses every 50 ms.
        while True:
            print("------------------------loop")
            prior_position = self.position
            time.sleep(0.05)  # wait for actuator to move
            if self.position >= target_pulses:  # if we've reached our target,
                print("target reached, bounceback initiated ... ")
                self.direction.value(self.opposite(direction)) # bounce back
                self.writeSpeed(0)  # turn off actuator
                break
            elif self.position == prior_position:  # if actuator is not moving
                stationary_counter = stationary_counter + 1
                if stationary_counter > 5:
                    print("edge detected")
                    if winch_speed > 0: # reverse direction only if currently winching
                        self.changeDirection()
                    self.writeSpeed(0)  # turn off actuator
                    break
            else:
                stationary_counter = 0


# change actuator direction
    def changeDirection(self):
        print("reversing direction")
        self.current_direction = self.opposite(self.current_direction)
        self.direction.value(self.current_direction) # reverse direction
        self.line_stack_state = self.opposite(self.line_stack_state)
        with open('stacking_state.txt') as infp:
            infp.write(str(self.line_stack_state))


# reed switch irq for rotation tracking
    def rotationTrackingReedSwitchTrigger(self,p):
        time.sleep_ms(10)
        current_reed_time = time.ticks_ms()
        if self.reed_sw.value() == 1 and (current_reed_time - self.last_reed_time ) > false_pulse_delay_reed_sw:
            self.last_reed_time = current_reed_time
            self.NeedToMoveActuator = True  # move actuator one cable width
            print("rotation reed switch triggered, time: ",current_reed_time*0.001)


# a useful function to flip booleans, ie 0 --> 1 and 1 --> 0
    def opposite(self, input):
        out = 1 - input
        return out


# actuator tick overshoot as a function of actuator speed
    def Overshoot(self, x):
        out = 0.695302 + 0.267472*x
        return out

# return optimal actuator speed for a given distance, winch speed, % rotation time for actuation
    def calculateSpeed(self, x, winch_speed, perc):
        # actuator speed in in/s = c0 + c1*P + c2*P^2
        c0 = 0.054397
        c1 = 0.0178693
        c2 = -0.0000678376
        SPR = self.secondsPerRotation(winch_speed)
        T = perc*SPR
        P = (-c1 + ( c1**2 + 4*c2*(x/T - c0) )**0.5 ) / (2*c2)
        return P

# seconds per spool rotation as a function of motor speed
    def secondsPerRotation(self, x):
        rpm = -11.8876 + 0.372714*x + 0.00207748*x**2.0
        out = 60/rpm
        return out
